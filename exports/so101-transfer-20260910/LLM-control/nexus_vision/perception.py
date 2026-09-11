"""Pure RGB-D geometry and rendered-color region proposals.

Pixel coordinates name pixel centers: ``[0, 0]`` is the center of the
top-left pixel.  ``cx`` and ``cy`` use that same convention.  Depth values
are metric optical-Z distances.  Camera rotation maps OpenGL camera-local
coordinates (right +X, up +Y, forward -Z) into world coordinates.

This module deliberately depends only on image arrays and camera calibration;
it has no access to simulator objects, poses, labels, or segmentation IDs.
"""

from collections.abc import Mapping

import numpy as np
from scipy import ndimage


_COLOR_HUE_RANGES = {
    "red": ((0.0, 15.0), (345.0, 360.0)),
    "orange": ((15.0, 45.0),),
    "yellow": ((45.0, 75.0),),
    "green": ((75.0, 170.0),),
    "blue": ((170.0, 270.0),),
}
_INTEGER_TOLERANCE = 1e-6


def _finite_float(value, name):
    raw = np.asarray(value)
    if raw.shape != () or np.iscomplexobj(raw):
        raise ValueError(f"calibration {name} must be a finite number")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"calibration {name} must be a finite number") from exc
    if not np.isfinite(result):
        raise ValueError(f"calibration {name} must be a finite number")
    return result


def _image_dimension(value, name):
    result = _finite_float(value, name)
    rounded = round(result)
    if result <= 0 or abs(result - rounded) > _INTEGER_TOLERANCE:
        raise ValueError(f"calibration {name} must be a positive integer")
    return int(rounded)


def _validated_calibration(calibration):
    if not isinstance(calibration, Mapping):
        raise ValueError("calibration must be a mapping")
    required = {"width", "height", "fx", "fy", "cx", "cy", "position", "rotation"}
    missing = required.difference(calibration)
    if missing:
        raise ValueError(f"calibration is missing {', '.join(sorted(missing))}")

    width = _image_dimension(calibration["width"], "width")
    height = _image_dimension(calibration["height"], "height")
    fx = _finite_float(calibration["fx"], "fx")
    fy = _finite_float(calibration["fy"], "fy")
    cx = _finite_float(calibration["cx"], "cx")
    cy = _finite_float(calibration["cy"], "cy")
    if fx <= 0 or fy <= 0:
        raise ValueError("calibration focal lengths must be positive")

    try:
        position = np.asarray(calibration["position"], dtype=float)
        rotation = np.asarray(calibration["rotation"], dtype=float)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("calibration position and rotation must be numeric") from exc
    if position.shape != (3,) or not np.all(np.isfinite(position)):
        raise ValueError("calibration position must be a finite length-3 vector")
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise ValueError("calibration rotation must be a finite 3x3 matrix")

    return width, height, fx, fy, cx, cy, position, rotation


def _validated_depth(depth, width, height):
    if not isinstance(depth, np.ndarray) or depth.ndim != 2:
        raise ValueError("depth must be a two-dimensional numpy array")
    if depth.shape != (height, width):
        raise ValueError("depth shape must match calibration height and width")
    if not np.issubdtype(depth.dtype, np.number) or np.issubdtype(
        depth.dtype, np.complexfloating
    ):
        raise ValueError("depth must contain real numeric values")


def _pixel_index(pixel, width, height):
    try:
        coordinates = np.asarray(pixel, dtype=float)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("pixel must contain two finite integer coordinates") from exc
    if coordinates.shape != (2,) or not np.all(np.isfinite(coordinates)):
        raise ValueError("pixel must contain two finite integer coordinates")
    rounded = np.rint(coordinates)
    if not np.all(np.abs(coordinates - rounded) <= _INTEGER_TOLERANCE):
        raise ValueError("pixel coordinates must be integer-valued")
    u, v = (int(value) for value in rounded)
    if not (0 <= u < width and 0 <= v < height):
        raise ValueError("pixel lies outside the calibrated image")
    return u, v


def unproject_pixel(depth, calibration, pixel):
    """Return the visible surface point at ``pixel`` in world coordinates.

    ``depth`` must be an HxW array registered to ``calibration``.  A sampled
    depth must be finite and strictly positive.  Integer-valued floats are
    accepted for JSON-friendly callers; fractional or out-of-bounds pixels
    raise :class:`ValueError`.
    """

    width, height, fx, fy, cx, cy, position, rotation = _validated_calibration(
        calibration
    )
    _validated_depth(depth, width, height)
    u, v = _pixel_index(pixel, width, height)
    metric_depth = float(depth[v, u])
    if not np.isfinite(metric_depth) or metric_depth <= 0:
        raise ValueError("depth at pixel must be finite and positive")

    local = np.array(
        [
            (u - cx) * metric_depth / fx,
            -(v - cy) * metric_depth / fy,
            -metric_depth,
        ]
    )
    return position + rotation @ local


def project_world(point, calibration):
    """Project a world point to ``[u, v, optical_depth]``.

    The returned pixel coordinates are continuous and may fall outside the
    image.  Points on or behind the camera plane have no visible projection
    and raise :class:`ValueError`.
    """

    _, _, fx, fy, cx, cy, position, rotation = _validated_calibration(calibration)
    try:
        world = np.asarray(point, dtype=float)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("point must be a finite length-3 vector") from exc
    if world.shape != (3,) or not np.all(np.isfinite(world)):
        raise ValueError("point must be a finite length-3 vector")

    # A rotation matrix maps local to world, so its inverse is its transpose.
    local = rotation.T @ (world - position)
    metric_depth = -float(local[2])
    if not np.isfinite(metric_depth) or metric_depth <= 0:
        raise ValueError("point must lie in front of the camera")
    u = cx + fx * local[0] / metric_depth
    v = cy - fy * local[1] / metric_depth
    return np.array([u, v, metric_depth])


def _normalized_rgb(rgb):
    if not isinstance(rgb, np.ndarray) or rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError("RGB image must be a numpy array with shape HxWx3")
    if not np.issubdtype(rgb.dtype, np.number) or np.issubdtype(
        rgb.dtype, np.complexfloating
    ):
        raise ValueError("RGB image must contain real numeric values")
    values = rgb.astype(float, copy=False)
    if not np.all(np.isfinite(values)):
        raise ValueError("RGB image must contain only finite values")
    minimum = float(values.min()) if values.size else 0.0
    maximum = float(values.max()) if values.size else 0.0
    if minimum < 0 or maximum > 255:
        raise ValueError("RGB values must lie in [0, 1] or [0, 255]")
    if maximum > 1.0:
        values = values / 255.0
    return values


def _hsv_channels(rgb):
    maximum = rgb.max(axis=2)
    minimum = rgb.min(axis=2)
    chroma = maximum - minimum
    saturation = np.divide(
        chroma,
        maximum,
        out=np.zeros_like(chroma),
        where=maximum > 0,
    )

    hue = np.zeros_like(maximum)
    colored = chroma > 0
    red_max = colored & (rgb[:, :, 0] == maximum)
    green_max = colored & ~red_max & (rgb[:, :, 1] == maximum)
    blue_max = colored & ~red_max & ~green_max
    hue[red_max] = (
        60.0 * (rgb[:, :, 1][red_max] - rgb[:, :, 2][red_max]) / chroma[red_max]
    ) % 360.0
    hue[green_max] = 60.0 * (
        (rgb[:, :, 2][green_max] - rgb[:, :, 0][green_max]) / chroma[green_max]
        + 2.0
    )
    hue[blue_max] = 60.0 * (
        (rgb[:, :, 0][blue_max] - rgb[:, :, 1][blue_max]) / chroma[blue_max]
        + 4.0
    )
    return hue, saturation, maximum


def locate_color(rgb, depth, calibration, color):
    """Return connected rendered-color proposals with surface coordinates.

    Supported colors are red, orange, yellow, green, and blue.  Each result
    contains a representative ``pixel``, a half-open ``bbox``, RGB mask
    ``area``, and ``surface_world`` derived from registered depth.  The
    representative is the valid-depth component pixel nearest its RGB
    centroid.  Components with no finite positive depth are omitted.
    Results are ordered by decreasing area, then image position.
    """

    if not isinstance(color, str) or color.strip().lower() not in _COLOR_HUE_RANGES:
        raise ValueError(f"unsupported color {color!r}")
    color = color.strip().lower()
    image = _normalized_rgb(rgb)
    width, height, *_ = _validated_calibration(calibration)
    if image.shape[:2] != (height, width):
        raise ValueError("RGB shape must match calibration height and width")
    if not isinstance(depth, np.ndarray) or depth.ndim != 2:
        raise ValueError("depth must be a two-dimensional numpy array registered to RGB")
    if depth.shape != image.shape[:2]:
        raise ValueError("depth and RGB must be registered with matching shapes")
    _validated_depth(depth, width, height)

    hue, saturation, value = _hsv_channels(image)
    mask = (saturation >= 0.35) & (value >= 0.20)
    hue_mask = np.zeros(mask.shape, dtype=bool)
    for lower, upper in _COLOR_HUE_RANGES[color]:
        hue_mask |= (hue >= lower) & (hue < upper)
    mask &= hue_mask

    labels, component_count = ndimage.label(
        mask, structure=np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]])
    )
    slices = ndimage.find_objects(labels)
    proposals = []
    for component_id in range(1, component_count + 1):
        coordinates = np.argwhere(labels == component_id)
        component_depth = depth[coordinates[:, 0], coordinates[:, 1]]
        valid = np.isfinite(component_depth) & (component_depth > 0)
        if not np.any(valid):
            continue

        centroid = coordinates.mean(axis=0)
        candidates = coordinates[valid]
        distances = np.sum((candidates - centroid) ** 2, axis=1)
        v, u = (int(value) for value in candidates[np.argmin(distances)])
        component_slice = slices[component_id - 1]
        proposals.append(
            {
                "pixel": [u, v],
                "bbox": [
                    component_slice[1].start,
                    component_slice[0].start,
                    component_slice[1].stop,
                    component_slice[0].stop,
                ],
                "area": int(coordinates.shape[0]),
                "surface_world": unproject_pixel(depth, calibration, [u, v]).tolist(),
            }
        )

    proposals.sort(key=lambda item: (-item["area"], item["bbox"][1], item["bbox"][0]))
    return proposals
