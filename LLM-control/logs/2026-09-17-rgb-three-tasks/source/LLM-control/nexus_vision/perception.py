"""Pure RGB(-D) geometry and rendered-color region proposals.

Two independent ways to get world coordinates out of an image live here.

:func:`unproject_pixel` needs registered depth and is exact for whatever
surface the pixel shows.

:func:`intersect_pixel_with_plane` and :func:`plane_support_estimate` need no
depth at all: they combine the camera calibration with an assumption the
caller states out loud, that the imaged surface lies on a known horizontal
plane, or that an object of known height stands upright on one.  That
assumption is what buys the third dimension, so it is carried in the result
and it is wrong exactly when the assumption is: a point on top of an object
of unknown height needs that height, not this plane.

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


def _camera_basis(calibration):
    """Ray origin and the local-to-world rotation, both validated."""
    _, _, fx, fy, cx, cy, position, rotation = _validated_calibration(calibration)
    return fx, fy, cx, cy, position, rotation


def _plane_intersections(calibration, pixels, plane_z):
    """World points where rays through ``pixels`` meet the plane ``z=plane_z``.

    ``pixels`` is an array of ``[u, v]`` pixel centres with any leading shape.
    Returns ``(points, valid)``; a ray parallel to the plane, or pointing away
    from it, is marked invalid and its point is meaningless.
    """

    fx, fy, cx, cy, position, rotation = _camera_basis(calibration)
    uv = np.asarray(pixels, dtype=float)
    if uv.shape[-1] != 2:
        raise ValueError("pixels must have a trailing axis of size 2")
    local = np.stack(
        [
            (uv[..., 0] - cx) / fx,
            -(uv[..., 1] - cy) / fy,
            -np.ones(uv.shape[:-1]),
        ],
        axis=-1,
    )
    # ``rotation`` maps local to world, so a batch multiplies by its transpose.
    direction = local @ rotation.T
    with np.errstate(divide="ignore", invalid="ignore"):
        distance = (float(plane_z) - position[2]) / direction[..., 2]
    valid = np.isfinite(distance) & (distance > 0)
    points = position + np.where(valid, distance, 0.0)[..., None] * direction
    return points, valid


def intersect_pixel_with_plane(calibration, pixel, plane_z=0.0):
    """Return where the ray through ``pixel`` meets the plane ``z=plane_z``.

    Depth-free monocular geometry: the calibration fixes the ray and the
    caller's plane fixes the range along it.  The result is a world point on
    that plane, correct only for image content that really lies on it.
    """

    width, height, *_ = _validated_calibration(calibration)
    plane_z = _finite_float(plane_z, "plane_z")
    u, v = _pixel_index(pixel, width, height)
    point, valid = _plane_intersections(calibration, [u, v], plane_z)
    if not valid:
        raise ValueError(
            "the ray through this pixel never reaches the assumed plane"
        )
    return point


def _project_points(calibration, points):
    """Project world points to ``(uv, depth)`` with any leading shape."""
    fx, fy, cx, cy, position, rotation = _camera_basis(calibration)
    local = (np.asarray(points, dtype=float) - position) @ rotation
    depth = -local[..., 2]
    with np.errstate(divide="ignore", invalid="ignore"):
        u = cx + fx * local[..., 0] / depth
        v = cy - fy * local[..., 1] / depth
    return np.stack([u, v], axis=-1), depth


def _mask_contains(mask, uv, depth):
    """True where a projection lands on a set mask pixel in front of the camera."""
    height, width = mask.shape
    column = np.rint(uv[..., 0])
    row = np.rint(uv[..., 1])
    inside = (
        np.isfinite(column)
        & np.isfinite(row)
        & (depth > 0)
        & (column >= 0)
        & (column < width)
        & (row >= 0)
        & (row < height)
    )
    column = np.where(inside, column, 0).astype(int)
    row = np.where(inside, row, 0).astype(int)
    return inside & mask[row, column]


def plane_support_estimate(mask, calibration, plane_z, object_height, cell=0.002):
    """Locate an upright object of known height standing on a known plane.

    A two-plane carve that uses only the colour mask and the calibration: a
    candidate point on the support plane can belong to the object's footprint
    only if both it *and* the point ``object_height`` above it project inside
    the mask.  One silhouette bounds the footprint from outside, and for an
    upright convex object the bound is tight, because the two constraints
    err in opposite directions -- the ground projection stretches away from
    the camera, the raised one toward it.

    Returns ``None`` when nothing survives (mask too small, object not on the
    plane, or the camera cannot see the plane), otherwise a dict with the
    footprint ``center`` on the plane, its ``extent`` in metres, the
    ``grasp_center`` at half the object's height, and the cell count.
    """

    plane_z = _finite_float(plane_z, "plane_z")
    object_height = _finite_float(object_height, "object_height")
    if object_height <= 0:
        raise ValueError("object_height must be positive")
    cell = _finite_float(cell, "cell")
    if cell <= 0:
        raise ValueError("cell must be positive")

    pixels = np.argwhere(mask)[:, ::-1]  # (u, v) order
    if not pixels.size:
        return None
    ground, valid = _plane_intersections(calibration, pixels, plane_z)
    ground = ground[valid]
    if not ground.size:
        return None

    low = ground[:, :2].min(axis=0) - cell
    high = ground[:, :2].max(axis=0) + cell
    # The silhouette's ground shadow can be long; cap the grid, never the area.
    steps = np.maximum(np.ceil((high - low) / cell).astype(int), 1)
    if steps.prod() > 250_000:
        cell = float(np.sqrt((high - low).prod() / 250_000))
        steps = np.maximum(np.ceil((high - low) / cell).astype(int), 1)
    axes = [low[axis] + cell * (np.arange(steps[axis]) + 0.5) for axis in (0, 1)]
    grid = np.stack(np.meshgrid(*axes, indexing="ij"), axis=-1).reshape(-1, 2)
    base = np.column_stack((grid, np.full(len(grid), plane_z)))
    raised = base + [0.0, 0.0, object_height]

    keep = _mask_contains(mask, *_project_points(calibration, base))
    keep &= _mask_contains(mask, *_project_points(calibration, raised))
    if not keep.any():
        return None

    footprint = grid[keep]
    center = footprint.mean(axis=0)
    extent = footprint.max(axis=0) - footprint.min(axis=0) + cell
    return {
        "center": [float(center[0]), float(center[1]), plane_z],
        "extent": [float(extent[0]), float(extent[1])],
        "grasp_center": [float(center[0]), float(center[1]),
                         plane_z + object_height / 2],
        "cells": int(keep.sum()),
        "cell_size": float(cell),
    }


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


def locate_color(rgb, depth, calibration, color, plane_z=None, object_height=None):
    """Return connected rendered-color proposals from an image.

    Supported colors are red, orange, yellow, green, and blue.  Each result
    contains a representative ``pixel``, a half-open ``bbox`` and RGB mask
    ``area``.

    ``depth`` may be ``None``, which is how an RGB-only camera reports that
    it has no range data.  The proposals are then purely two-dimensional:
    they carry no ``surface_world`` key, and the representative pixel is
    simply the one nearest the mask centroid.  With registered depth, every
    result also carries ``surface_world``, the representative is the nearest
    valid-depth pixel to the centroid, and components with no finite positive
    depth are omitted.

    ``object_height`` opts into the depth-free estimate of
    :func:`plane_support_estimate` for every component: each result then also
    carries ``support`` with the object's centre on the plane ``plane_z``
    (default 0) and a ``grasp_center`` at half its height.  It is an estimate
    under a stated assumption, not a measurement, and it is available with or
    without depth so the two can be compared.

    Results are ordered by decreasing area, then image position.
    """

    if not isinstance(color, str) or color.strip().lower() not in _COLOR_HUE_RANGES:
        raise ValueError(f"unsupported color {color!r}")
    color = color.strip().lower()
    image = _normalized_rgb(rgb)
    width, height, *_ = _validated_calibration(calibration)
    if image.shape[:2] != (height, width):
        raise ValueError("RGB shape must match calibration height and width")
    if depth is not None:
        if not isinstance(depth, np.ndarray) or depth.ndim != 2:
            raise ValueError(
                "depth must be a two-dimensional numpy array registered to RGB"
            )
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
        candidates = coordinates
        if depth is not None:
            component_depth = depth[coordinates[:, 0], coordinates[:, 1]]
            valid = np.isfinite(component_depth) & (component_depth > 0)
            if not np.any(valid):
                continue
            candidates = coordinates[valid]

        centroid = coordinates.mean(axis=0)
        distances = np.sum((candidates - centroid) ** 2, axis=1)
        v, u = (int(value) for value in candidates[np.argmin(distances)])
        component_slice = slices[component_id - 1]
        proposal = {
            "pixel": [u, v],
            "bbox": [
                component_slice[1].start,
                component_slice[0].start,
                component_slice[1].stop,
                component_slice[0].stop,
            ],
            "area": int(coordinates.shape[0]),
        }
        if depth is not None:
            proposal["surface_world"] = unproject_pixel(
                depth, calibration, [u, v]
            ).tolist()
        if object_height is not None:
            support = plane_support_estimate(
                labels == component_id, calibration,
                0.0 if plane_z is None else plane_z, object_height,
            )
            if support is not None:
                support["assumed_plane_z"] = 0.0 if plane_z is None else float(plane_z)
                support["assumed_object_height"] = float(object_height)
                support["method"] = "rgb_two_plane_carve"
            proposal["support"] = support
        proposals.append(proposal)

    proposals.sort(key=lambda item: (-item["area"], item["bbox"][1], item["bbox"][0]))
    return proposals
