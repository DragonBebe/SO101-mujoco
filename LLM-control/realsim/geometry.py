"""Projection, convex silhouettes and yaw conventions, in plain NumPy.

Everything here is deliberately free of OpenCV and MuJoCo so the same code
runs on a real undistorted frame and on a rendered one.

Camera convention
    Identical to :mod:`nexus_vision.perception`: ``rotation`` maps OpenGL
    camera axes (x right, y up, z backward) into the world, the optical axis
    is the camera's -z, and ``[0, 0]`` is the centre of the top-left pixel.
    :func:`project_points` is the batched form of
    ``perception.project_world`` and is tested against it.

Yaw convention
    A block lying flat on the support plane has only one rotational degree of
    freedom left, a rotation about the plane normal.  ``yaw`` is that angle in
    radians, counter-clockwise seen from above, applied in the table frame.

Symmetry
    A box's silhouette is unchanged by yaw steps that map the box onto
    itself: 90 degrees for a square footprint, 180 degrees otherwise.  An
    estimator cannot tell those apart, so every yaw this package reports is
    folded into ``[0, step)`` by :func:`canonical_yaw`.  Reporting a raw
    argmin instead is what makes an unmoved cube appear to jump by 90
    degrees between frames.
"""
import numpy as np


def project_points(points, calibration):
    """Return ``(uv, depth)`` for world points of any leading shape.

    ``depth`` is the metric optical-Z distance; points at or behind the
    camera plane get a non-positive depth and a meaningless ``uv``.
    """
    fx = float(calibration['fx'])
    fy = float(calibration['fy'])
    cx = float(calibration['cx'])
    cy = float(calibration['cy'])
    position = np.asarray(calibration['position'], dtype=float)
    rotation = np.asarray(calibration['rotation'], dtype=float)
    local = (np.asarray(points, dtype=float) - position) @ rotation
    depth = -local[..., 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        u = cx + fx * local[..., 0] / depth
        v = cy - fy * local[..., 1] / depth
    return np.stack([u, v], axis=-1), depth


def yaw_rotation(yaw):
    """Rotation about +z by ``yaw`` radians."""
    cos, sin = float(np.cos(yaw)), float(np.sin(yaw))
    return np.array([[cos, -sin, 0.0], [sin, cos, 0.0], [0.0, 0.0, 1.0]])


def yaw_quaternion(yaw):
    """MuJoCo-order quaternion ``[w, x, y, z]`` for a rotation about +z."""
    half = float(yaw) / 2.0
    return np.array([np.cos(half), 0.0, 0.0, np.sin(half)])


def quaternion_yaw(quaternion):
    """Inverse of :func:`yaw_quaternion` for a pure-yaw ``[w, x, y, z]``."""
    w, _, _, z = (float(value) for value in quaternion)
    return float(np.arctan2(2.0 * w * z, 1.0 - 2.0 * z * z))


def box_corners(half_extent, centre, yaw):
    """The eight corners of a yawed box, in the frame ``centre`` is given in.

    ``centre`` is the box's geometric centre, which is also the origin of a
    MuJoCo free-joint body holding a centred box geom, so no origin offset
    has to be applied anywhere else.
    """
    half = np.asarray(half_extent, dtype=float).reshape(3)
    signs = np.array([[sx, sy, sz] for sx in (-1.0, 1.0)
                      for sy in (-1.0, 1.0) for sz in (-1.0, 1.0)])
    return np.asarray(centre, dtype=float) + (signs * half) @ yaw_rotation(yaw).T


def convex_hull(points):
    """Counter-clockwise convex hull (monotone chain) of 2-D points.

    'Counter-clockwise' is in the coordinate system given; on an image, whose
    v axis points down, that reads as clockwise on screen.  Only consistency
    matters: :func:`polygon_coverage` and :func:`polygon_area` agree with it.
    """
    points = np.unique(np.asarray(points, dtype=float).reshape(-1, 2), axis=0)
    if len(points) <= 2:
        return points
    order = np.lexsort((points[:, 1], points[:, 0]))
    ordered = points[order]

    def chain(sequence):
        stack = []
        for point in sequence:
            while len(stack) >= 2:
                first, second = stack[-2], stack[-1]
                cross = ((second[0] - first[0]) * (point[1] - first[1]) -
                         (second[1] - first[1]) * (point[0] - first[0]))
                if cross > 0:
                    break
                stack.pop()
            stack.append(point)
        return stack

    lower = chain(ordered)
    upper = chain(ordered[::-1])
    return np.array(lower[:-1] + upper[:-1])


def polygon_area(polygon):
    """Shoelace area.  Exact, and unaffected by any rasterisation window."""
    polygon = np.asarray(polygon, dtype=float)
    if len(polygon) < 3:
        return 0.0
    x, y = polygon[:, 0], polygon[:, 1]
    return float(abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))) / 2.0)


def polygon_bounds(polygon):
    """Integer pixel bounds ``(u0, v0, u1, v1)`` containing a polygon."""
    polygon = np.asarray(polygon, dtype=float)
    low = np.floor(polygon.min(axis=0)).astype(int)
    high = np.ceil(polygon.max(axis=0)).astype(int) + 1
    return int(low[0]), int(low[1]), int(high[0]), int(high[1])


def polygon_coverage(polygon, window, samples=2):
    """Per-pixel area fraction of a convex polygon inside a pixel window.

    ``window`` is ``(u0, v0, width, height)`` in whole pixels.  Each pixel is
    probed on a ``samples x samples`` sub-grid, which turns the pixel-quantised
    overlap into a continuous function of the pose -- without it a local
    optimiser walks over a staircase and stops on the first flat step.
    """
    u0, v0, width, height = (int(value) for value in window)
    coverage = np.zeros((height, width), dtype=float)
    polygon = np.asarray(polygon, dtype=float)
    if len(polygon) < 3 or width <= 0 or height <= 0:
        return coverage
    offsets = (np.arange(samples) + 0.5) / samples - 0.5
    columns = u0 + np.arange(width)
    rows = v0 + np.arange(height)
    for du in offsets:
        for dv in offsets:
            u = (columns + du)[None, :]
            v = (rows + dv)[:, None]
            inside = np.ones((height, width), dtype=bool)
            for index in range(len(polygon)):
                ax, ay = polygon[index]
                bx, by = polygon[(index + 1) % len(polygon)]
                inside &= (bx - ax) * (v - ay) - (by - ay) * (u - ax) >= 0
                if not inside.any():
                    break
            coverage += inside
    return coverage / (samples * samples)


def silhouette(half_extent, centre, yaw, calibration):
    """Image-space outline of a yawed box, or ``None`` if it is not in front.

    A box is convex, so the convex hull of its projected corners is exactly
    its silhouette -- no hidden-surface reasoning is needed.
    """
    corners = box_corners(half_extent, centre, yaw)
    uv, depth = project_points(corners, calibration)
    if not np.all(np.isfinite(uv)) or np.any(depth <= 1e-6):
        return None
    return convex_hull(uv)


def canonical_yaw(yaw, step):
    """Fold ``yaw`` (radians) into ``[0, step)``; ``step`` is the symmetry."""
    step = float(step)
    if step <= 0:
        raise ValueError('symmetry step must be positive')
    return float(np.mod(float(yaw), step))


def yaw_difference(first, second, step):
    """Smallest angle between two yaws that the symmetry cannot distinguish.

    The number to quote as an orientation error: for a cube it is at most 45
    degrees however the two angles are written.
    """
    step = float(step)
    delta = np.mod(float(first) - float(second), step)
    return float(min(delta, step - delta))


def circular_mean(angles, step, weights=None):
    """Mean of angles that live on a circle of circumference ``step``.

    Mapping the angle onto a full turn before averaging is what keeps values
    just either side of the wrap point from averaging to the opposite side.
    """
    angles = np.asarray(angles, dtype=float)
    scaled = angles * (2 * np.pi / step)
    if weights is None:
        weights = np.ones_like(angles)
    weights = np.asarray(weights, dtype=float)
    mean = np.arctan2(float(np.sum(weights * np.sin(scaled))),
                      float(np.sum(weights * np.cos(scaled))))
    return canonical_yaw(mean * step / (2 * np.pi), step)


def convex_spans(polygon, rows):
    """Horizontal extent of a convex polygon at each scan line in ``rows``.

    Returns ``(left, right, hit)``.  Rasterising a convex silhouette this way
    costs one pass over its edges per scan line instead of a test per pixel,
    which is what makes a pose search over thousands of candidates practical.
    """
    polygon = np.asarray(polygon, dtype=float)
    if len(polygon) < 3:
        empty = np.zeros(len(rows))
        return empty, empty, np.zeros(len(rows), dtype=bool)
    start = polygon
    end = np.roll(polygon, -1, axis=0)
    ay, by = start[:, 1][:, None], end[:, 1][:, None]
    ax, bx = start[:, 0][:, None], end[:, 0][:, None]
    v = np.asarray(rows, dtype=float)[None, :]
    span = by - ay
    with np.errstate(divide='ignore', invalid='ignore'):
        fraction = (v - ay) / span
    crosses = (span != 0) & (fraction >= 0) & (fraction <= 1)
    x = ax + fraction * (bx - ax)
    left = np.where(crosses, x, np.inf).min(axis=0)
    right = np.where(crosses, x, -np.inf).max(axis=0)
    hit = crosses.any(axis=0) & np.isfinite(left) & np.isfinite(right) & (right >= left)
    return np.where(hit, left, 0.0), np.where(hit, right, 0.0), hit


class MaskScorer:
    """Overlap of convex silhouettes with one fixed boolean mask.

    The mask is summarised once as a per-row cumulative sum, so the area it
    shares with a polygon is read off by interpolating that sum at the
    polygon's left and right edge on each scan line.  Interpolating gives
    fractional pixels, which keeps the score a continuous function of the
    pose -- a pixel-quantised score is a staircase that a local optimiser
    cannot descend.

    Only the mask's own bounding box is summarised.  A polygon reaching
    outside it therefore contributes no intersection there, which is correct:
    the mask is empty outside its bounding box.
    """

    def __init__(self, mask, subrows=4):
        rows, columns = np.nonzero(mask)
        if not len(rows):
            raise ValueError('an empty mask cannot score a silhouette')
        self.origin = (int(columns.min()), int(rows.min()))
        self.width = int(columns.max()) - self.origin[0] + 1
        self.height = int(rows.max()) - self.origin[1] + 1
        patch = mask[self.origin[1]:self.origin[1] + self.height,
                     self.origin[0]:self.origin[0] + self.width].astype(float)
        self.cumulative = np.concatenate(
            [np.zeros((self.height, 1)), np.cumsum(patch, axis=1)], axis=1)
        self.mask_area = float(patch.sum())
        self.subrows = int(subrows)
        offsets = (np.arange(self.subrows) + 0.5) / self.subrows
        self.row_index = np.repeat(np.arange(self.height), self.subrows)
        # Pixel centres are integers, so pixel row ``r`` spans
        # ``[r - 0.5, r + 0.5)``; sampling ``[r, r + 1)`` instead would shift
        # every score half a pixel up the image.
        self.scan_rows = (self.origin[1] + self.row_index - 0.5 +
                          np.tile(offsets, self.height))

    def _lookup(self, position, rows):
        """Mask area of each row left of a fractional column position."""
        position = np.clip(position, 0.0, float(self.width))
        low = np.floor(position).astype(int)
        fraction = position - low
        low = np.minimum(low, self.width)
        high = np.minimum(low + 1, self.width)
        return (self.cumulative[rows, low] * (1.0 - fraction) +
                self.cumulative[rows, high] * fraction)

    def intersection(self, polygon):
        left, right, hit = convex_spans(polygon, self.scan_rows)
        if not hit.any():
            return 0.0
        rows = self.row_index[hit]
        # Pixel centres are integers, so pixel ``c`` spans ``[c-0.5, c+0.5]``
        # and the cumulative sum is indexed by the left edge of that span.
        offset = 0.5 - self.origin[0]
        covered = (self._lookup(right[hit] + offset, rows) -
                   self._lookup(left[hit] + offset, rows))
        return float(covered.sum() / self.subrows)

    def iou(self, polygon):
        """``(iou, model_area_px)``; the model area is exact, not rasterised."""
        model_area = polygon_area(polygon)
        if model_area <= 0:
            return 0.0, model_area
        shared = self.intersection(polygon)
        union = self.mask_area + model_area - shared
        return (shared / union if union > 0 else 0.0), model_area
