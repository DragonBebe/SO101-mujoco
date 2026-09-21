"""Drawing the estimate back onto the image it came from.

An overlay is the cheapest way to see that a pose is wrong, and the most
misleading way to argue that it is right: a silhouette can sit perfectly on a
blob while the metric position is centimetres off, because reprojection error
and world error are different quantities.  These pictures are for spotting
failures and for the record; the accuracy numbers come from independent
measurements, never from here.

The two virtual environments have different imaging libraries -- OpenCV on
the real side, Pillow on the simulation side -- so the drawing calls go
through a small backend that uses whichever is installed.
"""
import numpy as np

from . import geometry

COLOURS = {'valid': (60, 230, 60), 'invalid': (240, 60, 60),
           'stale': (250, 190, 40), 'axis_x': (255, 80, 80),
           'axis_y': (80, 255, 80), 'axis_z': (90, 160, 255),
           'text': (255, 255, 255), 'shadow': (0, 0, 0)}


class _Canvas:
    """Lines, polygons and labels on an RGB array, on either backend."""

    def __init__(self, rgb):
        self.array = np.ascontiguousarray(rgb.copy())
        try:
            from PIL import Image, ImageDraw

            self._image = Image.fromarray(self.array)
            self._draw = ImageDraw.Draw(self._image)
            self.backend = 'pillow'
        except ImportError:
            import cv2

            self._cv2 = cv2
            self.backend = 'opencv'

    def line(self, start, end, colour, width=2):
        if self.backend == 'pillow':
            self._draw.line([tuple(map(float, start)), tuple(map(float, end))],
                            fill=tuple(colour), width=width)
        else:
            self._cv2.line(self.array, tuple(np.round(start).astype(int)),
                           tuple(np.round(end).astype(int)), tuple(colour), width)

    def polygon(self, points, colour, width=2):
        points = np.asarray(points, dtype=float)
        for index in range(len(points)):
            self.line(points[index], points[(index + 1) % len(points)], colour, width)

    def rectangle(self, box, colour, width=1):
        u0, v0, u1, v1 = (float(value) for value in box)
        for start, end in (((u0, v0), (u1, v0)), ((u1, v0), (u1, v1)),
                           ((u1, v1), (u0, v1)), ((u0, v1), (u0, v0))):
            self.line(start, end, colour, width)

    def text(self, position, lines, colour=COLOURS['text']):
        x, y = (float(value) for value in position)
        for index, line in enumerate(lines):
            at = (x, y + 15 * index)
            if self.backend == 'pillow':
                self._draw.text((at[0] + 1, at[1] + 1), line, fill=COLOURS['shadow'])
                self._draw.text(at, line, fill=tuple(colour))
            else:
                point = (int(at[0]), int(at[1]) + 12)
                self._cv2.putText(self.array, line, point,
                                  self._cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                                  COLOURS['shadow'], 3, self._cv2.LINE_AA)
                self._cv2.putText(self.array, line, point,
                                  self._cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                                  tuple(colour), 1, self._cv2.LINE_AA)

    def result(self):
        if self.backend == 'pillow':
            return np.asarray(self._image)
        return self.array


def draw_axes(canvas, calibration, position, yaw, length=0.03):
    """The block's own frame, so a yaw error is visible rather than inferred."""
    rotation = geometry.yaw_rotation(yaw)
    origin = np.asarray(position, dtype=float)
    tips = [origin + length * rotation[:, axis] for axis in range(3)]
    uv, depth = geometry.project_points(np.vstack([origin] + tips), calibration)
    if np.any(depth <= 0) or not np.all(np.isfinite(uv)):
        return
    for index, key in enumerate(('axis_x', 'axis_y', 'axis_z')):
        canvas.line(uv[0], uv[index + 1], COLOURS[key], 2)


def annotate(rgb, calibration, entries, header=()):
    """Draw every block entry of a scene record on its own frame."""
    canvas = _Canvas(rgb)
    for entry in entries:
        pose = entry.get('pose_filtered') or entry.get('pose')
        state = entry.get('state', 'detected' if entry.get('valid') else 'invalid')
        colour = COLOURS.get(state, COLOURS['invalid'] if not entry.get('valid', True)
                             else COLOURS['valid'])
        if entry.get('bbox'):
            canvas.rectangle(entry['bbox'], colour, 1)
        outline = entry.get('outline_uv')
        if outline:
            canvas.polygon(np.asarray(outline, dtype=float), colour, 2)
        if pose:
            draw_axes(canvas, calibration, pose['position_m'],
                      pose['yaw_rad'] if 'yaw_rad' in pose
                      else np.radians(pose['yaw_deg']))
            x, y, z = pose['position_m']
            label = [f"{entry.get('id', entry.get('block_id', '?'))} [{state}]",
                     f'x {1000 * x:.0f} y {1000 * y:.0f} z {1000 * z:.0f} mm',
                     f"yaw {pose['yaw_deg']:.1f} deg (mod "
                     f"{pose['symmetry_step_deg']:.0f})"]
            quality = entry.get('quality') or {}
            if 'iou' in quality:
                label.append(f"iou {quality['iou']:.2f}")
            if entry.get('age_s'):
                label.append(f"age {entry['age_s']:.1f} s")
        else:
            label = [f"{entry.get('id', entry.get('block_id', '?'))} [{state}]"]
        for reason in (entry.get('reasons') or [])[:2]:
            label.append(reason[:64])
        anchor = entry.get('bbox') or [20, 20, 20, 20]
        canvas.text((anchor[0], max(0, anchor[1] - 16 * len(label))), label, colour)
    if header:
        canvas.text((12, 12), list(header))
    return canvas.result()


def save(rgb, path):
    from pathlib import Path

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        from PIL import Image

        Image.fromarray(rgb).save(path)
    except ImportError:
        import cv2

        if not cv2.imwrite(str(path), rgb[:, :, ::-1]):
            raise RuntimeError(f'failed to write {path}')
    return path


def side_by_side(left, right, gap=8):
    """One picture of the real frame and the mirrored scene, same scale."""
    left, right = np.asarray(left), np.asarray(right)
    if left.shape[0] != right.shape[0]:
        scale = left.shape[0] / right.shape[0]
        width = int(round(right.shape[1] * scale))
        rows = (np.arange(left.shape[0]) / scale).astype(int)
        rows = np.clip(rows, 0, right.shape[0] - 1)
        columns = (np.arange(width) / scale).astype(int)
        columns = np.clip(columns, 0, right.shape[1] - 1)
        right = right[rows][:, columns]
    spacer = np.full((left.shape[0], gap, 3), 30, dtype=left.dtype)
    return np.concatenate([left, spacer, right], axis=1)
