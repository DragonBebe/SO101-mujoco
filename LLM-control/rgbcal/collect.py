"""Stage 2: collecting board samples that actually constrain the model.

Thirty images of the board lying in the middle of the frame, all at the
same distance and angle, produce a calibration that fits beautifully and
predicts badly.  What fixes the focal length is *tilt*; what fixes the
distortion coefficients is board area near the *edges and corners*; what
exposes overfitting is a set of images the solver never saw.

So this collector is opinionated: it scores every candidate before it is
written, refuses blurred and near-duplicate frames, shows which ninth of
the frame is still empty, and assigns the held-out validation split at
capture time -- before any error is known, so the split cannot be chosen
to flatter the result.  Raw frames are written once and never rewritten.
"""
from pathlib import Path
import json

import cv2
import numpy as np

from . import boards as boards_module
from .capture import (RealCamera, exposure_report, save_frame, sharpness,
                      timestamp, transform_image)
from .preview import _put, FrameRate

#: Every Nth accepted sample is held out for validation.  Interleaving keeps
#: the two sets equally varied; a trailing block would hold out only the
#: poses that happened to come last.
VALIDATION_EVERY = 5
BINS = 3
KEY_HELP = 's save | h hud | q finish'


def quad(board, points):
    """The board's four outer corners, in detection order."""
    if board.kind == 'chessboard':
        grid = points.reshape(board.inner_rows, board.inner_cols, 2)
        return np.array([grid[0, 0], grid[0, -1], grid[-1, -1], grid[-1, 0]])
    hull = cv2.convexHull(points.astype(np.float32)).reshape(-1, 2)
    rect = cv2.minAreaRect(hull)
    return cv2.boxPoints(rect)


def planarity_residual(object_points, image_points):
    """How badly a flat-board model fails to explain the detected corners.

    A planar board seen through a pinhole maps to the image by a homography.
    Lens distortion bends that mapping, but a *bent board* breaks it far
    harder, and a sheet of paper held in the hand bends by millimetres.  The
    calibrator cannot tell the two apart, and silently charges the bending
    to the distortion coefficients, so this catches it at capture time.

    The residual also grows with real distortion near the frame edges, which
    is why the rejection threshold is generous: it is there to catch a bowed
    sheet, not to grade the lens.
    """
    source = np.asarray(object_points, dtype=np.float32)[:, :2]
    target = np.asarray(image_points, dtype=np.float32)
    homography, _ = cv2.findHomography(source, target, 0)
    if homography is None:
        return float('inf')
    predicted = cv2.perspectiveTransform(source.reshape(-1, 1, 2),
                                         homography).reshape(-1, 2)
    return float(np.sqrt(((predicted - target) ** 2).sum(axis=1).mean()))


def tilt_proxy(corners):
    """How far from fronto-parallel, without needing intrinsics.

    Under perspective the far edge of a tilted board images shorter than the
    near one.  The larger of the two opposite-edge ratios is a monotone
    stand-in for tilt: 1.0 is square-on, and samples that all sit near 1.0
    leave the focal length poorly determined.
    """
    edges = [np.linalg.norm(corners[index] - corners[(index + 1) % 4])
             for index in range(4)]
    horizontal = max(edges[0], edges[2]) / max(min(edges[0], edges[2]), 1e-6)
    vertical = max(edges[1], edges[3]) / max(min(edges[1], edges[3]), 1e-6)
    return float(max(horizontal, vertical))


class Collector:
    """Scores candidates and remembers what has already been accepted."""

    def __init__(self, board, width, height, min_sharpness=60.0,
                 min_separation=70.0, target=30, split=8, max_warp=2.0):
        self.board = board
        self.width, self.height = width, height
        self.min_sharpness = min_sharpness
        self.min_separation = min_separation
        self.max_warp = max_warp
        self.target, self.split = target, split
        self.samples = []
        self.coverage = np.zeros((BINS, BINS), dtype=int)

    def _separation(self, corners):
        """Smallest mean corner distance to any accepted sample."""
        if not self.samples:
            return float('inf')
        previous = np.array([sample['corners'] for sample in self.samples])
        return float(np.min(np.linalg.norm(previous - corners, axis=2).mean(axis=1)))

    def score(self, image):
        """Judge a candidate frame; returns ``(ok, reason, facts)``.

        Blur is measured on the board only.  A whole-frame score mostly
        measures the scene -- an empty white table scores low when perfectly
        sharp -- so it would reject a small, distant, sharp board and accept
        a large blurred one.
        """
        detection = boards_module.detect(self.board, image)
        focus = sharpness(image)
        if detection.ok:
            low = np.maximum(detection.image_points.min(axis=0) - 15, 0).astype(int)
            high = np.minimum(detection.image_points.max(axis=0) + 15,
                              [self.width, self.height]).astype(int)
            focus = sharpness(image[low[1]:high[1], low[0]:high[0]])
        facts = {'sharpness': focus, 'detected': detection.ok,
                 'points': detection.count, 'note': detection.note}
        if not detection.ok:
            return False, f'board not detected ({detection.note})', facts
        corners = quad(self.board, detection.image_points)
        separation = self._separation(corners)
        warp = planarity_residual(detection.object_points, detection.image_points)
        facts.update({'tilt': tilt_proxy(corners), 'separation_px': separation,
                      'warp_px': warp,
                      'centroid': detection.image_points.mean(axis=0).tolist(),
                      'corners': corners})
        if focus < self.min_sharpness:
            return False, f'too blurred ({focus:.0f} < {self.min_sharpness:.0f})', facts
        if separation < self.min_separation:
            return False, f'too similar to an accepted sample ({separation:.0f} px)', facts
        if warp > self.max_warp:
            return False, (f'board is not flat ({warp:.1f} px off a plane) - '
                           'mount the sheet on something rigid'), facts
        return True, 'ok', facts

    def accept(self, facts):
        index = len(self.samples) + 1
        role = 'validation' if index % VALIDATION_EVERY == 0 else 'calibration'
        centroid = facts['centroid']
        cell = (min(int(centroid[1] / self.height * BINS), BINS - 1),
                min(int(centroid[0] / self.width * BINS), BINS - 1))
        self.coverage[cell] += 1
        record = {'index': index, 'role': role, 'cell': list(cell),
                  'sharpness': facts['sharpness'], 'points': facts['points'],
                  'tilt': facts['tilt'], 'separation_px': facts['separation_px'],
                  'warp_px': facts['warp_px'],
                  'centroid': centroid, 'corners': facts['corners'],
                  'saved_at': timestamp()}
        self.samples.append(record)
        return record

    def counts(self):
        calibration = sum(1 for s in self.samples if s['role'] == 'calibration')
        validation = len(self.samples) - calibration
        return calibration, validation

    def advice(self):
        """The single most useful thing to do next."""
        if not self.samples:
            return 'start in the middle of the frame, board facing the camera'
        empty = [(row, column) for row in range(BINS) for column in range(BINS)
                 if self.coverage[row, column] == 0]
        if empty:
            names = {(0, 0): 'top-left', (0, 1): 'top-middle', (0, 2): 'top-right',
                     (1, 0): 'left', (1, 1): 'centre', (1, 2): 'right',
                     (2, 0): 'bottom-left', (2, 1): 'bottom-middle',
                     (2, 2): 'bottom-right'}
            return f'move the board to the {names[empty[0]]} of the frame'
        tilted = sum(1 for s in self.samples if s['tilt'] > 1.15)
        if tilted < len(self.samples) / 3:
            return 'tilt the board more; flat-on views leave the focal length loose'
        calibration, _ = self.counts()
        if calibration < self.target:
            return f'{self.target - calibration} calibration samples to go; vary distance'
        return 'target reached; a few more tilted edge samples still help'

    def serialisable(self):
        return [{**sample, 'corners': np.asarray(sample['corners']).tolist()}
                for sample in self.samples]


def draw_coverage(canvas, coverage, origin=(10, 170)):
    """Nine boxes showing where accepted samples have landed."""
    x, y, size = origin[0], origin[1], 26
    for row in range(BINS):
        for column in range(BINS):
            count = int(coverage[row, column])
            colour = (60, 60, 60) if count == 0 else (40, 160 + min(count, 5) * 18, 40)
            top_left = (x + column * size, y + row * size)
            bottom_right = (top_left[0] + size - 2, top_left[1] + size - 2)
            cv2.rectangle(canvas, top_left, bottom_right, colour, -1)
            cv2.putText(canvas, str(count), (top_left[0] + 7, top_left[1] + 19),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (235, 240, 250), 1, cv2.LINE_AA)
    cv2.putText(canvas, 'frame coverage', (x, y - 6), cv2.FONT_HERSHEY_SIMPLEX,
                0.45, (200, 205, 215), 1, cv2.LINE_AA)


def run(settings, directory, board, target=30, split=8, min_sharpness=60.0):
    """Interactive collection.  Returns the manifest it wrote."""
    directory = Path(directory)
    raw_dir = directory / 'samples'
    show_hud = True
    rate = FrameRate()
    rejected = 0
    last_message = ''

    with RealCamera(settings) as camera:
        camera.warm_up()
        camera.measure_fps(20)
        configuration = camera.configuration()
        width, height = configuration['resolution']
        collector = Collector(board, width, height, min_sharpness=min_sharpness,
                              target=target, split=split)
        directory.mkdir(parents=True, exist_ok=True)
        window = 'SO101 intrinsics collection'
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window, width, height)
        print(f'collecting into {raw_dir}')
        print(f'  board: {board.name} ({board.describe()["counts"]})')
        print(f'  keys: {KEY_HELP}')

        while True:
            try:
                raw = camera.frame()
            except RuntimeError as error:
                print(f'capture error: {error}')
                break
            view = transform_image(raw, settings)
            rate.tick()
            ok, reason, facts = collector.score(view)
            detection = boards_module.detect(board, view) if facts['detected'] else None
            canvas = (boards_module.draw(view, board, detection) if detection
                      else view.copy())
            border = (60, 200, 60) if ok else (40, 40, 200)
            cv2.rectangle(canvas, (2, 2), (width - 3, height - 3), border, 3)
            if show_hud:
                calibration, validation = collector.counts()
                lines = [
                    f'accepted {calibration} calibration + {validation} validation '
                    f'(target {target} + {split})   rejected {rejected}',
                    f'candidate: {"ACCEPTABLE - press s" if ok else "NOT usable: " + reason}',
                    f'sharpness {facts["sharpness"]:6.0f}   points {facts["points"]:3d}'
                    + (f'   tilt {facts["tilt"]:.2f}   flatness {facts["warp_px"]:.2f} px'
                       if 'tilt' in facts else ''),
                    f'NEXT: {collector.advice()}',
                    last_message or KEY_HELP,
                ]
                _put(canvas, lines)
                draw_coverage(canvas, collector.coverage)
            cv2.imshow(window, canvas)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == ord('h'):
                show_hud = not show_hud
            if key == ord('s'):
                if not ok:
                    rejected += 1
                    last_message = f'rejected: {reason}'
                    print(last_message)
                    continue
                record = collector.accept(facts)
                stem = f'{record["role"]}-{record["index"]:03d}'
                extra = {'stage': 'intrinsics', 'role': record['role'],
                         'board': board.describe(),
                         'image_transform': camera.transform(),
                         'sharpness': record['sharpness'], 'tilt': record['tilt'],
                         'warp_px': record['warp_px'],
                         'points': record['points'],
                         'exposure': exposure_report(view)}
                path = save_frame(raw, raw_dir, stem, extra, configuration)
                last_message = f'saved {stem} ({record["role"]})'
                print(f'{last_message}: {path}')
        cv2.destroyAllWindows()

    manifest = {'directory': str(directory), 'board': board.describe(),
                'camera': configuration, 'settings': settings.as_dict(),
                'validation_every': VALIDATION_EVERY,
                'split_rule': ('assigned when the sample was captured, before any '
                               'reprojection error was known'),
                'coverage': collector.coverage.tolist(),
                'samples': collector.serialisable(),
                'rejected': rejected, 'finished_at': timestamp()}
    (directory / 'samples.json').write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
    calibration, validation = collector.counts()
    print(f'\n{calibration} calibration + {validation} validation samples')
    print(f'manifest: {directory / "samples.json"}')
    return manifest
