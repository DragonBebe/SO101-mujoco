"""Calibration board geometry and detection.

A board is only a metric reference if the printed square really is the
size the configuration claims.  Printers rescale ("fit to page") without
saying so, so the square length here is a *measured* number the operator
supplies, and the presets below are what the source PDF specifies before
printing.  Everything is in metres.

Two board families are supported and they are counted differently, which
is the usual way a calibration silently fails:

* a plain chessboard is described by its **inner corner** counts
  (9 x 6 inner corners is a 10 x 7 square grid);
* a ChArUco board is described by its **square** counts (7 x 9 squares).
"""
from dataclasses import dataclass, asdict

import cv2
import numpy as np


@dataclass(frozen=True)
class Chessboard:
    """Plain chessboard, described by inner corners."""

    inner_cols: int = 9
    inner_rows: int = 6
    square: float = 0.025
    name: str = 'chessboard-9x6-25mm'
    kind: str = 'chessboard'

    @property
    def pattern(self):
        return (self.inner_cols, self.inner_rows)

    @property
    def point_count(self):
        return self.inner_cols * self.inner_rows

    def object_points(self):
        """Board-frame corner grid: X right, Y down the board, Z = 0."""
        grid = np.zeros((self.point_count, 3), np.float32)
        grid[:, :2] = np.mgrid[0:self.inner_cols, 0:self.inner_rows].T.reshape(-1, 2)
        return grid * self.square

    def size_m(self):
        """Printed extent of the full square grid, for checking the printout."""
        return ((self.inner_cols + 1) * self.square, (self.inner_rows + 1) * self.square)

    def describe(self):
        return dict(asdict(self), size_m=list(self.size_m()),
                    counts='inner corners')


@dataclass(frozen=True)
class Charuco:
    """ChArUco board, described by squares; markers fill the white squares."""

    squares_x: int = 7
    squares_y: int = 9
    square: float = 0.020
    marker: float = 0.015
    dictionary: str = 'DICT_4X4_50'
    first_id: int = 0
    name: str = 'charuco-7x9-20mm'
    kind: str = 'charuco'

    @property
    def marker_count(self):
        return (self.squares_x * self.squares_y) // 2

    @property
    def point_count(self):
        """ChArUco corners are the inner corners of the square grid."""
        return (self.squares_x - 1) * (self.squares_y - 1)

    def board(self):
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.dictionary))
        ids = np.arange(self.first_id, self.first_id + self.marker_count, dtype=np.int32)
        return cv2.aruco.CharucoBoard((self.squares_x, self.squares_y),
                                      self.square, self.marker, dictionary, ids)

    def size_m(self):
        return (self.squares_x * self.square, self.squares_y * self.square)

    def describe(self):
        return dict(asdict(self), size_m=list(self.size_m()),
                    marker_ids=[self.first_id, self.first_id + self.marker_count - 1],
                    counts='squares')


#: Presets matching the printed PDF, *before* any printer rescaling.
PRESETS = {
    'chessboard': Chessboard(),
    'charuco-large': Charuco(squares_x=7, squares_y=9, square=0.020, marker=0.015,
                             first_id=0, name='charuco-7x9-20mm'),
    'charuco-small': Charuco(squares_x=5, squares_y=7, square=0.015, marker=0.011,
                             first_id=32, name='charuco-5x7-15mm'),
}


def build(preset, square=None, marker=None):
    """A board from a preset name, with the *measured* sizes substituted in."""
    if preset not in PRESETS:
        raise ValueError(f'unknown board {preset!r}; choose from {", ".join(PRESETS)}')
    board = PRESETS[preset]
    changes = {}
    if square is not None:
        changes['square'] = float(square)
    if marker is not None:
        if board.kind != 'charuco':
            raise ValueError('marker length applies to ChArUco boards only')
        changes['marker'] = float(marker)
    if not changes:
        return board
    return type(board)(**{**asdict(board), **changes})


class Detection:
    """One board observation: image points, their board points, and coverage."""

    def __init__(self, ok, image_points=None, object_points=None, ids=None, note=''):
        self.ok = ok
        self.image_points = image_points
        self.object_points = object_points
        self.ids = ids
        self.note = note

    @property
    def count(self):
        return 0 if self.image_points is None else int(len(self.image_points))


def robust_detector_parameters():
    """Looser ArUco thresholds for markers that image small.

    Measured on this setup: the 15 mm markers of the large board cover about
    21 px at workspace distance, roughly 3.5 px per code cell, which is below
    what the default thresholds decode reliably.  These settings recovered
    most of the board; the cost is a higher chance of accepting a marginal
    decode, so they are opt-in and recorded when used.
    """
    parameters = cv2.aruco.DetectorParameters()
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 53
    parameters.adaptiveThreshWinSizeStep = 2
    parameters.minMarkerPerimeterRate = 0.008
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    parameters.perspectiveRemovePixelPerCell = 8
    parameters.maxErroneousBitsInBorderRate = 0.5
    parameters.errorCorrectionRate = 0.8
    return parameters


def detect(board, image, robust=False):
    """Detect ``board`` in a BGR frame.

    ChArUco detection is partial by design: a board half out of frame still
    contributes its visible corners, which is what makes edge coverage
    practical.  A plain chessboard is all-or-nothing.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    if board.kind == 'chessboard':
        flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY | cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCornersSB(gray, board.pattern, flags)
        if not found:
            found, corners = cv2.findChessboardCorners(
                gray, board.pattern,
                cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
            if found:
                corners = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        if not found:
            return Detection(False, note='chessboard not found')
        return Detection(True, corners.reshape(-1, 2).astype(np.float64),
                         board.object_points().astype(np.float64),
                         note='full chessboard')

    charuco = board.board()
    detector = (cv2.aruco.CharucoDetector(charuco, cv2.aruco.CharucoParameters(),
                                          robust_detector_parameters())
                if robust else cv2.aruco.CharucoDetector(charuco))
    corners, ids, marker_corners, marker_ids = detector.detectBoard(gray)
    if ids is None or len(ids) < 4:
        found = 0 if marker_ids is None else len(marker_ids)
        return Detection(False, note=f'only {found} markers, no usable ChArUco corners')
    object_points, image_points = charuco.matchImagePoints(corners, ids)
    return Detection(True, image_points.reshape(-1, 2).astype(np.float64),
                     object_points.reshape(-1, 3).astype(np.float64),
                     ids=ids.reshape(-1),
                     note=(f'{len(ids)}/{board.point_count} ChArUco corners'
                           + (' (robust thresholds)' if robust else '')))


def draw(image, board, detection):
    """Overlay detected points on a copy of the frame, for the live view."""
    canvas = image.copy()
    if not detection.ok:
        return canvas
    points = detection.image_points.reshape(-1, 1, 2).astype(np.float32)
    if board.kind == 'chessboard':
        cv2.drawChessboardCorners(canvas, board.pattern, points, True)
    else:
        for point in points.reshape(-1, 2):
            cv2.circle(canvas, tuple(np.int32(point)), 4, (0, 255, 255), -1)
            cv2.circle(canvas, tuple(np.int32(point)), 6, (20, 20, 20), 1)
    return canvas


def coverage(detections, width, height, bins=3):
    """How much of the frame the collected corners have actually seen.

    Intrinsics are fitted where the data is.  Distortion lives at the edges,
    so a pile of centre-only samples produces a confident, wrong model; this
    grid is what tells the operator to move the board outward.
    """
    grid = np.zeros((bins, bins), dtype=int)
    for detection in detections:
        if not detection.ok:
            continue
        for u, v in detection.image_points:
            column = min(int(u / width * bins), bins - 1)
            row = min(int(v / height * bins), bins - 1)
            if 0 <= column < bins and 0 <= row < bins:
                grid[row, column] += 1
    return grid


def mirror_probe(image, dictionary='DICT_4X4_50'):
    """Decide whether the stream is horizontally mirrored, from the image alone.

    ArUco codes are not mirror symmetric: a flipped frame decodes to nothing
    while its flipped copy decodes normally.  A plain chessboard cannot
    answer this question, which is why the check needs markers.

    A mirrored stream is fatal further down: it turns the camera frame into
    a left-handed one, so ``solvePnP`` would report a pose that no rigid
    transform to the robot base can match.
    """
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary)))
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    counts = {}
    for name, candidate in (('as_delivered', gray), ('flipped', cv2.flip(gray, 1))):
        _, ids, _ = detector.detectMarkers(candidate)
        counts[name] = 0 if ids is None else int(len(ids))
    direct, flipped = counts['as_delivered'], counts['flipped']
    if direct == 0 and flipped == 0:
        verdict = ('no markers in either orientation. This check needs an ArUco '
                   'page: the plain chessboard sheet carries no markers, so put '
                   'a ChArUco page in view and repeat')
    elif direct >= max(1, flipped * 2):
        verdict = 'NOT mirrored: markers decode in the frame as delivered'
    elif flipped >= max(1, direct * 2):
        verdict = 'MIRRORED: markers only decode after a horizontal flip'
    else:
        verdict = 'ambiguous; move the board fully into view and repeat'
    return {'markers': counts, 'verdict': verdict}
