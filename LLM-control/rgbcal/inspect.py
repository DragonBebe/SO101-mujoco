"""Stage 1 measurements on already-saved frames.

The viewpoint question is not answered by looking at a picture and liking
it.  What matters is measurable in the picture itself: how many pixels a
25 mm object covers at each part of the workspace, how much the view is
foreshortened, whether the board still detects near the edges, and how much
border is left for a lifted object.  Everything here works on the saved
raw samples, so a candidate viewpoint can be re-examined later.

No intrinsics are needed and none are assumed: these are image-space
numbers plus ratios, never a claim about metric camera pose.
"""
from pathlib import Path
import json

import cv2
import numpy as np

from . import boards as boards_module
from .capture import exposure_report, sharpness


def _square_edges(board, points):
    """One-square edges as ``(length_px, midpoint_row_px)`` pairs.

    Detection returns the grid in whatever order the board happened to be
    oriented, so "first row" says nothing about near or far.  Pairing each
    edge with its image row instead lets the caller compare the bottom of
    the image (near the camera in a downward view) with the top, which is
    what actually measures foreshortening.
    """
    grid = points.reshape(board.inner_rows, board.inner_cols, 2)
    lengths, rows = [], []
    for axis in (0, 1):
        difference = np.diff(grid, axis=axis)
        midpoint = (np.take(grid, np.arange(grid.shape[axis] - 1), axis=axis)
                    + difference / 2)
        lengths.append(np.linalg.norm(difference, axis=2).ravel())
        rows.append(midpoint[..., 1].ravel())
    return np.concatenate(lengths), np.concatenate(rows)


def analyse_frame(path, board, hflip=False, colours=()):
    """Board geometry, scale and exposure for one saved frame."""
    image = cv2.imread(str(path))
    if image is None:
        raise FileNotFoundError(f'cannot read {path}')
    if hflip:
        image = cv2.flip(image, 1)
    height, width = image.shape[:2]
    report = {'image': str(path), 'resolution': [width, height],
              'hflip_applied': bool(hflip),
              'sharpness': sharpness(image),
              'exposure': exposure_report(image)}
    if colours:
        report['objects'] = {colour: coloured_blobs(image, colour) for colour in colours}
    detection = boards_module.detect(board, image)
    report['board_detected'] = detection.ok
    report['board_note'] = detection.note
    report['board_points'] = detection.count
    if not detection.ok:
        return report

    points = detection.image_points
    low = points.min(axis=0)
    high = points.max(axis=0)
    report['board_bbox'] = [*low.round(1).tolist(), *high.round(1).tolist()]
    report['board_frame_fraction'] = {
        'width': float((high[0] - low[0]) / width),
        'height': float((high[1] - low[1]) / height)}
    report['board_margins_px'] = {
        'left': float(low[0]), 'right': float(width - high[0]),
        'top': float(low[1]), 'bottom': float(height - high[1])}

    if board.kind == 'chessboard':
        lengths, rows = _square_edges(board, points)
        square_px = float(np.median(lengths))
        split = np.median(rows)
        lower = float(np.median(lengths[rows >= split]))   # bottom of the image
        upper = float(np.median(lengths[rows < split]))    # top of the image
        ratio = lower / upper if upper else None
        report['square_px'] = {
            'median': square_px,
            'image_lower_half': lower,
            'image_upper_half': upper,
            'lower_over_upper': ratio}
        millimetre = square_px / (board.square * 1000.0)
        report['pixels_per_mm_at_board'] = millimetre
        report['cube_25mm_px_at_board'] = 25.0 * millimetre
        # Strong foreshortening across a 175 mm board means a grazing view;
        # a ratio near 1 means the camera is looking almost straight down.
        # In a downward-looking view the image bottom is nearer, so its squares
        # are larger.  This gradient grows with obliquity but also with the
        # board's depth relative to the camera distance, so it ranks candidate
        # viewpoints against each other; it is not a viewing angle.  The angle
        # is only available after intrinsics, from solvePnP.
        report['square_gradient_note'] = (
            'relative indicator only: depends on viewing angle AND on how much '
            'of the frame the board spans in depth; not a camera angle')
    return report


def analyse_session(directory, board, hflip=False, pattern='frames/*.png', colours=()):
    directory = Path(directory)
    reports = [analyse_frame(path, board, hflip, colours)
               for path in sorted(directory.glob(pattern))]
    return {'directory': str(directory), 'board': board.describe(),
            'frames': reports}


#: Hue ranges (OpenCV H is 0-179) for the coloured cubes the workspace uses.
_HUE = {'red': ((0, 10), (170, 180)), 'green': ((35, 85),), 'blue': ((90, 130),),
        'yellow': ((20, 35),), 'purple': ((130, 160),)}


def coloured_blobs(image, colour='red', min_area=200):
    """Pixel size of the coloured objects in a frame.

    Stage 1 asks a concrete question: does a 25 mm cube cover enough pixels
    at the far end of the workspace to be found and measured?  This answers
    it in the image, with no calibration involved, so it is a pixel count
    and never a position claim.
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for low, high in _HUE[colour]:
        mask |= cv2.inRange(hsv, (low, 90, 60), (high, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    blobs = []
    for index in range(1, count):
        x, y, width, height, area = stats[index]
        if area < min_area:
            continue
        blobs.append({'colour': colour, 'bbox': [int(x), int(y), int(width), int(height)],
                      'area_px': int(area),
                      'centroid': [round(float(centroids[index][0]), 1),
                                   round(float(centroids[index][1]), 1)],
                      'box_px': [int(width), int(height)]})
    blobs.sort(key=lambda blob: -blob['area_px'])
    return blobs


def annotate(path, board, output, hflip=False):
    """Write a copy with detections and framing guides, for side-by-side review."""
    from .preview import framing_guides

    image = cv2.imread(str(path))
    if hflip:
        image = cv2.flip(image, 1)
    detection = boards_module.detect(board, image)
    canvas = boards_module.draw(image, board, detection)
    framing_guides(canvas)
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), canvas)
    return detection
