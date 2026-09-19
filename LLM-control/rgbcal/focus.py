"""Finding a fixed focus for a camera that would rather refocus by itself.

Autofocus moves the lens between frames, and a moving lens is a moving
focal length and principal point: intrinsics measured under autofocus
describe no single frame.  So focus is switched to manual and pinned, and
the value is *chosen by measurement* over the region that matters -- the
workspace -- rather than guessed from a spec sheet.
"""
from pathlib import Path
import json
import time

import cv2
import numpy as np

from . import boards as boards_module
from .capture import RealCamera, sharpness, transform_image


def region_sharpness(image, box=None):
    if box is None:
        return sharpness(image)
    x0, y0, x1, y1 = box
    return sharpness(image[y0:y1, x0:x1])


def sweep(settings, directory, values, board=None, settle=12):
    """Try each focus value and score it on the workspace.

    With a board in view the score is taken over the board's own bounding
    box, which is by construction at working distance; without one, over
    the central half of the frame.
    """
    directory = Path(directory)
    results = []
    with RealCamera(settings) as camera:
        capture = camera.capture
        capture.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        camera.warm_up(5)
        reported_autofocus = capture.get(cv2.CAP_PROP_AUTOFOCUS)
        box = None
        for value in values:
            capture.set(cv2.CAP_PROP_FOCUS, float(value))
            for _ in range(settle):  # the lens motor needs a few frames
                camera.frame()
            view = transform_image(camera.frame(), settings)
            height, width = view.shape[:2]
            entry = {'focus_requested': float(value),
                     'focus_reported': float(capture.get(cv2.CAP_PROP_FOCUS))}
            if board is not None:
                detection = boards_module.detect(board, view, robust=True)
                entry['board_points'] = detection.count
                if detection.ok and box is None:
                    low = detection.image_points.min(axis=0).astype(int)
                    high = detection.image_points.max(axis=0).astype(int)
                    box = (max(low[0] - 20, 0), max(low[1] - 20, 0),
                           min(high[0] + 20, width), min(high[1] + 20, height))
            region = box or (width // 4, height // 4, 3 * width // 4, 3 * height // 4)
            entry['sharpness'] = region_sharpness(view, region)
            entry['region'] = [int(v) for v in region]
            results.append(entry)
            print(f"focus {value:5.0f} (reported {entry['focus_reported']:5.0f})  "
                  f"sharpness {entry['sharpness']:7.1f}"
                  + (f"  board points {entry['board_points']}" if board else ''))
    best = max(results, key=lambda item: item['sharpness'])
    report = {'autofocus_after_disable': reported_autofocus,
              'manual_focus_took_effect': len({r['focus_reported'] for r in results}) > 1,
              'results': results, 'best_focus': best['focus_requested'],
              'best_sharpness': best['sharpness'],
              'note': ('use --autofocus 0 --focus <best_focus> for every later '
                       'capture; the intrinsics are only valid at this focus')}
    directory.mkdir(parents=True, exist_ok=True)
    (directory / 'focus_sweep.json').write_text(json.dumps(report, indent=2) + '\n')
    return report
