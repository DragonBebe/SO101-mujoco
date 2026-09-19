"""Locking the viewpoint, and noticing later that it moved.

The environment camera's extrinsics are only valid while the camera stays
where it was.  A laptop gets nudged, a lid gets closed, a table gets
cleaned.  None of that announces itself, and a silently moved camera keeps
producing confident, wrong world coordinates.

So the accepted viewpoint is recorded here together with a set of image
patches from parts of the frame that do not move with the task (background
furniture, the far wall).  Re-finding those patches later measures how far
the view has shifted in pixels.  It is a change detector, not a
re-calibration: a shift means the extrinsics must be re-measured, and it
cannot repair them.
"""
from datetime import datetime, timezone
from pathlib import Path
import json

import cv2
import numpy as np

from . import devices as devices_module
from .capture import RealCamera, save_frame, sharpness, transform_image

#: Patch anchors as fractions of the frame, chosen along the top band where
#: the robot and the workspace do not reach.
ANCHORS = ((0.05, 0.02), (0.30, 0.02), (0.70, 0.02), (0.90, 0.02),
           (0.05, 0.25), (0.90, 0.25))
PATCH = (180, 120)


def _patches(image, size=80, grid=(8, 5), per_cell=3):
    """Texture-rich anchor patches, spread evenly over the frame.

    Two ways of choosing them failed on the real scene, and the reasons are
    the design.  Fixed positions along the top band landed on people walking
    by.  The strongest corners of the whole image all landed on the robot,
    whose black-on-white edges outshine everything else, and the robot is
    precisely what moves.  So corners are chosen per grid cell: every part of
    the frame gets anchors, static background included, and at check time
    the largest group that moves together decides.
    """
    height, width = image.shape[:2]
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    half = size // 2
    columns, rows = grid
    out = []
    for row in range(rows):
        for column in range(columns):
            x0 = max(int(column * width / columns), half + 1)
            x1 = min(int((column + 1) * width / columns), width - half - 1)
            y0 = max(int(row * height / rows), half + 1)
            y1 = min(int((row + 1) * height / rows), height - half - 1)
            if x1 - x0 < 10 or y1 - y0 < 10:
                continue
            cell = gray[y0:y1, x0:x1]
            corners = cv2.goodFeaturesToTrack(cell, per_cell, 0.05, 40)
            if corners is None:
                continue
            for x, y in corners.reshape(-1, 2).astype(int):
                out.append({'x': int(x0 + x - half), 'y': int(y0 + y - half),
                            'w': size, 'h': size})
    return out


def device_identity(device):
    """Kernel identity of a capture node, following /dev/v4l/by-id links.

    A stable by-id path is the right thing to configure, but the kernel lists
    nodes by their /dev/videoN name, so the link has to be resolved before
    the two can be matched.
    """
    resolved = str(Path(device).resolve())
    for entry in devices_module.enumerate_nodes():
        if entry['device'] == resolved:
            return {**entry, 'configured_as': device}
    raise RuntimeError(f'{device} (-> {resolved}) is not a known V4L2 node')


def lock(settings, directory, note='', frames=5):
    """Record the accepted viewpoint: identity, settings, reference frame."""
    directory = Path(directory)
    with RealCamera(settings) as camera:
        camera.warm_up(15)
        camera.measure_fps(20)
        configuration = camera.configuration()
        best, best_score = None, -1.0
        for _ in range(frames):
            raw = camera.frame()
            score = sharpness(transform_image(raw, settings))
            if score > best_score:
                best, best_score = raw, score
    reference = save_frame(best, directory, 'reference',
                           {'stage': 'locked-viewpoint', 'sharpness': best_score},
                           configuration)
    identity = device_identity(settings.device)
    record = {
        'locked_at': datetime.now(timezone.utc).astimezone().isoformat(timespec='seconds'),
        'note': note,
        'device': identity,
        'camera': configuration,
        'reference_image': str(reference),
        'anchors': _patches(cv2.imread(str(reference))),
        'valid_while': ('the laptop, its lid angle, the robot base and the table '
                        'stay exactly as they are; any of those moving '
                        'invalidates the extrinsics, not the intrinsics'),
    }
    (directory / 'setup.json').write_text(json.dumps(record, indent=2, ensure_ascii=False) + '\n')
    return record


def drift(record, image):
    """Pixel shift of ``image`` against the locked reference.

    Each anchor patch is searched for in a window around its original place.
    A confident, consistent shift means the camera moved; disagreeing or
    low-confidence matches usually mean the background itself changed.
    """
    reference = cv2.imread(record['reference_image'])
    if reference is None:
        raise FileNotFoundError(record['reference_image'])
    if image.shape != reference.shape:
        return {'comparable': False,
                'reason': f'resolution changed: {reference.shape[:2][::-1]} -> '
                          f'{image.shape[:2][::-1]}'}
    results = []
    search = 160
    anchors = record['anchors']
    if len(anchors) < 40:
        # Records locked before anchors were chosen by texture: rebuild them from
        # the reference image rather than trust the old fixed positions.
        anchors = _patches(reference)
    for anchor in anchors:
        x, y, w, h = anchor['x'], anchor['y'], anchor['w'], anchor['h']
        patch = reference[y:y + h, x:x + w]
        x0, y0 = max(x - search, 0), max(y - search, 0)
        x1, y1 = min(x + w + search, image.shape[1]), min(y + h + search, image.shape[0])
        window = image[y0:y1, x0:x1]
        if window.shape[0] < h or window.shape[1] < w:
            continue
        match = cv2.matchTemplate(window, patch, cv2.TM_CCOEFF_NORMED)
        _, score, _, location = cv2.minMaxLoc(match)
        results.append({'dx': int(x0 + location[0] - x), 'dy': int(y0 + location[1] - y),
                        'score': float(score)})
    trusted = [item for item in results if item['score'] >= 0.7]
    if len(trusted) < 6:
        return {'comparable': True, 'anchors': results, 'confident': False,
                'matched_anchors': len(trusted),
                'verdict': (f'only {len(trusted)} anchors matched; the scene may have '
                            'changed too much. Compare the reference by eye')}
    # Static structure moves as one: every anchor on it reports the same shift.
    # Anchors on people, the arm or moved objects report scattered shifts.  So
    # the answer is the largest group of anchors that agree with each other,
    # not an average that the moving ones can drag.
    best = []
    for candidate in trusted:
        group = [item for item in trusted
                 if abs(item['dx'] - candidate['dx']) <= 2
                 and abs(item['dy'] - candidate['dy']) <= 2]
        if len(group) > len(best):
            best = group
    agreeing = best
    consensus = len(agreeing) / len(trusted)
    dx = float(np.median([item['dx'] for item in agreeing]))
    dy = float(np.median([item['dy'] for item in agreeing]))
    if len(agreeing) < 6:
        return {'comparable': True, 'anchors': results, 'confident': False,
                'matched_anchors': len(trusted), 'agreeing_anchors': len(agreeing),
                'consensus': consensus,
                'verdict': 'no large group of anchors agrees; too much of the scene changed'}
    moved = abs(dx) > 3 or abs(dy) > 3
    return {'comparable': True, 'anchors': results, 'confident': True,
            'matched_anchors': len(trusted), 'agreeing_anchors': len(agreeing),
            'consensus': consensus, 'shift_px': {'dx': dx, 'dy': dy},
            'verdict': ('camera appears UNMOVED (shift within 3 px)' if not moved else
                        f'camera MOVED by about ({dx:+.0f}, {dy:+.0f}) px: '
                        'extrinsics must be re-measured before any motion')}


def check(settings, setup_directory):
    record = json.loads((Path(setup_directory) / 'setup.json').read_text())
    with RealCamera(settings) as camera:
        camera.warm_up(10)
        image = camera.frame()
    result = drift(record, image)
    result['locked_at'] = record['locked_at']
    return result
