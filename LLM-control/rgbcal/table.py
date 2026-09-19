"""Measuring the table, instead of assuming z = 0 is the table.

The robot's base frame has its origin at the base, and nothing guarantees
that the support surface sits at z = 0 in it: the base plate has
thickness, the table may not be level, and the arm may stand on something.
The RGB localisation in this project takes an explicit ``plane_z``, so
that number has to be measured, with the board's own thickness accounted
for rather than ignored.
"""
from pathlib import Path
import json

import cv2
import numpy as np

from . import boards as boards_module
from . import transforms as tf
from .capture import (RealCamera, check_compatible, save_frame, timestamp,
                      transform_image)
from .handeye import board_pose


def plane_pose(board, image, matrix, distortion, robust=True):
    """Board pose for measuring a plane.

    A plain chessboard may come back rotated by 180 degrees in its own plane,
    which would corrupt a hand-eye solve but cannot change the plane it lies
    in: the normal and the height are the same either way.  So for this one
    purpose a chessboard is accepted.
    """
    if board.kind == 'charuco':
        return board_pose(board, image, matrix, distortion, robust)
    detection = boards_module.detect(board, image)
    if not detection.ok:
        return None, detection
    ok, rvec, tvec = cv2.solvePnP(detection.object_points,
                                  detection.image_points.reshape(-1, 1, 2),
                                  matrix, distortion)
    if not ok:
        return None, detection
    return tf.from_rvec_tvec(rvec, tvec), detection


def measure_images(paths, board, intrinsics_path, extrinsics_path,
                   board_thickness_m=0.0, robust=True):
    """The same plane measurement on frames already saved to disk.

    The frames must come from the camera configuration the intrinsics were
    measured under; their sidecars are checked for that.
    """
    calibration = json.loads(Path(intrinsics_path).read_text())
    extrinsics = json.loads(Path(extrinsics_path).read_text())
    if extrinsics.get('status') == 'EXPERIMENTAL-UNVERIFIED':
        raise RuntimeError('refusing to measure the table with an experimental extrinsic')
    matrix = np.array(calibration['K'])
    distortion = np.array(calibration['D'])
    base_camera = np.array(extrinsics['T_base_camera'])
    results = []
    for path in paths:
        path = Path(path)
        sidecar = path.with_suffix('.json')
        if sidecar.exists():
            recorded = json.loads(sidecar.read_text()).get('camera', {})
            requested = recorded.get('requested', {})
            imaging = calibration.get('imaging', {})
            for key in ('device', 'autofocus', 'focus', 'width', 'height'):
                if key in imaging and key in requested and imaging[key] != requested[key]:
                    raise RuntimeError(f'{path.name}: {key} {requested[key]!r} differs '
                                       f'from the intrinsics ({imaging[key]!r})')
        image = cv2.imread(str(path))
        if image is None:
            raise FileNotFoundError(path)
        pose, detection = plane_pose(board, image, matrix, distortion, robust)
        if pose is None:
            results.append({'image': str(path), 'detected': False,
                            'note': detection.note})
            continue
        object_points = detection.object_points
        corners_base = (base_camera @ np.column_stack(
            [(pose[:3, :3] @ object_points.T).T + pose[:3, 3],
             np.ones(len(object_points))]).T).T[:, :3]
        normal = (base_camera[:3, :3] @ pose[:3, :3])[:, 2]
        normal = normal / np.linalg.norm(normal)
        if normal[2] < 0:
            normal = -normal
        centre = corners_base.mean(axis=0)
        results.append({
            'image': str(path), 'detected': True,
            'board_centre_base_m': centre.tolist(),
            'board_distance_from_base_m': float(np.linalg.norm(centre[:2])),
            'plane_z_m': float(np.median(corners_base[:, 2])) - board_thickness_m,
            'surface_normal_base': normal.tolist(),
            'tilt_from_base_z_deg': float(np.degrees(np.arccos(np.clip(normal[2], -1, 1)))),
        })
    return results


def measure(settings, directory, board, intrinsics_path, extrinsics_path,
            board_thickness_m=0.0, frames=10, robust=True):
    """Where the plane the board rests on lies, in robot base coordinates."""
    directory = Path(directory)
    calibration = json.loads(Path(intrinsics_path).read_text())
    extrinsics = json.loads(Path(extrinsics_path).read_text())
    check_compatible(calibration, settings)
    matrix = np.array(calibration['K'])
    distortion = np.array(calibration['D'])
    base_camera = np.array(extrinsics['T_base_camera'])
    if extrinsics.get('status') == 'EXPERIMENTAL-UNVERIFIED':
        raise RuntimeError('refusing to measure the table with an experimental extrinsic')

    poses, corner_sets = [], []
    with RealCamera(settings) as camera:
        camera.warm_up(15)
        configuration = camera.configuration()
        for index in range(frames):
            raw = camera.frame()
            view = transform_image(raw, settings)
            pose, detection = plane_pose(board, view, matrix, distortion, robust)
            if pose is None:
                continue
            if index == 0:
                save_frame(raw, directory, 'table-reference',
                           {'stage': 'table-plane'}, configuration)
            poses.append(pose)
            corner_sets.append(detection.object_points)

    if len(poses) < 3:
        raise RuntimeError('the board was not detected often enough; is it in view '
                           'and lying flat on the table?')

    heights, normals = [], []
    for pose, object_points in zip(poses, corner_sets):
        corners_base = (base_camera @ np.column_stack(
            [(pose[:3, :3] @ object_points.T).T + pose[:3, 3],
             np.ones(len(object_points))]).T).T[:, :3]
        heights.append(corners_base[:, 2])
        # The board's own z axis, expressed in base coordinates, is the surface
        # normal; a level table makes it parallel to the base z axis.
        normals.append((base_camera[:3, :3] @ pose[:3, :3])[:, 2])

    per_frame = np.array([np.median(frame) for frame in heights])
    heights = np.concatenate(heights)
    normal = np.mean(normals, axis=0)
    normal = normal / np.linalg.norm(normal)
    if normal[2] < 0:
        normal = -normal
    surface_z = float(np.median(heights))
    report = {
        'schema': 'rgbcal/table/1',
        'measured_at': timestamp(),
        'frames_used': len(poses),
        'board_surface_z_m': surface_z,
        'board_thickness_m': float(board_thickness_m),
        'plane_z_m': surface_z - float(board_thickness_m),
        'plane_z_note': ('plane_z is the support surface itself: the measured board '
                         'surface minus the thickness of whatever the pattern is '
                         'mounted on'),
        'frame_to_frame_height_std_mm': float(per_frame.std() * 1000),
        'corner_height_range_mm': float((heights.max() - heights.min()) * 1000),
        'corner_height_range_note': ('max minus min over every corner of every frame; '
                                     'includes the height change across the board '
                                     'caused by any tilt, so it is not a noise figure'),
        'surface_normal_base': normal.tolist(),
        'tilt_from_base_z_deg': float(np.degrees(np.arccos(np.clip(normal[2], -1, 1)))),
        'frames': {'plane_z_m': 'height of the support surface in ROBOT BASE '
                                'coordinates, the value RGB localisation needs'},
        'intrinsics': str(intrinsics_path), 'extrinsics': str(extrinsics_path),
        'board': board.describe(),
        'uncertainty_note': ('plane_z inherits the extrinsic uncertainty; the '
                             'height spread above is only the spread across frames'),
    }
    (directory / 'table.json').write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    return report
