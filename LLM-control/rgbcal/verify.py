"""Stage 2 verification: evidence that the fit generalises.

A small training RMS proves only that the solver found a minimum; with
enough distortion coefficients it can fit its own input almost exactly and
still be wrong everywhere else.  The checks here are deliberately ones the
fit could not have optimised:

* the held-out samples, whose corners the solver never saw;
* how the error behaves with distance from the principal point, because
  distortion is a radial story and edge behaviour is where it fails;
* the board's own right angles, recovered in 3D, which test the shape of
  the camera model without depending on the printed scale at all.

Undistorted images are written next to the report so the correction can be
looked at, not just scored.
"""
from pathlib import Path
import json

import cv2
import numpy as np

from .solve import load_samples


def _pnp(sample, matrix, distortion):
    object_points = sample['object_points'].astype(np.float32)
    image_points = sample['image_points'].astype(np.float32).reshape(-1, 1, 2)
    ok, rvec, tvec = cv2.solvePnP(object_points, image_points, matrix, distortion,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    projected, _ = cv2.projectPoints(object_points, rvec, tvec, matrix, distortion)
    residual = projected.reshape(-1, 2) - image_points.reshape(-1, 2)
    return {'rvec': rvec, 'tvec': tvec, 'residual': residual,
            'rms': float(np.sqrt((residual ** 2).sum(axis=1).mean())),
            'distance_m': float(np.linalg.norm(tvec))}


def radial_profile(entries, matrix, bins=5):
    """Reprojection error against distance from the principal point.

    Errors that climb steeply in the outer bins mean the distortion model
    is not describing the edge of this lens, whatever the overall RMS says.
    """
    centre = np.array([matrix[0, 2], matrix[1, 2]])
    radii, errors = [], []
    for entry in entries:
        points = entry['sample']['image_points']
        radii.append(np.linalg.norm(points - centre, axis=1))
        errors.append(np.linalg.norm(entry['pnp']['residual'], axis=1))
    radii = np.concatenate(radii)
    errors = np.concatenate(errors)
    limit = radii.max()
    profile = []
    for index in range(bins):
        low, high = limit * index / bins, limit * (index + 1) / bins
        mask = (radii >= low) & (radii <= high if index == bins - 1 else radii < high)
        profile.append({
            'radius_px': [round(float(low), 1), round(float(high), 1)],
            'points': int(mask.sum()),
            'rms_px': round(float(np.sqrt((errors[mask] ** 2).mean())), 4)
            if mask.any() else None})
    return profile


def board_geometry(sample, pnp, board):
    """Recover the board in 3D and check its right angles.

    Scale-free on purpose: the angles and the ratio of opposite sides do not
    depend on how big the printed square really is, so this tests the camera
    model even though the print scale has not been measured.
    """
    if board.kind != 'chessboard':
        return None
    rotation, _ = cv2.Rodrigues(pnp['rvec'])
    points = sample['object_points'].astype(np.float64)
    image_points = sample['image_points'].astype(np.float32).reshape(-1, 1, 2)
    undistorted = cv2.undistortPoints(image_points, board_geometry.matrix,
                                      board_geometry.distortion).reshape(-1, 2)
    # Ray through each corner, intersected with the board plane in camera space.
    origin = pnp['tvec'].reshape(3)
    normal = rotation[:, 2]
    rays = np.column_stack([undistorted, np.ones(len(undistorted))])
    scale = (normal @ origin) / (rays @ normal)
    world = rays * scale[:, None]
    grid = world.reshape(board.inner_rows, board.inner_cols, 3)
    corners = [grid[0, 0], grid[0, -1], grid[-1, -1], grid[-1, 0]]
    edges = [corners[(index + 1) % 4] - corners[index] for index in range(4)]
    angles = []
    for index in range(4):
        a, b = edges[index], -edges[(index - 1) % 4]
        cosine = (a @ b) / (np.linalg.norm(a) * np.linalg.norm(b))
        angles.append(float(np.degrees(np.arccos(np.clip(cosine, -1, 1)))))
    lengths = [float(np.linalg.norm(edge)) for edge in edges]
    return {'corner_angles_deg': [round(angle, 2) for angle in angles],
            'max_angle_error_deg': round(max(abs(angle - 90) for angle in angles), 3),
            'opposite_side_ratio': [round(lengths[0] / lengths[2], 4),
                                    round(lengths[1] / lengths[3], 4)],
            'measured_sides_m': [round(length, 4) for length in lengths]}


def write_undistorted(sample, matrix, distortion, directory):
    """Before/after pair at the same intrinsics the pipeline will use."""
    image = cv2.imread(sample['path'])
    height, width = image.shape[:2]
    new_matrix, _ = cv2.getOptimalNewCameraMatrix(matrix, distortion, (width, height),
                                                  alpha=0)
    undistorted = cv2.undistort(image, matrix, distortion, None, new_matrix)
    directory.mkdir(parents=True, exist_ok=True)
    name = Path(sample['path']).stem
    pair = np.hstack([image, undistorted])
    cv2.line(pair, (width, 0), (width, height), (0, 255, 255), 2)
    cv2.putText(pair, 'as delivered', (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                (0, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(pair, 'undistorted (alpha=0)', (width + 20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2, cv2.LINE_AA)
    path = directory / f'{name}-undistorted.png'
    cv2.imwrite(str(path), pair)
    return {'pair_image': str(path),
            'new_camera_matrix': new_matrix.tolist(),
            'note': ('the undistorted image has its OWN intrinsics '
                     '(new_camera_matrix). Never mix it with K/D of the raw image')}


def run(directory, intrinsics=None):
    directory = Path(directory)
    path = Path(intrinsics) if intrinsics else directory / 'intrinsics.json'
    calibration = json.loads(path.read_text())
    matrix = np.array(calibration['K'])
    distortion = np.array(calibration['D'])
    board_geometry.matrix, board_geometry.distortion = matrix, distortion

    manifest, board, samples = load_samples(directory)
    groups = {}
    for role in ('validation', 'calibration'):
        entries = []
        for sample in samples:
            if not sample['ok'] or sample['role'] != role:
                continue
            pnp = _pnp(sample, matrix, distortion)
            if pnp:
                entries.append({'sample': sample, 'pnp': pnp})
        groups[role] = entries

    if not groups['validation']:
        raise RuntimeError('no usable validation samples; nothing independent to check')

    report = {'intrinsics': str(path), 'resolution': calibration['resolution'],
              'model': calibration['model']}
    for role, entries in groups.items():
        errors = [entry['pnp']['rms'] for entry in entries]
        report[role] = {
            'images': len(entries),
            'rms_px': round(float(np.sqrt(np.mean(np.square(errors)))), 4),
            'worst_px': round(float(max(errors)), 4),
            'per_image': {Path(entry['sample']['path']).name:
                          round(entry['pnp']['rms'], 4) for entry in entries},
            'distance_range_m': [round(min(e['pnp']['distance_m'] for e in entries), 3),
                                 round(max(e['pnp']['distance_m'] for e in entries), 3)],
        }
    report['independent_check'] = (
        'validation images were held out at capture time and their corners were '
        'never used to fit K or D')
    report['radial_profile_validation'] = radial_profile(groups['validation'], matrix)

    # How far out the data actually reaches.  Beyond that radius the distortion
    # curve is extrapolated, and no error figure in this report covers it.
    centre = np.array([matrix[0, 2], matrix[1, 2]])
    observed = np.concatenate([np.linalg.norm(entry['sample']['image_points'] - centre,
                                              axis=1)
                               for entries in groups.values() for entry in entries])
    width = calibration['resolution']['width']
    height = calibration['resolution']['height']
    furthest = float(max(np.linalg.norm(np.array(corner) - centre) for corner in
                         ((0, 0), (width, 0), (0, height), (width, height))))
    report['radial_coverage'] = {
        'max_observed_radius_px': round(float(observed.max()), 1),
        'image_corner_radius_px': round(furthest, 1),
        'fraction_of_image_radius_covered': round(float(observed.max()) / furthest, 3),
        'note': ('distortion beyond the observed radius is extrapolated. Keep '
                 'localisation inside it, or collect corner samples and re-solve')}
    geometry = [board_geometry(entry['sample'], entry['pnp'], board)
                for entry in groups['validation']]
    geometry = [item for item in geometry if item]
    if geometry:
        report['board_right_angles'] = {
            'max_angle_error_deg': round(max(item['max_angle_error_deg']
                                             for item in geometry), 3),
            'median_angle_error_deg': round(float(np.median(
                [item['max_angle_error_deg'] for item in geometry])), 3),
            'per_image': geometry,
            'note': ('scale-free: tests the camera model, not the printed square '
                     'size. A square print scale error leaves these angles right')}
    report['undistortion_examples'] = [
        write_undistorted(entry['sample'], matrix, distortion, directory / 'undistorted')
        for entry in groups['validation'][:3]]
    verdict = []
    validation_rms = report['validation']['rms_px']
    calibration_rms = report['calibration']['rms_px']
    verdict.append('PASS: held-out error matches the fitted error'
                   if validation_rms <= calibration_rms * 1.5 + 0.1 else
                   'FAIL: held-out error much worse than fitted error (overfit)')
    outer = [b['rms_px'] for b in report['radial_profile_validation'][-2:]
             if b['rms_px'] is not None]
    inner = [b['rms_px'] for b in report['radial_profile_validation'][:2]
             if b['rms_px'] is not None]
    if outer and inner:
        ratio = max(outer) / max(max(inner), 1e-6)
        verdict.append(f'edge/centre error ratio {ratio:.2f}: '
                       + ('acceptable' if ratio < 2.5 else
                          'the edges are much worse; distortion model or edge coverage'))
    report['verdict'] = verdict
    (directory / 'verification.json').write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    report['written'] = str(directory / 'verification.json')
    return report
