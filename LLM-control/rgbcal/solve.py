"""Stage 2: fitting K and D, and writing down what they are valid for.

The output of this module is not "the camera's intrinsics".  It is the
intrinsics *of one imaging configuration*: one device, one resolution, one
pixel format, one declared pixel transform, one board.  All of that travels
inside the calibration file, because a K applied to a differently sized or
mirrored frame is wrong in a way nothing downstream can detect.

Only samples the collector marked ``calibration`` are fitted.  The
validation samples are left untouched here; see :mod:`rgbcal.verify`.
"""
from pathlib import Path
import json

import cv2
import numpy as np

from . import boards as boards_module
from .capture import transform_image

#: Distortion models.  A laptop webcam is usually well served by the
#: standard five-coefficient model; the others are there for a lens that
#: the residuals show it cannot describe.
MODELS = {
    'standard': 0,
    'rational': cv2.CALIB_RATIONAL_MODEL,
    'thin-prism': cv2.CALIB_RATIONAL_MODEL | cv2.CALIB_THIN_PRISM_MODEL,
}


class _Settings:
    """Minimal stand-in so saved frames go through the same transform."""

    def __init__(self, hflip):
        self.hflip = bool(hflip)


def load_samples(directory, role=None):
    """Re-detect the board in the archived raw frames.

    Detection is redone rather than trusting numbers recorded at capture
    time: the raw images are the evidence, and re-running makes the whole
    fit reproducible from them alone.
    """
    directory = Path(directory)
    manifest = json.loads((directory / 'samples.json').read_text())
    board = boards_module.build(
        'chessboard' if manifest['board']['kind'] == 'chessboard'
        else ('charuco-large' if manifest['board'].get('squares_x', 0) == 7
              else 'charuco-small'),
        square=manifest['board']['square'],
        marker=manifest['board'].get('marker'))
    settings = _Settings(manifest['camera']['image_transform']['hflip'])
    samples = []
    for record in manifest['samples']:
        if role and record['role'] != role:
            continue
        path = directory / 'samples' / f'{record["role"]}-{record["index"]:03d}.png'
        image = cv2.imread(str(path))
        if image is None:
            raise FileNotFoundError(path)
        image = transform_image(image, settings)
        detection = boards_module.detect(board, image)
        if not detection.ok:
            samples.append({'path': str(path), 'role': record['role'],
                            'ok': False, 'note': detection.note})
            continue
        samples.append({'path': str(path), 'role': record['role'], 'ok': True,
                        'image_points': detection.image_points,
                        'object_points': detection.object_points,
                        'shape': image.shape[:2]})
    return manifest, board, samples


def per_view_errors(object_points, image_points, rvecs, tvecs, matrix, distortion):
    """RMS reprojection error per image, in pixels."""
    errors = []
    for index, points in enumerate(object_points):
        projected, _ = cv2.projectPoints(points, rvecs[index], tvecs[index],
                                         matrix, distortion)
        difference = projected.reshape(-1, 2) - image_points[index].reshape(-1, 2)
        errors.append(float(np.sqrt((difference ** 2).sum(axis=1).mean())))
    return errors


def planarity_after_undistortion(samples, matrix, distortion):
    """Per-image flatness check with the fitted distortion removed.

    After undistortion a flat board maps to the image by a homography
    exactly, so what is left is the board's own deviation from a plane plus
    detection noise.  A median far above a few tenths of a pixel means the
    board was bent, and no distortion model can repair that.
    """
    residuals = {}
    for sample in samples:
        points = sample['image_points'].astype(np.float32).reshape(-1, 1, 2)
        undistorted = cv2.undistortPoints(points, matrix, distortion,
                                          P=matrix).reshape(-1, 2)
        source = sample['object_points'][:, :2].astype(np.float32)
        homography, _ = cv2.findHomography(source, undistorted, 0)
        if homography is None:
            continue
        predicted = cv2.perspectiveTransform(source.reshape(-1, 1, 2),
                                             homography).reshape(-1, 2)
        residuals[Path(sample['path']).name] = float(
            np.sqrt(((predicted - undistorted) ** 2).sum(axis=1).mean()))
    return residuals


def run(directory, model='standard', fix_aspect=False, output=None, max_planarity=None):
    directory = Path(directory)
    manifest, board, samples = load_samples(directory)
    usable = [s for s in samples if s['ok'] and s['role'] == 'calibration']
    failed = [s for s in samples if not s['ok']]
    if len(usable) < 6:
        raise RuntimeError(f'only {len(usable)} usable calibration samples; '
                           'collect more before solving')
    height, width = usable[0]['shape']
    object_points = [s['object_points'].astype(np.float32) for s in usable]
    image_points = [s['image_points'].astype(np.float32).reshape(-1, 1, 2)
                    for s in usable]
    flags = MODELS[model] | (cv2.CALIB_FIX_ASPECT_RATIO if fix_aspect else 0)

    result = cv2.calibrateCameraExtended(
        object_points, image_points, (width, height), None, None, flags=flags)
    rms, matrix, distortion, rvecs, tvecs, std_intrinsics = result[:6]

    excluded = {}
    if max_planarity is not None:
        # A frame whose board was bent violates the planar-target model the fit
        # assumes.  The rule is fixed in advance, applied to calibration frames
        # only, and the held-out validation set is never filtered -- so whether
        # dropping them helped is judged by data the rule never touched.
        planarity = planarity_after_undistortion(usable, matrix, distortion)
        keep = [sample for sample in usable
                if planarity[Path(sample['path']).name] <= max_planarity]
        excluded = {Path(sample['path']).name: round(planarity[Path(sample['path']).name], 3)
                    for sample in usable if sample not in keep}
        if len(keep) < 6:
            raise RuntimeError(f'only {len(keep)} frames pass the planarity rule')
        usable = keep
        object_points = [s['object_points'].astype(np.float32) for s in usable]
        image_points = [s['image_points'].astype(np.float32).reshape(-1, 1, 2)
                        for s in usable]
        result = cv2.calibrateCameraExtended(
            object_points, image_points, (width, height), None, None, flags=flags)
        rms, matrix, distortion, rvecs, tvecs, std_intrinsics = result[:6]
    errors = per_view_errors(object_points, image_points, rvecs, tvecs,
                             matrix, distortion)

    fx, fy = float(matrix[0, 0]), float(matrix[1, 1])
    cx, cy = float(matrix[0, 2]), float(matrix[1, 2])
    calibration = {
        'schema': 'rgbcal/intrinsics/1',
        'camera_id': 'environment',
        'device': manifest['camera']['requested']['device'],
        'device_name': manifest.get('device_name'),
        'resolution': {'width': width, 'height': height},
        'pixel_format': manifest['camera']['pixel_format'],
        'image_transform': manifest['camera']['image_transform'],
        # Everything that fixes the imaging geometry, so a later capture can be
        # checked against it instead of trusted to match.
        'imaging': {key: manifest['settings'].get(key) for key in
                    ('device', 'width', 'height', 'autofocus', 'focus', 'hflip')},
        'model': model,
        'distortion_model': ('opencv-5 (k1,k2,p1,p2,k3)' if model == 'standard'
                             else f'opencv-{model}'),
        'K': matrix.tolist(),
        'D': distortion.reshape(-1).tolist(),
        'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy,
        'fov_deg': {
            'horizontal': float(np.degrees(2 * np.arctan(width / (2 * fx)))),
            'vertical': float(np.degrees(2 * np.arctan(height / (2 * fy))))},
        'std_intrinsics': [float(v) for v in np.asarray(std_intrinsics).reshape(-1)[:4]],
        'reprojection_rms_px': float(rms),
        'per_image_rms_px': {Path(s['path']).name: error
                             for s, error in zip(usable, errors)},
        'planarity_residual_px': planarity_after_undistortion(usable, matrix, distortion),
        'planarity_rule': ({'max_planarity_px': max_planarity,
                            'excluded_calibration_frames': excluded,
                            'validation_filtered': False,
                            'why': ('frames whose board deviates from a plane violate '
                                    'the planar-target model; validation is untouched '
                                    'so it can judge whether excluding them helped')}
                           if max_planarity is not None else None),
        'samples': {'calibration_used': len(usable),
                    'validation_held_out': sum(1 for s in samples
                                               if s['role'] == 'validation'),
                    'failed_detection': [s['path'] for s in failed]},
        'board': board.describe(),
        'board_scale_note': ('square length is the printed nominal value unless it '
                             'was measured; it does not affect K or D, only metric '
                             'extrinsics, and rescales them linearly'),
        'source_session': str(directory),
        'valid_for': ('this device at this resolution, pixel format and declared '
                      'transform only; re-calibrate if any of those change'),
    }
    path = Path(output) if output else directory / 'intrinsics.json'
    path.write_text(json.dumps(calibration, indent=2, ensure_ascii=False) + '\n')
    return {'calibration': calibration, 'path': str(path),
            'summary': {'model': model, 'samples_used': len(usable),
                        'reprojection_rms_px': round(float(rms), 4),
                        'fx': round(fx, 2), 'fy': round(fy, 2),
                        'cx': round(cx, 2), 'cy': round(cy, 2),
                        'fov_deg': {k: round(v, 2)
                                    for k, v in calibration['fov_deg'].items()},
                        'D': [round(v, 5) for v in calibration['D']],
                        'worst_image': max(calibration['per_image_rms_px'].items(),
                                           key=lambda item: item[1]),
                        'excluded_for_planarity': excluded,
                        'planarity_median_px': round(float(np.median(list(
                            calibration['planarity_residual_px'].values()))), 3),
                        'written': str(path)}}
