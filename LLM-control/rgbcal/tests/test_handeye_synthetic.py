"""Round-trip checks on synthetic data, where the right answer is known.

These exist because the failure modes of hand-eye calibration are silent.
An inverted transform, a swapped frame convention or an OpenGL/OpenCV mixup
all produce a plausible-looking matrix; only a case with a known answer
shows that the pipeline returns *that* answer.
"""
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from rgbcal import boards, handeye, robot, transforms as tf

INTRINSICS = Path(__file__).resolve().parents[2] / 'calib/intrinsics-02/intrinsics.json'
rng = np.random.default_rng(7)


def _true_camera():
    """A camera in front of the robot, raised and looking down at the table."""
    position = np.array([0.55, 0.05, 0.34])
    forward = -position / np.linalg.norm(position)          # looks at the base
    right = np.cross([0, 0, 1.0], forward)
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    return tf.transform(np.column_stack([right, down, forward]), position)


def _synthetic_session(poses=16, noise_px=0.0):
    calibration = json.loads(INTRINSICS.read_text())
    matrix, distortion = np.array(calibration['K']), np.array(calibration['D'])
    board = boards.build('charuco-large')
    object_points = board.board().getChessboardCorners().astype(np.float64)
    kinematics = robot.Kinematics()
    models = robot.default_joint_models()
    arm = list(robot.ARM_JOINTS)

    true_signs = {'shoulder_pan': 1, 'shoulder_lift': -1, 'elbow_flex': 1,
                  'wrist_flex': -1, 'wrist_roll': 1}
    # Only the identifiable offsets get a non-zero truth: the first and last
    # joints' offsets are not separable from the camera pose and the board
    # mounting, and the solver pins them for exactly that reason.
    true_offsets = {name: 0.0 for name in arm}
    for name, value in zip(robot.IDENTIFIABLE_OFFSETS,
                           rng.uniform(-0.06, 0.06, len(robot.IDENTIFIABLE_OFFSETS))):
        true_offsets[name] = float(value)
    camera = _true_camera()
    mount = tf.transform(cv2.Rodrigues(np.array([0.1, -0.2, 0.05]))[0],
                         [0.02, 0.0, 0.06])

    # Only poses where the board would actually have been detected are kept:
    # the real collector cannot save a frame whose board is out of frame or
    # behind the camera, so a synthetic set that contains them is not a model
    # of this problem.
    entries = []
    attempts = 0
    while len(entries) < poses and attempts < 4000:
        attempts += 1
        angles = dict(zip(arm, rng.uniform(-0.6, 0.6, len(arm))))
        ticks = {}
        for name in arm:
            model = models[name]
            ticks[model.servo_id] = float(model.zero_ticks + (
                (angles[name] - true_offsets[name]) / true_signs[name]
            ) * robot.TICKS_PER_TURN / (2 * np.pi))
        fk = kinematics.T_base_gripper(angles)
        camera_board = tf.invert(camera) @ fk @ mount
        rvec, tvec = tf.to_rvec_tvec(camera_board)
        projected, _ = cv2.projectPoints(object_points, rvec, tvec, matrix, distortion)
        depths = (camera_board[:3, :3] @ object_points.T).T + camera_board[:3, 3]
        image_points = projected.reshape(-1, 2)
        visible = (depths[:, 2] > 0.20) & (depths[:, 2] < 1.20) \
            & (image_points[:, 0] > 0) & (image_points[:, 0] < calibration['resolution']['width']) \
            & (image_points[:, 1] > 0) & (image_points[:, 1] < calibration['resolution']['height'])
        # ChArUco detection is partial by design, so keep the visible corners
        # and require as many as the real collector demands.
        if visible.sum() < 8:
            continue
        seen_object = object_points[visible]
        seen_image = image_points[visible]
        if noise_px:
            seen_image = seen_image + rng.normal(0, noise_px, seen_image.shape)
        entries.append({'role': 'solve', 'ticks': ticks, 'T_camera_board': camera_board,
                        'object_points': seen_object, 'image_points': seen_image,
                        'path': f'synthetic-{len(entries):03d}'})
    if len(entries) < poses:
        raise RuntimeError(f'only {len(entries)} visible synthetic poses')
    return (entries, kinematics, models, matrix, distortion, camera, mount,
            true_signs, true_offsets)


@pytest.mark.parametrize('noise_px', [0.0, 0.3])
def test_recovers_camera_pose_signs_and_offsets(noise_px):
    (entries, kinematics, models, matrix, distortion, camera, mount,
     true_signs, true_offsets) = _synthetic_session(noise_px=noise_px)

    ranked = handeye.rank_sign_patterns(entries, kinematics, models)
    assert ranked[0]['signs'] == true_signs, 'the cheap screen ranked the wrong signs first'
    results = sorted(
        (handeye._fit(entries, kinematics, models, item['signs'], matrix,
                      distortion, max_nfev=400) for item in ranked[:4]),
        key=lambda item: item['rms_px'])
    best = handeye._fit(entries, kinematics, models, results[0]['signs'],
                        matrix, distortion)

    assert best['signs'] == true_signs, 'the sign search picked the wrong convention'
    assert best['rms_px'] < max(noise_px * 2, 0.05)
    # The camera position must come back in metres in the base frame.
    assert np.allclose(best['T_base_camera'][:3, 3], camera[:3, 3], atol=2e-3)
    assert np.allclose(best['T_base_camera'][:3, :3], camera[:3, :3], atol=5e-3)
    assert np.allclose(best['T_gripper_board'], mount, atol=5e-3)
    for name in robot.IDENTIFIABLE_OFFSETS:
        assert abs(best['offsets_rad'][name] - true_offsets[name]) < 0.01, name
    assert set(best['offsets_rad']) == set(robot.IDENTIFIABLE_OFFSETS)
    # The runner-up must be clearly worse, or the choice is not evidence.
    assert results[1]['rms_px'] > best['rms_px'] * 3


def test_motion_consistency_detects_a_wrong_sign():
    (entries, kinematics, models, *_rest, true_signs, true_offsets) = \
        _synthetic_session(noise_px=0.0)
    board_poses = [entry['T_camera_board'] for entry in entries]
    ticks = [entry['ticks'] for entry in entries]

    right = handeye._fk_poses(kinematics, models, ticks, true_offsets, true_signs)
    assert handeye.motion_consistency(right, board_poses)['max_deg'] < 1e-6

    wrong = dict(true_signs)
    wrong['elbow_flex'] *= -1
    flipped = handeye._fk_poses(kinematics, models, ticks, true_offsets, wrong)
    assert handeye.motion_consistency(flipped, board_poses)['median_deg'] > 1.0


def test_eye_to_hand_initial_guess_has_the_documented_direction():
    """``initial_guess`` must return base<-camera, not its inverse."""
    (entries, kinematics, models, *_rest, true_signs, true_offsets) = \
        _synthetic_session(noise_px=0.0)
    fk = handeye._fk_poses(kinematics, models, [e['ticks'] for e in entries],
                           true_offsets, true_signs)
    guess = handeye.initial_guess(fk, [e['T_camera_board'] for e in entries])
    camera = _true_camera()
    assert np.allclose(guess, camera, atol=1e-6)
    # The camera origin in base coordinates is the translation column.
    assert np.allclose(guess[:3, 3], camera[:3, 3], atol=1e-9)
