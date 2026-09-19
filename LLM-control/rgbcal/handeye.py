"""Stage 3: measuring where the camera is, in robot base coordinates.

Eye-to-hand: the camera is fixed in the world and a board rides on the
gripper.  For every pose ``i``

    T_base_camera @ T_camera_board(i) == T_base_gripper(i) @ T_gripper_board

with ``T_camera_board`` measured by PnP, ``T_base_gripper`` computed by
forward kinematics, and the two constants unknown.  ``T_base_camera`` is
the answer; ``T_gripper_board`` is how the board happens to sit in the
jaws, which nobody has to measure because the solve returns it too.

What makes this honest rather than hopeful is that forward kinematics is
*not* trusted as given.  The sign of each joint and the offset between the
servo's calibrated zero and the kinematic model's zero are unknown facts
about this particular arm, so they are solved for here alongside the two
transforms, and the residual is reported in pixels on poses the solve
never saw.  A wrong sign does not hide: it cannot be absorbed by the
constants, and it shows up as a residual an order of magnitude too large.
"""
from itertools import product
from pathlib import Path
import json

import cv2
import numpy as np
from scipy.optimize import least_squares

from . import boards as boards_module
from . import robot as robot_module
from . import transforms as tf
from .capture import (RealCamera, check_compatible, save_frame, sharpness,
                      timestamp, transform_image)
from .preview import _put, FrameRate

#: Poses are held out for validation the same way intrinsics samples are:
#: decided at capture time, before any residual exists.
VALIDATION_EVERY = 4
KEY_HELP = 's capture pose | h hud | q finish'
HOLD_KEY_HELP = ('l lock all | 1-5 free one joint | s capture | '
                 'r release all (support the arm!) | q finish')


def board_pose(board, image, matrix, distortion, robust=True):
    """``T_camera_board`` from one image, or ``None`` if the board is not seen.

    ChArUco is required: its markers carry identity, so the board's origin
    and orientation are unambiguous.  A plain chessboard would be detected
    just as well and could silently come back rotated by 180 degrees between
    two frames, which would corrupt the solve rather than fail it.
    """
    if board.kind != 'charuco':
        raise ValueError('hand-eye needs a ChArUco board; a plain chessboard has '
                         'no unique origin and can flip between frames')
    detection = boards_module.detect(board, image, robust=robust)
    if not detection.ok or detection.count < 6:
        return None, detection
    ok, rvec, tvec = cv2.solvePnP(
        detection.object_points.astype(np.float64),
        detection.image_points.astype(np.float64).reshape(-1, 1, 2),
        matrix, distortion, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None, detection
    rvec, tvec = cv2.solvePnPRefineLM(
        detection.object_points.astype(np.float64),
        detection.image_points.astype(np.float64).reshape(-1, 1, 2),
        matrix, distortion, rvec, tvec)
    return tf.from_rvec_tvec(rvec, tvec), detection


def stable_capture(camera, bus, board, matrix, distortion, settings, robust=True,
                   frames=6, flush=4, pose_tolerance_deg=0.4, pose_tolerance_mm=1.5,
                   tick_tolerance=3):
    """A frame and a joint reading that describe the *same* instant.

    A webcam hands over a frame that was exposed some time ago, and an arm
    held by hand keeps settling.  Pressing a key does not make the two
    agree, and a pose recorded while either was still moving is not wrong
    by a little: it breaks the rigid relation the whole calibration rests
    on.  So the image buffer is flushed, several frames and several servo
    reads are taken, and the capture is only accepted if both have stopped
    changing.
    """
    for _ in range(flush):
        camera.frame()
    poses, images, readings = [], [], []
    for _ in range(frames):
        raw = camera.frame()
        view = transform_image(raw, settings)
        pose, detection = board_pose(board, view, matrix, distortion, robust)
        if pose is None:
            return None, None, None, 'board not detected while settling'
        poses.append(pose)
        images.append((raw, view, detection))
        readings.append(bus.read_positions())

    spread_deg = max(np.degrees(tf.rotation_angle(tf.invert(poses[0]) @ pose))
                     for pose in poses[1:])
    spread_mm = max(np.linalg.norm(poses[0][:3, 3] - pose[:3, 3])
                    for pose in poses[1:]) * 1000
    tick_spread = max(max(reading[servo] for reading in readings)
                      - min(reading[servo] for reading in readings)
                      for servo in readings[0])
    if spread_deg > pose_tolerance_deg or spread_mm > pose_tolerance_mm:
        return None, None, None, (f'still moving: board drifted {spread_deg:.2f} deg / '
                                  f'{spread_mm:.1f} mm over {frames} frames')
    if tick_spread > tick_tolerance:
        return None, None, None, f'still moving: joints drifted {tick_spread} ticks'
    middle = len(poses) // 2
    return (poses[middle], images[middle], readings[middle],
            f'stable ({spread_deg:.2f} deg, {spread_mm:.1f} mm, {tick_spread} ticks)')


def collect(settings, directory, board, intrinsics_path, port='/dev/ttyACM0',
            calibration_file=None, robust=True, hold=False):
    """Capture (image, servo ticks) pairs for hand-eye calibration."""
    directory = Path(directory)
    calibration = json.loads(Path(intrinsics_path).read_text())
    check_compatible(calibration, settings)
    matrix = np.array(calibration['K'])
    distortion = np.array(calibration['D'])
    models = (robot_module.joint_models_from_lerobot(calibration_file)
              if calibration_file else robot_module.default_joint_models())

    samples = []
    manifest_path = directory / 'poses.json'
    if manifest_path.exists():
        # Resume: the poses already on disk stay, numbering continues, and the
        # rigidity check picks up from the last one.  Only valid if nothing
        # moved in between -- camera, robot base, board mount -- which the
        # operator has to guarantee; the next rigid check is the first test.
        previous = json.loads(manifest_path.read_text())
        if previous['camera']['requested'].get('device') != settings.device or \
                previous['camera']['resolution'] != [settings.width, settings.height]:
            raise RuntimeError('this session was captured with a different camera '
                               'configuration; start a new session instead')
        samples = previous['samples']
        print(f'resuming {directory.name}: {len(samples)} poses already captured')
    show_hud = True
    rate = FrameRate()
    last_message = ''
    kinematics = robot_module.Kinematics()
    fk_history, board_history = [], []
    previous_ticks = {}
    if samples:
        last = samples[-1]
        previous_ticks = {int(k): v for k, v in last['servo_ticks'].items()}
        fk_history.append(kinematics.T_base_gripper(
            robot_module.angles_from_ticks(previous_ticks, models)))
        board_history.append(np.array(last['pose_T_camera_board']))
    configuration, state, manifest = None, {}, None
    try:
        with robot_module.ServoBus(port) as bus, RealCamera(settings) as camera:
            state = bus.read_state()
            torque = [servo_id for servo_id, entry in state.items()
                      if entry['torque_enabled']]
            print(f'servo bus {port}: {len(state)} servos responding')
            if torque:
                print(f'WARNING: torque is enabled on servos {torque}. Support the arm '
                      'before it is released, or it will drop.')
            held = bool(torque) and hold
            if hold:
                print('hold mode: position the arm by hand, press l to lock it, take your '
                      'hands OFF the arm, press s, then SUPPORT the arm and press r')
            camera.warm_up()
            configuration = camera.configuration()
            window = 'SO101 hand-eye capture'
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window, settings.width, settings.height)
            print(f'capturing into {directory}\n  keys: {KEY_HELP}')
            while True:
                try:
                    raw = camera.frame()
                except RuntimeError as error:
                    print(f'capture error: {error}')
                    break
                view = transform_image(raw, settings)
                rate.tick()
                pose, detection = board_pose(board, view, matrix, distortion, robust)
                canvas = boards_module.draw(view, board, detection)
                usable = pose is not None and sharpness(view) > 60
                cv2.rectangle(canvas, (2, 2), (settings.width - 3, settings.height - 3),
                              (60, 200, 60) if usable else (40, 40, 200), 3)
                if show_hud:
                    lines = [f'poses captured {len(samples)}'
                             + (('   ARM LOCKED - hands off, press s' if held else
                                 '   ARM FREE - support it; press l to lock') if hold else ''),
                             f'board: {detection.note}',
                             (f'distance {np.linalg.norm(pose[:3, 3]):.3f} m'
                              if pose is not None else 'no pose'),
                             f'sharpness {sharpness(view):6.0f}',
                             last_message or (HOLD_KEY_HELP if hold else KEY_HELP)]
                    _put(canvas, lines)
                cv2.imshow(window, canvas)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):
                    break
                if key == ord('h'):
                    show_hud = not show_hud
                if hold and key == ord('l'):
                    try:
                        result = bus.hold()
                        held = True
                        last_message = ('locked; largest joint change '
                                        f'{max(abs(v) for v in result["moved_ticks"].values())} '
                                        'ticks. Hands off, then s')
                    except RuntimeError as error:
                        held = False
                        last_message = f'lock failed: {error}'
                    print(last_message)
                if hold and key == ord('r'):
                    try:
                        alarms = bus.release()
                        last_message = 'released: the arm is limp' + (
                            f' (servo alarms seen: {alarms})' if alarms else '')
                    except RuntimeError as error:
                        last_message = f'RELEASE PROBLEM: {error}'
                    held = False
                    print(last_message)
                if hold and ord('1') <= key <= ord('5'):
                    # Free a single joint and keep the rest locked, so a single-joint
                    # move really is one: the motors, not a hand, keep the others still.
                    joint = robot_module.ARM_JOINTS[key - ord('1')]
                    try:
                        alarms = bus.release([robot_module.JOINT_IDS[joint]])
                        last_message = (f'{joint} is free, the others stay locked. Move it, '
                                        'then l to lock, hands off, s')
                        if alarms:
                            last_message = (f'{joint} released, but it reported {alarms}: '
                                            'it may have tripped while holding. Let it rest')
                    except RuntimeError as error:
                        last_message = f'RELEASE PROBLEM: {error}'
                    held = False
                    print(last_message)
                if key == ord('s') and hold and not held:
                    last_message = 'rejected: lock the arm first (l), then hands off'
                    print(last_message)
                    continue
                if key == ord('s') and hold:
                    alarms = bus.alarms([robot_module.JOINT_IDS[n]
                                         for n in robot_module.ARM_JOINTS])
                    if alarms:
                        held = False
                        last_message = (f'rejected: servo alarm {alarms} - it dropped its '
                                        'torque, so the arm is not in the locked pose')
                        print(last_message)
                        continue
                if key == ord('s'):
                    if not usable:
                        last_message = 'rejected: board not usable in this frame'
                        print(last_message)
                        continue
                    pose, captured, second, note = stable_capture(
                        camera, bus, board, matrix, distortion, settings, robust)
                    if pose is None:
                        last_message = f'rejected: {note}'
                        print(last_message)
                        continue
                    raw, view, detection = captured
                    index = len(samples) + 1
                    role = 'validation' if index % VALIDATION_EVERY == 0 else 'solve'
                    stem = f'{role}-{index:03d}'
                    extra = {'stage': 'handeye', 'role': role, 'index': index,
                             'servo_ticks': {str(k): v for k, v in second.items()},
                             'stability': note,
                             'held_by_torque': bool(held),
                             'board': board.describe(),
                             'image_transform': camera.transform(),
                             'pose_T_camera_board': pose.tolist(),
                             'board_points': detection.count,
                             'sharpness': sharpness(view)}
                    save_frame(raw, directory / 'poses', stem, extra, configuration)
                    samples.append(extra)
                    last_message = f'captured {stem} ({role}), {detection.count} corners'
                    print(last_message)

                    # The board has to ride on the gripper.  If it is lying on the
                    # table instead, every pose still looks perfectly detectable
                    # while the calibration is quietly meaningless, so compare how
                    # far the arm turned with how far the board turned.
                    fk_history.append(kinematics.T_base_gripper(
                        robot_module.angles_from_ticks(second, models)))
                    board_history.append(pose)
                    if len(fk_history) >= 2:
                        # A rigid board must turn by exactly as much as the arm did.
                        # Checking it now, pair by pair, is the difference between
                        # noticing a loose mount immediately and discovering it after
                        # fifty poses.
                        arm_turn = np.degrees(tf.rotation_angle(
                            tf.invert(fk_history[-2]) @ fk_history[-1]))
                        board_turn = np.degrees(tf.rotation_angle(
                            tf.invert(board_history[-2]) @ board_history[-1]))
                        moved = [servo for servo in second
                                 if servo != robot_module.JOINT_IDS['gripper']
                                 and abs(second[servo] - previous_ticks.get(servo, second[servo])) > 11]
                        if arm_turn > 5:
                            mismatch = abs(arm_turn - board_turn)
                            kind = ('single joint' if len(moved) == 1 else
                                    'several joints: depends on the unverified convention')
                            detail = (f'arm {arm_turn:.1f} deg vs board {board_turn:.1f} '
                                      f'deg (off by {mismatch:.1f}; {kind})')
                            if mismatch > 2.0:
                                last_message = f'WARNING: {detail} - mount may be loose'
                                # The jaw is a separate body from the gripper frame the
                                # kinematics reports, and with torque off it swings free.
                                # A board clamped between the jaws rides that, not the arm.
                                jaw = abs(second[robot_module.JOINT_IDS['gripper']]
                                          - previous_ticks[robot_module.JOINT_IDS['gripper']])
                                if jaw > 8:
                                    last_message += (f'; the gripper itself moved {jaw} '
                                                     'ticks - fix the board to the wrist, '
                                                     'not between the jaws')
                            else:
                                last_message = f'rigid check OK: {detail}'
                            print(last_message)
                    previous_ticks = second
            if hold and held:
                # Left holding on purpose: releasing here would drop an arm nobody
                # is necessarily supporting.  Releasing is an explicit act.
                print('the arm is still LOCKED. Support it, then run: '
                      'bash LLM-control/run_rgbcal.sh arm-release')
    finally:
        # Whatever ended the loop -- q, a servo alarm, a camera error -- the
        # poses already written to disk are recorded, so none are orphaned.
        cv2.destroyAllWindows()
        if configuration is not None:
            manifest = write_manifest(directory, configuration, settings, board,
                                      intrinsics_path, port, models, state, samples)
            print(f'\n{len(samples)} poses -> {directory / "poses.json"}')
    return manifest


def write_manifest(directory, configuration, settings, board, intrinsics_path, port,
                   models, state, samples):
    manifest = {'directory': str(directory), 'camera': configuration,
                'settings': settings.as_dict(), 'board': board.describe(),
                'intrinsics': str(intrinsics_path), 'port': port,
                'joint_models': robot_module.models_to_dict(models),
                'validation_every': VALIDATION_EVERY,
                'servo_state_at_start': {str(k): v for k, v in state.items()},
                'samples': samples, 'finished_at': timestamp()}
    (Path(directory) / 'poses.json').write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
    return manifest


def rebuild_manifest(directory):
    """Re-derive poses.json from the per-pose sidecars.

    Every capture writes its own sidecar with the servo reading and the board
    pose, so a session that ended without writing its manifest has lost
    nothing but the index.  The header is taken from the previous manifest.
    """
    directory = Path(directory)
    manifest = json.loads((directory / 'poses.json').read_text())
    samples = []
    for sidecar in sorted((directory / 'poses').glob('*.json')):
        record = json.loads(sidecar.read_text())
        if record.get('stage') != 'handeye':
            continue
        samples.append({key: value for key, value in record.items()
                        if key not in ('image', 'saved_at', 'shape', 'dtype', 'camera')})
    samples.sort(key=lambda item: item['index'])
    recovered = len(samples) - len(manifest['samples'])
    manifest['samples'] = samples
    manifest['rebuilt_at'] = timestamp()
    (directory / 'poses.json').write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + '\n')
    return len(samples), recovered


def _fk_poses(kinematics, models, ticks_list, offsets, signs):
    """``T_base_gripper`` for every pose under a candidate joint convention."""
    poses = []
    for ticks in ticks_list:
        angles = {}
        for name, model in models.items():
            if model.servo_id not in ticks:
                continue
            sign = signs.get(name, model.sign)
            offset = offsets.get(name, 0.0)
            angles[name] = sign * (ticks[model.servo_id] - model.zero_ticks) \
                * 2 * np.pi / robot_module.TICKS_PER_TURN + offset
        poses.append(kinematics.T_base_gripper(angles))
    return poses


def park_hand_eye(motions_a, motions_b):
    """Closed-form ``AX = XB`` after Park and Martin (1994).

    Written out here because OpenCV 5 ships the ``CALIB_HAND_EYE_*`` constants
    but does not expose ``calibrateHandEye`` to Python.  It is only the
    starting point for the non-linear refinement, and the synthetic
    round-trip test is what shows it returns ``T_base_camera`` and not its
    inverse.
    """
    scatter = np.zeros((3, 3))
    for motion_a, motion_b in zip(motions_a, motions_b):
        alpha = cv2.Rodrigues(motion_a[:3, :3])[0].reshape(3)
        beta = cv2.Rodrigues(motion_b[:3, :3])[0].reshape(3)
        scatter += np.outer(beta, alpha)
    eigenvalues, eigenvectors = np.linalg.eigh(scatter.T @ scatter)
    inverse_sqrt = eigenvectors @ np.diag(
        1.0 / np.sqrt(np.maximum(eigenvalues, 1e-12))) @ eigenvectors.T
    rotation = inverse_sqrt @ scatter.T
    # Project onto SO(3); noise leaves the closed form slightly off the manifold.
    u, _, vt = np.linalg.svd(rotation)
    rotation = u @ vt
    if np.linalg.det(rotation) < 0:
        rotation = u @ np.diag([1.0, 1.0, -1.0]) @ vt

    left, right = [], []
    for motion_a, motion_b in zip(motions_a, motions_b):
        left.append(motion_a[:3, :3] - np.eye(3))
        right.append(rotation @ motion_b[:3, 3] - motion_a[:3, 3])
    translation = np.linalg.lstsq(np.vstack(left), np.concatenate(right),
                                  rcond=None)[0]
    return tf.transform(rotation, translation)


def initial_guess(fk_poses, board_poses):
    """Closed-form ``T_base_camera`` from the eye-to-hand relative motions.

    Per pose, ``inv(T_base_gripper) @ T_base_camera @ T_camera_board`` is the
    same constant board mounting.  Eliminating it between two poses leaves
    ``A X = X B`` with ``A = FK_j inv(FK_i)``, ``B = board_j inv(board_i)``
    and ``X = T_base_camera``.
    """
    motions_a, motions_b = [], []
    for stride in (1, 2, 3):
        for index in range(len(fk_poses) - stride):
            other = index + stride
            motions_a.append(fk_poses[other] @ tf.invert(fk_poses[index]))
            motions_b.append(board_poses[other] @ tf.invert(board_poses[index]))
    return park_hand_eye(motions_a, motions_b)


def motion_consistency(fk_poses, board_poses):
    """Convention check that needs no solution at all.

    ``A X = X B`` forces the rotation angle of every relative arm motion to
    equal the rotation angle of the matching relative board motion.  That
    equality holds whatever the camera pose and the board mounting are, so
    a mismatch means the forward kinematics is wrong -- the wrong sign, the
    wrong zero, or the wrong tick scale -- before any calibration is
    attempted.
    """
    angle_errors, pitch_errors = [], []
    for i in range(len(fk_poses) - 1):
        arm = fk_poses[i + 1] @ tf.invert(fk_poses[i])
        board = board_poses[i + 1] @ tf.invert(board_poses[i])
        arm_angle, arm_pitch = tf.screw(arm)
        board_angle, board_pitch = tf.screw(board)
        angle_errors.append(abs(arm_angle - board_angle))
        pitch_errors.append(abs(arm_pitch - board_pitch))
    angle_errors = np.degrees(angle_errors)
    pitch_errors = np.asarray(pitch_errors) * 1000.0
    if not len(angle_errors):
        return {'pairs': 0, 'median_deg': None, 'max_deg': None}
    return {'pairs': int(len(angle_errors)),
            'median_deg': float(np.median(angle_errors)),
            'max_deg': float(np.max(angle_errors)),
            'median_pitch_mm': float(np.median(pitch_errors)),
            'max_pitch_mm': float(np.max(pitch_errors)),
            'meaning': ('relative arm motion vs relative board motion, compared as '
                        'screw parameters. A X = X B makes the two conjugate, so '
                        'both the rotation angle and the translation along the axis '
                        'must agree for any correct forward kinematics')}


def _residuals(parameters, kinematics, models, arm_joints, ticks_list, detections,
               signs, matrix, distortion):
    """Reprojection error, in pixels, of every detected board corner."""
    camera_base = tf.exp_se3(parameters[0:6])         # T_base_camera
    gripper_board = tf.exp_se3(parameters[6:12])      # T_gripper_board
    offsets = dict(zip(arm_joints, parameters[12:]))
    base_camera = tf.invert(camera_base)
    residuals = []
    for ticks, detection in zip(ticks_list, detections):
        angles = {}
        for name, model in models.items():
            if model.servo_id not in ticks:
                continue
            angles[name] = signs.get(name, model.sign) * (
                ticks[model.servo_id] - model.zero_ticks
            ) * 2 * np.pi / robot_module.TICKS_PER_TURN + offsets.get(name, 0.0)
        predicted = base_camera @ kinematics.T_base_gripper(angles) @ gripper_board
        rvec, tvec = tf.to_rvec_tvec(predicted)
        projected, _ = cv2.projectPoints(detection['object_points'], rvec, tvec,
                                         matrix, distortion)
        residuals.append((projected.reshape(-1, 2) - detection['image_points']).ravel())
    return np.concatenate(residuals) if residuals else np.zeros(1)


def _load(directory, intrinsics=None, robust=True):
    directory = Path(directory)
    manifest = json.loads((directory / 'poses.json').read_text())
    calibration = json.loads(Path(intrinsics or manifest['intrinsics']).read_text())
    if calibration['resolution']['width'] != manifest['camera']['resolution'][0]:
        raise RuntimeError('intrinsics resolution does not match the captured poses')
    matrix = np.array(calibration['K'])
    distortion = np.array(calibration['D'])
    board = boards_module.build(
        'charuco-large' if manifest['board'].get('squares_x') == 7 else 'charuco-small',
        square=manifest['board']['square'], marker=manifest['board']['marker'])
    models = robot_module.models_from_dict(manifest['joint_models'])
    settings = type('S', (), {'hflip': manifest['camera']['image_transform']['hflip']})()
    entries = []
    for sample in manifest['samples']:
        path = directory / 'poses' / f'{sample["role"]}-{sample["index"]:03d}.png'
        image = transform_image(cv2.imread(str(path)), settings)
        pose, detection = board_pose(board, image, matrix, distortion, robust)
        if pose is None:
            continue
        entries.append({
            'path': str(path), 'role': sample['role'], 'index': sample['index'],
            'ticks': {int(k): v for k, v in sample['servo_ticks'].items()},
            'T_camera_board': pose,
            'object_points': detection.object_points.astype(np.float64),
            'image_points': detection.image_points.astype(np.float64),
            'corners': detection.count})
    return manifest, calibration, board, models, matrix, distortion, entries


def rank_sign_patterns(entries, kinematics, models, offset_bound_deg=None, step_deg=10.0):
    """Order the 32 sign patterns by a test that needs no optimisation.

    ``motion_consistency`` compares the rotation magnitude of each relative
    arm motion with that of the matching board motion.  That comparison is
    independent of where the camera is and of how the board is mounted, so
    it separates joint conventions before a single fit is run -- which
    matters because a hopeless sign pattern makes the non-linear fit wander
    for a minute before failing.
    """
    ticks = [entry['ticks'] for entry in entries]
    board_poses = [entry['T_camera_board'] for entry in entries]
    # With a bound, each pattern is scored at its best offsets inside it, so a
    # pattern is not dismissed only because its zero is a few degrees away.
    if offset_bound_deg:
        values = np.radians(np.arange(-offset_bound_deg, offset_bound_deg + 1e-9,
                                      step_deg))
        grid = list(product(values, repeat=len(robot_module.IDENTIFIABLE_OFFSETS)))
    else:
        grid = [tuple(0.0 for _ in robot_module.IDENTIFIABLE_OFFSETS)]
    ranked = []
    for pattern in product((1, -1), repeat=len(robot_module.ARM_JOINTS)):
        signs = dict(zip(robot_module.ARM_JOINTS, pattern))
        best = None
        for offsets in grid:
            fk = _fk_poses(kinematics, models, ticks,
                           dict(zip(robot_module.IDENTIFIABLE_OFFSETS, offsets)), signs)
            trial = motion_consistency(fk, board_poses)
            value = trial['median_deg'] + trial['median_pitch_mm']
            if best is None or value < best[0]:
                best = (value, trial)
        score = best[1]
        # One number to sort by: a degree of angle error and a millimetre of
        # screw-translation error are treated as comparably bad.
        ranked.append({'signs': signs, 'median_deg': score['median_deg'],
                       'max_deg': score['max_deg'],
                       'median_pitch_mm': score['median_pitch_mm'],
                       'score': score['median_deg'] + score['median_pitch_mm']})
    ranked.sort(key=lambda item: item['score'])
    return ranked


#: Robust loss for real data.  A few poses are known to be bad in ways no
#: model term describes (the board slipping on a wrist-roll move, the arm
#: settling inside gear backlash); a squared loss lets each of them pull the
#: camera pose by centimetres.  Huber with a scale near the typical residual
#: keeps them in the fit but caps their influence.  Fixed before fitting,
#: applied to every pose alike.
ROBUST_LOSS = 'huber'
ROBUST_SCALE_PX = 15.0


#: How far the kinematic model's zero may sit from the servo-calibrated zero.
#:
#: lerobot's calibration defines zero at the arm's middle pose and the
#: so101_new_calib model is built around that same pose; from this arm's own
#: calibration file the two differ by at most ~8 degrees.  Without a bound,
#: flipping the three pitch joints' signs and adding ~180 degrees to the
#: elbow and wrist reproduces almost the same motion (the elbow-flipped
#: mirror of the arm), and the solver can prefer it while placing the camera
#: behind the robot.  The bound encodes a fact about this robot, not a wish
#: about the answer, and it is recorded in the result.
MAX_OFFSET_DEG = 30.0


def _fit(entries, kinematics, models, signs, matrix, distortion, free_offsets=True,
         max_nfev=4000, loss='linear', f_scale=1.0, offset_bound_deg=None):
    """One complete fit under a fixed sign pattern; returns parameters and cost."""
    arm_joints = list(robot_module.IDENTIFIABLE_OFFSETS)
    ticks = [entry['ticks'] for entry in entries]
    fk = _fk_poses(kinematics, models, ticks, {}, signs)
    guess = initial_guess(fk, [entry['T_camera_board'] for entry in entries])
    gripper_board = tf.invert(fk[0]) @ guess @ entries[0]['T_camera_board']
    start = np.concatenate([tf.log_se3(guess), tf.log_se3(gripper_board),
                            np.zeros(len(arm_joints))])
    if not free_offsets:
        start = start[:12]

    def residual(parameters):
        full = (parameters if free_offsets
                else np.concatenate([parameters, np.zeros(len(arm_joints))]))
        return _residuals(full, kinematics, models, arm_joints, ticks, entries,
                          signs, matrix, distortion)

    if offset_bound_deg is not None and free_offsets:
        limit = np.radians(offset_bound_deg)
        lower = np.concatenate([np.full(12, -np.inf), np.full(len(arm_joints), -limit)])
        upper = np.concatenate([np.full(12, np.inf), np.full(len(arm_joints), limit)])
        solution = least_squares(residual, start, method='trf', bounds=(lower, upper),
                                 loss=loss, f_scale=f_scale, x_scale='jac',
                                 max_nfev=max_nfev)
    elif loss == 'linear':
        solution = least_squares(residual, start, method='lm', max_nfev=max_nfev)
    else:
        # Start the robust fit from the least-squares one: Huber is only
        # needed to stop outliers dominating, not to find the basin.
        first = least_squares(residual, start, method='lm', max_nfev=max_nfev)
        solution = least_squares(residual, first.x, method='trf', loss=loss,
                                 f_scale=f_scale, x_scale='jac', max_nfev=max_nfev)
    parameters = (solution.x if free_offsets
                  else np.concatenate([solution.x, np.zeros(len(arm_joints))]))
    errors = np.linalg.norm(residual(solution.x).reshape(-1, 2), axis=1)
    return {'parameters': parameters, 'rms_px': float(np.sqrt((errors ** 2).mean())),
            'max_px': float(errors.max()), 'signs': dict(signs),
            'T_base_camera': tf.exp_se3(parameters[0:6]),
            'T_gripper_board': tf.exp_se3(parameters[6:12]),
            'offsets_rad': dict(zip(arm_joints, parameters[12:]))}


EXPERIMENTAL_FILE = 'extrinsics-experimental.json'
VERIFIED_FILE = 'extrinsics.json'


def solve(directory, intrinsics=None, search_signs=True, robust=True,
          experimental=False, min_poses=8, loss=ROBUST_LOSS, f_scale=ROBUST_SCALE_PX,
          bootstrap=0, offset_bound_deg=MAX_OFFSET_DEG):
    """Fit ``T_base_camera``, the board mounting and the joint convention."""
    directory = Path(directory)
    manifest, calibration, board, models, matrix, distortion, entries = _load(
        directory, intrinsics, robust)
    fitting = [entry for entry in entries if entry['role'] == 'solve']
    holdout = [entry for entry in entries if entry['role'] == 'validation']
    if len(fitting) < min_poses:
        raise RuntimeError(f'only {len(fitting)} usable poses (need {min_poses}); '
                           'capture more')
    kinematics = robot_module.Kinematics()
    arm_joints = list(robot_module.ARM_JOINTS)
    identifiable = list(robot_module.IDENTIFIABLE_OFFSETS)

    baseline = {name: models[name].sign for name in arm_joints}
    if search_signs:
        ranked = rank_sign_patterns(fitting, kinematics, models,
                                    offset_bound_deg=offset_bound_deg)
        candidates = [item['signs'] for item in ranked[:4]]
    else:
        ranked, candidates = [], [baseline]
    results = []
    for signs in candidates:
        try:
            results.append(_fit(fitting, kinematics, models, signs, matrix,
                                distortion, max_nfev=400, loss=loss, f_scale=f_scale,
                                offset_bound_deg=offset_bound_deg))
        except Exception:  # a hopeless pattern can still make the fit diverge
            continue
    if not results:
        raise RuntimeError('no sign pattern produced a usable fit')
    results.sort(key=lambda item: item['rms_px'])
    # Refine only the winner, now that the convention is settled.
    best = _fit(fitting, kinematics, models, results[0]['signs'], matrix, distortion,
                loss=loss, f_scale=f_scale, offset_bound_deg=offset_bound_deg)
    runner_up = results[1] if len(results) > 1 else None

    # Per-pose residuals of the final fit, so a bad pose is named, not hidden.
    per_pose = {}
    base_camera = tf.invert(best['T_base_camera'])
    for entry, fk in zip(fitting, _fk_poses(kinematics, models,
                                            [e['ticks'] for e in fitting],
                                            best['offsets_rad'], best['signs'])):
        predicted = base_camera @ fk @ best['T_gripper_board']
        rvec, tvec = tf.to_rvec_tvec(predicted)
        projected, _ = cv2.projectPoints(entry['object_points'], rvec, tvec,
                                         matrix, distortion)
        difference = projected.reshape(-1, 2) - entry['image_points']
        per_pose[f"{entry['role']}-{entry['index']:03d}"] = float(
            np.sqrt((difference ** 2).sum(axis=1).mean()))
    typical = float(np.median(list(per_pose.values())))

    uncertainty = None
    if bootstrap:
        rng = np.random.default_rng(0)
        positions, rotations = [], []
        for _ in range(bootstrap):
            sample = [fitting[i] for i in rng.integers(0, len(fitting), len(fitting))]
            try:
                again = _fit(sample, kinematics, models, best['signs'], matrix,
                             distortion, loss=loss, f_scale=f_scale, max_nfev=600,
                             offset_bound_deg=offset_bound_deg)
            except Exception:
                continue
            positions.append(again['T_base_camera'][:3, 3])
            rotations.append(np.degrees(tf.rotation_angle(
                tf.invert(best['T_base_camera']) @ again['T_base_camera'])))
        if positions:
            positions = np.array(positions)
            uncertainty = {
                'resamples': len(positions),
                'camera_position_std_mm': (positions.std(axis=0) * 1000).tolist(),
                'camera_position_spread_mm_95': float(np.percentile(
                    np.linalg.norm(positions - best['T_base_camera'][:3, 3], axis=1),
                    95) * 1000),
                'camera_rotation_spread_deg_95': float(np.percentile(rotations, 95)),
                'meaning': ('how much T_base_camera moves when the fit is repeated on '
                            'resampled poses: an error bar on the extrinsic itself, '
                            'not on the images'),
            }

    fk_best = _fk_poses(kinematics, models, [e['ticks'] for e in fitting],
                        best['offsets_rad'], best['signs'])
    consistency = motion_consistency(fk_best,
                                     [e['T_camera_board'] for e in fitting])
    camera_position = best['T_base_camera'][:3, 3]
    rotation_cv = best['T_base_camera'][:3, :3]
    report = {
        'schema': 'rgbcal/extrinsics/1',
        # An experimental result is written to its own file and says so, so it
        # can never be picked up where a verified extrinsic is expected.
        'status': 'EXPERIMENTAL-UNVERIFIED' if experimental else 'solved',
        'usable_for_motion': False,
        'usable_for_motion_note': ('set only after held-out verification passes and '
                                   'a supervised hover test confirms it'),
        'frames': {
            'T_base_camera': ('maps OpenCV camera coordinates into robot base '
                              'coordinates: p_base = T_base_camera @ p_camera'),
            'T_gripper_board': 'maps board coordinates into gripper-frame coordinates',
            'gripper_frame': "the model's gripperframe site (the simulation's TCP)",
        },
        'T_base_camera': best['T_base_camera'].tolist(),
        'camera_position_base_m': camera_position.tolist(),
        'camera_rotation_opencv': rotation_cv.tolist(),
        'camera_rotation_opengl': tf.opencv_rotation_to_opengl(rotation_cv).tolist(),
        'T_gripper_board': best['T_gripper_board'].tolist(),
        'joint_convention': {
            'signs': best['signs'],
            'zero_offsets_rad': best['offsets_rad'],
            'zero_offsets_deg': {name: float(np.degrees(value))
                                 for name, value in best['offsets_rad'].items()},
            'offsets_fitted': identifiable,
            'offsets_pinned_to_servo_zero': [name for name in arm_joints
                                             if name not in identifiable],
            'why_pinned': ("the first joint's zero is indistinguishable from a "
                           'rotation of the camera about the base z axis, and the '
                           "last joint's from the board mounting; fitting them "
                           'would make the camera pose unidentifiable'),
            'ticks_per_turn': robot_module.TICKS_PER_TURN,
            'solved': True,
            'note': ('sign and zero offset were solved from the observations, not '
                     'copied from a configuration file'),
        },
        'fit': {'poses_used': len(fitting), 'rms_px': best['rms_px'],
                'max_px': best['max_px'], 'loss': loss, 'loss_scale_px': f_scale,
                'median_pose_rms_px': typical,
                'per_pose_rms_px': per_pose,
                'poses_over_3x_median': sorted(name for name, value in per_pose.items()
                                               if value > 3 * typical)},
        'uncertainty': uncertainty,
        'offset_bound_deg': offset_bound_deg,
        'plausibility': plausibility(best['T_base_camera']),
        'sign_search': {
            'patterns_screened': len(ranked),
            'patterns_fitted': len(results),
            'consistency_ranking_deg': [
                {'signs': item['signs'], 'median_deg': round(item['median_deg'], 3),
                 'median_pitch_mm': round(item['median_pitch_mm'], 2)}
                for item in ranked[:4]],
            'best_rms_px': best['rms_px'],
            'runner_up_rms_px': runner_up['rms_px'] if runner_up else None,
            'separation': (runner_up['rms_px'] / best['rms_px']
                           if runner_up and best['rms_px'] > 0 else None),
            'meaning': ('a wrong sign cannot be absorbed by the unknown transforms; '
                        'a clear gap to the runner-up is what makes the choice evidence'),
        },
        'motion_consistency': consistency,
        'intrinsics': str(intrinsics or manifest['intrinsics']),
        'source_session': str(directory),
        'validation_poses_held_out': len(holdout),
        'valid_while': ('the camera, the robot base and the table stay where they '
                        'were when this was measured'),
    }
    if experimental:
        report['experimental_reason'] = (
            'solved on data known to violate the rigid board-on-gripper assumption, '
            'at the operator\'s request, to see how far off it is')
    name = EXPERIMENTAL_FILE if experimental else VERIFIED_FILE
    (directory / name).write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
    report['written'] = str(directory / name)
    return report


def plausibility(base_camera):
    """Physical sanity of a solved camera pose, stated as checks, not hopes.

    The environment camera sits above the table and looks at the robot.  A
    solution that puts it under the base plane, or facing away from the base,
    is a mirror or a divergence however small its residual.
    """
    position = base_camera[:3, 3]
    axis = base_camera[:3, 2]                      # OpenCV optical axis in base
    toward_base = -position / max(np.linalg.norm(position), 1e-9)
    checks = {
        'above_base_plane': bool(position[2] > 0),
        'looks_down': bool(axis[2] < 0),
        'faces_the_robot': bool(axis @ toward_base > 0.5),
    }
    checks['ok'] = all(checks.values())
    return checks


def verify(directory, intrinsics=None, robust=True, experimental=False):
    """Check the solved extrinsics on poses the fit never saw."""
    directory = Path(directory)
    name = EXPERIMENTAL_FILE if experimental else VERIFIED_FILE
    report = json.loads((directory / name).read_text())
    manifest, calibration, board, models, matrix, distortion, entries = _load(
        directory, intrinsics, robust)
    kinematics = robot_module.Kinematics()
    base_camera = tf.invert(np.array(report['T_base_camera']))
    gripper_board = np.array(report['T_gripper_board'])
    signs = {name: int(value) for name, value in
             report['joint_convention']['signs'].items()}
    offsets = report['joint_convention']['zero_offsets_rad']

    groups = {}
    for role in ('validation', 'solve'):
        chosen = [entry for entry in entries if entry['role'] == role]
        if not chosen:
            continue
        fk = _fk_poses(kinematics, models, [e['ticks'] for e in chosen], offsets, signs)
        pixel_errors, position_errors = [], []
        for entry, pose in zip(chosen, fk):
            predicted = base_camera @ pose @ gripper_board
            rvec, tvec = tf.to_rvec_tvec(predicted)
            projected, _ = cv2.projectPoints(entry['object_points'], rvec, tvec,
                                             matrix, distortion)
            difference = projected.reshape(-1, 2) - entry['image_points']
            pixel_errors.append(float(np.sqrt((difference ** 2).sum(axis=1).mean())))
            # Where the board centre is predicted to be, against where it was seen.
            observed = entry['T_camera_board'][:3, 3]
            position_errors.append(float(np.linalg.norm(predicted[:3, 3] - observed)))
        groups[role] = {
            'poses': len(chosen),
            'reprojection_rms_px': round(float(np.sqrt(np.mean(
                np.square(pixel_errors)))), 4),
            'reprojection_worst_px': round(float(max(pixel_errors)), 4),
            'board_position_error_mm': {
                'median': round(float(np.median(position_errors)) * 1000, 2),
                'max': round(float(max(position_errors)) * 1000, 2)},
            'per_pose_px': {Path(entry['path']).name: round(error, 3)
                            for entry, error in zip(chosen, pixel_errors)},
        }
    result = {'extrinsics': str(directory / name), **groups,
              'independent_check': ('validation poses were assigned at capture '
                                    'time and were not used in the fit'),
              'camera_position_base_m': report['camera_position_base_m'],
              'sign_search': report['sign_search'],
              'motion_consistency': report['motion_consistency']}
    verdict = []
    if 'validation' in groups and 'solve' in groups:
        ratio = groups['validation']['reprojection_rms_px'] / max(
            groups['solve']['reprojection_rms_px'], 1e-9)
        verdict.append(f'held-out / fitted reprojection ratio {ratio:.2f}: '
                       + ('consistent' if ratio < 2.0 else 'held-out much worse'))
    separation = report['sign_search'].get('separation')
    if separation:
        verdict.append(f'best sign pattern is {separation:.1f}x better than the '
                       'runner-up' + (' (decisive)' if separation > 3 else
                                      ' (NOT decisive: capture more varied poses)'))
    consistency = report['motion_consistency']['median_deg']
    if consistency is not None:
        verdict.append(f'arm/board relative rotation agreement {consistency:.2f} deg '
                       'median' + (' (good)' if consistency < 1.0 else
                                   ' (forward kinematics still suspect)'))
    checks = report.get('plausibility') or plausibility(np.array(report['T_base_camera']))
    verdict.append('camera pose is physically plausible' if checks['ok'] else
                   f'camera pose FAILS physical checks: {checks}')
    result['plausibility'] = checks
    result['verdict'] = verdict
    result['status'] = report.get('status')
    out = ('extrinsics-experimental_verification.json' if experimental
           else 'extrinsics_verification.json')
    (directory / out).write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n')
    return result


def single_joint_pairs(entries, models, min_ticks=90, still_ticks=11):
    """Consecutive pose pairs in which exactly one arm joint moved.

    Only these test the arm without assuming anything about it: whatever
    the joint's sign, zero or axis, a rigid board has to turn by exactly the
    joint's own angle, about a fixed line, with no slide along that line.
    """
    names = {model.servo_id: name for name, model in models.items()}
    pairs = []
    for before, after in zip(entries, entries[1:]):
        moved = {names[servo]: after['ticks'][servo] - before['ticks'][servo]
                 for servo in before['ticks']
                 if names.get(servo) in robot_module.ARM_JOINTS}
        active = [name for name, delta in moved.items() if abs(delta) > still_ticks]
        if len(active) != 1 or abs(moved[active[0]]) < min_ticks:
            continue
        joint = active[0]
        motion = after['T_camera_board'] @ tf.invert(before['T_camera_board'])
        board_angle, pitch = tf.screw(motion)
        servo_angle = abs(moved[joint]) * 360.0 / robot_module.TICKS_PER_TURN
        pairs.append({'joint': joint, 'from': before['index'], 'to': after['index'],
                      'servo_deg': servo_angle, 'board_deg': float(np.degrees(board_angle)),
                      'ratio': float(np.degrees(board_angle)) / servo_angle,
                      'error_deg': float(np.degrees(board_angle)) - servo_angle,
                      'pitch_mm': pitch * 1000.0,
                      'start_ticks': before['ticks']})
    return pairs


def joint_report(directory, intrinsics=None, robust=True):
    """Per-joint evidence: does each joint behave like the revolute joint the
    model says it is, and by how much does the board disagree with it?"""
    manifest, calibration, board, models, matrix, distortion, entries = _load(
        directory, intrinsics, robust)
    entries.sort(key=lambda entry: entry['index'])
    pairs = single_joint_pairs(entries, models)
    report = {}
    for joint in robot_module.ARM_JOINTS:
        chosen = [pair for pair in pairs if pair['joint'] == joint]
        if not chosen:
            report[joint] = {'pairs': 0}
            continue
        errors = np.array([pair['error_deg'] for pair in chosen])
        report[joint] = {
            'pairs': len(chosen),
            'median_ratio': float(np.median([pair['ratio'] for pair in chosen])),
            'median_abs_error_deg': float(np.median(np.abs(errors))),
            'max_abs_error_deg': float(np.max(np.abs(errors))),
            'median_abs_pitch_mm': float(np.median([abs(p['pitch_mm']) for p in chosen])),
            'pairs_detail': [{k: (round(v, 2) if isinstance(v, float) else v)
                              for k, v in pair.items() if k != 'start_ticks'}
                             for pair in chosen],
        }
    return report
