"""End-to-end check: where the camera says an object is, against where the
arm actually is when it holds that object.

No motion is commanded.  The operator places a cube, the camera estimates
its grasp centre from the undistorted frame and the measured table, and
then the operator closes the gripper around the cube by hand exactly as a
grasp would.  The servo reading, turned into a TCP position with the
*solved* joint convention, is compared with the camera's estimate.

The difference is the number that matters for grasping.  It lumps together
the camera, the extrinsic, the arm's kinematics and the gap between the
model's TCP and the real grasp centre, precisely because a real grasp
suffers all of them at once, and it does so without assuming where on the
gripper the TCP is.
"""
from pathlib import Path
import json

import cv2
import numpy as np

from nexus_vision import perception
from . import realcam
from . import robot as robot_module
from .capture import save_frame, timestamp

CUBE_EDGE_M = 0.025
#: Colour blobs whose footprint is far from a cube's are rejected: a hand or
#: an arm in frame is red to the colour threshold as well.
FOOTPRINT_RANGE_M = (0.012, 0.045)


def solved_arm(extrinsics):
    """Joint models plus the solved signs and offsets, as one FK function."""
    session = Path(extrinsics['source_session'])
    manifest = json.loads((session / 'poses.json').read_text())
    models = robot_module.models_from_dict(manifest['joint_models'])
    convention = extrinsics['joint_convention']
    signs = {name: int(value) for name, value in convention['signs'].items()}
    offsets = {name: float(value) for name, value in convention['zero_offsets_rad'].items()}
    kinematics = robot_module.Kinematics()

    def tcp(ticks):
        angles = {}
        for name, model in models.items():
            if model.servo_id not in ticks or name not in robot_module.ARM_JOINTS:
                continue
            angles[name] = signs.get(name, model.sign) * (
                ticks[model.servo_id] - model.zero_ticks
            ) * 2 * np.pi / robot_module.TICKS_PER_TURN + offsets.get(name, 0.0)
        return kinematics.T_base_gripper(angles), angles

    return tcp


def cube_candidates(frame, calibration, colour='red', support_height=0.0):
    """Cube estimates in BASE coordinates from one undistorted frame."""
    rgb = cv2.cvtColor(frame['undistorted'], cv2.COLOR_BGR2RGB)
    # plane_z is measured from the TABLE along its normal: 0 for a cube on the
    # table, 0.025 for a cube resting on another 25 mm cube.
    proposals = perception.locate_color(rgb, None, frame['calibration'], colour,
                                        plane_z=support_height, object_height=CUBE_EDGE_M)
    found = []
    for proposal in proposals:
        support = proposal.get('support')
        if not support:
            continue
        extent = support['extent']
        plausible = all(FOOTPRINT_RANGE_M[0] <= value <= FOOTPRINT_RANGE_M[1]
                        for value in extent)
        found.append({
            'pixel': proposal['pixel'], 'bbox': proposal['bbox'], 'area': proposal['area'],
            'footprint_m': extent, 'plausible_cube': plausible,
            'grasp_centre_base': realcam.table_to_base(
                calibration, support['grasp_center']).tolist(),
            'footprint_centre_base': realcam.table_to_base(
                calibration, support['center']).tolist(),
            'assumptions': {'plane': f'measured table + {support_height * 1000:.0f} mm '
                                     '(table frame z = support height)',
                            'object_height_m': CUBE_EDGE_M, 'method': support['method']},
        })
    return found


def run(settings, directory, port='/dev/ttyACM0', support_height=0.0):
    directory = Path(directory)
    calibration = realcam.load()
    status = realcam.motion_status(calibration, settings)
    tcp = solved_arm(calibration.extrinsics)
    records, estimate, held, message = [], None, False, ''
    frames_dir = directory / 'frames'
    # Resume: earlier points stay, and numbering continues from what is on disk,
    # so pressing c twice or restarting never collides with a saved frame.
    previous = directory / 'point_check.json'
    if previous.exists():
        records = json.loads(previous.read_text()).get('records', [])
        if records:
            print(f'resuming {directory.name}: {len(records)} points already recorded')

    def next_stem(prefix):
        existing = list(frames_dir.glob(f'{prefix}-*.png')) if frames_dir.exists() else []
        return f'{prefix}-{len(existing) + 1:03d}'

    window = 'SO101 point check'
    with robot_module.ServoBus(port) as bus, \
            realcam.EnvironmentCamera(settings, calibration) as camera:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window, 1280, 720)
        print('c = camera estimate (cube alone, arm away) | l = lock arm | '
              's = record the arm (gripper closed around the cube) | '
              'r = release (support the arm!) | q = quit')
        print('motion use:', status)
        try:
            while True:
                frame = camera.frame()
                view = frame['undistorted'].copy()
                candidates = cube_candidates(frame, calibration,
                                             support_height=support_height)
                for candidate in candidates:
                    x0, y0, x1, y1 = candidate['bbox']
                    colour = (60, 220, 60) if candidate['plausible_cube'] else (60, 60, 220)
                    cv2.rectangle(view, (x0, y0), (x1, y1), colour, 2)
                    g = candidate['grasp_centre_base']
                    cv2.putText(view, f'({g[0]*1000:.0f},{g[1]*1000:.0f},{g[2]*1000:.0f}) mm',
                                (x0, max(y0 - 8, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                                colour, 2, cv2.LINE_AA)
                lines = ['camera estimate: ' + ('none - press c with the cube alone'
                                                if estimate is None else
                                                str(np.round(np.array(estimate['grasp_centre_base']) * 1000, 1)) + ' mm'),
                         'arm: ' + ('LOCKED - hands off, press s' if held else 'free'),
                         message or 'c estimate | l lock | s record | r release | q quit']
                for index, line in enumerate(lines):
                    cv2.putText(view, line, (20, 40 + 36 * index), cv2.FONT_HERSHEY_SIMPLEX,
                                1.0, (255, 255, 255), 3, cv2.LINE_AA)
                    cv2.putText(view, line, (20, 40 + 36 * index), cv2.FONT_HERSHEY_SIMPLEX,
                                1.0, (20, 20, 20), 1, cv2.LINE_AA)
                cv2.imshow(window, view)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):
                    break
                if key == ord('c'):
                    cubes = [c for c in candidates if c['plausible_cube']]
                    if len(cubes) != 1:
                        message = (f'need exactly one cube-sized red blob, found {len(cubes)} '
                                   f'(of {len(candidates)} red regions)')
                    else:
                        estimate = cubes[0]
                        estimate['frame'] = str(save_frame(
                            frame['raw'], frames_dir, next_stem('estimate'),
                            {'stage': 'point-check', 'estimate': estimate}))
                        message = ('estimate stored (press c again to replace it); now '
                                   'close the gripper around the cube')
                    print(message)
                if key == ord('l'):
                    try:
                        # The gripper is locked with the arm: a cube clamped the same
                        # way every time sits at the same place in the gripper frame,
                        # which is what makes the grasp centre measurable at all.
                        bus.hold(list(robot_module.JOINT_IDS.values()))
                        held, message = True, 'arm AND gripper locked; hands off, then s'
                    except RuntimeError as error:
                        held, message = False, f'lock failed: {error}'
                    print(message)
                if key == ord('r'):
                    try:
                        alarms = bus.release(list(robot_module.JOINT_IDS.values()))
                        message = 'released' + (f' (alarms {alarms})' if alarms else '')
                    except RuntimeError as error:
                        message = f'RELEASE PROBLEM: {error}'
                    held = False
                    print(message)
                if key == ord('s'):
                    if estimate is None:
                        message = ('no camera estimate for this placement: support the '
                                   'arm, r, move it out of view, then c')
                    elif not held:
                        message = 'lock the arm first (l), hands off'
                    else:
                        ticks = bus.read_positions()
                        pose, angles = tcp(ticks)
                        arm_point = pose[:3, 3]
                        camera_point = np.array(estimate['grasp_centre_base'])
                        difference = camera_point - arm_point
                        record = {'index': len(records) + 1, 'recorded_at': timestamp(),
                                  'gripper_locked': True,
                                  'grasp_in_gripper_frame_m': (
                                      pose[:3, :3].T @ (np.array(estimate['grasp_centre_base'])
                                                        - pose[:3, 3])).tolist(),
                                  'camera_grasp_centre_base_m': camera_point.tolist(),
                                  'arm_tcp_base_m': arm_point.tolist(),
                                  'camera_minus_arm_mm': (difference * 1000).tolist(),
                                  'distance_mm': float(np.linalg.norm(difference) * 1000),
                                  'horizontal_mm': float(np.linalg.norm(difference[:2]) * 1000),
                                  'servo_ticks': {str(k): v for k, v in ticks.items()},
                                  'joint_angles_rad': angles, 'estimate': estimate}
                        records.append(record)
                        record['frame'] = str(save_frame(
                            frame['raw'], frames_dir, next_stem('arm'),
                            {'stage': 'point-check', 'record': record['index']}))
                        # Write after every point, so a later crash loses nothing.
                        directory.mkdir(parents=True, exist_ok=True)
                        (directory / 'point_check.json').write_text(json.dumps(
                            {'records': records, 'partial': True}, indent=2) + '\n')
                        message = (f'#{record["index"]}: camera - arm = '
                                   f'{np.round(difference * 1000, 1)} mm '
                                   f'(horizontal {record["horizontal_mm"]:.1f} mm, '
                                   f'vertical {difference[2] * 1000:+.1f} mm)')
                        print(message)
                        # The estimate belongs to this placement only; reusing it
                        # for the next one would pair a stale camera reading with
                        # a new arm pose.
                        estimate = None
                        message = ('NEXT: support the arm, r, move it away, move the '
                                   'cube, then c')
                    print(message)
        finally:
            cv2.destroyAllWindows()
            if held:
                print('the arm is still LOCKED. Support it, then run: '
                      'bash LLM-control/run_rgbcal.sh arm-release')
            directory.mkdir(parents=True, exist_ok=True)
            summary = {'records': records, 'motion_status': status,
                       'support_height_m': support_height,
                       'calibration': calibration.sources,
                       'finished_at': timestamp()}
            if records:
                distances = [r['distance_mm'] for r in records]
                horizontal = [r['horizontal_mm'] for r in records]
                summary['summary'] = {
                    'points': len(records),
                    'distance_mm': {'median': float(np.median(distances)),
                                    'max': float(np.max(distances))},
                    'horizontal_mm': {'median': float(np.median(horizontal)),
                                      'max': float(np.max(horizontal))},
                    'mean_offset_mm': np.mean([r['camera_minus_arm_mm'] for r in records],
                                              axis=0).tolist()}
            (directory / 'point_check.json').write_text(
                json.dumps(summary, indent=2, ensure_ascii=False) + '\n')
            print(f'{len(records)} points -> {directory / "point_check.json"}')
    return records


def report(directory):
    """The grasp centre in the gripper frame, and the error left after using it.

    The model's TCP is a point on the gripper, not where a clamped cube sits.
    Each record says where the camera saw the cube *in the gripper frame*;
    their mean is the measured grasp centre, and their spread is how
    repeatable a grasp is.  The end-to-end error is then judged leave-one-out:
    each record is predicted from the grasp centre estimated WITHOUT it, so
    no record grades itself.
    """
    data = json.loads((Path(directory) / 'point_check.json').read_text())
    records = [r for r in data['records'] if r.get('gripper_locked')]
    if len(records) < 3:
        raise RuntimeError(f'{len(records)} clamped grasps recorded; need at least 3')
    local = np.array([r['grasp_in_gripper_frame_m'] for r in records])
    rotations, positions = [], []
    kinematics_poses = []
    for record in records:
        ticks = {int(k): v for k, v in record['servo_ticks'].items()}
        kinematics_poses.append(ticks)
    calibration = realcam.load()
    tcp = solved_arm(calibration.extrinsics)
    poses = [tcp(ticks)[0] for ticks in kinematics_poses]
    errors = []
    for index, record in enumerate(records):
        others = np.delete(local, index, axis=0).mean(axis=0)
        predicted = poses[index][:3, :3] @ others + poses[index][:3, 3]
        camera = np.array(record['camera_grasp_centre_base_m'])
        errors.append((camera - predicted) * 1000)
    errors = np.array(errors)
    distance = np.linalg.norm(errors, axis=1)
    horizontal = np.linalg.norm(errors[:, :2], axis=1)
    return {
        'grasps': len(records),
        'grasp_centre_in_gripper_frame_mm': (local.mean(axis=0) * 1000).tolist(),
        'grasp_centre_spread_mm': (local.std(axis=0) * 1000).tolist(),
        'leave_one_out_error_mm': {
            'per_grasp': errors.round(1).tolist(),
            'distance_median': float(np.median(distance)),
            'distance_max': float(distance.max()),
            'horizontal_median': float(np.median(horizontal)),
            'horizontal_max': float(horizontal.max())},
        'meaning': ('the camera estimate of the cube against where the arm would put '
                    'the measured grasp centre, each grasp predicted from the others'),
    }
