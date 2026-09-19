"""Command line for the real environment-camera calibration.

Run through ``LLM-control/run_rgbcal.sh <command>``.  Stage order matters
and the commands follow it: identify the device, check the viewpoint, then
collect board samples, then solve, then register to the robot.
"""
from pathlib import Path
import argparse
import os
import json
import sys

from . import boards as boards_module
from . import devices as devices_module
from .capture import CameraSettings

ROOT = Path(__file__).resolve().parent.parent
#: Calibration sessions live outside the source tree and are never rewritten.
SESSIONS = ROOT / 'calib'
#: Where lerobot keeps this arm's servo calibration, if it was made with lerobot.
LEROBOT_CALIBRATION = str(Path.home() / '.cache/huggingface/lerobot/calibration/'
                                          'robots/so101_follower/follower.json')


def _path(value):
    """Resolve a file argument against the directory the user typed it in.

    The launcher changes directory so the package imports cleanly, which
    would otherwise silently break every relative path on the command line.
    """
    if value is None:
        return None
    path = Path(value)
    if path.is_absolute():
        return str(path)
    return str((Path(os.environ.get('RGBCAL_CWD', Path.cwd())) / path).resolve())


#: Used only when neither a profile nor the command line says otherwise.
CAMERA_DEFAULTS = {'device': '/dev/video0', 'width': 1280, 'height': 720, 'fps': 30.0,
                   'fourcc': 'MJPG', 'autofocus': None, 'focus': None,
                   'auto_exposure': None, 'exposure': None, 'auto_wb': None,
                   'hflip': False}
PROFILES = SESSIONS / 'profiles'


def _profile(name):
    """A saved imaging configuration, so no capture can forget a setting.

    Forgetting ``--autofocus 0`` once is enough to collect samples at a
    different focal length than the intrinsics describe, and nothing would
    say so.  A profile makes the configuration one name instead of eight
    flags.
    """
    path = Path(name) if name.endswith('.json') else PROFILES / f'{name}.json'
    if not path.exists():
        raise FileNotFoundError(f'no camera profile {path}')
    data = json.loads(path.read_text())
    return {key: value for key, value in data.items() if key in CAMERA_DEFAULTS}


def _settings(args):
    """Defaults, then the profile, then whatever was typed explicitly."""
    values = dict(CAMERA_DEFAULTS)
    if getattr(args, 'profile', None):
        values.update(_profile(args.profile))
    for key in CAMERA_DEFAULTS:
        typed = getattr(args, key, None)
        if typed is not None and typed is not False:
            values[key] = typed
    for key in ('autofocus', 'auto_exposure', 'auto_wb'):
        if isinstance(values[key], int) and not isinstance(values[key], bool):
            values[key] = bool(values[key])
    return CameraSettings(**values)


def _board(args):
    if getattr(args, 'board', None) in (None, 'none'):
        return None
    return boards_module.build(args.board, square=args.square, marker=args.marker)


def _session_dir(args, default_prefix):
    name = args.session or default_prefix
    return SESSIONS / name


def command_devices(args):
    nodes = devices_module.survey(probe=not args.no_probe)
    print(f'{"device":<14}{"name":<28}{"usb":<12}{"hint":<16}status')
    for entry in nodes:
        capture = entry.get('capture', {})
        if entry['busy']:
            status = 'BUSY: ' + ', '.join(f'{h["command"]}({h["pid"]})' for h in entry['busy'])
        elif capture.get('reads'):
            status = f'reads frames {capture["shape"][1]}x{capture["shape"][0]}'
        elif capture.get('opens'):
            status = 'opens but returns no image (metadata node)'
        else:
            status = 'cannot open'
        print(f'{entry["device"]:<14}{entry["name"][:27]:<28}{entry["usb_id"]:<12}'
              f'{entry["role_hint"]:<16}{status}')
    print('\nrole_hint comes from the reported device name only. Confirm which node is '
          'the laptop built-in camera before calibrating; the wrist USB camera is a '
          'different device and must not be used for the environment calibration.')
    if args.json:
        print(json.dumps(nodes, indent=2, ensure_ascii=False))
    return 0


def command_modes(args):
    """Which resolutions the driver actually grants, not which it advertises."""
    import cv2

    candidates = [(640, 480), (800, 600), (848, 480), (960, 540), (1024, 576),
                  (1280, 720), (1280, 960), (1600, 896), (1920, 1080)]
    print(f'probing {args.device} with FOURCC {args.fourcc}')
    print(f'{"requested":<14}{"granted":<14}{"fps":<8}{"format"}')
    capture = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not capture.isOpened():
        print('cannot open device', file=sys.stderr)
        return 1
    try:
        for width, height in candidates:
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*args.fourcc))
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            ok, frame = capture.read()
            if not ok or frame is None:
                print(f'{width}x{height:<9}{"no frame":<14}')
                continue
            from .capture import fourcc_text
            print(f'{f"{width}x{height}":<14}{f"{frame.shape[1]}x{frame.shape[0]}":<14}'
                  f'{capture.get(cv2.CAP_PROP_FPS):<8.1f}'
                  f'{fourcc_text(capture.get(cv2.CAP_PROP_FOURCC))}')
    finally:
        capture.release()
    return 0


def command_snap(args):
    from .preview import snap

    directory = _session_dir(args, 'viewpoint')
    configuration, results = snap(_settings(args), directory, count=args.count,
                                  label=args.label, board=_board(args))
    print(json.dumps({'directory': str(directory), 'camera': configuration,
                      'frames': results}, indent=2))
    return 0


def command_preview(args):
    from .preview import run

    directory = _session_dir(args, 'viewpoint')
    run(_settings(args), directory, board=_board(args), label=args.label,
        margin=args.margin, guides=not args.no_guides,
        labels=[name.strip() for name in args.labels.split(',')] if args.labels else None)
    print(f'session: {directory}')
    return 0


def command_mirror_check(args):
    """Is the delivered image mirrored? Needs the ChArUco board in view."""
    from .capture import RealCamera, save_frame
    from . import boards as boards_module
    import cv2

    directory = _session_dir(args, 'viewpoint')
    if _path(args.image):
        # Re-check a frame that is already on disk instead of taking the camera.
        frame = cv2.imread(_path(args.image))
        if frame is None:
            print(f'cannot read {args.image}', file=sys.stderr)
            return 1
        result = boards_module.mirror_probe(frame, args.dictionary)
        result['image'] = _path(args.image)
        print(json.dumps(result, indent=2))
        return 0
    with RealCamera(_settings(args)) as camera:
        camera.warm_up(15)
        frame = camera.frame()
        configuration = camera.configuration()
    result = boards_module.mirror_probe(frame, args.dictionary)
    path = save_frame(frame, directory / 'frames', args.label,
                      {'stage': 'mirror-check', **result}, configuration)
    result['image'] = str(path)
    print(json.dumps(result, indent=2))
    return 0


def command_inspect(args):
    """Measure saved frames: board scale, foreshortening, margins, exposure."""
    from .inspect import analyse_session

    directory = _session_dir(args, 'viewpoint')
    board = _board(args)
    if board is None:
        print('inspect needs --board', file=sys.stderr)
        return 1
    report = analyse_session(directory, board, hflip=args.hflip,
                             colours=tuple(args.colour or ()))
    print(json.dumps(report, indent=2))
    return 0


def command_lock(args):
    """Stage 1 exit: record the accepted viewpoint and its device identity."""
    from .setup import lock

    directory = _session_dir(args, 'setup')
    record = lock(_settings(args), directory, note=args.note)
    print(json.dumps({'setup': str(directory / 'setup.json'),
                      'reference_image': record['reference_image'],
                      'device': record['device']['name'],
                      'resolution': record['camera']['resolution'],
                      'pixel_format': record['camera']['pixel_format']}, indent=2))
    return 0


def command_check_setup(args):
    """Has the camera moved since the viewpoint was locked?"""
    from .setup import check

    result = check(_settings(args), _session_dir(args, 'setup'))
    print(json.dumps(result, indent=2))
    return 0


def command_arm(args):
    """Read the servo bus: positions, limits, and whether torque is on."""
    from . import robot as robot_module

    with robot_module.ServoBus(args.port) as bus:
        state = bus.read_state()
    models = (robot_module.joint_models_from_lerobot(_path(args.lerobot_calibration))
              if args.lerobot_calibration else robot_module.default_joint_models())
    angles = robot_module.angles_from_ticks(
        {int(k): v['present_position'] for k, v in state.items()}, models)
    kinematics = robot_module.Kinematics()
    print(json.dumps({
        'port': args.port,
        'servos': {str(k): v for k, v in state.items()},
        'joint_models': robot_module.models_to_dict(models),
        'angles_rad_UNVERIFIED': angles,
        'T_base_gripper_UNVERIFIED': kinematics.T_base_gripper(angles).tolist(),
        'warning': ('angles and kinematics here use an unverified joint convention; '
                    'hand-eye calibration is what establishes it'),
    }, indent=2))
    return 0


def command_arm_release(args):
    """Make the arm limp.  Only after the operator is supporting it."""
    from . import robot as robot_module

    with robot_module.ServoBus(args.port) as bus:
        before = bus.torque_enabled()
        if not any(before.values()):
            print('torque is already off on every arm joint')
            return 0
        if not args.yes:
            answer = input('The arm will go limp. Are you supporting it? [y/N] ')
            if answer.strip().lower() not in ('y', 'yes'):
                print('not released')
                return 1
        bus.release()
        print('released: torque off on', [k for k, v in before.items() if v])
    return 0


def command_handeye_collect(args):
    from .handeye import collect

    directory = _session_dir(args, 'handeye')
    board = _board(args)
    if board is None or board.kind != 'charuco':
        print('hand-eye needs a ChArUco board (--board charuco-large)', file=sys.stderr)
        return 1
    collect(_settings(args), directory, board, _path(args.intrinsics), port=args.port,
            calibration_file=_path(args.lerobot_calibration), hold=args.hold)
    return 0


def command_handeye_solve(args):
    from .handeye import solve

    report = solve(_session_dir(args, 'handeye'), intrinsics=_path(args.intrinsics),
                   experimental=args.experimental, min_poses=args.min_poses,
                   loss=args.loss, f_scale=args.loss_scale, bootstrap=args.bootstrap,
                   offset_bound_deg=args.max_offset_deg)
    print(json.dumps({'status': report['status'], 'written': report['written'],
                      'T_base_camera': report['T_base_camera'],
                      'camera_position_base_m': report['camera_position_base_m'],
                      'joint_convention': report['joint_convention'],
                      'fit': {k: v for k, v in report['fit'].items()
                              if k != 'per_pose_rms_px'},
                      'uncertainty': report['uncertainty'],
                      'plausibility': report['plausibility'],
                      'sign_search': report['sign_search'],
                      'motion_consistency': report['motion_consistency']}, indent=2))
    return 0


def command_handeye_verify(args):
    from .handeye import verify

    print(json.dumps(verify(_session_dir(args, 'handeye'),
                            intrinsics=_path(args.intrinsics),
                            experimental=args.experimental), indent=2))
    return 0


def command_handeye_joints(args):
    """Per-joint check from single-joint moves, independent of any convention."""
    from .handeye import joint_report

    report = joint_report(_session_dir(args, 'handeye'), intrinsics=_path(args.intrinsics))
    for joint, entry in report.items():
        if not entry['pairs']:
            print(f'{joint:15s} no single-joint moves')
            continue
        print(f"{joint:15s} pairs {entry['pairs']:2d}  board/servo {entry['median_ratio']:.3f}  "
              f"|error| median {entry['median_abs_error_deg']:.2f} max {entry['max_abs_error_deg']:.2f} deg  "
              f"pitch {entry['median_abs_pitch_mm']:.1f} mm")
        if args.detail:
            for pair in entry['pairs_detail']:
                print(f"    {pair['from']:3d}->{pair['to']:3d}  servo {pair['servo_deg']:6.2f}  "
                      f"board {pair['board_deg']:6.2f}  error {pair['error_deg']:+6.2f} deg  "
                      f"pitch {pair['pitch_mm']:+6.1f} mm")
    return 0


def command_point_check(args):
    """Camera estimate against the arm holding the object; no motion commanded."""
    from .pointcheck import run

    run(_settings(args), _session_dir(args, 'point-check'), port=args.port,
        support_height=args.support_height)
    return 0


def command_point_report(args):
    from .pointcheck import report

    print(json.dumps(report(_session_dir(args, 'point-check')), indent=2))
    return 0


def command_sim_camera(args):
    from .simcam import export

    output, record = export(_path(args.output), _path(args.intrinsics),
                            _path(args.extrinsics), _path(args.table))
    summary = {key: record[key] for key in ('position', 'look_down_deg', 'fov_deg',
                                            'fx', 'fy', 'cx', 'cy', 'image')}
    print(json.dumps({'written': str(output), **summary}, indent=2))
    return 0


def command_table(args):
    from .table import measure

    board = _board(args)
    report = measure(_settings(args), _session_dir(args, 'table'), board,
                     _path(args.intrinsics), _path(args.extrinsics),
                     board_thickness_m=args.thickness)
    print(json.dumps(report, indent=2))
    return 0


def command_focus(args):
    """Pick a fixed focus by measuring sharpness on the workspace."""
    from .focus import sweep

    values = [float(v) for v in args.values.split(',')] if args.values else \
        list(range(int(args.start), int(args.stop) + 1, int(args.step)))
    report = sweep(_settings(args), _session_dir(args, 'focus'), values,
                   board=_board(args))
    print(json.dumps({k: report[k] for k in ('autofocus_after_disable',
                                              'manual_focus_took_effect',
                                              'best_focus', 'best_sharpness', 'note')},
                     indent=2))
    return 0


def command_collect(args):
    from .collect import run

    directory = _session_dir(args, 'intrinsics')
    board = _board(args)
    if board is None:
        print('collect needs --board', file=sys.stderr)
        return 1
    run(_settings(args), directory, board, target=args.target, split=args.split,
        min_sharpness=args.min_sharpness)
    print(f'session: {directory}')
    return 0


def command_solve(args):
    from .solve import run

    directory = _session_dir(args, 'intrinsics')
    result = run(directory, model=args.model, fix_aspect=args.fix_aspect,
                 output=_path(args.output), max_planarity=args.max_planarity)
    print(json.dumps(result['summary'], indent=2))
    return 0


def command_verify(args):
    from .verify import run

    directory = _session_dir(args, 'intrinsics')
    result = run(directory, intrinsics=_path(args.intrinsics))
    print(json.dumps(result, indent=2))
    return 0


def build_parser():
    parser = argparse.ArgumentParser(
        prog='rgbcal', description='Real SO101 environment RGB camera calibration')
    subparsers = parser.add_subparsers(dest='command', required=True)

    def add_camera(sub, device_required=True):
        sub.add_argument('--profile', default=None,
                         help='saved camera profile under calib/profiles (e.g. c920)')
        sub.add_argument('--device', default=None,
                         help='V4L2 node; prefer a /dev/v4l/by-id path or a profile')
        sub.add_argument('--width', type=int, default=None)
        sub.add_argument('--height', type=int, default=None)
        sub.add_argument('--fps', type=float, default=None)
        sub.add_argument('--fourcc', default=None)
        sub.add_argument('--autofocus', type=int, default=None,
                         help='1 auto, 0 manual; omit to leave the driver alone')
        sub.add_argument('--focus', type=float, default=None)
        sub.add_argument('--auto-exposure', dest='auto_exposure', type=int, default=None,
                         help='1 auto, 0 manual; omit to leave the driver alone')
        sub.add_argument('--exposure', type=float, default=None)
        sub.add_argument('--auto-wb', dest='auto_wb', type=int, default=None)
        sub.add_argument('--hflip', action='store_true',
                         help='the camera delivers a mirrored stream; undo it '
                              '(declared in every sidecar and calibration file)')
        sub.add_argument('--session', default=None, help='session directory name under calib/')

    def add_board(sub, default=None):
        sub.add_argument('--board', default=default,
                         choices=[*boards_module.PRESETS, 'none'],
                         help='printed board preset')
        sub.add_argument('--square', type=float, default=None,
                         help='MEASURED square length in metres (overrides the preset)')
        sub.add_argument('--marker', type=float, default=None,
                         help='MEASURED ChArUco marker length in metres')

    sub = subparsers.add_parser('devices', help='list cameras and who holds them')
    sub.add_argument('--json', action='store_true')
    sub.add_argument('--no-probe', action='store_true')
    sub.set_defaults(func=command_devices)

    sub = subparsers.add_parser('modes', help='which resolutions the driver grants')
    sub.add_argument('--device', default='/dev/video0')
    sub.add_argument('--fourcc', default='MJPG')
    sub.set_defaults(func=command_modes)

    sub = subparsers.add_parser('snap', help='headless capture of raw frames')
    add_camera(sub)
    add_board(sub)
    sub.add_argument('--count', type=int, default=1)
    sub.add_argument('--label', default='snap')
    sub.set_defaults(func=command_snap)

    sub = subparsers.add_parser('preview', help='stage 1: live viewpoint check')
    add_camera(sub)
    add_board(sub)
    sub.add_argument('--label', default='view')
    sub.add_argument('--labels', default=None,
                     help='comma-separated names for the shots to take, in order; '
                          'each save uses the next name and the window shows it')
    sub.add_argument('--margin', type=float, default=0.10)
    sub.add_argument('--no-guides', action='store_true')
    sub.set_defaults(func=command_preview)

    sub = subparsers.add_parser('lock', help='stage 1 exit: record the accepted viewpoint')
    add_camera(sub)
    sub.add_argument('--note', default='')
    sub.set_defaults(func=command_lock)

    sub = subparsers.add_parser('check-setup',
                                help='has the camera moved since it was locked?')
    add_camera(sub)
    sub.set_defaults(func=command_check_setup)

    sub = subparsers.add_parser('arm', help='stage 3: read the servo bus (read-only)')
    sub.add_argument('--port', default='/dev/ttyACM0')
    sub.add_argument('--lerobot-calibration', default=LEROBOT_CALIBRATION)
    sub.set_defaults(func=command_arm)

    sub = subparsers.add_parser('arm-release',
                                help='turn arm torque off (support the arm first)')
    sub.add_argument('--port', default='/dev/ttyACM0')
    sub.add_argument('--yes', action='store_true')
    sub.set_defaults(func=command_arm_release)

    sub = subparsers.add_parser('handeye-collect',
                                help='stage 3: capture poses for eye-to-hand')
    add_camera(sub)
    add_board(sub, default='charuco-large')
    sub.add_argument('--intrinsics', required=True)
    sub.add_argument('--port', default='/dev/ttyACM0')
    sub.add_argument('--lerobot-calibration', default=LEROBOT_CALIBRATION)
    sub.add_argument('--hold', action='store_true',
                     help='lock the arm with torque at its present position for each '
                          'capture, so no hand touches it while the image is taken')
    sub.set_defaults(func=command_handeye_collect)

    sub = subparsers.add_parser('handeye-solve', help='stage 3: solve T_base_camera')
    sub.add_argument('--session', default=None)
    sub.add_argument('--intrinsics', default=None)
    sub.add_argument('--experimental', action='store_true',
                     help='solve data known to be flawed; written to its own file '
                          'and marked EXPERIMENTAL-UNVERIFIED')
    sub.add_argument('--min-poses', type=int, default=8)
    sub.add_argument('--loss', default='huber', choices=['linear', 'huber', 'soft_l1'])
    sub.add_argument('--loss-scale', type=float, default=15.0,
                     help='robust loss scale in pixels')
    sub.add_argument('--bootstrap', type=int, default=0,
                     help='resampled refits for an error bar on T_base_camera')
    sub.add_argument('--max-offset-deg', type=float, default=30.0,
                     help='bound on how far the model zero may sit from the servo '
                          'zero; excludes the elbow-flipped mirror solution')
    sub.set_defaults(func=command_handeye_solve)

    sub = subparsers.add_parser('handeye-verify',
                                help='stage 3: check on held-out poses')
    sub.add_argument('--session', default=None)
    sub.add_argument('--intrinsics', default=None)
    sub.add_argument('--experimental', action='store_true')
    sub.set_defaults(func=command_handeye_verify)

    sub = subparsers.add_parser('handeye-joints',
                                help='stage 3: per-joint check from single-joint moves')
    sub.add_argument('--session', default=None)
    sub.add_argument('--intrinsics', default=None)
    sub.add_argument('--detail', action='store_true')
    sub.set_defaults(func=command_handeye_joints)

    sub = subparsers.add_parser('point-check',
                                help='stage 6: camera estimate vs arm holding the cube')
    add_camera(sub)
    sub.add_argument('--port', default='/dev/ttyACM0')
    sub.add_argument('--support-height', type=float, default=0.0,
                     help='height of what the cube rests on, above the table (m); '
                          '0.025 for a cube stacked on another 25 mm cube')
    sub.set_defaults(func=command_point_check)

    sub = subparsers.add_parser('point-report',
                                help='stage 6: measured grasp centre and held-out error')
    sub.add_argument('--session', default=None)
    sub.set_defaults(func=command_point_report)

    sub = subparsers.add_parser('sim-camera',
                                help='export the calibrated camera for the MuJoCo simulation')
    sub.add_argument('--intrinsics', default=None)
    sub.add_argument('--extrinsics', default=None)
    sub.add_argument('--table', default=None)
    sub.add_argument('--output', default=None,
                     help='default: calib/c920-sim-camera/sim_camera.json')
    sub.set_defaults(func=command_sim_camera)

    sub = subparsers.add_parser('table-plane',
                                help='stage 3: measure the support surface height')
    add_camera(sub)
    add_board(sub, default='charuco-large')
    sub.add_argument('--intrinsics', required=True)
    sub.add_argument('--extrinsics', required=True)
    sub.add_argument('--thickness', type=float, default=0.0,
                     help='thickness in metres of whatever the pattern is mounted on')
    sub.set_defaults(func=command_table)

    sub = subparsers.add_parser('focus-sweep',
                                help='measure sharpness across manual focus values')
    add_camera(sub)
    add_board(sub)
    sub.add_argument('--start', type=float, default=0)
    sub.add_argument('--stop', type=float, default=80)
    sub.add_argument('--step', type=float, default=5)
    sub.add_argument('--values', default=None, help='explicit comma-separated list')
    sub.set_defaults(func=command_focus)

    sub = subparsers.add_parser('inspect', help='stage 1: measure saved frames')
    add_board(sub, default='chessboard')
    sub.add_argument('--session', default=None)
    sub.add_argument('--hflip', action='store_true')
    sub.add_argument('--colour', action='append',
                     help='also measure coloured objects, e.g. --colour red')
    sub.set_defaults(func=command_inspect)

    sub = subparsers.add_parser('mirror-check',
                                help='stage 1: detect a horizontally mirrored stream')
    add_camera(sub)
    sub.add_argument('--dictionary', default='DICT_4X4_50')
    sub.add_argument('--label', default='mirror-check')
    sub.add_argument('--image', default=None,
                     help='check a saved frame instead of opening the camera')
    sub.set_defaults(func=command_mirror_check)

    sub = subparsers.add_parser('collect', help='stage 2: collect board samples')
    add_camera(sub)
    add_board(sub, default='chessboard')
    sub.add_argument('--target', type=int, default=30, help='calibration samples wanted')
    sub.add_argument('--split', type=int, default=8, help='held-out validation samples')
    sub.add_argument('--min-sharpness', type=float, default=100.0,
                     help='Laplacian variance over the board region')
    sub.set_defaults(func=command_collect)

    sub = subparsers.add_parser('solve', help='stage 2: fit K and D')
    sub.add_argument('--session', default=None)
    sub.add_argument('--model', default='standard',
                     choices=['standard', 'rational', 'thin-prism'])
    sub.add_argument('--fix-aspect', action='store_true')
    sub.add_argument('--max-planarity', type=float, default=None,
                     help='exclude CALIBRATION frames whose board is bent by more '
                          'than this (px); validation frames are never filtered')
    sub.add_argument('--output', default=None)
    sub.set_defaults(func=command_solve)

    sub = subparsers.add_parser('verify', help='stage 2: check held-out samples')
    sub.add_argument('--session', default=None)
    sub.add_argument('--intrinsics', default=None)
    sub.set_defaults(func=command_verify)
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except (RuntimeError, FileNotFoundError, FileExistsError, ValueError) as error:
        # These are the expected operator-facing failures -- a device that will
        # not open, a configuration that does not match, a sample that would be
        # overwritten. A traceback buries the one line that says what to do.
        print(f'error: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
