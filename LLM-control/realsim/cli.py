"""Command line for the real-to-simulation block mapping.

Run through ``LLM-control/run_realsim.sh <command>``, which picks the virtual
environment each command needs: the real side wants OpenCV, the simulation
side wants MuJoCo and ``so101_nexus``, and this project keeps them apart.

The commands follow the order the work is done in::

    check       what calibration and which blocks this setup has
    drift       how far the camera has moved since the setup was locked
    scale-check two blocks pushed flush together, as an independent scale test
    colours     check each block's colour band against a real image
    measure     fit a block's size from an image, to cross-check the catalogue
    observe     one frame -> one scene state (live camera or a saved frame)
    live        a stream of scene states, appended to a JSONL log
    mirror      drive a MuJoCo scene from a scene state, once or continuously
    snapshot    freeze the current estimate into a steppable simulation
    export      write the captured scene as a standalone MJCF file
    replay      re-run a recorded log into the mirror, offline
    selftest    render known poses and measure what the estimator recovers
"""
from pathlib import Path
import argparse
import json
import os
import sys
import time

# Only cv2-free, MuJoCo-free names at module level: this parser is built by
# both virtual environments, and each is missing what the other side needs.
from realsim import DISPLACEMENT_LIMIT_PX

ROOT = Path(__file__).resolve().parent.parent
#: Sessions live beside the calibration sessions they depend on.
SESSIONS = ROOT / 'calib'


def _path(value):
    """Resolve a path against the directory the user typed the command in."""
    if value is None:
        return None
    path = Path(value)
    if path.is_absolute():
        return str(path)
    return str((Path(os.environ.get('REALSIM_CWD', Path.cwd())) / path).resolve())


def _session(args, default):
    return Path(_path(args.session) if args.session and
                ('/' in args.session or args.session.endswith('.json'))
                else SESSIONS / (args.session or default))


def _catalogue(args):
    from realsim import blocks

    return blocks.load(_path(getattr(args, 'blocks', None)))


def _settings(args):
    """Camera settings: the rgbcal profile, so one setup is described once."""
    from rgbcal.cli import _settings as rgbcal_settings

    class Holder:
        pass

    holder = Holder()
    holder.profile = args.profile
    for key in ('device', 'width', 'height', 'fps', 'fourcc', 'autofocus', 'focus',
                'auto_exposure', 'exposure', 'auto_wb', 'hflip'):
        setattr(holder, key, getattr(args, key, None))
    return rgbcal_settings(holder)


def _fit_settings(args):
    from realsim import estimate

    changes = {}
    if getattr(args, 'no_height_check', False):
        changes['check_height'] = False
    if getattr(args, 'min_iou', None) is not None:
        changes['min_iou'] = float(args.min_iou)
    return estimate.FitSettings(**changes) if changes else estimate.FitSettings()


def _support_heights(values):
    """``--stacked red_cube=0.025`` -- a support height the caller states."""
    heights = {}
    for item in values or ():
        if '=' not in item:
            raise ValueError(f'--stacked wants block_id=height_m, got {item!r}')
        name, height = item.split('=', 1)
        heights[name.strip()] = float(height)
    return heights


def command_check(args):
    from rgbcal import realcam
    from realsim import observe

    catalogue = _catalogue(args)
    settings = _settings(args)
    calibration = realcam.load()
    new_matrix, _ = calibration.undistortion()
    report = observe.calibration_report(calibration, settings, new_matrix)
    unmeasured = [block.id for block in catalogue.blocks if not block.size_measured]
    print(json.dumps({
        'blocks': [block.describe() for block in catalogue.blocks],
        'blocks_whose_size_was_never_measured': unmeasured,
        'calibration': {key: report[key] for key in
                        ('sources', 'versions', 'intrinsics_resolution',
                         'usable_for_motion', 'usable_for_motion_reasons')},
        'camera_in_table_frame_m': calibration.T_table_camera[:3, 3].tolist(),
        'table_fit_residual_rms_mm': report['table_fit_residual_rms_mm'],
    }, indent=2, ensure_ascii=False))
    if unmeasured:
        print('\nnote: the sizes of ' + ', '.join(unmeasured) + ' are nominal. Every '
              'position this pipeline reports scales with them; measure them before '
              'quoting an accuracy.', file=sys.stderr)
    return 0


def command_observe(args):
    from rgbcal import realcam
    from realsim import observe, scene

    catalogue = _catalogue(args)
    settings = _settings(args)
    directory = _session(args, 'realsim-observe')
    observer = observe.RealObserver(settings, catalogue, directory=directory,
                                    fit_settings=_fit_settings(args))
    robot = observe.read_arm(args.port) if args.with_arm else None
    if args.image:
        raw, captured_at = observe.read_raw(_path(args.image))
        source = {'raw_frame': _path(args.image), 'live': False,
                  'captured_at_from': 'the archived frame\'s own sidecar'
                                      if captured_at else 'unknown; no sidecar'}
    else:
        with realcam.EnvironmentCamera(settings, observer.calibration) as camera:
            raw = camera.camera.canonical_frame()
        captured_at, source = None, {'raw_frame': 'live capture', 'live': True}
    record = observer.process(raw, captured_at=captured_at,
                              support_heights=_support_heights(args.stacked),
                              robot=robot)
    record['source'] = source
    output = Path(_path(args.output)) if args.output else directory / 'scene.json'
    scene.write(record, output)
    scene.append(record, directory / 'scene.jsonl')
    print(json.dumps(_summary(record, output), indent=2, ensure_ascii=False))
    return 0


def _summary(record, output=None):
    summary = {'written': str(output) if output else None,
               'captured_at': record['captured_at'],
               'frame_index': record['frame_index'],
               'reference_frame': record['reference_frame'],
               'objects': []}
    for entry in record['objects']:
        row = {'id': entry['id'], 'state': entry['state']}
        if entry.get('pose'):
            position = entry['pose']['position_m']
            row.update({
                'position_mm': [round(1000 * value, 1) for value in position],
                'yaw_deg': round(entry['pose']['yaw_deg'], 1),
                'symmetry_deg': entry['pose']['symmetry_step_deg'],
                'iou': round(entry['quality'].get('iou', 0), 3),
                'age_s': entry.get('age_s')})
            if entry.get('pose_base'):
                row['position_base_mm'] = [round(1000 * value, 1)
                                           for value in entry['pose_base']['position_m']]
        else:
            row['reason'] = entry.get('reason', 'not detected in this frame')
        summary['objects'].append(row)
    summary['rejected'] = [{'reasons': entry['reasons'], 'bbox': entry['bbox']}
                           for entry in record.get('rejected', [])]
    summary['warnings'] = record.get('warnings')
    summary['images'] = record.get('images')
    summary['timing'] = record.get('timing')
    return summary


def command_drift(args):
    from rgbcal import realcam
    from realsim import drift, observe

    settings = _settings(args)
    calibration = realcam.load()
    reference = Path(_path(args.reference) if args.reference
                     else SESSIONS / args.setup / 'reference.png')
    if args.image:
        current, _ = observe.read_raw(_path(args.image))
    else:
        with realcam.EnvironmentCamera(settings, calibration) as camera:
            current = camera.camera.canonical_frame()
    result = drift.report(reference, current, calibration, args.limit)
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 2 if result['camera_moved'] else 0


def command_scale_check(args):
    from rgbcal import realcam
    from realsim import observe, scene

    catalogue = _catalogue(args)
    settings = _settings(args)
    directory = _session(args, 'realsim-scale-check')
    observer = observe.RealObserver(settings, catalogue, directory=directory,
                                    fit_settings=_fit_settings(args))
    records = []
    if args.image:
        raw, captured_at = observe.read_raw(_path(args.image))
        records.append(observer.process(raw, captured_at=captured_at, label='scale'))
    else:
        with realcam.EnvironmentCamera(settings, observer.calibration) as camera:
            for index in range(args.frames):
                records.append(observer.process(
                    camera.camera.canonical_frame(), label='scale',
                    save_images=(index == 0)))
    for record in records:
        scene.append(record, directory / 'scene.jsonl')
    report = scene.contact_check(records, args.pair[0], args.pair[1], catalogue,
                                 args.tolerance)
    report.pop('rows', None)
    scene.write({'schema': scene.SCHEMA, 'contact_check': report,
                 'captured_at': records[-1]['captured_at'],
                 'calibration': records[-1]['calibration'],
                 'frames': FRAMES_NOTE, 'objects': records[-1]['objects'],
                 'reference_frame': 'table', 'catalogue': catalogue.describe()},
                directory / 'scale_check.json')
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report.get('passed') else 2


FRAMES_NOTE = {'reference': 'table'}


def command_colours(args):
    from rgbcal import realcam
    from realsim import estimate, observe

    catalogue = _catalogue(args)
    settings = _settings(args)
    observer = observe.RealObserver(settings, catalogue,
                                    fit_settings=_fit_settings(args))
    if args.image:
        raw, _ = observe.read_raw(_path(args.image))
    else:
        with realcam.EnvironmentCamera(settings, observer.calibration) as camera:
            raw = camera.camera.canonical_frame()
    import cv2

    rgb = cv2.cvtColor(observer.undistort(raw), cv2.COLOR_BGR2RGB)
    print(json.dumps([estimate.colour_report(rgb, block)
                      for block in catalogue.blocks], indent=2, ensure_ascii=False))
    return 0


def command_measure(args):
    from rgbcal import realcam
    from realsim import estimate, observe

    catalogue = _catalogue(args)
    settings = _settings(args)
    observer = observe.RealObserver(settings, catalogue,
                                    fit_settings=_fit_settings(args))
    if args.image:
        raw, _ = observe.read_raw(_path(args.image))
    else:
        with realcam.EnvironmentCamera(settings, observer.calibration) as camera:
            raw = camera.camera.canonical_frame()
    import cv2

    rgb = cv2.cvtColor(observer.undistort(raw), cv2.COLOR_BGR2RGB)
    heights = _support_heights(args.stacked)
    rows = []
    for block in catalogue.blocks:
        for result in estimate.measure_size(
                rgb, observer.nexus_calibration, block,
                plane_z=heights.get(block.id, 0.0),
                settings=_fit_settings(args))[:args.regions]:
            rows.append({
                'block_id': block.id,
                'catalogue_edge_mm': round(result['catalogue_size_mm'][0], 2),
                'fitted_edge_mm': round(result['fitted_size_mm'][0], 2),
                'size_bracket_mm': ([round(value, 2)
                                     for value in result['size_bracket_mm']]
                                    if result['size_bracket_mm'] else None),
                'bracket_note': result['bracket_note'],
                'iou_at_catalogue_size': round(result['iou_at_catalogue_size'], 3),
                'iou_at_fitted_size': round(result['iou_at_fitted_size'], 3),
                'position_mm': [round(1000 * value, 1)
                                for value in result['position_m']],
                'bbox': result['bbox']})
    print(json.dumps({'measured': rows,
                      'note': ('a cross-check on the catalogue, in the metric scale '
                               'of the calibration chain. If it disagrees with the '
                               'catalogue, measure the block with a calliper; do not '
                               'copy this number in')},
                     indent=2, ensure_ascii=False))
    return 0


def command_live(args):
    from rgbcal import realcam
    from realsim import observe, scene

    catalogue = _catalogue(args)
    settings = _settings(args)
    directory = _session(args, 'realsim-live')
    observer = observe.RealObserver(settings, catalogue, directory=directory,
                                   fit_settings=_fit_settings(args))
    log = directory / 'scene.jsonl'
    latest = directory / 'scene.json'
    heights = _support_heights(args.stacked)
    deadline = time.monotonic() + args.duration if args.duration else None
    period = 1.0 / args.rate if args.rate else 0.0
    window = 'realsim live' if args.window else None
    if window:
        import cv2

        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window, 1280, 720)
    frames, started = 0, time.monotonic()
    print(f'logging to {log}; ctrl-c or q to stop')
    try:
        with realcam.EnvironmentCamera(settings, observer.calibration) as camera:
            while deadline is None or time.monotonic() < deadline:
                loop_started = time.monotonic()
                raw = camera.camera.canonical_frame()
                captured = time.monotonic()
                record = observer.process(
                    raw, support_heights=heights,
                    save_images=(frames % max(args.image_every, 1) == 0),
                    label='live')
                # End-to-end latency as a consumer of the log experiences it:
                # from the frame leaving the camera to the state being written.
                record['timing']['capture_s'] = round(captured - loop_started, 3)
                record['timing']['frame_to_record_s'] = round(
                    time.monotonic() - captured, 3)
                scene.append(record, log)
                scene.write(record, latest)
                frames += 1
                states = ' '.join(f"{entry['id']}:{entry['state']}"
                                  for entry in record['objects'])
                print(f"\rframe {frames} {states} "
                      f"{record['timing']['estimate_s'] * 1000:.0f} ms", end='',
                      flush=True)
                if window:
                    import cv2

                    from realsim import overlay

                    picture = overlay.annotate(
                        cv2.cvtColor(observer.undistort(raw), cv2.COLOR_BGR2RGB),
                        observer.nexus_calibration, record['objects'])
                    cv2.imshow(window, picture[:, :, ::-1])
                    if (cv2.waitKey(1) & 0xFF) in (ord('q'), 27):
                        break
                remaining = period - (time.monotonic() - loop_started)
                if remaining > 0:
                    time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    finally:
        if window:
            import cv2

            cv2.destroyAllWindows()
    elapsed = time.monotonic() - started
    print(f'\n{frames} frames in {elapsed:.1f} s '
          f'({frames / max(elapsed, 1e-6):.2f} Hz effective); log {log}')
    return 0


def command_arm(args):
    from realsim import observe

    print(json.dumps(observe.read_arm(args.port), indent=2, ensure_ascii=False))
    return 0


def _mirror(args, catalogue, directory):
    from nexus_vision.cameras import CameraSuite
    from realsim import mirror

    cameras = CameraSuite(modality='rgb', placement=args.placement)
    return mirror.MirrorSimulation(catalogue, directory / 'sim', cameras=cameras,
                                   viewer=args.viewer, realtime=False,
                                   camera_viewer=False)


def _compare(simulation, record, directory, index):
    """Save the mirrored view, next to the real overlay when there is one."""
    from realsim import overlay

    rendered, _ = simulation.render(directory / 'sim' / f'mirror-{index:04d}.png')
    real = (record.get('images') or {}).get('overlay')
    if not real or not Path(real).exists():
        return {'sim': str(rendered)}
    try:
        from PIL import Image
        import numpy as np

        pair = overlay.side_by_side(np.asarray(Image.open(real).convert('RGB')),
                                    np.asarray(Image.open(rendered).convert('RGB')))
        path = overlay.save(pair, directory / 'compare' / f'compare-{index:04d}.png')
        return {'sim': str(rendered), 'real': real, 'compare': str(path)}
    except Exception as error:                      # noqa: BLE001 - reporting only
        return {'sim': str(rendered), 'real': real, 'compare_error': str(error)}


def command_mirror(args):
    from realsim import scene

    catalogue = _catalogue(args)
    directory = _session(args, 'realsim-mirror')
    source = Path(_path(args.scene))
    simulation = _mirror(args, catalogue, directory)
    index, seen = 0, 0
    try:
        while True:
            record = scene.read(source)
            stamp = record.get('written_at')
            if stamp != seen:
                seen = stamp
                index += 1
                applied = simulation.apply(record, use_filtered=not args.raw,
                                           allow_stale=args.allow_stale,
                                           with_arm=not args.no_arm)
                images = _compare(simulation, record, directory, index)
                report = {'applied': [entry['id'] for entry in applied['placed']],
                          'parked': applied['skipped'],
                          'arm': bool(applied['arm']),
                          'reprojection_check': simulation.reprojection_check(record),
                          'from_frame': record.get('frame_index'),
                          'captured_at': record.get('captured_at'),
                          'images': images,
                          'sim_state': simulation.state()}
                print(json.dumps(report, indent=2, ensure_ascii=False)
                      if not args.follow else
                      f"frame {record.get('frame_index')}: placed "
                      f"{len(applied['placed'])}, parked {len(applied['skipped'])}"
                      f" -> {images.get('compare', images['sim'])}")
            if not args.follow:
                break
            time.sleep(args.poll)
    except KeyboardInterrupt:
        print('\nstopped')
    finally:
        simulation.close()
    return 0


def command_snapshot(args):
    from realsim import scene

    catalogue = _catalogue(args)
    directory = _session(args, 'realsim-snapshot')
    record = scene.read(Path(_path(args.scene)))
    simulation = _mirror(args, catalogue, directory)
    try:
        applied = simulation.apply(record, use_filtered=not args.raw,
                                   allow_stale=args.allow_stale,
                                   with_arm=not args.no_arm)
        before = simulation.state()
        settled = simulation.settle(args.settle) if args.settle else None
        output = Path(_path(args.output)) if args.output else directory / 'snapshot.json'
        path, snapshot = simulation.snapshot(output)
        rendered, _ = simulation.render(directory / 'snapshot.png')
        print(json.dumps({
            'written': str(path), 'render': str(rendered),
            'from_frame': record.get('frame_index'),
            'captured_at': record.get('captured_at'),
            'placed': [entry['id'] for entry in applied['placed']],
            'parked': applied['skipped'],
            'state_as_mirrored': before,
            'settled': settled,
            'state_after_settling': simulation.state() if settled else None,
            'note': ('the snapshot steps on its own from here; it predicts and does '
                     'not observe, and it is never written back to the real side'),
        }, indent=2, ensure_ascii=False))
    finally:
        simulation.close()
    return 0


def command_export(args):
    from nexus_vision.cameras import CameraSuite
    from realsim import export, scene

    catalogue = _catalogue(args)
    record = scene.read(Path(_path(args.scene)))
    camera = None
    if not args.no_camera:
        camera = CameraSuite(modality='rgb', placement='calibrated',
                             calibrated_scale=args.camera_scale).calibrated_camera()
    output = Path(_path(args.output) if args.output
                  else _session(args, 'realsim-export') / 'scene.xml')
    detail = export.write(record, catalogue, output, camera,
                          use_filtered=not args.raw, allow_stale=args.allow_stale)
    print(json.dumps({
        'written': detail['path'],
        'placed': [entry['id'] for entry in detail['placed']],
        'not_placed': detail['skipped'],
        'camera': ('calibrated, with the measured intrinsics' if detail['camera']
                   else 'none'),
        'open_it_with': [
            f"{sys.executable.split('/')[-1]} -m mujoco.viewer --mjcf={detail['path']}",
            f"simulate {detail['path']}   # if you have MuJoCo's simulate binary",
        ],
        'note': ('the file carries the block poses on the bodies, so it opens in the '
                 'captured state; the vendored robot is embedded with the recorded '
                 'table/base transform; meshes retain absolute paths'),
    }, indent=2, ensure_ascii=False))
    return 0


def command_replay(args):
    from realsim import scene

    catalogue = _catalogue(args)
    directory = _session(args, 'realsim-replay')
    records = scene.read_all(Path(_path(args.log)))
    simulation = _mirror(args, catalogue, directory)
    rows = []
    try:
        end = args.start + args.count if args.count else None
        for index, record in enumerate(records[args.start:end], start=1):
            applied = simulation.apply(record, use_filtered=not args.raw,
                                       allow_stale=args.allow_stale,
                                       with_arm=not args.no_arm)
            images = (_compare(simulation, record, directory, index)
                      if index % max(args.render_every, 1) == 0 else {})
            rows.append({'frame': record.get('frame_index'),
                         'captured_at': record.get('captured_at'),
                         'placed': [entry['id'] for entry in applied['placed']],
                         'parked': [entry['id'] for entry in applied['skipped']],
                         'images': images})
            if args.realtime and index < len(records):
                time.sleep(args.realtime)
    finally:
        simulation.close()
    report = {'log': _path(args.log), 'frames': len(rows), 'directory': str(directory),
              'rows': rows[-args.tail:] if args.tail else rows}
    (directory / 'replay.json').parent.mkdir(parents=True, exist_ok=True)
    (directory / 'replay.json').write_text(
        json.dumps({'log': _path(args.log), 'rows': rows}, indent=2) + '\n')
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


def command_selftest(args):
    from realsim import selftest

    directory = _session(args, 'realsim-selftest')
    report = selftest.run(_catalogue(args), directory, placements=args.grid,
                          fit_settings=_fit_settings(args),
                          placement=args.placement, lifts=not args.no_lift_check)
    print(json.dumps(report['summary'], indent=2, ensure_ascii=False))
    print(f"\nfull report: {directory / 'selftest.json'}")
    return 0 if report['summary']['passed'] else 2


def command_arm_stream(args):
    from rgbcal.robot import ServoBus
    from realsim.arm_feedback import FeedbackPublisher
    import os

    if os.environ.get('SO101_FEEDBACK_PATH'):
        raise ValueError('arm-stream owns publication; unset SO101_FEEDBACK_PATH')
    with ServoBus(args.port) as bus:
        publisher = FeedbackPublisher(bus, _path(args.state), args.rate,
                                      _path(args.log) if args.log else None).start()
        started = time.monotonic()
        try:
            while not args.duration or time.monotonic() - started < args.duration:
                time.sleep(.2)
                if publisher.last:
                    print(json.dumps(publisher.last), flush=True)
        except KeyboardInterrupt:
            pass
        finally:
            publisher.close()
    return 0


def command_arm_sync(args):
    from realsim.arm_sync import run
    for key in ('scene', 'xml', 'state', 'log', 'record', 'gripper_map', 'status_output'):
        value = getattr(args, key, None)
        if value:
            setattr(args, key, _path(value))
    return run(args)


def build_parser():
    parser = argparse.ArgumentParser(
        prog='run_realsim.sh', description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    subparsers = parser.add_subparsers(dest='command', required=True)

    def add_common(sub):
        sub.add_argument('--blocks', default=None,
                         help='block catalogue JSON (default realsim/config/blocks.json)')
        sub.add_argument('--session', default=None,
                         help='session directory name under calib/, or a path')

    def add_camera(sub):
        sub.add_argument('--profile', default='c920',
                         help='rgbcal camera profile; must match the intrinsics')
        for name in ('device', 'fourcc'):
            sub.add_argument(f'--{name}', default=None)
        for name in ('width', 'height'):
            sub.add_argument(f'--{name}', type=int, default=None)
        sub.add_argument('--fps', type=float, default=None)
        sub.add_argument('--autofocus', type=int, default=None)
        sub.add_argument('--focus', type=float, default=None)
        sub.add_argument('--auto-exposure', dest='auto_exposure', type=int, default=None)
        sub.add_argument('--exposure', type=float, default=None)
        sub.add_argument('--auto-wb', dest='auto_wb', type=int, default=None)
        sub.add_argument('--hflip', action='store_true')

    def add_fit(sub):
        sub.add_argument('--stacked', action='append', default=[],
                         metavar='ID=HEIGHT_M',
                         help='support height of a block that rests on another; '
                              'without it every block is assumed to be on the table')
        sub.add_argument('--no-height-check', action='store_true',
                         help='skip the on-plane check (about 4x faster, and it '
                              'stops refusing lifted blocks)')
        sub.add_argument('--min-iou', type=float, default=None)

    def add_sim(sub):
        sub.add_argument('--placement', default='calibrated',
                         choices=['calibrated', 'side', 'overhead'],
                         help='which simulated environment camera to render')
        sub.add_argument('--viewer', action='store_true',
                         help='open the interactive MuJoCo window')
        sub.add_argument('--raw', action='store_true',
                         help='mirror the unfiltered single-frame estimate')
        sub.add_argument('--allow-stale', action='store_true',
                         help='mirror poses that are no longer current; they are '
                              'refused by default')
        sub.add_argument('--no-arm', action='store_true',
                         help='do not copy the recorded joint angles')

    sub = subparsers.add_parser('check', help='calibration and block catalogue status')
    add_common(sub)
    add_camera(sub)
    sub.set_defaults(func=command_check, side='real')

    sub = subparsers.add_parser('observe', help='one frame -> one scene state')
    add_common(sub)
    add_camera(sub)
    add_fit(sub)
    sub.add_argument('--image', default=None,
                     help='a saved RAW frame to map instead of opening the camera')
    sub.add_argument('--output', default=None)
    sub.add_argument('--with-arm', action='store_true',
                     help='also read the servo bus (read-only) into the record')
    sub.add_argument('--port', default='/dev/ttyACM0')
    sub.set_defaults(func=command_observe, side='real')

    sub = subparsers.add_parser(
        'drift', help='how far the camera has moved since the setup was locked')
    add_camera(sub)
    sub.add_argument('--setup', default='setup-03',
                     help='the locked setup session holding reference.png')
    sub.add_argument('--reference', default=None, help='a reference frame directly')
    sub.add_argument('--image', default=None, help='a saved RAW frame to compare')
    sub.add_argument('--limit', type=float, default=DISPLACEMENT_LIMIT_PX,
                     help='pixels of displacement to tolerate')
    sub.set_defaults(func=command_drift, side='real')

    sub = subparsers.add_parser(
        'scale-check',
        help='push two blocks flush together: their centres must be one edge apart')
    add_common(sub)
    add_camera(sub)
    add_fit(sub)
    sub.add_argument('--pair', nargs=2, required=True, metavar='BLOCK_ID',
                     help='the two blocks that are touching')
    sub.add_argument('--frames', type=int, default=10)
    sub.add_argument('--tolerance', type=float, default=1.0,
                     help='millimetres of error to accept')
    sub.add_argument('--image', default=None, help='a saved RAW frame instead')
    sub.set_defaults(func=command_scale_check, side='real')

    sub = subparsers.add_parser(
        'colours', help='check each block\'s colour band against a real image')
    add_common(sub)
    add_camera(sub)
    add_fit(sub)
    sub.add_argument('--image', default=None, help='a saved RAW frame')
    sub.set_defaults(func=command_colours, side='real')

    sub = subparsers.add_parser(
        'measure', help='fit a block\'s own size, to cross-check the catalogue')
    add_common(sub)
    add_camera(sub)
    add_fit(sub)
    sub.add_argument('--image', default=None, help='a saved RAW frame')
    sub.add_argument('--regions', type=int, default=1,
                     help='how many colour regions per block to report')
    sub.set_defaults(func=command_measure, side='real')

    sub = subparsers.add_parser('live', help='continuous scene states into a JSONL log')
    add_common(sub)
    add_camera(sub)
    add_fit(sub)
    sub.add_argument('--duration', type=float, default=0.0, help='seconds; 0 = until q')
    sub.add_argument('--rate', type=float, default=0.0, help='cap in Hz; 0 = as fast as it runs')
    sub.add_argument('--window', action='store_true', help='live overlay window')
    sub.add_argument('--image-every', type=int, default=10,
                     help='archive a raw frame and an overlay every N frames')
    sub.set_defaults(func=command_live, side='real')

    sub = subparsers.add_parser('arm', help='read the servo bus (read-only)')
    sub.add_argument('--port', default='/dev/ttyACM0')
    sub.set_defaults(func=command_arm, side='real')

    sub = subparsers.add_parser('mirror', help='drive a MuJoCo scene from a scene state')
    add_common(sub)
    add_sim(sub)
    sub.add_argument('--scene', required=True, help='scene.json or scene.jsonl')
    sub.add_argument('--follow', action='store_true',
                     help='keep re-reading the file and re-applying new states')
    sub.add_argument('--poll', type=float, default=0.2)
    sub.set_defaults(func=command_mirror, side='sim')

    sub = subparsers.add_parser('snapshot',
                                help='freeze the estimate into a steppable simulation')
    add_common(sub)
    add_sim(sub)
    sub.add_argument('--scene', required=True)
    sub.add_argument('--output', default=None)
    sub.add_argument('--settle', type=float, default=0.0,
                     help='seconds of physics to run after loading (snapshot only)')
    sub.set_defaults(func=command_snapshot, side='sim')

    sub = subparsers.add_parser(
        'export', help='write the captured scene as a standalone MJCF file')
    add_common(sub)
    sub.add_argument('--scene', required=True, help='scene.json or scene.jsonl')
    sub.add_argument('--output', default=None, help='where to write the .xml')
    sub.add_argument('--raw', action='store_true',
                     help='export the unfiltered single-frame estimate')
    sub.add_argument('--allow-stale', action='store_true')
    sub.add_argument('--no-camera', action='store_true',
                     help='leave out the calibrated camera element')
    sub.add_argument('--camera-scale', type=float, default=0.5,
                     help='fraction of the real 1920x1080 the camera renders')
    sub.set_defaults(func=command_export, side='sim')

    sub = subparsers.add_parser('replay', help='re-run a recorded log into the mirror')
    add_common(sub)
    add_sim(sub)
    sub.add_argument('--log', required=True)
    sub.add_argument('--start', type=int, default=0)
    sub.add_argument('--count', type=int, default=0)
    sub.add_argument('--render-every', type=int, default=1)
    sub.add_argument('--realtime', type=float, default=0.0,
                     help='seconds to pause between frames')
    sub.add_argument('--tail', type=int, default=10)
    sub.set_defaults(func=command_replay, side='sim')

    sub = subparsers.add_parser(
        'selftest', help='render known poses and measure what is recovered')
    add_common(sub)
    add_fit(sub)
    sub.add_argument('--placement', default='calibrated',
                     choices=['calibrated', 'side', 'overhead'])
    sub.add_argument('--grid', type=int, default=9,
                     help='how many ground-truth placements to render')
    sub.add_argument('--no-lift-check', action='store_true')
    sub.add_argument('--viewer', action='store_true')
    sub.set_defaults(func=command_selftest, side='sim')
    sub = subparsers.add_parser('arm-stream', help='exclusive read-only actual joint feedback')
    sub.add_argument('--port', default='/dev/ttyACM0')
    sub.add_argument('--state', required=True, help='atomic latest feedback JSON')
    sub.add_argument('--log', help='new raw feedback JSONL (refuses overwrite)')
    sub.add_argument('--rate', type=float, default=30.)
    sub.add_argument('--duration', type=float, default=0.)
    sub.set_defaults(func=command_arm_stream, side='real')

    for name in ('sync', 'arm-replay'):
        sub = subparsers.add_parser(name, help='joint feedback mirror (no physics stepping)')
        sub.add_argument('--scene', required=True, help='scene.json with calibration provenance')
        sub.add_argument('--xml', required=True, help='already exported scene.xml')
        if name == 'sync':
            sub.add_argument('--state', required=True)
        else:
            sub.add_argument('--log', required=True, help='raw arm-stream / publisher JSONL')
            sub.add_argument('--speed', type=float, default=1.)
        sub.add_argument('--record', help='new conversion/status audit JSONL')
        sub.add_argument('--status-output', help='latest status JSON for headless monitoring')
        sub.add_argument('--gripper-map', help='measured two-point jaw calibration JSON')
        sub.add_argument('--display-rate', type=float, default=60.)
        sub.add_argument('--stale-after', type=float, default=.5)
        sub.add_argument('--disconnect-after', type=float, default=2.)
        sub.add_argument('--duration', type=float, default=0.)
        sub.add_argument('--viewer', action='store_true')
        sub.set_defaults(func=command_arm_sync, side='sim')
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return args.func(args)
    except (RuntimeError, FileNotFoundError, FileExistsError, ValueError, KeyError) as error:
        print(f'error: {error}', file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
