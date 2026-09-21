"""Feedback to MuJoCo qpos, with independent latest-state refresh and replay."""
from contextlib import nullcontext
from pathlib import Path
import json
import math
import time
import numpy as np
from .arm_mapping import ArmMapping, finite_number, rigid_transform
from .arm_feedback import SCHEMA, clock_id, atomic_json


class FeedbackState:
    def __init__(self, mapping, stale_after=.5, disconnect_after=2.):
        if not 0 < stale_after < disconnect_after:
            raise ValueError('require 0 < stale-after < disconnect-after')
        self.mapping = mapping
        self.stale_after, self.disconnect_after = stale_after, disconnect_after
        self.joints, self.gripper = {}, {'status': 'uncalibrated'}
        self.last_valid = None
        self.last_sampled_at = None
        self.error, self.explicit_disconnect = None, False
        self.latest_stamp = -math.inf
        self.generation = 0

    def accept(self, record, now=None, monotonic=None, clock_id=None, replay=False):
        now = time.time() if now is None else now
        monotonic = time.monotonic() if monotonic is None else monotonic
        try:
            if not isinstance(record, dict):
                raise ValueError('feedback must be a JSON object')
            if record.get('schema') != SCHEMA:
                raise ValueError('unsupported feedback schema')
            if (not isinstance(record.get('stream_id'), str) or not record['stream_id']
                    or type(record.get('sequence')) is not int or record['sequence'] < 1):
                raise ValueError('feedback requires stream_id and positive sequence')
            if record.get('valid') is not True:
                self.explicit_disconnect = record.get('status') == 'disconnected'
                raise ValueError(record.get('error', 'invalid feedback'))
            sampled = record['sampled_at']
            stamp = record['sampled_monotonic']
            if not all(finite_number(v) for v in (sampled, stamp, record['received_at'])):
                raise ValueError('invalid sample timestamps')
            if not replay and record.get('clock_id') != clock_id:
                raise ValueError('feedback is from another boot/host; use arm-replay')
            age = monotonic - stamp
            if age < 0 or age > self.stale_after:
                raise ValueError(f'sample age {age:.3f}s is future or stale')
            if stamp < self.latest_stamp:
                raise ValueError('out-of-order feedback')
            if 'registers' in record:
                self.mapping.check_registers(record['registers'])
            else:
                raise ValueError('feedback lacks calibration registers')
            joints, gripper = self.mapping.convert(record['ticks'])
            self.joints, self.gripper = joints, gripper
            self.last_valid = monotonic - age
            self.last_sampled_at = sampled
            self.latest_stamp = stamp
            self.generation += 1
            self.error, self.explicit_disconnect = None, False
            return True
        except (KeyError, TypeError, ValueError) as error:
            self.error = str(error)
            return False

    def status(self, monotonic=None):
        monotonic = time.monotonic() if monotonic is None else monotonic
        age = None if self.last_valid is None else max(0, monotonic - self.last_valid)
        state = ('disconnected' if self.explicit_disconnect or age is None or age > self.disconnect_after
                 else 'stale' if age > self.stale_after
                 else 'invalid' if self.error else 'live')
        return {'status': state, 'age_s': age, 'last_valid_sampled_at': self.last_sampled_at,
                'error': self.error, 'gripper': self.gripper,
                'mapping_zero_status': {name: entry['zero_status']
                                        for name, entry in self.mapping.entries.items()},
                'joint_angles_rad': self.joints}


def align_base(model, data, record):
    import mujoco
    target = np.linalg.inv(rigid_transform(record['calibration']['T_base_table']))
    body = model.body('base').id
    site = model.site('baseframe').id
    if model.body_parentid[body] != 0 or model.site_bodyid[site] != body:
        raise ValueError('expected world-attached base with local baseframe site')
    local = np.eye(4)
    rot = np.empty(9)
    mujoco.mju_quat2Mat(rot, model.site_quat[site])
    local[:3, :3] = rot.reshape(3, 3)
    local[:3, 3] = model.site_pos[site]
    transform = target @ np.linalg.inv(local)
    model.body_pos[body] = transform[:3, 3]
    mujoco.mju_mat2Quat(model.body_quat[body], transform[:3, :3].flatten())
    mujoco.mj_forward(model, data)


class ArmMirror:
    def __init__(self, xml, record, mapping):
        import mujoco
        self.mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(xml))
        self.data = mujoco.MjData(self.model)
        self.addresses, self.ranges = {}, {}
        for name in [*mapping.entries, 'gripper']:
            joint = self.model.joint(name)
            if joint.type[0] != mujoco.mjtJoint.mjJNT_HINGE:
                raise ValueError(f'{name}: expected hinge')
            self.addresses[name] = int(joint.qposadr[0])
            self.ranges[name] = tuple(joint.range)
        align_base(self.model, self.data, record)

    def apply(self, joints):
        # Validate the entire update before changing any qpos.
        for name, value in joints.items():
            if name not in self.addresses or not finite_number(value):
                raise ValueError(f'invalid joint: {name}')
            low, high = self.ranges[name]
            if not low <= value <= high:
                raise ValueError(f'{name}: {value} rad outside model range [{low}, {high}]')
        for name, value in joints.items():
            self.data.qpos[self.addresses[name]] = value
        self.data.qvel[:] = 0
        self.mj.mj_forward(self.model, self.data)


def json_safe(value):
    """Retain malformed nonfinite feedback as labelled text in strict JSON logs."""
    if isinstance(value, float) and not math.isfinite(value):
        return repr(value)
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, list):
        return [json_safe(v) for v in value]
    return value


def positive(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError('rate and timeout values must be finite and positive')
    return value


class ReplayClock:
    """Schedule recorded arrivals; all validity uses the same virtual clock."""
    def __init__(self, rows, speed, started):
        if not rows:
            raise ValueError('empty feedback log')
        self.rows, self.speed, self.started = rows, positive(speed), started
        self.arrivals = []
        stamps = []
        for row in rows:
            stamp = row['sampled_monotonic']
            # Older raw logs omitted the final disconnect's received_monotonic.
            arrival = row.get('received_monotonic', stamp + max(
                0., row['received_at'] - row['sampled_at']))
            if not all(finite_number(v) for v in (stamp, arrival)) or arrival < stamp:
                raise ValueError('invalid replay sampling/receipt timestamps')
            stamps.append(stamp)
            self.arrivals.append(arrival)
        if (any(b < a for a, b in zip(stamps, stamps[1:]))
                or any(b < a for a, b in zip(self.arrivals, self.arrivals[1:]))):
            raise ValueError('replay timestamps must be ordered')
        self.origin = stamps[0]
        self.index = 0

    def now(self, real_monotonic):
        return self.origin + (real_monotonic - self.started) * self.speed

    def latest(self, real_monotonic):
        record = None
        while self.index < len(self.rows) and self.arrivals[self.index] <= self.now(real_monotonic):
            record = self.rows[self.index]
            self.index += 1
        return record

    def finished(self, real_monotonic, tail):
        return self.index == len(self.rows) and self.now(real_monotonic) > self.arrivals[-1] + tail


def run(args):
    record = json.loads(Path(args.scene).read_text())
    mapping = ArmMapping.from_scene(record)
    if args.gripper_map:
        mapping.set_gripper(json.loads(Path(args.gripper_map).read_text()))
    sim = ArmMirror(args.xml, record, mapping)
    rate = positive(args.display_rate)
    state = FeedbackState(mapping, positive(args.stale_after), positive(args.disconnect_after))
    replay = args.command == 'arm-replay'
    rows = None
    if replay:
        rows = [json.loads(line) for line in Path(args.log).read_text().splitlines() if line.strip()]
    replay_clock = ReplayClock(rows, args.speed, 0.) if replay else None
    if args.duration < 0 or not math.isfinite(args.duration):
        raise ValueError('duration must be finite and non-negative')
    if args.record:
        Path(args.record).parent.mkdir(parents=True, exist_ok=True)
    output = open(args.record, 'x', buffering=1) if args.record else None
    if output:
        output.write(json.dumps({'metadata': {'mapping': mapping.entries,
                     'provenance': mapping.provenance, 'gripper': mapping.gripper,
                     'xml': str(args.xml), 'scene': str(args.scene)}}) + '\n')
    viewer = None
    started = time.monotonic()
    if replay_clock:
        replay_clock.started = started
    frames, samples, applied, last_seen = 0, 0, 0, None
    next_report = started
    boot = clock_id()
    last_status = None
    print(json.dumps({'mapping': mapping.entries, 'provenance': mapping.provenance,
                      'gripper': mapping.gripper or 'UNCALIBRATED: raw ticks only',
                      'joint_ranges_rad': sim.ranges, 'mode': 'replay' if replay else 'live'}, default=float))
    try:
        if args.viewer:
            import mujoco.viewer
            viewer = mujoco.viewer.launch_passive(sim.model, sim.data)
        if replay_clock:
            # Opening a desktop window must not skip the beginning of a replay.
            started = time.monotonic()
            replay_clock.started = started
            next_report = started
        while not viewer or viewer.is_running():
            tick = time.monotonic()
            state_time = replay_clock.now(tick) if replay else tick
            if args.duration and tick - started >= args.duration:
                break
            incoming = None
            try:
                if replay:
                    incoming = replay_clock.latest(tick)
                    if replay_clock.finished(tick, args.disconnect_after):
                        break
                else:
                    incoming = json.loads(Path(args.state).read_text())
                if incoming is not None and not isinstance(incoming, dict):
                    incoming = None
                    raise ValueError('feedback must be a JSON object')
            except (OSError, ValueError) as error:
                state.error = str(error)
            if incoming is not None:
                identity = (incoming.get('stream_id'), incoming.get('sequence'))
                if identity != last_seen:
                    last_seen = identity
                    samples += 1
                    received = time.time()
                    old = (state.joints.copy(), state.gripper, state.last_valid, state.last_sampled_at,
                           state.latest_stamp, state.generation)
                    valid = state.accept(incoming, now=received, monotonic=state_time,
                                         clock_id=boot, replay=replay)
                    if valid:
                        try:
                            with viewer.lock() if viewer else nullcontext():
                                sim.apply(state.joints)
                            applied += 1
                        except ValueError as error:
                            (state.joints, state.gripper, state.last_valid, state.last_sampled_at,
                             state.latest_stamp, state.generation) = old
                            state.error = str(error)
                            valid = False
                    event = {'raw': incoming, 'received_at': received,
                             'received_monotonic': tick, 'valid': valid,
                             'processing_s': time.monotonic() - tick,
                             'converted': state.joints if valid else None,
                             'calibration_angles_rad': {
                                 name: value - mapping.entries[name]['model_offset_rad']
                                 for name, value in state.joints.items()
                                 if name in mapping.entries} if valid else None,
                             'status': state.status(state_time if replay else None), 'replay': replay}
                    if output:
                        output.write(json.dumps(json_safe(event), allow_nan=False) + '\n')
            status = state.status(state_time if replay else None)
            if viewer:
                text = f"{'REPLAY ' if replay else ''}{status['status']} age={status['age_s']} gripper={state.gripper['status']}"
                uncertain = ', '.join(f'{name}: {quality}' for name, quality
                                      in status['mapping_zero_status'].items()
                                      if quality not in ('handeye_fitted', 'measured'))
                viewer.set_texts([(sim.mj.mjtFontScale.mjFONTSCALE_150,
                                   sim.mj.mjtGridPos.mjGRID_TOPLEFT, text,
                                   f"last valid: {status['last_valid_sampled_at']}\n{uncertain}\n{status['error'] or ''}")])
                viewer.sync()
            frames += 1
            if tick >= next_report or status['status'] != last_status:
                report = {**status, 'elapsed_s': tick - started, 'refresh_hz': (frames - 1) / max(tick - started, 1e-6),
                          'received_hz': max(0, samples - 1) / max(tick - started, 1e-6), 'applied': applied,
                          'replay': replay}
                print(json.dumps(report), flush=True)
                if args.status_output:
                    atomic_json(args.status_output, report)
                if output:
                    output.write(json.dumps({'status_event': report}) + '\n')
                next_report = tick + 1
                last_status = status['status']
            time.sleep(max(0, 1 / rate - (time.monotonic() - tick)))
    except KeyboardInterrupt:
        pass
    finally:
        final_state = state.status(replay_clock.now(time.monotonic()) if replay else None)
        shutdown = {**final_state, 'feedback_status': final_state['status'],
                    'status': 'disconnected', 'error': 'mirror stopped',
                    'stopped_at': time.time(), 'replay': replay}
        try:
            if output:
                output.write(json.dumps({'status_event': shutdown}) + '\n')
            if args.status_output:
                atomic_json(args.status_output, shutdown)
        finally:
            if viewer:
                viewer.close()
            if output:
                output.close()
    elapsed = time.monotonic() - started
    print(json.dumps({'elapsed_s': elapsed, 'refresh_hz': frames / max(elapsed, 1e-6),
                      'received_hz': samples / max(elapsed, 1e-6), 'applied': applied,
                      'final': state.status(replay_clock.now(time.monotonic()) if replay else None), 'replay': replay}), flush=True)
    return 0
