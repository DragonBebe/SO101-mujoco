"""Synthetic feedback only: conversion, freshness and named MuJoCo state."""
import copy
import math
from pathlib import Path
import json
import numpy as np
import pytest
from realsim.arm_mapping import ArmMapping
from realsim.arm_sync import FeedbackState

ROOT = Path(__file__).resolve().parents[2]


def mapping():
    return ArmMapping.from_scene(json.loads((ROOT / 'calib/realsim-model/scene.json').read_text()))


def sample(t=100.):
    return {'schema': 'realsim/arm-feedback/1', 'valid': True,
            'sampled_at': t, 'sampled_monotonic': t, 'received_at': t,
            'registers': mapping().expected_registers,
            'sequence': 1, 'stream_id': 'synthetic-test', 'clock_id': 'test',
            'ticks': {'1': 2101, '2': 2087, '3': 1984, '4': 1956, '5': 2048, '6': 2020}}


def test_calibrated_zero_offsets_and_positive_quarter_turn():
    m = mapping()
    q, info = m.convert(sample()['ticks'])
    assert q['shoulder_pan'] == 0
    assert q['shoulder_lift'] == pytest.approx(0.05352945636905725)
    assert q['elbow_flex'] == pytest.approx(-0.06207982012055717)
    assert q['wrist_flex'] == pytest.approx(0.18247878128862513)
    assert 'gripper' not in q and info['status'] == 'uncalibrated'
    ticks = sample()['ticks']; ticks['1'] += 1000
    q, _ = m.convert(ticks)
    assert q['shoulder_pan'] == pytest.approx(1.5343553863686413)


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -1, 4096, '2048', True])
def test_bad_ticks_rejected(bad):
    ticks = sample()['ticks']; ticks['3'] = bad
    with pytest.raises(ValueError): mapping().convert(ticks)


def test_missing_joint_rejected():
    ticks = sample()['ticks']; del ticks['4']
    with pytest.raises(ValueError): mapping().convert(ticks)


def test_explicit_gripper_two_point_calibration():
    m = mapping()
    m.set_gripper({'ticks': [2020, 3000], 'radians': [0, 1.2],
                   'source': 'SYNTHETIC fixture, not hardware calibration'})
    ticks = sample()['ticks']; ticks['6'] = 2510
    q, _ = m.convert(ticks)
    assert q['gripper'] == pytest.approx(.6)
    ticks['6'] = 3001
    with pytest.raises(ValueError): m.convert(ticks)


def test_stale_invalid_disconnect_and_recovery_keep_last_valid():
    state = FeedbackState(mapping(), stale_after=.5, disconnect_after=2)
    state.accept(sample(), now=100., monotonic=100., clock_id='test')
    assert state.status(100.1)['status'] == 'live'
    previous = state.joints.copy()
    bad = sample(100.2); bad['ticks']['1'] = float('nan')
    state.accept(bad, now=100.2, monotonic=100.2, clock_id='test')
    assert state.status(100.2)['status'] == 'invalid'
    assert state.joints == previous
    assert state.status(100.6)['status'] == 'stale'
    assert state.status(102.1)['status'] == 'disconnected'
    state.accept(sample(103), now=103, monotonic=103, clock_id='test')
    assert state.status(103.1)['status'] == 'live'


def test_old_or_future_samples_never_look_fresh():
    for stamp in (90, 110):
        state = FeedbackState(mapping())
        state.accept(sample(stamp), now=100, monotonic=100, clock_id='test')
        assert state.status(100)['status'] != 'live'
        assert not state.joints


def test_named_qpos_base_alignment_and_no_physics(tmp_path):
    mj = pytest.importorskip('mujoco')
    pytest.importorskip('so101_nexus')
    from realsim.arm_sync import ArmMirror
    xml = ROOT / 'calib/realsim-model/scene.xml'
    record = json.loads((ROOT / 'calib/realsim-model/scene.json').read_text())
    sim = ArmMirror(xml, record, mapping())
    before = sim.data.qpos.copy()
    q, _ = mapping().convert(sample()['ticks'])
    sim.apply(q)
    assert sim.data.time == 0
    for name, value in q.items():
        assert sim.data.qpos[sim.model.joint(name).qposadr[0]] == value
    jid = sim.model.joint('red_cube_joint').qposadr[0]
    np.testing.assert_array_equal(sim.data.qpos[jid:jid+7], before[jid:jid+7])
    base = sim.model.site('baseframe').id
    expected = np.linalg.inv(np.array(record['calibration']['T_base_table']))
    np.testing.assert_allclose(sim.data.site_xpos[base], expected[:3, 3], atol=1e-9)
    np.testing.assert_allclose(sim.data.site_xmat[base].reshape(3,3), expected[:3,:3], atol=1e-9)
    previous = sim.data.qpos.copy()
    with pytest.raises(ValueError): sim.apply({'shoulder_pan': 100})
    np.testing.assert_array_equal(sim.data.qpos, previous)


def test_export_aligns_base_without_moving_objects(tmp_path):
    mj = pytest.importorskip('mujoco'); pytest.importorskip('so101_nexus')
    from realsim import export, blocks
    record = json.loads((ROOT / 'calib/realsim-model/scene.json').read_text())
    path = tmp_path / 'aligned.xml'
    export.write(record, blocks.load(), path)
    model = mj.MjModel.from_xml_path(str(path)); data = mj.MjData(model)
    mj.mj_forward(model, data)
    expected = np.linalg.inv(np.array(record['calibration']['T_base_table']))
    np.testing.assert_allclose(data.site_xpos[model.site('baseframe').id], expected[:3,3], atol=1e-8)


def test_publisher_records_errors_recovers_and_closes_without_commands(tmp_path):
    import time
    from realsim.arm_feedback import FeedbackPublisher
    class ReadOnlyBus:
        count = 0
        def read_state(self):
            self.count += 1
            if self.count == 2:
                raise RuntimeError('synthetic disconnect')
            return {i: {'present_position': 2000 + i} for i in range(1,7)}
    path, log = tmp_path / 'latest.json', tmp_path / 'raw.jsonl'
    publisher = FeedbackPublisher(ReadOnlyBus(), path, rate=100, log=log).start()
    deadline = time.monotonic() + 2
    while publisher.sequence < 4 and time.monotonic() < deadline:
        time.sleep(.01)
    publisher.close()
    rows = [json.loads(line) for line in log.read_text().splitlines()]
    assert rows[0]['ticks']['1'] == 2001
    assert rows[1]['valid'] is False and 'synthetic disconnect' in rows[1]['error']
    assert rows[2]['valid'] is True
    assert rows[-1]['status'] == 'disconnected'
    assert json.loads(path.read_text()) == rows[-1]
    assert all(row['received_at'] >= row['sampled_at'] for row in rows)


def test_calibration_register_change_rejected():
    state = FeedbackState(mapping())
    row = sample(); row['registers']['1']['homing_offset'] += 1
    assert not state.accept(row, now=100, monotonic=100, clock_id='test')
    assert not state.joints
    assert 'homing_offset' in state.error


def test_no_second_open_of_existing_serial_descriptor():
    import os
    import pty
    from rgbcal.bus_access import PortOwnership
    master, slave = pty.openpty()
    try:
        with pytest.raises(RuntimeError, match='already open'):
            PortOwnership(os.ttyname(slave))
    finally:
        os.close(master); os.close(slave)


def test_bus_publisher_serializes_sdk_and_never_writes_on_lifecycle(tmp_path, monkeypatch):
    """Fake servo SDK + real PTY: any overlapping transaction/register write fails."""
    import os, pty, sys, threading, time, types
    serial = pytest.importorskip('serial')
    from rgbcal.robot import ServoBus
    master, slave = pty.openpty(); port = os.ttyname(slave); os.close(slave)
    active = threading.Lock()
    calls = []
    class Port:
        def __init__(self, port): self.port, self.ser = port, None
        def openPort(self): self.ser = serial.Serial(self.port); return True
        def setBaudRate(self, baud): return True
        def closePort(self): self.ser.close()
    class Packet:
        def read2ByteTxRx(self, port, motor, address):
            assert active.acquire(blocking=False), 'SDK transactions overlapped'
            try:
                time.sleep(.0001); calls.append(('read', address))
                return 2048, 0, 0
            finally: active.release()
        read1ByteTxRx = read2ByteTxRx
        def write2ByteTxRx(self, *args): raise AssertionError('unexpected servo write')
        write1ByteTxRx = write2ByteTxRx
    monkeypatch.setitem(sys.modules, 'scservo_sdk', types.SimpleNamespace(
        PortHandler=Port, PacketHandler=lambda _: Packet(), COMM_SUCCESS=0))
    monkeypatch.setenv('SO101_FEEDBACK_PATH', str(tmp_path / 'state.json'))
    monkeypatch.delenv('SO101_FEEDBACK_LOG', raising=False)
    try:
        with ServoBus(port) as bus:
            for _ in range(5): bus.read_positions()
        assert calls and all(kind == 'read' for kind, _ in calls)
        assert not bus.port_handler.ser.is_open
        assert json.loads((tmp_path / 'state.json').read_text())['status'] == 'disconnected'
    finally: os.close(master)


@pytest.mark.parametrize('name,servo_id', [('shoulder_pan','1'), ('shoulder_lift','2'),
                         ('elbow_flex','3'), ('wrist_flex','4'), ('wrist_roll','5')])
@pytest.mark.parametrize('delta', [-100,100])
def test_each_joint_moves_only_its_named_target(name, servo_id, delta):
    m = mapping(); ticks = sample()['ticks']; before, _ = m.convert(ticks)
    ticks[servo_id] += delta; after, _ = m.convert(ticks)
    assert after[name] - before[name] == pytest.approx(.15343553863686413 if delta > 0 else -.15343553863686413)
    for other in set(after) - {name}: assert after[other] == before[other]


@pytest.mark.parametrize('field', ['ticks', 'registers'])
def test_malformed_feedback_structure_retains_pose(field):
    state = FeedbackState(mapping())
    assert state.accept(sample(), now=100, monotonic=100, clock_id='test')
    previous = state.joints.copy()
    row = sample(100.1); row[field] = []
    assert not state.accept(row, now=100.1, monotonic=100.1, clock_id='test')
    assert state.joints == previous


@pytest.mark.parametrize('speed', [.1, 1, 10])
def test_replay_uses_virtual_age_and_keeps_acquisition_delay(speed):
    from realsim.arm_sync import ReplayClock
    row = sample(100); row['received_monotonic'] = 101
    replay = ReplayClock([row], speed=speed, started=1000)
    assert replay.latest(1000 + .4 / speed) is None
    incoming = replay.latest(1000 + 1.1 / speed)
    assert incoming == row
    state = FeedbackState(mapping())
    assert not state.accept(incoming, monotonic=replay.now(1000 + 1.1 / speed), replay=True)
    assert not state.joints
    row = sample(100); row['received_monotonic'] = 100
    replay = ReplayClock([row], speed=speed, started=1000)
    state.accept(replay.latest(1000), monotonic=replay.now(1000), replay=True)
    assert state.status(replay.now(1000 + .6 / speed))['status'] == 'stale'
    assert state.status(replay.now(1000 + 2.1 / speed))['status'] == 'disconnected'


def test_model_zero_correction_is_separate_from_handeye_and_applied_once():
    record = json.loads((ROOT / 'calib/realsim-model/scene.json').read_text())
    record.pop('model_joint_offsets', None)
    original = ArmMapping.from_scene(record)
    record['model_joint_offsets'] = {'wrist_roll': {
        'radians': -math.pi / 2, 'status': 'visual_estimate',
        'source': 'SYNTHETIC quarter-turn fixture, not a hardware measurement'}}
    corrected = ArmMapping.from_scene(record)
    ticks = sample()['ticks']; ticks['5'] = 2151
    q, _ = corrected.convert(ticks)
    assert math.degrees(q['wrist_roll']) == pytest.approx(-80.9010989010989)
    before, _ = original.convert(ticks)
    assert {k: v for k, v in q.items() if k != 'wrist_roll'} == {
        k: v for k, v in before.items() if k != 'wrist_roll'}
    assert corrected.entries['wrist_roll']['calibration_offset_rad'] == 0
    assert corrected.entries['wrist_roll']['zero_status'] == 'visual_estimate'
    assert original.entries['wrist_roll']['zero_status'] == 'unverified_pinned'
    state = FeedbackState(corrected)
    state.accept(sample(), now=100, monotonic=100, clock_id='test')
    assert state.status(100)['mapping_zero_status']['wrist_roll'] == 'visual_estimate'
    assert corrected.convert(ticks)[0] == q


@pytest.mark.parametrize('entry', [
    {'radians': float('nan'), 'status': 'visual_estimate', 'source': 'test'},
    {'radians': -.5, 'status': 'visual_estimate'},
    {'radians': -.5, 'status': 'solved', 'source': 'test'},
    {'radians': True, 'status': 'measured', 'source': 'test'},
])
def test_invalid_model_zero_correction_rejected(entry):
    record = json.loads((ROOT / 'calib/realsim-model/scene.json').read_text())
    record['model_joint_offsets'] = {'wrist_roll': entry}
    with pytest.raises(ValueError): ArmMapping.from_scene(record)


def test_unknown_model_joint_offset_rejected():
    record = json.loads((ROOT / 'calib/realsim-model/scene.json').read_text())
    record['model_joint_offsets'] = {'wrist_rol': {
        'radians': -.5, 'status': 'visual_estimate', 'source': 'test'}}
    with pytest.raises(ValueError): ArmMapping.from_scene(record)


def test_model_zero_rotates_actual_gripper_geometry_without_moving_base_or_objects():
    mj = pytest.importorskip('mujoco')
    from realsim.arm_sync import ArmMirror
    record = json.loads((ROOT / 'calib/realsim-model/scene.json').read_text())
    record.pop('model_joint_offsets', None)
    original = ArmMapping.from_scene(record)
    record['model_joint_offsets'] = {'wrist_roll': {
        'radians': -math.pi / 2, 'status': 'visual_estimate', 'source': 'SYNTHETIC'}}
    corrected = ArmMapping.from_scene(record)
    sim = ArmMirror(ROOT / 'calib/realsim-model/scene.xml', record, corrected)
    sim.apply(original.convert(sample()['ticks'])[0])
    body = sim.model.body('gripper').id
    rotation_before = sim.data.xmat[body].reshape(3, 3).copy()
    mount = sim.data.xpos[sim.model.body('camera_mount').id].copy()
    base = sim.data.site_xpos[sim.model.site('baseframe').id].copy()
    objects = sim.data.qpos[6:].copy()
    sim.apply(corrected.convert(sample()['ticks'])[0])
    rotation_after = sim.data.xmat[body].reshape(3, 3)
    np.testing.assert_allclose(rotation_before.T @ rotation_after,
                               [[0, 1, 0], [-1, 0, 0], [0, 0, 1]], atol=1e-10)
    np.testing.assert_allclose(sim.data.xpos[sim.model.body('camera_mount').id], mount)
    np.testing.assert_array_equal(sim.data.site_xpos[sim.model.site('baseframe').id], base)
    np.testing.assert_array_equal(sim.data.qpos[6:], objects)
    assert sim.data.time == 0
