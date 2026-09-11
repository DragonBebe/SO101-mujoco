import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simulation import Simulation


@pytest.fixture
def sim():
    return Simulation()


def test_scene_settles_on_table_and_cannot_claim_success(sim):
    obs = sim.observe()
    assert obs['objects']['RedBox']['position'][2] > 0.75
    assert np.linalg.norm(sim.data.body('base').xpos - [1.0, 0.6, 0.71]) < 0.002
    assert not obs['success']


@pytest.mark.parametrize('action', [
    {'action': 'move', 'position': [float('nan'), 0, 1]},
    {'action': 'move', 'position': [1, 2]},
    {'action': 'gripper', 'opening': -1},
    {'action': 'wait', 'seconds': 1000},
    {'action': '__import__'},
])
def test_invalid_action_does_not_advance_simulation(sim, action):
    before = sim.data.qpos.copy()
    with pytest.raises(ValueError):
        sim.execute(action)
    np.testing.assert_array_equal(sim.data.qpos, before)


def test_unreachable_move_does_not_change_state(sim):
    before = sim.data.qpos.copy()
    with pytest.raises(ValueError, match='reach|workspace|IK'):
        sim.execute({'action': 'move', 'position': [2, 2, 2]})
    np.testing.assert_array_equal(sim.data.qpos, before)


def test_contact_grasp_lifts_free_cube(sim):
    initial_z = sim.observe()['objects']['RedBox']['position'][2]
    for command in [
        {'action': 'move', 'position': [1.2, 0.6, 0.805]},
        {'action': 'move', 'position': [1.2, 0.6, 0.755]},
        {'action': 'gripper', 'opening': 0},
        {'action': 'move', 'position': [1.2, 0.6, 0.81]},
    ]:
        obs = sim.execute(command)
    red = obs['objects']['RedBox']
    assert red['position'][2] > initial_z + 0.04
    assert len(red['gripper_contacts']) == 2
    assert not obs['success']  # Held in the air is not placed.


def test_full_pick_place_remains_released_and_stable(sim):
    initial = sim.data.qpos[sim.model.joint('RedBox').qposadr[0]:][:3].copy()
    for command in [
        {'action': 'move', 'position': [1.2, 0.6, 0.805]},
        {'action': 'move', 'position': [1.2, 0.6, 0.755]},
        {'action': 'gripper', 'opening': 0},
        {'action': 'move', 'position': [1.2, 0.6, 0.81]},
        {'action': 'move', 'position': [1.16, 0.79, 0.81]},
        {'action': 'move', 'position': [1.16, 0.79, 0.755]},
        {'action': 'gripper', 'opening': 1},
        {'action': 'move', 'position': [1.16, 0.79, 0.81]},
        {'action': 'wait', 'seconds': 2},
    ]:
        obs = sim.execute(command)
    assert obs['success']
    red = obs['objects']['RedBox']
    np.testing.assert_allclose(red['position'], [1.16, 0.79, 0.755], atol=0.01)
    assert np.linalg.norm(np.array(red['position'])[:2] - initial[:2]) > 0.15
    assert red['speed'] < 0.001
    assert not red['gripper_contacts']
    assert sim.model.neq == 1  # Only the original base weld, no object attachment.


def test_persistent_service_rejects_bad_input_and_retains_state(tmp_path):
    import json
    import subprocess
    import time
    import socket
    cli = Path(__file__).resolve().parents[1] / 'control.py'
    assert cli.exists(), 'Persistent Codex control entry point is missing'
    address = str(tmp_path / 'sim.sock')
    process = subprocess.Popen([sys.executable, str(cli), 'serve', '--socket', address,
                                '--record', str(tmp_path / 'record')],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        for _ in range(200):
            if Path(address).exists() or process.poll() is not None:
                break
            time.sleep(0.05)
        assert process.poll() is None, process.communicate()

        def send(payload):
            with socket.socket(socket.AF_UNIX) as connection:
                connection.connect(address)
                connection.sendall(payload.encode() + b'\n')
                return json.loads(connection.makefile('rb').readline())

        first = send('{"action":"observe"}')['observation']
        assert not send('not json')['ok']
        assert not send('{"action":"wait","seconds":0.2,"extra":1e400}')['ok']
        second = send('{"action":"wait","seconds":0.2}')['observation']
        assert second['time'] == pytest.approx(first['time'] + 0.2)
        assert send('{"action":"shutdown"}')['ok']
        process.wait(timeout=5)
        assert process.returncode == 0
    finally:
        if process.poll() is None:
            process.terminate()
            process.wait(timeout=5)


def test_failed_screenshot_preserves_successful_motion_in_log(tmp_path, monkeypatch):
    import json
    from control import Session
    session = Session(tmp_path, capture=True)
    try:
        def fail_snapshot(path):
            raise RuntimeError('No graphics context')
        monkeypatch.setattr(session.sim, 'snapshot', fail_snapshot)
        initial_time = session.sim.data.time
        result = session.execute({'action': 'wait', 'seconds': 0.2})
        assert result['ok']
        assert 'capture_error' in result
        assert result['observation']['time'] == pytest.approx(initial_time + 0.2)
        entry = json.loads((tmp_path / 'actions.jsonl').read_text())
        assert entry['ok']  # Replay must retain the executed action.
    finally:
        session.close()
