"""Exercise the actual CLI, socket and persistent physical world."""
import json
import os
from pathlib import Path
import select
import socket
import subprocess
import sys

import numpy as np
import pytest


@pytest.mark.parametrize('gui', [False, True])
def test_two_tasks_over_socket_and_invalid_json_recovery(tmp_path, gui):
    if gui and (os.environ.get('MUJOCO_GL') != 'glfw' or not os.environ.get('DISPLAY')):
        pytest.skip('Dual-window test requires a GLFW desktop')
    root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(root))
    from nexus_vision.bridge import send
    address = str(tmp_path / 'loop.sock')
    errors = (tmp_path / 'server.stderr').open('w')
    process = subprocess.Popen([sys.executable, str(root / 'loop.py'), 'serve',
                                '--socket', address, '--record', str(tmp_path / 'run'),
                                *(['--viewer', '--camera-viewer'] if gui else [])],
                               stdout=subprocess.PIPE, stderr=errors,
                               text=True, env=os.environ.copy())
    try:
        assert select.select([process.stdout], [], [], 20)[0], 'Server did not announce readiness'
        ready = process.stdout.readline()
        assert ready, (tmp_path / 'server.stderr').read_text()
        assert json.loads(ready)['ready']
        assert Path(address).stat().st_mode & 0o777 == 0o600
        for invalid in (b'{"action":"wait","seconds":NaN}\n', b'[]\n'):
            with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
                client.settimeout(5)
                client.connect(address)
                client.sendall(invalid)
                assert not json.loads(client.recv(65536))['ok']
        first = send(address, {'action': 'begin', 'instruction': '夹爪上移三厘米'})
        initial = first['observation']['robot']['tcp'][:3]
        target = [initial[0], initial[1], initial[2] + .03]
        moved = send(address, {'action': 'step', 'task_id': first['task']['id'],
                              'frame_id': first['frame_id'], 'expected_step': 0,
                              'command': {'action': 'move', 'position': target,
                                          'orientation': 'position', 'seconds': .5}})
        assert moved['ok'], moved
        actual = moved['observation']['robot']['tcp'][:3]
        np.testing.assert_allclose(actual, target, atol=.002)
        assert moved['frame_id'] != first['frame_id']
        done = send(address, {'action': 'complete', 'task_id': first['task']['id'],
                             'frame_id': moved['frame_id'], 'outcome': 'succeeded',
                             'evidence': 'TCP 上升约三厘米，当前图像可复核。'})
        assert done['ok']
        second = send(address, {'action': 'begin', 'instruction': '保持这个姿态'})
        assert second['world'] == first['world']
        assert second['observation']['time'] == moved['observation']['time']
        assert send(address, {'action': 'cancel'})['ok']
        reset = send(address, {'action': 'reset', 'seed': 7})
        assert reset['ok'], reset
        assert reset['world'] == 2 and reset['task'] is None
        assert not send(address, {'action': 'localize', 'frame_id': moved['frame_id'],
                                  'pixel': [50, 50]})['ok']
        assert send(address, {'action': 'shutdown'})['ok']
        assert process.wait(timeout=10) == 0
        assert not Path(address).exists()
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        errors.close()
