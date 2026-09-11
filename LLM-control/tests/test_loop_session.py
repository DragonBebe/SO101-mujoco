"""Real RGB-D/physics tests for persistent conversation tasks."""
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


@pytest.fixture
def session(tmp_path):
    from nexus_vision.loop_session import LoopSession
    value = LoopSession(tmp_path)
    yield value
    value.close()


def begin(session, **extra):
    result = session.execute({'action': 'begin', 'instruction': '抬高夹爪，然后等待', **extra})
    assert result['ok'], result
    return result


def step(session, command, **extra):
    state = session.execute({'action': 'status'})['task']
    return session.execute({'action': 'step', 'task_id': state['id'],
                            'frame_id': session.sim.frame_id,
                            'expected_step': state['steps'], 'command': command, **extra})


def test_consecutive_tasks_preserve_world_and_log_evidence(session):
    start = begin(session)
    assert start['observation']['instruction'] == '抬高夹爪，然后等待'
    moved = step(session, {'action': 'wait', 'seconds': 0.1})
    assert moved['ok'] and moved['task']['steps'] == 1
    before = session.sim.data.qpos.copy()
    done = session.execute({'action': 'complete', 'task_id': moved['task']['id'],
                            'frame_id': session.sim.frame_id, 'outcome': 'succeeded',
                            'evidence': '查看当前双相机图像，机械臂保持稳定。'})
    assert done['ok'] and done['task']['verification'] == 'codex_visual'
    second = begin(session)
    assert second['task']['id'] != start['task']['id']
    np.testing.assert_array_equal(before, session.sim.data.qpos)
    rows = [json.loads(row) for row in (session.directory / 'events.jsonl').read_text().splitlines()]
    assert any(row['command']['action'] == 'complete' for row in rows)
    assert json.loads((session.directory / 'session.json').read_text())['task']['id'] == second['task']['id']


def test_stale_and_duplicate_steps_never_advance_physics(session):
    begin(session)
    before = session.sim.data.time
    assert not step(session, {'action': 'wait'}, frame_id='old')['ok']
    assert session.sim.data.time == before
    assert step(session, {'action': 'wait', 'seconds': .02})['ok']
    before = session.sim.data.time
    assert not step(session, {'action': 'wait'}, expected_step=0)['ok']
    assert session.sim.data.time == before


def test_pause_resume_and_budget(session):
    begin(session, max_steps=1)
    assert session.execute({'action': 'pause'})['ok']
    before = session.sim.data.time
    assert not step(session, {'action': 'wait'})['ok']
    assert session.sim.data.time == before
    assert session.execute({'action': 'resume'})['ok']
    assert step(session, {'action': 'wait', 'seconds': .02})['ok']
    before = session.sim.data.time
    assert not step(session, {'action': 'wait'})['ok']
    assert session.sim.data.time == before
    assert session.execute({'action': 'status'})['task']['status'] == 'paused'


def test_errors_are_recorded_and_repeated_failure_pauses(session):
    begin(session)
    before = session.sim.data.time
    for _ in range(3):
        result = step(session, {'action': 'move', 'position': [20, 0, 0]})
        assert not result['ok']
    assert result['task']['status'] == 'paused'
    assert result['task']['steps'] == 3
    assert session.sim.data.time == before


def test_invalid_commands_and_completion_cannot_mutate_task(session):
    assert not session.execute([])['ok']
    for bad in ('', None, 4):
        assert not session.execute({'action': 'begin', 'instruction': bad})['ok']
    begin(session)
    for cmd in ({'action': 'begin', 'instruction': '覆盖任务'},
                {'action': 'complete', 'outcome': 'succeeded'},
                {'action': 'reset'}, {'action': 'move', 'position': [.2, 0, .15]}):
        assert not session.execute(cmd)['ok']
    before = session.sim.data.time
    assert not step(session, {'action': 'shutdown'})['ok']
    assert session.running and session.sim.data.time == before


def test_workspace_has_multiple_physical_objects_and_rgbd(session):
    obs = session.execute({'action': 'observe'})['observation']
    assert 'object_position' not in str(obs)
    for camera in obs['cameras'].values():
        assert Path(camera['rgb']).exists()
        assert np.load(camera['depth']).shape == (480, 640)
    slots = [*session.sim.env._slots, *session.sim.env._distractor_slots]
    assert len(slots) >= 4
    for slot in slots:
        assert session.sim.model.geom_contype[slot.geom_id] != 0
        assert session.sim.data.qpos[slot.qpos_addr + 2] > 0
