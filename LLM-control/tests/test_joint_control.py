"""The direct joint-space action: no inverse kinematics between command and pose.

``joints`` sets arm joint targets directly (delta or absolute, in degrees),
the way a teleoperator or a per-joint script would drive the real servos. It
never calls ``solve_ik``; whatever TCP pose results is read back afterwards
from ``observe``, not solved for in advance.
"""
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nexus_vision.simulation import JOINT_NAMES, MAX_JOINT_DELTA_DEG
from nexus_vision.workspace import WorkspaceSimulation


@pytest.fixture
def sim(tmp_path):
    value = WorkspaceSimulation(tmp_path)
    yield value
    value.close()


def test_delta_moves_only_the_named_joints_by_exactly_that_amount(sim):
    before = np.degrees(sim.target[:5].copy())
    sim.execute({'action': 'joints', 'delta_deg': {'shoulder_pan': 10, 'wrist_roll': -5},
                'seconds': 0.3})
    after = np.degrees(sim.target[:5])
    expected = before.copy()
    expected[JOINT_NAMES.index('shoulder_pan')] += 10
    expected[JOINT_NAMES.index('wrist_roll')] -= 5
    np.testing.assert_allclose(after, expected, atol=0.05)
    # Reached, not just commanded: the actual qpos tracks the target closely
    # once the joint has time to settle.
    reached = np.degrees(sim.data.qpos[sim.qadr[:5]])
    np.testing.assert_allclose(reached, expected, atol=1.5)


def test_target_deg_is_absolute_regardless_of_current_pose(sim):
    # wrist_flex rests at 30 deg (rest_qpos_deg); +15 then straight to 60 both
    # stay well inside its +-95 deg range.
    sim.execute({'action': 'joints', 'delta_deg': {'wrist_flex': 15}, 'seconds': 0.2})
    sim.execute({'action': 'joints', 'target_deg': {'wrist_flex': 60}, 'seconds': 0.3})
    reached = np.degrees(sim.target[JOINT_NAMES.index('wrist_flex')])
    assert reached == pytest.approx(60, abs=0.05)


def test_unspecified_joints_hold_their_target(sim):
    before = sim.target[:5].copy()
    sim.execute({'action': 'joints', 'delta_deg': {'wrist_flex': 8}, 'seconds': 0.2})
    after = sim.target[:5]
    for i, name in enumerate(JOINT_NAMES):
        if name != 'wrist_flex':
            assert after[i] == pytest.approx(before[i]), name


def test_delta_beyond_the_per_step_bound_is_rejected_and_does_not_move(sim):
    before = sim.target[:5].copy()
    with pytest.raises(ValueError, match='shoulder_pan'):
        sim.execute({'action': 'joints',
                    'delta_deg': {'shoulder_pan': MAX_JOINT_DELTA_DEG + 1}})
    np.testing.assert_array_equal(sim.target[:5], before)


def test_target_beyond_joint_limits_is_rejected_and_does_not_move(sim):
    before = sim.target[:5].copy()
    limit = np.degrees(sim.high[JOINT_NAMES.index('shoulder_lift')]) + 5
    with pytest.raises(ValueError, match='joint limits'):
        sim.execute({'action': 'joints', 'target_deg': {'shoulder_lift': limit}})
    np.testing.assert_array_equal(sim.target[:5], before)


def test_exactly_one_of_delta_or_target_is_required(sim):
    with pytest.raises(ValueError, match='exactly one of delta_deg or target_deg'):
        sim.execute({'action': 'joints'})
    with pytest.raises(ValueError, match='exactly one of delta_deg or target_deg'):
        sim.execute({'action': 'joints', 'delta_deg': {'wrist_roll': 1},
                    'target_deg': {'wrist_roll': 1}})


def test_unknown_joint_name_is_rejected(sim):
    with pytest.raises(ValueError, match='JOINT_NAMES|subset'):
        sim.execute({'action': 'joints', 'delta_deg': {'wrist_pitch': 5}})


def test_gripper_is_not_settable_through_joints(sim):
    with pytest.raises(ValueError):
        sim.execute({'action': 'joints', 'delta_deg': {'gripper': 5}})


def test_observation_reports_joint_names_degrees_and_limits(sim):
    obs = sim.observe()
    robot = obs['robot']
    assert tuple(robot['joint_names']) == JOINT_NAMES
    np.testing.assert_allclose(robot['joints_deg'],
                               np.degrees(robot['joints'][:5]), atol=1e-9)
    low, high = robot['joint_limits_deg']['low'], robot['joint_limits_deg']['high']
    assert len(low) == len(high) == 5
    assert all(lo < hi for lo, hi in zip(low, high))


def test_no_analytic_ik_is_involved(sim, monkeypatch):
    """The defining property: solve_ik is never called for this action."""
    calls = []
    monkeypatch.setattr(sim, 'solve_ik', lambda *a, **k: calls.append((a, k)))
    sim.execute({'action': 'joints', 'delta_deg': {'elbow_flex': 5}, 'seconds': 0.2})
    assert not calls, 'joints must not go through inverse kinematics'


def test_step_in_a_task_accepts_joints_as_a_motion(tmp_path):
    from nexus_vision.loop_session import LoopSession

    session = LoopSession(tmp_path)
    try:
        session.execute({'action': 'begin', 'instruction': '直接关节控制测试'})
        state = session.execute({'action': 'status'})['task']
        result = session.execute({'action': 'step', 'task_id': state['id'],
                                  'frame_id': session.sim.frame_id,
                                  'expected_step': state['steps'],
                                  'command': {'action': 'joints',
                                             'delta_deg': {'wrist_roll': 5},
                                             'seconds': 0.2}})
        assert result['ok'], result
        assert result['task']['steps'] == 1
    finally:
        session.close()
