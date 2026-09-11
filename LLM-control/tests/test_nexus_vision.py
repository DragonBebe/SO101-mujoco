"""Run with .venv-vision: real upstream Nexus, without privileged policy state."""
import sys
from pathlib import Path
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from nexus_vision.simulation import VisualSimulation


def test_visual_observation_has_images_and_no_object_coordinates(tmp_path):
    with VisualSimulation('Touch', tmp_path, seed=4) as sim:
        obs = sim.observe()
        assert set(obs) == {'task', 'instruction', 'frame_id', 'time', 'robot', 'cameras'}
        for camera in obs['cameras'].values():
            assert Path(camera['rgb']).exists()
            assert np.load(camera['depth']).shape == (480, 640)
        assert len(obs['robot']['joints']) == 6
        # Even reset info and action feedback cannot expose privileged state.
        assert 'privileged' not in str(obs)
        assert 'object_position' not in str(obs)


def test_invalid_move_keeps_physics_unchanged(tmp_path):
    with VisualSimulation('Touch', tmp_path, seed=4) as sim:
        before = sim.env.data.qpos.copy()
        with pytest.raises(ValueError):
            sim.execute({'action':'move','position':[20,0,0]})
        np.testing.assert_array_equal(sim.env.data.qpos, before)


def test_pixel_selection_requires_matching_observation(tmp_path):
    with VisualSimulation('Touch', tmp_path, seed=4) as sim:
        obs = sim.observe()
        with pytest.raises(ValueError, match='frame'):
            sim.execute({'action':'localize','camera':'overhead','pixel':[320,240], 'frame_id':'stale-frame'})


def test_episode_reset_cannot_alias_old_frames_or_points(tmp_path):
    from nexus_vision.bridge import VisualSession
    session = VisualSession(tmp_path)
    try:
        first = session.execute({'action':'observe'})['result']
        old = session.execute({'action':'localize','pixel':[50,50], 'frame_id':first['frame_id']})['result']
        second = session.execute({'action':'start','task':'PickLift','seed':4})['result']
        second = session.execute({'action':'observe'})['result']  # Match the old episode's frame counter.
        assert first['frame_id'] != second['frame_id']
        assert not session.execute({'action':'localize','pixel':[50,50], 'frame_id':first['frame_id']})['ok']
        new = session.execute({'action':'localize','pixel':[50,50], 'frame_id':second['frame_id']})['result']
        assert old['point_id'] != new['point_id']
        assert not session.execute({'action':'move','point_id':old['point_id']})['ok']
        session.execute({'action':'finish'})
        time = session.sim.data.time
        assert not session.execute({'action':'wait','seconds':0.2})['ok']
        assert session.sim.data.time == time
    finally:
        session.close()


def test_far_plane_depth_does_not_localize_as_surface(tmp_path):
    with VisualSimulation('PickLift',tmp_path,seed=4) as sim:
        with pytest.raises(ValueError, match='depth'):
            sim.execute({'action':'localize','camera':'wrist','pixel':[600,400],'frame_id':sim.frame_id})


def test_visual_replay_reprojects_pixels_and_finishes(tmp_path):
    from nexus_vision.replay import replay_case
    case = {'task':'Move','seed':4,'commands':[
        {'action':'localize','pixel':[587,252],'save_as':'goal'},
        {'action':'move','point_id':'goal','offset':[0,0,-0.015],'orientation':'position'},
        {'action':'finish'},
    ]}
    result = replay_case(case,tmp_path)
    assert result['success']
    assert result['metrics']['move_displacement'] > 0.09


@pytest.mark.parametrize('commands', [
    [{'action':'finish'}, {'action':'start','task':'Touch'}],
    [{'action':'finish'}, {'action':'observe'}],
    [{'action':'finish'}, {'action':'finish'}],
    [{'action':'observe'}],
])
def test_replay_rejects_nonterminal_or_multiple_episodes_before_start(tmp_path, commands):
    from nexus_vision.replay import replay_case
    directory = tmp_path/'must-not-start'
    with pytest.raises(ValueError, match='single episode'):
        replay_case({'task':'Move','commands':commands},directory)
    assert not directory.exists()
