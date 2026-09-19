"""Real MuJoCo checks for the camera modality and environment placement.

Ground-truth object poses appear only as a test oracle, never as an input to
the code under test: the perception path still sees images and calibration.
"""
from pathlib import Path
import sys

import numpy as np
import pytest
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nexus_vision.cameras import CameraSuite, workspace_points
from nexus_vision.loop_session import LoopSession
from nexus_vision.perception import project_world
from nexus_vision.workspace import WorkspaceSimulation

COMBINATIONS = [(modality, placement)
                for modality in ('rgb', 'rgbd')
                for placement in ('side', 'overhead')]


def workspace(directory, modality='rgbd', placement='overhead', **extra):
    return WorkspaceSimulation(directory, cameras=CameraSuite(
        modality=modality, placement=placement, **extra))


def red_cube_pose(sim):
    """Oracle only: the carried cube's true centre and half extent."""
    slot = sim.env._slots[0]
    return sim.data.qpos[slot.qpos_addr:slot.qpos_addr + 3].copy(), sim.env.cube_half_size


@pytest.mark.parametrize(('modality', 'placement'), COMBINATIONS)
def test_every_mode_renders_real_images_from_the_configured_viewpoint(
        tmp_path, modality, placement):
    with workspace(tmp_path / 'world', modality, placement) as sim:
        observation = sim.observe()
        cameras = observation['cameras']
        assert sorted(cameras) == sorted((placement, 'wrist'))
        for name, entry in cameras.items():
            image = np.asarray(Image.open(entry['rgb']))
            assert image.shape == (480, 640, 3)
            # A cleared or constant buffer would mean nothing was rendered.
            assert image.std() > 5
            assert entry['width'] == 640 and entry['height'] == 480
            assert entry['modality'] == modality
            assert entry['mounting'] == ('world_fixed' if name == placement else 'robot_wrist')
        eye = np.asarray(cameras[placement]['calibration']['position'])
        if placement == 'overhead':
            assert eye[2] > 0.5 and np.linalg.norm(eye[:2] - [0.14, 0]) < 0.01
        else:
            # Off to one side of the table and well below the overhead height.
            assert eye[1] < -0.2 and 0.2 < eye[2] < 0.6 and eye[0] > 0.2
        assert observation['perception']['modality'] == modality
        assert observation['perception']['environment_camera'] == placement


def test_side_camera_frames_the_whole_reachable_workspace(tmp_path):
    with workspace(tmp_path / 'world', placement='side') as sim:
        calibration = sim.frames['side'][2]
        config = sim.env.config
        points = workspace_points(config.spawn_center, config.spawn_max_radius,
                                  config.spawn_angle_half_range_deg,
                                  sim.cameras.side_margin, sim.cameras.side_height)
        for point in points:
            u, v, depth = project_world(point, calibration)
            assert 0 <= u < 640 and 0 <= v < 480 and depth > 0, point


def test_side_depth_reprojects_onto_the_object_the_image_shows(tmp_path):
    with workspace(tmp_path / 'world', 'rgbd', 'side') as sim:
        result = sim.execute({'action': 'propose', 'frame_id': sim.frame_id, 'color': 'red'})
        assert result['depth_available'] and result['proposals']
        surface = np.asarray(result['proposals'][0]['surface_world'])
        centre, half = red_cube_pose(sim)
        # A surface point of a cube of this size, not merely somewhere nearby:
        # a miscalibrated side view would miss by far more than one half-width.
        assert np.all(np.abs(surface - centre) <= half + 0.004), (surface, centre)


def test_rgb_mode_has_no_depth_anywhere_the_controller_can_reach(tmp_path):
    with workspace(tmp_path / 'world', 'rgb', 'side') as sim:
        observation = sim.observe()
        for entry in observation['cameras'].values():
            assert entry['depth'] is None and 'depth_note' in entry
        assert not list(Path(sim.directory).glob('*.npy'))
        for name, (_, depth, _) in sim.frames.items():
            assert depth is None, name
        with pytest.raises(ValueError, match="'depth' is not supported"):
            sim.execute({'action': 'localize', 'frame_id': sim.frame_id,
                         'pixel': [320, 240], 'method': 'depth'})
        # The plane estimate is available, but only once the caller says which
        # plane: guessing one silently is the failure mode being prevented.
        with pytest.raises(ValueError, match='explicit plane_z'):
            sim.execute({'action': 'localize', 'frame_id': sim.frame_id, 'pixel': [320, 240]})
        proposals = sim.execute({'action': 'propose', 'frame_id': sim.frame_id,
                                 'color': 'red'})
        assert proposals['depth_available'] is False
        assert proposals['proposals'], 'RGB colour regions must still be found'
        assert all('surface_world' not in item for item in proposals['proposals'])


def test_environment_camera_stays_fixed_while_the_wrist_camera_moves(tmp_path):
    with workspace(tmp_path / 'world', 'rgb', 'side') as sim:
        frame = sim.frame_id
        before = {name: (rgb.copy(), np.asarray(cal['position']))
                  for name, (rgb, _, cal) in sim.frames.items()}
        moved = sim.execute({'action': 'move', 'position': [0.20, -0.05, 0.16],
                             'orientation': 'position', 'seconds': 0.5})
        assert moved['frame_id'] != frame
        np.testing.assert_allclose(moved['robot']['tcp'][:3], [0.20, -0.05, 0.16], atol=.005)
        after = {name: (rgb, np.asarray(cal['position']))
                 for name, (rgb, _, cal) in sim.frames.items()}
        np.testing.assert_array_equal(before['side'][1], after['side'][1])
        assert np.linalg.norm(after['wrist'][1] - before['wrist'][1]) > 0.02
        # The fixed camera keeps its pose but must still show the arm moving.
        assert not np.array_equal(before['side'][0], after['side'][0])
        assert not np.array_equal(before['wrist'][0], after['wrist'][0])


def test_restarting_in_another_mode_cannot_reuse_frames_points_or_cameras(tmp_path):
    first = LoopSession(tmp_path / 'rgbd', camera_modality='rgbd',
                        environment_camera='overhead')
    try:
        observation = first.execute({'action': 'observe'})['observation']
        located = first.execute({'action': 'localize', 'frame_id': observation['frame_id'],
                                 'pixel': [320, 240]})['result']
    finally:
        first.close()

    second = LoopSession(tmp_path / 'rgb', camera_modality='rgb',
                         environment_camera='side')
    try:
        state = second.execute({'action': 'status'})
        assert state['cameras']['modality'] == 'rgb'
        assert state['cameras']['environment_camera'] == 'side'
        stale = second.execute({'action': 'propose', 'color': 'red',
                                'frame_id': observation['frame_id']})
        assert not stale['ok'] and 'frame_id' in stale['error']
        current = second.execute({'action': 'observe'})['observation']['frame_id']
        gone = second.execute({'action': 'propose', 'camera': 'overhead',
                               'color': 'red', 'frame_id': current})
        assert not gone['ok'] and 'overhead' in gone['error']
        begun = second.execute({'action': 'begin', 'instruction': '不要复用旧定位点'})
        replay = second.execute({'action': 'step', 'task_id': begun['task']['id'],
                                 'frame_id': begun['frame_id'], 'expected_step': 0,
                                 'command': {'action': 'move',
                                             'point_id': located['point_id']}})
        assert not replay['ok'] and 'point' in replay['error']
        assert not list((second.directory / 'world-001').glob('*.npy'))
    finally:
        second.close()


def test_preview_panel_follows_the_mode_without_touching_missing_depth():
    from nexus_vision.camera_viewer import compose_panel, panel_cells, panel_size
    rgb_view = {'modality': 'rgb', 'cameras': ('side', 'wrist'),
                'environment_camera': 'side'}
    assert panel_cells(rgb_view) == [('side', 'rgb'), ('wrist', 'rgb')]
    assert panel_size(rgb_view) == (960, 438)
    # RGB-D keeps the original two-row layout and window size.
    assert panel_size({'modality': 'rgbd', 'cameras': ('overhead', 'wrist')}) == (960, 836)

    cameras = {name: (np.full((480, 640, 3), 40, dtype=np.uint8), None, {})
               for name in ('side', 'wrist')}
    panel = compose_panel(cameras, 1.25, rgb_view)
    assert panel.shape == (438, 960, 3)
