"""The calibrated environment camera: the measured real C920 in simulation.

Ground-truth object poses appear only as a test oracle, never as an input to
the code under test: the perception path still sees images and calibration.
"""
from pathlib import Path
import json
import math
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nexus_vision.cameras import (CALIBRATED_CAMERA_FILE, CameraSuite,
                                  load_calibrated_camera)
from nexus_vision.camera_viewer import TILE, fit_tile
from nexus_vision.perception import project_world
from nexus_vision.workspace import WorkspaceSimulation

needs_file = pytest.mark.skipif(not CALIBRATED_CAMERA_FILE.exists(),
                                reason='run `run_rgbcal.sh sim-camera` first')


def red_cube_pose(sim):
    """Oracle only: the red cube's true centre and half extent."""
    slot = sim.env._slots[0]
    return sim.data.qpos[slot.qpos_addr:slot.qpos_addr + 3].copy(), sim.env.cube_half_size


def test_calibrated_placement_is_rgb_only():
    with pytest.raises(ValueError, match="only available with camera modality 'rgb'"):
        CameraSuite(modality='rgbd', placement='calibrated')


def test_missing_export_is_reported_not_guessed(tmp_path):
    suite = CameraSuite(modality='rgb', placement='calibrated',
                        calibrated_file=str(tmp_path / 'absent.json'))
    with pytest.raises(ValueError, match='sim-camera'):
        suite.calibrated_camera()


@needs_file
def test_resampling_keeps_the_field_of_view_and_principal_point():
    record = json.loads(CALIBRATED_CAMERA_FILE.read_text())
    full = load_calibrated_camera(CALIBRATED_CAMERA_FILE, 1.0)
    half = load_calibrated_camera(CALIBRATED_CAMERA_FILE, 0.5)
    assert (full['width'], full['height']) == (record['image']['width'],
                                               record['image']['height'])
    assert (half['width'], half['height']) == (full['width'] // 2, full['height'] // 2)

    def fov(camera):
        return (math.atan((camera['cx'] + 0.5) / camera['fx']) +
                math.atan((camera['width'] - 0.5 - camera['cx']) / camera['fx']))
    assert fov(half) == pytest.approx(fov(full), abs=1e-9)
    assert (half['cx'] + 0.5) / half['width'] == pytest.approx(
        (full['cx'] + 0.5) / full['width'])


@needs_file
def test_calibrated_camera_renders_the_measured_intrinsics_and_pose(tmp_path):
    suite = CameraSuite(modality='rgb', placement='calibrated')
    expected = suite.calibrated_camera()
    with WorkspaceSimulation(tmp_path / 'world', cameras=suite) as sim:
        observation = sim.observe()
        entry = observation['cameras']['calibrated']
        calibration = entry['calibration']
        assert entry['mounting'] == 'world_fixed' and entry['depth'] is None
        assert (entry['width'], entry['height']) == (expected['width'], expected['height'])
        assert sim.frames['calibrated'][0].shape == (expected['height'], expected['width'], 3)
        # Read back from the GL camera that rendered the frame (float32).
        for key in ('fx', 'fy', 'cx', 'cy'):
            assert calibration[key] == pytest.approx(expected[key], abs=1e-3), key
        np.testing.assert_allclose(calibration['position'], expected['position'], atol=1e-6)
        np.testing.assert_allclose(calibration['rotation'], expected['rotation'], atol=1e-6)
        # The wrist camera keeps its own size and aspect.
        assert (observation['cameras']['wrist']['width'],
                observation['cameras']['wrist']['height']) == (suite.width, suite.height)
        assert sim.camera_report()['calibrated_resolved']['source'] == expected['source']


@needs_file
def test_plane_estimate_through_the_calibrated_camera_finds_the_cube(tmp_path):
    """The image really is what K and the pose say: pixels land on the object.

    If the frustum did not match the reported intrinsics (e.g. an off-centre
    principal point ignored, or a wrong horizontal scale), the carve would
    miss the cube by centimetres.
    """
    with WorkspaceSimulation(tmp_path / 'world',
                             cameras=CameraSuite(modality='rgb',
                                                 placement='calibrated')) as sim:
        centre, half = red_cube_pose(sim)
        calibration = sim.frames['calibrated'][2]
        result = sim.execute({'action': 'propose', 'frame_id': sim.frame_id,
                              'color': 'red', 'object_height': 2 * half})
        proposal = result['proposals'][0]
        u, v, _ = project_world(centre, calibration)
        assert abs(proposal['pixel'][0] - u) <= 3 and abs(proposal['pixel'][1] - v) <= 3
        support = np.asarray(proposal['support']['center'])
        assert np.hypot(*(support[:2] - centre[:2])) < 0.004, (support, centre)


def test_preview_tiles_keep_each_cameras_aspect():
    size, offset = fit_tile(960, 540)
    assert size == (TILE[0], 270) and offset == (0, (TILE[1] - 270) // 2)
    assert fit_tile(640, 480) == (TILE, (0, 0))
