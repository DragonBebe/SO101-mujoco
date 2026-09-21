"""The exported MJCF: does it open anywhere, and is its camera the real one.

Needs MuJoCo and ``so101_nexus``, so it is skipped on the real side.
"""
import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')
pytest.importorskip('so101_nexus')

from nexus_vision.cameras import CameraSuite                       # noqa: E402
from nexus_vision.simulation import VisualSimulation               # noqa: E402
from realsim import blocks, export, geometry, scene, track         # noqa: E402
from realsim.tests.test_track_scene import candidate, record_for   # noqa: E402


@pytest.fixture(scope='module')
def catalogue():
    return blocks.load()


@pytest.fixture(scope='module')
def record(catalogue):
    tracker = track.Tracker(catalogue)
    poses = {'red_cube': (0.30, -0.06, 12.0), 'green_cube': (0.26, 0.05, 71.0)}
    for block_id, (x, y, yaw) in poses.items():
        tracker.tracks[block_id].update(
            candidate(x, y, yaw, catalogue.by_id(block_id)), 100.0, tracker.settings)
    return record_for(tracker, 100.0)


def test_the_file_opens_on_its_own_from_any_directory(record, catalogue, tmp_path):
    """The robot's meshdir is relative; the export has to override it."""
    deep = tmp_path / 'somewhere' / 'else'
    detail = export.write(record, catalogue, deep / 'scene.xml')
    model = mujoco.MjModel.from_xml_path(detail['path'])
    assert model.nmesh > 0, 'the robot meshes did not resolve'
    names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, index)
             for index in range(model.nbody)]
    assert 'red_cube' in names and 'green_cube' in names
    # A block the camera never saw must not appear at all.
    assert 'blue_cube' not in names


def test_the_blocks_are_already_in_place_without_loading_a_keyframe(record, catalogue,
                                                                    tmp_path):
    detail = export.write(record, catalogue, tmp_path / 'scene.xml')
    model = mujoco.MjModel.from_xml_path(detail['path'])
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    for entry in detail['placed']:
        body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, entry['id'])
        np.testing.assert_allclose(data.xpos[body], entry['position_m'], atol=1e-9)
        yaw = geometry.quaternion_yaw(entry['quaternion_wxyz'])
        assert geometry.yaw_difference(
            geometry.quaternion_yaw(data.xquat[body]), yaw,
            np.pi / 2) < 1e-9


def test_the_exported_camera_reproduces_the_calibrated_intrinsics(record, catalogue,
                                                                  tmp_path):
    """Locks MuJoCo's principalpixel sign convention, which is not obvious.

    Both axes run opposite to the image axes; getting either wrong shifts the
    view by twice the principal-point offset, which still looks like a
    plausible picture.
    """
    camera = CameraSuite(modality='rgb', placement='calibrated').calibrated_camera()
    detail = export.write(record, catalogue, tmp_path / 'scene.xml', camera)
    model = mujoco.MjModel.from_xml_path(detail['path'])
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    index = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'calibrated')
    width, height = (int(value) for value in model.cam_resolution[index])
    assert (width, height) == (camera['width'], camera['height'])
    renderer = mujoco.Renderer(model, height=height, width=width)
    try:
        renderer.update_scene(data, camera='calibrated')
        actual = VisualSimulation._calibration(renderer, (height, width))
    finally:
        renderer.close()
    for key in ('fx', 'fy', 'cx', 'cy'):
        assert actual[key] == pytest.approx(camera[key], abs=1e-3), key
    np.testing.assert_allclose(actual['position'], camera['position'], atol=1e-9)
    np.testing.assert_allclose(actual['rotation'], camera['rotation'], atol=1e-9)


def test_a_stale_block_is_named_in_the_header_instead_of_being_placed(catalogue):
    tracker = track.Tracker(catalogue)
    tracker.tracks['red_cube'].update(
        candidate(0.30, 0.0, 10.0, catalogue.by_id('red_cube')), 100.0,
        tracker.settings)
    stale = record_for(tracker, 103.0)
    text, detail = export.mjcf(stale, catalogue)
    assert detail['placed'] == []
    assert 'NOT PLACED' in text and 'red_cube' in text


def test_the_header_says_what_was_measured_and_what_was_assumed(record, catalogue):
    text, _ = export.mjcf(record, catalogue)
    assert 'measured by the camera' in text and 'x, y and yaw' in text
    assert 'assumed, not measured' in text
    assert 'never been identified' in text          # mass and friction
