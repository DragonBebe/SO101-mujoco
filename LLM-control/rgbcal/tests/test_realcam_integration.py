"""The real-camera calibration, driven through the unmodified RGB pipeline.

Each test answers one way the integration could be silently wrong: metres
versus millimetres, a transform used in the wrong direction, OpenCV axes
handed to code that expects OpenGL, a raw pixel paired with undistorted
intrinsics, or a capture configuration that no longer matches.
"""
import json
from pathlib import Path

import cv2
import numpy as np
import pytest

from nexus_vision import perception
from rgbcal import realcam, transforms as tf
from rgbcal.capture import CameraSettings

CALIB = Path(__file__).resolve().parents[2] / 'calib'
pytestmark = pytest.mark.skipif(
    not (CALIB / 'c920-handeye-03/extrinsics.json').exists(),
    reason='needs the measured C920 calibration')


@pytest.fixture(scope='module')
def calibration():
    return realcam.load()


@pytest.fixture(scope='module')
def undistorted(calibration):
    return calibration.undistortion()


def _raw_pixel(calibration, point_base):
    """Where a base-frame point lands in the RAW (distorted) image."""
    camera_base = tf.invert(calibration.T_base_camera)
    rvec, tvec = tf.to_rvec_tvec(camera_base)
    pixel, _ = cv2.projectPoints(np.asarray(point_base, float).reshape(1, 3),
                                 rvec, tvec, calibration.K, calibration.D)
    return pixel.reshape(2)


def _to_undistorted(calibration, new_matrix, raw_pixel):
    return cv2.undistortPoints(np.asarray(raw_pixel, float).reshape(1, 1, 2),
                               calibration.K, calibration.D, P=new_matrix).reshape(2)


def _table_point(calibration, x, y):
    """A point on the measured table, in base coordinates."""
    plane = calibration.table['plane']['z_of_xy']
    return np.array([x, y, plane['a'] * x + plane['b'] * y + plane['c_m']])


def test_units_are_metres(calibration):
    height = calibration.T_table_camera[2, 3]
    assert 0.2 < height < 1.0, 'camera height above the table should be tens of cm, in m'
    assert 0.3 < np.linalg.norm(calibration.T_base_camera[:3, 3]) < 2.0


def test_table_frame_really_puts_the_table_at_z0(calibration):
    for x, y in [(0.18, -0.05), (0.30, 0.00), (0.40, 0.08)]:
        point = np.append(_table_point(calibration, x, y), 1.0)
        in_table = tf.invert(calibration.T_base_table) @ point
        assert abs(in_table[2]) < 1e-9


@pytest.mark.parametrize('x,y', [(0.18, -0.06), (0.27, 0.0), (0.38, 0.05), (0.30, -0.10)])
def test_table_point_round_trips_through_the_unmodified_pipeline(calibration,
                                                                 undistorted, x, y):
    new_matrix, _ = undistorted
    truth = _table_point(calibration, x, y)
    pixel = _to_undistorted(calibration, new_matrix, _raw_pixel(calibration, truth))
    nexus = calibration.nexus_calibration(new_matrix, frame='table')
    # perception takes integer pixels: rounding is the only error left
    found_table = perception.intersect_pixel_with_plane(nexus, np.rint(pixel), plane_z=0.0)
    found_base = realcam.table_to_base(calibration, found_table)
    assert np.linalg.norm(found_base - truth) < 0.0015, found_base - truth


def test_camera_axes_reach_perception_as_opengl(calibration, undistorted):
    """A point on the optical axis must project to the principal point, and a
    point to the camera's right (OpenCV +x) must land right of it."""
    new_matrix, _ = undistorted
    nexus = calibration.nexus_calibration(new_matrix, frame='base')
    pose = calibration.T_base_camera
    ahead = pose[:3, 3] + 0.6 * pose[:3, 2]
    right = ahead + 0.05 * pose[:3, 0]
    below = ahead + 0.05 * pose[:3, 1]
    u, v, depth = perception.project_world(ahead, nexus)
    assert abs(u - new_matrix[0, 2]) < 1e-6 and abs(v - new_matrix[1, 2]) < 1e-6
    assert abs(depth - 0.6) < 1e-9
    assert perception.project_world(right, nexus)[0] > u
    assert perception.project_world(below, nexus)[1] > v


@pytest.mark.parametrize('raw', [(150, 950), (1800, 950), (150, 650), (1800, 600),
                                 (960, 1040), (960, 700)])
def test_points_across_the_whole_frame_round_trip(calibration, undistorted, raw):
    """Edges included: that is where distortion is largest."""
    new_matrix, _ = undistorted
    nexus = calibration.nexus_calibration(new_matrix, frame='table')
    pixel = _to_undistorted(calibration, new_matrix, raw)
    truth = realcam.table_to_base(
        calibration, perception.intersect_pixel_with_plane(nexus, np.rint(pixel), 0.0))
    back = _raw_pixel(calibration, truth)
    assert np.linalg.norm(back - np.asarray(raw, float)) < 1.0


def test_image_remap_agrees_with_point_undistortion(calibration, undistorted):
    """The runtime undistorts IMAGES; the maths above undistorts POINTS.

    If the two disagreed, a colour blob found in the undistorted frame would
    be paired with the wrong ray even though every point test passed.
    """
    new_matrix, maps = undistorted
    width, height = calibration.size
    for raw in [(200, 900), (1700, 950), (960, 540), (300, 300)]:
        image = np.zeros((height, width), np.uint8)
        cv2.circle(image, raw, 6, 255, -1)
        warped = cv2.remap(image, *maps, cv2.INTER_LINEAR)
        moments = cv2.moments(warped)
        found = np.array([moments['m10'] / moments['m00'], moments['m01'] / moments['m00']])
        expected = _to_undistorted(calibration, new_matrix, raw)
        assert np.linalg.norm(found - expected) < 0.5, (raw, found, expected)


def test_object_height_is_measured_from_the_table(calibration, undistorted):
    """The top of a 25 mm cube: plane_z = 0.025 in the table frame."""
    new_matrix, _ = undistorted
    table_point = _table_point(calibration, 0.28, 0.02)
    top = table_point + 0.025 * calibration.T_base_table[:3, 2]
    pixel = _to_undistorted(calibration, new_matrix, _raw_pixel(calibration, top))
    nexus = calibration.nexus_calibration(new_matrix, frame='table')
    found = realcam.table_to_base(
        calibration, perception.intersect_pixel_with_plane(nexus, np.rint(pixel), 0.025))
    assert np.linalg.norm(found - top) < 0.0015
    # the same pixel read against the table itself lands centimetres away
    wrong = realcam.table_to_base(
        calibration, perception.intersect_pixel_with_plane(nexus, np.rint(pixel), 0.0))
    assert np.linalg.norm(wrong - top) > 0.01


def test_motion_is_refused_without_a_motion_check_or_with_other_settings(calibration,
                                                                          tmp_path):
    profile = json.loads((CALIB / 'profiles/c920.json').read_text())
    matching = CameraSettings(**{k: profile[k] for k in
                                 ('device', 'width', 'height', 'fps', 'fourcc',
                                  'autofocus', 'focus')})
    status = realcam.motion_status(calibration, matching,
                                   motion_check=tmp_path / 'none.json')
    assert not status['usable_for_motion']
    assert any('motion check' in reason for reason in status['reasons'])

    other = CameraSettings(device=matching.device, width=1280, height=720,
                           autofocus=False, focus=0.0)
    status = realcam.motion_status(calibration, other,
                                   motion_check=tmp_path / 'none.json')
    assert any('resolution' in reason for reason in status['reasons'])


def test_experimental_extrinsics_cannot_be_loaded(tmp_path):
    source = json.loads((CALIB / 'c920-handeye-03/extrinsics.json').read_text())
    source['status'] = 'EXPERIMENTAL-UNVERIFIED'
    path = tmp_path / 'extrinsics-experimental.json'
    path.write_text(json.dumps(source))
    with pytest.raises(RuntimeError, match='experimental'):
        realcam.load(extrinsics=path, table=tmp_path / 'unused.json')


def test_drift_detector_follows_the_background_not_the_moving_objects():
    """A frame where the camera moved AND the arm moved: the verdict must be
    the camera's shift, decided by the static background."""
    from rgbcal.setup import drift

    rng = np.random.default_rng(3)
    background = (rng.random((1080, 1920, 3)) * 255).astype(np.uint8)
    background = cv2.GaussianBlur(background, (7, 7), 0)
    reference = background.copy()
    cv2.rectangle(reference, (900, 300), (1300, 800), (20, 20, 20), -1)   # "the robot"
    shifted = np.roll(np.roll(background, 5, axis=0), 2, axis=1)
    cv2.rectangle(shifted, (1000, 250), (1400, 750), (20, 20, 20), -1)    # it moved too
    import tempfile, os
    with tempfile.TemporaryDirectory() as folder:
        path = os.path.join(folder, 'reference.png')
        cv2.imwrite(path, reference)
        result = drift({'reference_image': path, 'anchors': []}, shifted)
    assert result['confident'] and result['shift_px'] == {'dx': 2.0, 'dy': 5.0}
