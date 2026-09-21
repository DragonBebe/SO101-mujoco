"""Geometry the whole mapping rests on: projection, hulls, yaw, overlap.

These run under either virtual environment: nothing here needs OpenCV or
MuJoCo.
"""
import math

import numpy as np
import pytest

from nexus_vision import perception
from realsim import geometry


def camera(position=(0.6, 0.0, 0.4), look_down_deg=35.0, yaw_deg=180.0):
    """A pinhole camera in the table frame, in the convention perception uses."""
    pitch = math.radians(-look_down_deg)
    yaw = math.radians(yaw_deg)
    # OpenGL camera axes: +x right, +y up, -z forward.
    forward = np.array([math.cos(pitch) * math.cos(yaw),
                        math.cos(pitch) * math.sin(yaw), math.sin(pitch)])
    right = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    up = np.cross(right, forward)
    rotation = np.column_stack([right, up, -forward])
    return {'width': 1920, 'height': 1080, 'fx': 1400.0, 'fy': 1400.0,
            'cx': 959.5, 'cy': 539.5, 'position': list(position),
            'rotation': rotation.tolist()}


def test_projection_agrees_with_perception_point_for_point():
    calibration = camera()
    points = np.array([[0.25, 0.0, 0.0125], [0.3, -0.1, 0.05], [0.2, 0.12, 0.0]])
    uv, depth = geometry.project_points(points, calibration)
    for index, point in enumerate(points):
        expected = perception.project_world(point, calibration)
        assert uv[index] == pytest.approx(expected[:2], abs=1e-9)
        assert depth[index] == pytest.approx(expected[2], abs=1e-12)


def test_a_pixel_ray_returns_to_its_own_plane_point():
    """Projection and perception's plane intersection are inverse on the plane."""
    calibration = camera()
    point = np.array([0.27, -0.06, 0.025])
    uv, _ = geometry.project_points(point, calibration)
    back = perception.intersect_pixel_with_plane(
        calibration, np.rint(uv).astype(int).tolist(), plane_z=0.025)
    # Rounding to a whole pixel is the only error here; at this range one
    # pixel is well under a millimetre.
    assert np.linalg.norm(back - point) < 0.001


def test_convex_hull_of_a_projected_box_is_its_outline():
    calibration = camera()
    corners = geometry.box_corners([0.0125] * 3, [0.25, 0.0, 0.0125], 0.3)
    uv, _ = geometry.project_points(corners, calibration)
    hull = geometry.convex_hull(uv)
    # A box seen from an oblique angle shows six of its corners on the
    # outline; the two remaining ones are strictly inside it.
    assert 4 <= len(hull) <= 6
    for point in uv:
        inside = True
        for index in range(len(hull)):
            a, b = hull[index], hull[(index + 1) % len(hull)]
            cross = ((b[0] - a[0]) * (point[1] - a[1]) -
                     (b[1] - a[1]) * (point[0] - a[0]))
            inside &= cross >= -1e-6
        assert inside


def test_polygon_area_matches_its_rasterisation():
    polygon = np.array([[10.0, 10.0], [40.0, 12.0], [44.0, 35.0], [12.0, 38.0]])
    coverage = geometry.polygon_coverage(polygon, (0, 0, 60, 60), samples=8)
    assert coverage.sum() == pytest.approx(geometry.polygon_area(polygon), rel=0.01)


def test_mask_scorer_matches_brute_force_rasterisation():
    polygon = np.array([[10.0, 10.0], [40.0, 12.0], [44.0, 35.0], [12.0, 38.0]])
    mask = geometry.polygon_coverage(polygon, (0, 0, 60, 60), samples=8) > 0.5
    scorer = geometry.MaskScorer(mask, subrows=8)
    shifted = polygon + [3.0, -2.0]
    brute = geometry.polygon_coverage(shifted, (0, 0, 60, 60), samples=8)
    expected = float((brute * mask).sum())
    assert scorer.intersection(shifted) == pytest.approx(expected, rel=0.02)
    iou, area = scorer.iou(shifted)
    assert area == pytest.approx(geometry.polygon_area(shifted), rel=1e-9)
    assert 0.5 < iou < 0.95


def test_a_silhouette_scores_one_against_its_own_mask():
    calibration = camera()
    outline = geometry.silhouette([0.0125] * 3, [0.25, 0.0, 0.0125], 0.4, calibration)
    window = geometry.polygon_bounds(outline)
    mask = np.zeros((calibration['height'], calibration['width']), dtype=bool)
    coverage = geometry.polygon_coverage(
        outline, (window[0], window[1], window[2] - window[0], window[3] - window[1]),
        samples=8)
    mask[window[1]:window[3], window[0]:window[2]] = coverage > 0.5
    iou, _ = geometry.MaskScorer(mask, subrows=8).iou(outline)
    assert iou > 0.97


def test_yaw_is_folded_into_the_symmetry_and_compared_within_it():
    step = math.pi / 2
    assert geometry.canonical_yaw(math.radians(100), step) == pytest.approx(
        math.radians(10), abs=1e-9)
    assert geometry.canonical_yaw(math.radians(-5), step) == pytest.approx(
        math.radians(85), abs=1e-9)
    # 89 and 1 degree are the same cube pose, one degree apart, not 88.
    assert math.degrees(geometry.yaw_difference(
        math.radians(89), math.radians(1), step)) == pytest.approx(2.0, abs=1e-9)
    assert math.degrees(geometry.yaw_difference(
        math.radians(0), math.radians(45), step)) == pytest.approx(45.0, abs=1e-9)


def test_circular_mean_does_not_average_across_the_wrap():
    step = math.pi / 2
    angles = [math.radians(89.0), math.radians(1.0), math.radians(0.0)]
    mean = math.degrees(geometry.circular_mean(angles, step))
    assert mean > 88.0 or mean < 2.0


def test_quaternion_round_trip():
    for degrees in (0.0, 17.0, 89.9, 180.0):
        quaternion = geometry.yaw_quaternion(math.radians(degrees))
        assert np.linalg.norm(quaternion) == pytest.approx(1.0)
        recovered = math.degrees(geometry.quaternion_yaw(quaternion)) % 360
        assert recovered == pytest.approx(degrees % 360, abs=1e-6)
