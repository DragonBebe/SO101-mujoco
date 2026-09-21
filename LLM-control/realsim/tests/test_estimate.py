"""The single-frame estimator, on images whose true pose is known exactly.

These images are drawn by projecting the block model itself, so they test the
estimator and the conventions around it, not the colour threshold's tolerance
to real lighting.  Rendered-scene and real-photograph checks are separate and
are not replaced by these.
"""
import math

import numpy as np
import pytest

from realsim import blocks, estimate, geometry
from realsim.tests.test_geometry import camera

CUBE = blocks.Block(id='red_cube', colour='red', size_m=(0.025, 0.025, 0.025),
                    size_source='synthetic', size_measured=True, mass_kg=0.01,
                    mass_source='test', friction=(1.0, 0.005, 0.0001),
                    friction_source='test')
BAR = blocks.Block(id='green_bar', colour='green', size_m=(0.050, 0.025, 0.025),
                   size_source='synthetic', size_measured=True, mass_kg=0.01,
                   mass_source='test', friction=(1.0, 0.005, 0.0001),
                   friction_source='test')
COLOURS = {'red': (220, 20, 20), 'green': (20, 190, 20)}


def draw(calibration, items, background=(120, 120, 125)):
    """An image showing each ``(block, x, y, yaw, elevation)`` as a solid blob."""
    image = np.full((calibration['height'], calibration['width'], 3),
                    background, dtype=np.uint8)
    for block, x, y, yaw, elevation in items:
        centre = [x, y, elevation + block.half_extent[2]]
        outline = geometry.silhouette(block.half_extent, centre, yaw, calibration)
        u0, v0, u1, v1 = geometry.polygon_bounds(outline)
        coverage = geometry.polygon_coverage(outline, (u0, v0, u1 - u0, v1 - v0),
                                             samples=4)
        image[v0:v1, u0:u1][coverage > 0.5] = COLOURS[block.colour]
    return image


def test_a_flat_cube_is_recovered_in_position_and_yaw():
    calibration = camera()
    for true_yaw_deg in (0.0, 17.0, 43.0, 71.0):
        truth = (0.26, -0.04, math.radians(true_yaw_deg))
        image = draw(calibration, [(CUBE, truth[0], truth[1], truth[2], 0.0)])
        best = estimate.estimate_block(image, calibration, CUBE)[0]
        assert best['valid'], best['reasons']
        position = np.array(best['position'])
        assert np.linalg.norm(position[:2] - truth[:2]) < 0.0015
        # The centre height is the stated assumption, exactly.
        assert position[2] == pytest.approx(CUBE.height_m / 2, abs=1e-12)
        error = math.degrees(geometry.yaw_difference(
            best['yaw_rad'], truth[2], CUBE.symmetry_step_rad))
        assert error < 2.0
        assert best['quality']['iou'] > 0.95


def test_a_cube_yaw_is_reported_inside_its_own_symmetry():
    calibration = camera()
    image = draw(calibration, [(CUBE, 0.25, 0.0, math.radians(95.0), 0.0)])
    best = estimate.estimate_block(image, calibration, CUBE)[0]
    assert 0.0 <= best['yaw_deg'] < 90.0
    assert best['symmetry_step_deg'] == pytest.approx(90.0)


def test_a_rectangular_block_keeps_a_180_degree_symmetry():
    calibration = camera()
    truth_yaw = math.radians(20.0)
    image = draw(calibration, [(BAR, 0.26, 0.02, truth_yaw, 0.0)])
    best = estimate.estimate_block(image, calibration, BAR)[0]
    assert best['valid'], best['reasons']
    assert best['symmetry_step_deg'] == pytest.approx(180.0)
    assert math.degrees(geometry.yaw_difference(
        best['yaw_rad'], truth_yaw, BAR.symmetry_step_rad)) < 2.0


def test_a_lifted_block_is_refused_rather_than_placed_on_the_table():
    calibration = camera()
    image = draw(calibration, [(CUBE, 0.26, 0.0, math.radians(10.0), 0.040)])
    best = estimate.estimate_block(image, calibration, CUBE)[0]
    assert not best['valid']
    assert any('not resting on it' in reason for reason in best['reasons'])
    height = best['quality']['height_check']
    assert height['best_elevation_m'] > 0.02


def test_a_stacked_block_is_correct_once_its_support_height_is_stated():
    calibration = camera()
    truth = (0.255, 0.01, math.radians(30.0))
    image = draw(calibration, [(CUBE, truth[0], truth[1], truth[2], 0.025)])
    on_table = estimate.estimate_block(image, calibration, CUBE, plane_z=0.0)[0]
    assert not on_table['valid']
    stacked = estimate.estimate_block(image, calibration, CUBE, plane_z=0.025)[0]
    assert stacked['valid'], stacked['reasons']
    assert np.linalg.norm(np.array(stacked['position'][:2]) - truth[:2]) < 0.0015
    assert stacked['position'][2] == pytest.approx(0.025 + CUBE.height_m / 2)


def test_a_half_occluded_block_is_refused():
    calibration = camera()
    image = draw(calibration, [(CUBE, 0.26, 0.0, 0.2, 0.0)])
    coloured = np.all(image == COLOURS['red'], axis=2)
    rows, columns = np.nonzero(coloured)
    cut = (columns.min() + columns.max()) // 2
    image[:, cut:][coloured[:, cut:]] = (120, 120, 125)
    candidates = estimate.estimate_block(image, calibration, CUBE)
    assert candidates and not candidates[0]['valid']


def test_two_blocks_of_one_colour_are_both_returned_when_expected():
    calibration = camera()
    image = draw(calibration, [(CUBE, 0.24, -0.06, 0.1, 0.0),
                               (CUBE, 0.27, 0.07, 0.6, 0.0)])
    one = estimate.estimate_block(image, calibration, CUBE, expected_count=1)
    assert sum(entry['valid'] for entry in one) == 1
    two = estimate.estimate_block(image, calibration, CUBE, expected_count=2)
    assert sum(entry['valid'] for entry in two) == 2


def test_an_empty_table_yields_nothing_rather_than_a_guess():
    calibration = camera()
    image = draw(calibration, [])
    assert estimate.estimate_block(image, calibration, CUBE) == []
