"""Identity, ageing and what a scene record is allowed to hand the simulator."""
import math

import numpy as np
import pytest

from realsim import blocks, geometry, scene, track
from realsim.tests.test_estimate import CUBE


def catalogue(*items):
    return blocks.Catalogue(blocks=list(items or (CUBE,)),
                            tolerances={'position_mm': 5.0, 'yaw_deg': 10.0})


def candidate(x, y, yaw_deg, block=CUBE, iou=0.95):
    yaw = geometry.canonical_yaw(math.radians(yaw_deg), block.symmetry_step_rad)
    return {'block_id': block.id, 'valid': True, 'reasons': [],
            'position': [x, y, block.half_extent[2]], 'yaw_rad': yaw,
            'yaw_deg': math.degrees(yaw),
            'symmetry_step_deg': math.degrees(block.symmetry_step_rad),
            'quaternion_wxyz': geometry.yaw_quaternion(yaw).tolist(),
            'quality': {'iou': iou, 'area_ratio': 1.0, 'mask_area_px': 2000.0,
                        'model_area_px': 2000.0},
            'pixel': [100, 100], 'bbox': [90, 90, 110, 110], 'outline_uv': None,
            'measured': ['x', 'y', 'yaw'], 'assumed': {'support_plane_z_m': 0.0},
            'method': 'test', 'observed_at': '2026-09-21T12:00:00+02:00'}


def record_for(tracker, now, rejected=()):
    return scene.build(tracker, now,
                       {'captured_at': '2026-09-21T12:00:00+02:00', 'frame_index': 1,
                        'camera': {'width': 640, 'height': 480},
                        'to_base': lambda point: np.asarray(point) + [0.0, 0.0, 0.01]},
                       {'usable_for_motion': False}, rejected)


def test_a_pose_ages_from_detected_through_stale_to_lost():
    tracker = track.Tracker(catalogue())
    tracker.tracks[CUBE.id].update(candidate(0.25, 0.0, 10), 100.0, tracker.settings)
    states = [tracker.tracks[CUBE.id].state(time, tracker.settings)[0]
              for time in (100.1, 101.0, 110.0)]
    assert states == ['detected', 'stale', 'lost']


def test_a_stale_pose_keeps_the_time_it_was_measured():
    tracker = track.Tracker(catalogue())
    tracker.tracks[CUBE.id].update(candidate(0.25, 0.0, 10), 100.0, tracker.settings)
    record = record_for(tracker, 101.5)
    entry = record['objects'][0]
    assert entry['state'] == 'stale' and entry['fresh'] is False
    assert entry['observed_at'] == '2026-09-21T12:00:00+02:00'
    assert entry['age_s'] == pytest.approx(1.5)
    assert 'not a current measurement' in entry['warning']


def test_a_stale_or_lost_pose_is_refused_by_the_simulator_unless_asked_for():
    tracker = track.Tracker(catalogue())
    tracker.tracks[CUBE.id].update(candidate(0.25, 0.0, 10), 100.0, tracker.settings)
    stale = record_for(tracker, 101.5)
    assert scene.placements(stale)[0] == []
    assert scene.placements(stale)[1][0]['reason'].startswith('state is stale')
    allowed, _ = scene.placements(stale, allow_stale=True)
    assert len(allowed) == 1 and allowed[0]['age_s'] == pytest.approx(1.5)
    lost = record_for(tracker, 130.0)
    assert scene.placements(lost, allow_stale=True)[0] == []


def test_the_filter_smooths_noise_but_jumps_with_the_block():
    tracker = track.Tracker(catalogue())
    now = 100.0
    for _ in range(6):
        tracker.tracks[CUBE.id].update(candidate(0.250, 0.0, 10), now,
                                       tracker.settings)
        now += 0.1
    assert tracker.tracks[CUBE.id].filtered_position[0] == pytest.approx(0.250)
    # One noisy frame is damped, not followed.
    tracker.tracks[CUBE.id].update(candidate(0.253, 0.0, 10), now, tracker.settings)
    assert 0.250 < tracker.tracks[CUBE.id].filtered_position[0] < 0.253
    # A real move resets it instead of dragging a trail behind the block.
    reset = tracker.tracks[CUBE.id].update(candidate(0.320, 0.05, 10), now + 0.1,
                                           tracker.settings)
    assert reset and tracker.tracks[CUBE.id].filtered_position[0] == pytest.approx(0.320)


def test_yaw_is_filtered_around_its_own_symmetry():
    tracker = track.Tracker(catalogue())
    now = 100.0
    for yaw in (89.0, 0.5, 89.5, 0.2):
        tracker.tracks[CUBE.id].update(candidate(0.25, 0.0, yaw), now,
                                       tracker.settings)
        now += 0.1
    filtered = math.degrees(tracker.tracks[CUBE.id].filtered_yaw)
    # The average of these is 0/90, never the 45 an arithmetic mean gives.
    assert filtered > 85.0 or filtered < 5.0


def test_two_blocks_of_one_colour_keep_their_identities_when_they_move():
    second = blocks.Block(id='red_cube_b', colour='red', size_m=CUBE.size_m,
                          size_source='test', size_measured=True, mass_kg=0.01,
                          mass_source='t', friction=(1, 0, 0), friction_source='t')
    tracker = track.Tracker(catalogue(CUBE, second))
    first = tracker._assign([CUBE, second],
                            [candidate(0.22, -0.05, 10), candidate(0.30, 0.05, 40)])
    assert set(first) == {CUBE.id, second.id}
    for block_id, item in first.items():
        tracker.tracks[block_id].update(item, 100.0, tracker.settings)
    # Both shift a little; each must stay with the track it was nearest to.
    again = tracker._assign([CUBE, second],
                            [candidate(0.305, 0.052, 41), candidate(0.222, -0.048, 11)])
    assert again[CUBE.id]['position'][0] == pytest.approx(
        first[CUBE.id]['position'][0], abs=0.01)
    assert again[second.id]['position'][0] == pytest.approx(
        first[second.id]['position'][0], abs=0.01)
    assert all(entry['association']['method'] == 'nearest previous pose'
               for entry in again.values())


def test_first_sight_of_identical_blocks_says_it_is_a_convention():
    second = blocks.Block(id='red_cube_b', colour='red', size_m=CUBE.size_m,
                          size_source='test', size_measured=True, mass_kg=0.01,
                          mass_source='t', friction=(1, 0, 0), friction_source='t')
    tracker = track.Tracker(catalogue(CUBE, second))
    assignment = tracker._assign([CUBE, second],
                                 [candidate(0.22, -0.05, 10), candidate(0.30, 0.05, 40)])
    assert all('convention' in entry['association'].get('note', '')
               for entry in assignment.values())


def test_the_record_states_what_was_measured_and_what_was_assumed():
    tracker = track.Tracker(catalogue())
    tracker.tracks[CUBE.id].update(candidate(0.25, 0.0, 10), 100.0, tracker.settings)
    entry = record_for(tracker, 100.0)['objects'][0]
    assert entry['measured'] == ['x', 'y', 'yaw']
    assert entry['assumed']['support_plane_z_m'] == 0.0
    assert entry['pose_base']['position_m'][2] == pytest.approx(
        entry['pose']['position_m'][2] + 0.01)
    assert entry['pose']['symmetry_step_deg'] == pytest.approx(90.0)


def test_a_never_seen_block_has_no_pose_at_all():
    tracker = track.Tracker(catalogue())
    entry = record_for(tracker, 100.0)['objects'][0]
    assert entry['state'] == 'lost' and entry['pose'] is None
    assert scene.placements(record_for(tracker, 100.0))[0] == []


def contact_record(distance_m, yaw_deg=0.0, lateral_m=0.0, second_yaw_deg=None,
                   state='detected'):
    """Two blocks of the catalogue's size, placed a stated distance apart."""
    second = blocks.Block(id='cube_b', colour='blue', size_m=CUBE.size_m,
                          size_source='t', size_measured=True, mass_kg=0.01,
                          mass_source='t', friction=(1, 0, 0), friction_source='t')
    tracker = track.Tracker(catalogue(CUBE, second))
    yaw = math.radians(yaw_deg)
    offset = np.array([math.cos(yaw), math.sin(yaw)]) * distance_m
    offset += np.array([-math.sin(yaw), math.cos(yaw)]) * lateral_m
    tracker.tracks[CUBE.id].update(candidate(0.25, 0.0, yaw_deg), 100.0,
                                   tracker.settings)
    tracker.tracks['cube_b'].update(
        candidate(0.25 + offset[0], 0.0 + offset[1],
                  yaw_deg if second_yaw_deg is None else second_yaw_deg, second),
        100.0 if state == 'detected' else 90.0, tracker.settings)
    return tracker.catalogue, record_for(tracker, 100.0)


def test_two_flush_blocks_one_edge_apart_pass_the_scale_check():
    cubes, record = contact_record(CUBE.size_m[0])
    report = scene.contact_check([record], CUBE.id, 'cube_b', cubes)
    assert report['passed']
    assert report['expected_mm'] == pytest.approx(1000 * CUBE.size_m[0])
    assert report['error_mm'] == pytest.approx(0.0, abs=1e-6)


def test_the_scale_check_reports_a_real_error_in_millimetres():
    """The failure this exists to catch: a stable estimate that is stably wrong."""
    cubes, record = contact_record(CUBE.size_m[0] * 1.04, yaw_deg=23.0)
    report = scene.contact_check([record], CUBE.id, 'cube_b', cubes)
    assert not report['passed']
    assert report['error_mm'] == pytest.approx(0.04 * 1000 * CUBE.size_m[0], abs=0.01)
    assert report['error_percent'] == pytest.approx(4.0, abs=0.05)


def test_blocks_that_are_not_flush_are_reported_as_a_placement_failure():
    cubes, record = contact_record(CUBE.size_m[0], lateral_m=0.010)
    report = scene.contact_check([record], CUBE.id, 'cube_b', cubes)
    assert not report['blocks_were_flush'] and not report['passed']
    assert report['lateral_mm'] == pytest.approx(10.0, abs=0.01)
    # The offset is reported as an offset, not counted as scale error.
    assert report['error_mm'] == pytest.approx(0.0, abs=1e-6)


def test_the_check_measures_along_the_contact_normal_not_the_straight_line():
    cubes, record = contact_record(CUBE.size_m[0], yaw_deg=40.0, lateral_m=0.004)
    report = scene.contact_check([record], CUBE.id, 'cube_b', cubes)
    straight = math.hypot(1000 * CUBE.size_m[0], 4.0)
    assert report['measured_mm'] == pytest.approx(1000 * CUBE.size_m[0], abs=0.01)
    assert straight > report['measured_mm'] + 0.2


def test_a_stale_block_is_not_used_for_the_scale_check():
    cubes, record = contact_record(CUBE.size_m[0], state='stale')
    assert scene.contact_check([record], CUBE.id, 'cube_b', cubes)['frames'] == 0
