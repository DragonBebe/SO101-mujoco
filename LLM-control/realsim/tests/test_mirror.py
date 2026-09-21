"""The simulation half: what a mirror is allowed to do, and what it is not.

Needs MuJoCo and ``so101_nexus``, so it is skipped under the real side's
virtual environment.
"""
import math

import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')
pytest.importorskip('so101_nexus')

from realsim import blocks, geometry, scene, track            # noqa: E402
from realsim.tests.test_track_scene import candidate, catalogue, record_for  # noqa: E402


@pytest.fixture(scope='module')
def simulation(tmp_path_factory):
    from realsim.mirror import MirrorSimulation

    cubes = blocks.load()
    sim = MirrorSimulation(cubes, tmp_path_factory.mktemp('mirror'))
    yield sim
    sim.close()


def record_with(cubes, tracker, poses, now=100.0, at=100.0):
    for block_id, (x, y, yaw_deg) in poses.items():
        block = cubes.by_id(block_id)
        tracker.tracks[block_id].update(
            candidate(x, y, yaw_deg, block), at, tracker.settings)
    return record_for(tracker, now)


def test_a_mirrored_pose_lands_exactly_where_the_observation_put_it(simulation):
    cubes = simulation.catalogue
    tracker = track.Tracker(cubes)
    record = record_with(cubes, tracker, {'red_cube': (0.26, -0.04, 33.0)})
    applied = simulation.apply(record, with_arm=False)
    assert [entry['id'] for entry in applied['placed']] == ['red_cube']
    assert applied['stepped'] is False
    state = {entry['id']: entry for entry in simulation.state()['blocks']}
    np.testing.assert_allclose(state['red_cube']['position_m'][:2], [0.26, -0.04],
                               atol=1e-12)
    assert geometry.quaternion_yaw(state['red_cube']['quaternion_wxyz']) == \
        pytest.approx(math.radians(33.0), abs=1e-9)
    # A block that was never seen is parked, not left somewhere plausible.
    assert state['green_cube']['parked']


def test_a_mirror_never_integrates_physics(simulation):
    cubes = simulation.catalogue
    tracker = track.Tracker(cubes)
    # Deliberately floating: only stepping could move it, and a mirror must not.
    record = record_with(cubes, tracker, {'red_cube': (0.26, 0.0, 0.0)})
    record['objects'][0]['pose']['position_m'][2] = 0.20
    record['objects'][0]['pose_filtered']['position_m'][2] = 0.20
    simulation.apply(record, with_arm=False)
    before = simulation.state()['blocks'][0]['position_m'][2]
    for _ in range(5):
        simulation.apply(record, with_arm=False)
    after = simulation.state()['blocks'][0]['position_m'][2]
    assert before == pytest.approx(0.20) and after == pytest.approx(0.20)
    velocity = simulation.data.qvel[simulation.slots['red_cube'].dof_addr:
                                    simulation.slots['red_cube'].dof_addr + 6]
    assert np.allclose(velocity, 0.0)


def test_a_snapshot_steps_on_its_own_and_reloads(simulation, tmp_path):
    cubes = simulation.catalogue
    tracker = track.Tracker(cubes)
    record = record_with(cubes, tracker, {'red_cube': (0.26, 0.0, 0.0)})
    record['objects'][0]['pose']['position_m'][2] = 0.12
    record['objects'][0]['pose_filtered']['position_m'][2] = 0.12
    simulation.apply(record, with_arm=False)
    path, snapshot = simulation.snapshot(tmp_path / 'snapshot.json')
    assert snapshot['physics_parameters']['source'].startswith('defaults')
    simulation.settle(0.6)
    fallen = simulation.state()['blocks'][0]['position_m'][2]
    assert fallen < 0.05, 'a snapshot is meant to run its own physics'
    simulation.restore(path)
    assert simulation.state()['blocks'][0]['position_m'][2] == pytest.approx(0.12)


def test_a_stale_observation_is_parked_unless_it_is_asked_for(simulation):
    cubes = simulation.catalogue
    tracker = track.Tracker(cubes)
    record = record_with(cubes, tracker, {'red_cube': (0.26, 0.0, 10.0)},
                         now=102.0, at=100.0)
    assert record['objects'][0]['state'] == 'stale'
    applied = simulation.apply(record, with_arm=False)
    assert applied['placed'] == [] and applied['skipped'][0]['id'] == 'red_cube'
    assert simulation.state()['blocks'][0]['parked']
    applied = simulation.apply(record, allow_stale=True, with_arm=False)
    assert [entry['id'] for entry in applied['placed']] == ['red_cube']
    assert applied['placed'][0]['age_s'] == pytest.approx(2.0)


def test_recorded_joint_angles_are_copied_into_the_arm(simulation):
    from nexus_vision.simulation import JOINT_NAMES

    cubes = simulation.catalogue
    tracker = track.Tracker(cubes)
    record = record_with(cubes, tracker, {'red_cube': (0.26, 0.0, 10.0)})
    angles = {name: math.radians(value) for name, value
              in zip(JOINT_NAMES, (12.0, -40.0, 50.0, 20.0, 5.0))}
    record['robot'] = {'joint_angles_rad': angles, 'gripper_fraction': 0.5,
                       'source': 'test', 'read_at': 'test'}
    previous_gripper = simulation.data.qpos[simulation.qadr[5]]
    applied = simulation.apply(record, with_arm=True)
    assert simulation.data.qpos[simulation.qadr[5]] == previous_gripper
    assert 'gripper' not in applied['arm']['applied']
    assert applied['arm']['applied'].keys() >= set(JOINT_NAMES)
    actual = simulation.data.qpos[simulation.qadr[:5]]
    np.testing.assert_allclose(actual, [angles[name] for name in JOINT_NAMES],
                               atol=1e-12)
    # The jaw is not a joint angle in the record; it arrives as a fraction.
    assert applied['arm']['gripper_note'] is None or 'verified' in \
        applied['arm']['gripper_note']


def test_the_reprojection_check_agrees_with_the_observation(simulation):
    cubes = simulation.catalogue
    tracker = track.Tracker(cubes)
    record = record_with(cubes, tracker, {'red_cube': (0.26, -0.03, 20.0)})
    # The observed pixel is where the camera model itself puts that pose.
    from realsim import geometry as geometry_module

    camera = record['camera'] = dict(
        simulation.cameras.calibrated_camera(),
        position=list(simulation.cameras.calibrated_camera()['position']),
        rotation=[list(row) for row in
                  simulation.cameras.calibrated_camera()['rotation']])
    uv, _ = geometry_module.project_points(
        np.array(record['objects'][0]['pose']['position_m']), camera)
    record['objects'][0]['pixel'] = [float(uv[0]), float(uv[1])]
    simulation.apply(record, with_arm=False)
    rows = simulation.reprojection_check(record)
    assert rows and rows[0]['difference_px'] < 1.0


def test_a_rectangular_block_is_refused_rather_than_rounded_to_a_cube():
    from realsim.mirror import scene_objects

    bar = blocks.Block(id='bar', colour='green', size_m=(0.05, 0.025, 0.025),
                       size_source='test', size_measured=True, mass_kg=0.01,
                       mass_source='t', friction=(1, 0, 0), friction_source='t')
    with pytest.raises(ValueError, match='cubes'):
        scene_objects(blocks.Catalogue(blocks=[bar]))


def test_snapshot_restore_keeps_aligned_base(simulation, tmp_path):
    from realsim.arm_sync import align_base
    # Synthetic tilted-base transform: restoration must preserve model state too.
    angle = .1
    transform = [[1,0,0,0], [0,math.cos(angle),-math.sin(angle),0],
                 [0,math.sin(angle),math.cos(angle),-.02], [0,0,0,1]]
    align_base(simulation.model, simulation.data, {'calibration': {'T_base_table': transform}})
    position = simulation.model.body('base').pos.copy()
    rotation = simulation.model.body('base').quat.copy()
    path, _ = simulation.snapshot(tmp_path / 'aligned.json')
    simulation.model.body('base').pos[:] = 0
    simulation.model.body('base').quat[:] = [1,0,0,0]
    simulation.restore(path)
    np.testing.assert_allclose(simulation.model.body('base').pos, position)
    np.testing.assert_allclose(simulation.model.body('base').quat, rotation)
