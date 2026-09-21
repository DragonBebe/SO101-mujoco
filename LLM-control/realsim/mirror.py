"""The simulation half: a MuJoCo scene driven by the real observation.

Two different things are built here and they are deliberately not the same
object, because confusing them is how a "digital twin" quietly starts
inventing its own facts:

Mirror
    The virtual blocks are *set* from the latest valid observation and the
    world is only re-derived, never integrated.  Velocities are zeroed and
    ``mj_step`` is never called, so nothing in the simulation can move a
    block away from where the camera says it is.  A block whose track is
    lost -- or stale, unless the caller opts in -- is parked off-world
    instead of being left at an old pose that would look current.

Snapshot
    The same state, taken once, in a simulation that then steps on its own.
    It is where a planned action may be tried.  It never writes back: the
    real side of this package reads the camera and the servo bus and nothing
    here can change either.

The scene reuses the existing ``nexus_vision`` simulation rather than
building a second one, so the robot model, the renderer, the camera suite
and the calibrated viewpoint are the ones already in use.  The model is
compiled once; applying an observation writes free-joint ``qpos`` and calls
``mj_forward``, which is what keeps a 1 Hz update from recompiling a scene.
"""
from contextlib import nullcontext
from pathlib import Path
import json

import mujoco
import numpy as np
from so101_nexus.objects import CubeObject

from nexus_vision.cameras import CameraSuite
from nexus_vision.simulation import VisualSimulation, JOINT_NAMES

from . import scene as scene_module

#: Where a block goes when its real counterpart is not currently observed.
PARKED = (0.0, 0.0, -10.0)


def scene_objects(catalogue):
    """``so101_nexus`` objects for a catalogue, in catalogue order.

    Only cubes for now: the upstream primitives take a single half-size, so a
    block with unequal sides has no body to mirror onto and is refused rather
    than silently rounded to a cube.
    """
    objects = []
    for block in catalogue.blocks:
        if not block.is_cube:
            raise ValueError(
                f'{block.id} is {1000 * block.size_m[0]:.1f} x '
                f'{1000 * block.size_m[1]:.1f} x {1000 * block.size_m[2]:.1f} mm. '
                'The simulation primitives available here are cubes; mirroring a '
                'rectangular block needs a box body added to the scene builder.')
        objects.append(CubeObject(half_size=float(block.half_extent[0]),
                                  mass=float(block.mass_kg), color=block.colour))
    return objects


class MirrorSimulation(VisualSimulation):
    """A tabletop whose blocks are the catalogue's, placed by observation.

    The world is the measured table frame, which is what the calibrated
    camera export already assumes, so an observed pose is applied unchanged.
    """

    def __init__(self, catalogue, directory, cameras=None, viewer=False,
                 realtime=False, camera_viewer=False, seed=0):
        self.catalogue = catalogue
        objects = scene_objects(catalogue)
        super().__init__(
            'PickLift', directory, seed=seed, viewer=viewer, realtime=realtime,
            camera_viewer=camera_viewer,
            config_overrides={'objects': objects,
                              'n_distractors': len(objects) - 1,
                              'min_object_separation': 0.02,
                              'reset_settle_frames': 20},
            cameras=cameras or CameraSuite(modality='rgb', placement='calibrated'))
        # config.objects and env._slots share an order, so a catalogue entry
        # and its body are matched by index, not by searching for a colour.
        self.slots = {block.id: slot for block, slot
                      in zip(catalogue.blocks, self.env._slots)}
        self.paint()
        self.applied = None

    def paint(self):
        """Render each block in its own colour rather than a canonical one.

        ``so101_nexus`` names colours from a fixed palette, which is fine for
        a simulated scene but wrong for a mirror: a real teal cube rendered
        as pure green is a cube the real colour band would not find, so a
        self-test on that render would pass while the real detector failed.
        The rendered colour is taken from the block's own band.
        """
        for block in self.catalogue.blocks:
            rgba = list(block.colour_spec.representative_rgb()) + [1.0]
            self.model.geom_rgba[list(self.slots[block.id].geom_ids)] = rgba

    def park(self, block_id):
        slot = self.slots[block_id]
        self.data.qpos[slot.qpos_addr:slot.qpos_addr + 3] = PARKED
        self.data.qpos[slot.qpos_addr + 3:slot.qpos_addr + 7] = [1.0, 0.0, 0.0, 0.0]
        self.data.qvel[slot.dof_addr:slot.dof_addr + 6] = 0.0

    def place(self, block_id, position, quaternion):
        slot = self.slots[block_id]
        self.data.qpos[slot.qpos_addr:slot.qpos_addr + 3] = np.asarray(position, float)
        self.data.qpos[slot.qpos_addr + 3:slot.qpos_addr + 7] = np.asarray(
            quaternion, float)
        self.data.qvel[slot.dof_addr:slot.dof_addr + 6] = 0.0

    def apply(self, record, use_filtered=True, allow_stale=False, with_arm=True):
        """Set the scene from one observation record.  No physics is stepped.

        Returns what was placed and what was refused, so a caller can report
        an incomplete mirror instead of showing a plausible-looking scene.
        """
        placed, skipped = scene_module.placements(record, use_filtered, allow_stale)
        arm = None
        with self.viewer.lock() if self.viewer else nullcontext():
            if record.get('calibration', {}).get('T_base_table') is not None:
                from .arm_sync import align_base
                align_base(self.model, self.data, record)
            for entry in skipped:
                self.park(entry['id'])
            for entry in placed:
                self.place(entry['id'], entry['position_m'], entry['quaternion_wxyz'])
            if with_arm:
                arm = self.apply_arm(record.get('robot'))
            # Derived quantities only: positions, frames, contacts and the
            # renderer's view of them.  Integration would move the blocks.
            mujoco.mj_forward(self.model, self.data)
        if self.viewer:
            self.viewer.sync()
        self.applied = {'placed': placed, 'skipped': skipped, 'arm': arm,
                        'from_frame_index': record.get('frame_index'),
                        'captured_at': record.get('captured_at'),
                        'stepped': False,
                        'note': 'mirror: state is set from the observation and only '
                                'mj_forward is called, so the simulation cannot move '
                                'a block away from where the camera saw it'}
        return self.applied

    def apply_arm(self, robot):
        """Copy the real joint angles, if the record carries any.

        The five arm joints share their order and sign convention with the
        hand-eye solution, so they transfer directly.  The jaw does not: what
        travels is a servo reading, and turning it into the model's joint
        angle requires an explicit gripper_angle_rad. Legacy unverified
        gripper_fraction is never applied.
        """
        if not robot or not robot.get('joint_angles_rad'):
            return None
        angles = robot['joint_angles_rad']
        applied = {}
        for index, name in enumerate(JOINT_NAMES):
            if name in angles:
                self.data.qpos[self.qadr[index]] = float(angles[name])
                applied[name] = float(angles[name])
        gripper = robot.get('gripper_angle_rad')
        if gripper is not None:
            low, high = float(self.low[5]), float(self.high[5])
            value = float(gripper)
            if not np.isfinite(value) or not low <= value <= high:
                raise ValueError('gripper_angle_rad outside model range')
            self.data.qpos[self.qadr[5]] = value
            applied['gripper'] = value
        self.data.qvel[self.dadr] = 0.0
        self.target = self.data.qpos[self.qadr].copy()
        return {'applied': applied, 'source': robot.get('source'),
                'read_at': robot.get('read_at'),
                'gripper_note': robot.get('gripper_note')}

    def settle(self, seconds):
        """Step the simulation on its own -- snapshot use only, never mirror."""
        steps = int(round(float(seconds) / float(self.model.opt.timestep)))
        for _ in range(steps):
            mujoco.mj_step(self.model, self.data)
        if self.viewer:
            self.viewer.sync()
        if self.applied:
            self.applied['stepped'] = True
        return {'seconds': float(seconds), 'steps': steps}

    def state(self):
        """Where the virtual blocks actually are, read back from MuJoCo."""
        entries = []
        for block in self.catalogue.blocks:
            slot = self.slots[block.id]
            qpos = self.data.qpos[slot.qpos_addr:slot.qpos_addr + 7].copy()
            entries.append({'id': block.id,
                            'position_m': qpos[:3].tolist(),
                            'quaternion_wxyz': qpos[3:].tolist(),
                            'parked': bool(qpos[2] < -1.0)})
        return {'time_s': float(self.data.time), 'blocks': entries,
                'joints_deg': np.degrees(self.data.qpos[self.qadr[:5]]).tolist()}

    def reprojection_check(self, record):
        """Where the mirrored blocks land in the simulated camera, in pixels.

        Both sides use the same camera model, so this is a consistency check
        on the mapping, not a measurement of accuracy: it catches a pose
        applied to the wrong body, a quaternion in the wrong order or a frame
        confused with another, and it says nothing about whether the block is
        really there.  A small reprojection error with a large world error is
        exactly the trap this project keeps warning about.
        """
        from . import geometry

        name = self.cameras.environment_camera
        renderer, camera, fov = self._camera(name)
        with self._render_fov(fov):
            renderer.disable_depth_rendering()
            renderer.update_scene(self.data, camera=camera)
            if name == 'calibrated':
                self._apply_calibrated_camera(renderer)
            calibration = self._calibration(
                renderer, (self.cameras.resolution(name)['height'],
                           self.cameras.resolution(name)['width']))
        scale = calibration['width'] / float(record['camera']['width'])
        rows = []
        for entry in record.get('objects', []):
            if entry.get('pose') is None or entry['id'] not in self.slots:
                continue
            slot = self.slots[entry['id']]
            position = self.data.qpos[slot.qpos_addr:slot.qpos_addr + 3]
            if position[2] < -1.0:
                continue
            uv, depth = geometry.project_points(np.asarray(position), calibration)
            if depth <= 0:
                continue
            observed = np.asarray(entry['pixel'], dtype=float) * scale
            rows.append({
                'id': entry['id'],
                'observed_pixel_scaled': observed.tolist(),
                'mirrored_pixel': uv.tolist(),
                'difference_px': float(np.linalg.norm(uv - observed)),
                'image_scale': scale,
                'note': ('the observed pixel is the colour region\'s representative '
                         'pixel, not the projected centre, so a few pixels of '
                         'difference is expected even when the mapping is exact')})
        return rows

    def snapshot(self, path):
        """Write a MuJoCo keyframe-style state that can be reloaded and stepped."""
        record = {'schema': 'realsim/snapshot/1',
                  'written_at': scene_module.timestamp(),
                  'catalogue': self.catalogue.describe(),
                  'world_frame': scene_module.FRAMES['table'],
                  'applied': self.applied,
                  'state': self.state(),
                  'base_pose': {'position': self.model.body('base').pos.tolist(),
                                'quaternion': self.model.body('base').quat.tolist()},
                  'qpos': self.data.qpos.tolist(),
                  'qvel': self.data.qvel.tolist(),
                  'ctrl': self.data.ctrl.tolist(),
                  'time_s': float(self.data.time),
                  'note': ('a simulation initialised from the observation; stepping '
                           'it predicts, it does not observe, and nothing here is '
                           'written back to the real side'),
                  'physics_parameters': {
                      'source': 'defaults, not identified from the real blocks',
                      'blocks': [{'id': block.id, 'mass_kg': block.mass_kg,
                                  'mass_source': block.mass_source,
                                  'friction': list(block.friction),
                                  'friction_source': block.friction_source}
                                 for block in self.catalogue.blocks]}}
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(record, indent=2, ensure_ascii=False) + '\n')
        return path, record

    def restore(self, snapshot_path):
        """Reload a snapshot into this simulation so it can be stepped again."""
        record = json.loads(Path(snapshot_path).read_text())
        if record.get('schema') != 'realsim/snapshot/1':
            raise ValueError(f'{snapshot_path} is not a realsim snapshot')
        for name, values in (('qpos', record['qpos']), ('qvel', record['qvel']),
                             ('ctrl', record['ctrl'])):
            target = getattr(self.data, name)
            values = np.asarray(values, dtype=float)
            if values.shape != target.shape:
                raise ValueError(
                    f'{snapshot_path} has {values.size} {name} values but this model '
                    f'has {target.size}: it was taken with a different catalogue')
            target[:] = values
        if 'base_pose' in record:
            base = record['base_pose']
            position = np.asarray(base['position'], dtype=float)
            quaternion = np.asarray(base['quaternion'], dtype=float)
            if (position.shape != (3,) or quaternion.shape != (4,)
                    or not np.isfinite(position).all() or not np.isfinite(quaternion).all()
                    or not np.isclose(np.linalg.norm(quaternion), 1.)):
                raise ValueError('invalid snapshot base pose')
            self.model.body('base').pos[:] = position
            self.model.body('base').quat[:] = quaternion
        self.data.time = float(record.get('time_s', 0.0))
        mujoco.mj_forward(self.model, self.data)
        self.target = self.data.qpos[self.qadr].copy()
        if self.viewer:
            self.viewer.sync()
        return record

    def render(self, path):
        """Save the environment camera's view of the mirrored scene."""
        from PIL import Image

        name = self.cameras.environment_camera
        rgb, _, calibration = self.capture_cameras()[name]
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(rgb).save(path)
        return path, calibration
