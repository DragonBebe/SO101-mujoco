"""SO101 Cartesian controls using the existing MuJoCo scene and real contacts."""
from pathlib import Path
import math

import mujoco
import numpy as np
from scipy.optimize import least_squares

SCENE = Path(__file__).resolve().parents[1] / 'manipulator_grasp/assets/SO101/scene_table_cubes.xml'
OBJECTS = ('RedBox', 'GreenBox', 'BlueBox')


class Simulation:
    def __init__(self, on_step=None):
        spec = mujoco.MjSpec.from_file(str(SCENE))
        # A single convex hull of the concave finger assembly fills the gap.
        # Replace only those two collision hulls with finger-sized boxes;
        # retain original visual meshes, motor collisions and inertial data.
        fixed = spec.body('gripper')
        fixed.geoms[-1].contype = fixed.geoms[-1].conaffinity = 0
        moving = spec.body('moving_jaw_so101_v1')
        moving.geoms[-1].contype = moving.geoms[-1].conaffinity = 0
        for body, name, pos, size in [
            (fixed, 'fixed_finger_pad', [-0.0129, -0.0002, -0.09], [0.005, 0.0075, 0.014]),
            (moving, 'moving_finger_pad', [-0.0073, -0.0666, 0.019], [0.005, 0.014, 0.0075]),
        ]:
            body.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, pos=pos, size=size,
                          contype=1, conaffinity=1, condim=6, friction=[1.5, 0.005, 0.0001],
                          solref=[0.004, 1], solimp=[0.95, 0.99, 0.001, 0.5, 2],
                          rgba=[0.15, 0.15, 0.15, 1], group=3)
        self.model = spec.compile()
        m = self.model
        # The source weld encodes the table-external original base position.
        m.eq_data[m.equality('attach_so101_to_mocap').id, 3:6] = 0
        m.opt.solver = mujoco.mjtSolver.mjSOL_NEWTON
        m.opt.iterations = 100
        self.data = mujoco.MjData(m)
        self.ik_data = mujoco.MjData(m)
        self.qadr = np.array([m.joint(str(i)).qposadr[0] for i in range(1, 7)])
        self.dadr = np.array([m.joint(str(i)).dofadr[0] for i in range(1, 7)])
        self.aids = np.array([m.actuator(str(i)).id for i in range(1, 7)])
        self.limits = np.array([m.joint(str(i)).range for i in range(1, 6)])
        self.site = m.site('gripper').id
        # Source site lies on the fixed finger's inner edge, not in the gap.
        # Center a 30 mm cube between that edge and the opposing moving jaw.
        m.site_pos[self.site] = [0.0071, -0.000218121, -0.095]
        self.gripper_body = m.body('gripper').id
        self.jaw_bodies = {self.gripper_body, m.body('moving_jaw_so101_v1').id}
        # Better tracking with damping; preserve the actuator torque limits.
        m.actuator_gainprm[self.aids[:5], 0] = 80
        m.actuator_biasprm[self.aids[:5], 1] = -80
        m.actuator_biasprm[self.aids[:5], 2] = -4
        m.actuator_forcerange[self.aids[5]] = [-0.5, 0.5]
        for name in OBJECTS:
            m.geom_solref[m.geom(name).id] = [0.004, 1]
        self.target = np.zeros(6)
        self.on_step = on_step
        self.goal = np.array([1.16, 0.79, 0.755])
        m.body_pos[m.body('zone_drop').id] = [*self.goal[:2], 0.741]
        m.geom_size[m.geom('zone_drop').id] = [0.03, 0.03, 0.001]
        m.geom_rgba[m.geom('zone_drop').id] = [1, 0.6, 0, 0.45]
        self.reset()

    def reset(self):
        m, d = self.model, self.data
        mujoco.mj_resetData(m, d)
        d.mocap_pos[0] = [1.0, 0.6, 0.71]
        d.qpos[:3] = d.mocap_pos[0]
        d.qpos[3:7] = [math.sqrt(0.5), 0, 0, math.sqrt(0.5)]
        self.target[:] = [0, -1.5, 0, 0, 0, 1.0]
        d.qpos[self.qadr] = self.target
        # Reset-only tabletop layout: avoid dropping the source's stack through
        # the arm. The same three free bodies and meshes remain in use.
        for name, position in zip(OBJECTS, ([1.2, 0.6, 0.755], [1.29, 0.7, 0.755], [1.29, 0.8, 0.755])):
            adr = m.joint(name).qposadr[0]
            d.qpos[adr:adr+3] = position
        self.initial_positions = {}
        self.max_lift = {name: 0.0 for name in OBJECTS}
        self.lift_with_contact = {name: False for name in OBJECTS}
        self.step_count = 0
        mujoco.mj_forward(m, d)
        self._step(1500)
        self.initial_positions = {name: d.body(name).xpos.copy() for name in OBJECTS}
        return self.observe()

    def _step(self, count):
        m, d = self.model, self.data
        for _ in range(count):
            ctrl = self.target.copy()
            ctrl[:5] += d.qfrc_bias[self.dadr[:5]] / 80
            d.ctrl[self.aids] = np.clip(ctrl, m.actuator_ctrlrange[self.aids, 0], m.actuator_ctrlrange[self.aids, 1])
            mujoco.mj_step(m, d)
            self.step_count += 1
            if not np.isfinite(d.qpos).all() or np.any(d.warning.number):
                raise RuntimeError('MuJoCo reported unstable simulation')
            for name, initial in self.initial_positions.items():
                height = float(d.body(name).xpos[2] - initial[2])
                self.max_lift[name] = max(self.max_lift[name], height)
                if height >= 0.04 and self._contacts(name):
                    self.lift_with_contact[name] = True
            if self.on_step and self.step_count % 20 == 0:
                self.on_step(self)
        mujoco.mj_forward(m, d)

    def _contacts(self, name):
        bid = self.model.body(name).id
        contacts = set()
        for contact in self.data.contact:
            if contact.dist > 0.0005:
                continue
            bodies = {int(self.model.geom_bodyid[g]) for g in contact.geom}
            if bid in bodies:
                contacts.update(self.model.body(b).name for b in bodies & self.jaw_bodies)
        return sorted(contacts)

    def solve_ik(self, position):
        target = np.asarray(position, dtype=float)
        if target.shape != (3,) or not np.isfinite(target).all():
            raise ValueError('position must contain three finite coordinates')
        if not (0.65 <= target[0] <= 1.45 and 0.25 <= target[1] <= 0.95 and 0.755 <= target[2] <= 1.25):
            raise ValueError('Target outside workspace')
        m, scratch = self.model, self.ik_data
        scratch.qpos[:] = self.data.qpos
        scratch.mocap_pos[:] = self.data.mocap_pos
        scratch.mocap_quat[:] = self.data.mocap_quat

        def residual(q):
            scratch.qpos[self.qadr[:5]] = q
            mujoco.mj_forward(m, scratch)
            # Body -Z points down into the object; yaw remains unconstrained.
            z_axis = scratch.xmat[self.gripper_body].reshape(3, 3)[:, 2]
            return np.r_[scratch.site_xpos[self.site] - target, 0.1 * (z_axis - [0, 0, 1]), 0.1*q[4]]

        seeds = [self.target[:5], [0, 0.4, 0.6, 1.0, 0], [0, -0.8, 1.0, 1.0, 0]]
        best = None
        for seed in seeds:
            sol = least_squares(residual, np.clip(seed, self.limits[:, 0]+1e-7, self.limits[:, 1]-1e-7),
                                bounds=(self.limits[:, 0], self.limits[:, 1]),
                                max_nfev=150, ftol=1e-10, xtol=1e-10, gtol=1e-10)
            error = np.linalg.norm(residual(sol.x))
            if best is None or error < best[0]:
                best = (error, sol.x)
            if error < 0.0002:
                break
        if best[0] > 0.002:
            raise ValueError(f'IK target not reachable with downward gripper (error {best[0]:.4f} m)')
        return best[1].copy()

    @staticmethod
    def _number(value, low, high, label):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f'{label} must be a finite number in [{low}, {high}]')
        return float(value)

    def execute(self, command):
        if not isinstance(command, dict):
            raise ValueError('Command must be a JSON object')
        action = command.get('action')
        if action == 'observe':
            return self.observe()
        if action == 'reset':
            return self.reset()
        if action == 'wait':
            seconds = self._number(command.get('seconds', 1), 0.01, 5, 'seconds')
            self._step(round(seconds / self.model.opt.timestep))
        elif action == 'gripper':
            opening = self._number(command.get('opening'), 0, 1, 'opening')
            seconds = self._number(command.get('seconds', 1.2), 0.2, 5, 'seconds')
            self.target[5] = -0.15 + 1.35 * opening
            self._step(round(seconds / self.model.opt.timestep))
        elif action == 'move':
            seconds = self._number(command.get('seconds', 2.0), 0.3, 8, 'seconds')
            end = self.solve_ik(command.get('position'))
            start = self.target[:5].copy()
            count = round(seconds / self.model.opt.timestep)
            for i in range(count):
                t = (i+1)/count
                self.target[:5] = start + (end-start) * (t*t*(3-2*t))
                self._step(1)
            self._step(250)
        else:
            raise ValueError(f'Unknown action: {action!r}')
        return self.observe()

    def observe(self):
        m, d = self.model, self.data
        objects = {}
        for name in OBJECTS:
            position = d.body(name).xpos.copy()
            dof = m.joint(name).dofadr[0]
            velocity = float(np.linalg.norm(d.qvel[dof:dof+3]))
            objects[name] = {'position': position.tolist(), 'speed': velocity,
                             'gripper_contacts': self._contacts(name), 'max_lift': self.max_lift[name]}
        red = objects['RedBox']
        pos = np.array(red['position'])
        displaced = np.linalg.norm(pos[:2] - self.initial_positions.get('RedBox', pos)[:2])
        success = (self.lift_with_contact['RedBox'] and displaced > 0.08 and
                   np.linalg.norm(pos[:2] - self.goal[:2]) < 0.02 and abs(pos[2] - self.goal[2]) < 0.01 and
                   red['speed'] < 0.01 and not red['gripper_contacts'] and self.target[5] > 0.7)
        return {'time': float(d.time), 'end_effector': d.site_xpos[self.site].tolist(),
                'joints': d.qpos[self.qadr].tolist(), 'objects': objects,
                'goal': self.goal.tolist(), 'success': bool(success)}

    def snapshot(self, path):
        camera = mujoco.MjvCamera()
        camera.lookat[:] = [1.08, 0.65, 0.88]
        camera.distance = 0.85
        camera.azimuth = 135
        camera.elevation = -30
        import imageio.v3 as iio
        with mujoco.Renderer(self.model, height=720, width=960) as renderer:
            renderer.update_scene(self.data, camera=camera)
            iio.imwrite(path, renderer.render())
        return str(path)
