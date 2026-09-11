"""Camera/proprioception boundary and bounded low-level robot controls."""
from pathlib import Path
import json
import math
import os
import time
import uuid

import mujoco
import numpy as np
from PIL import Image
from scipy.optimize import least_squares
from so101_nexus import (
    PickConfig, PickAndPlaceConfig, PickAndPlaceV2Config, StackCubeConfig,
    TouchConfig, LookAtConfig, MoveConfig, RobotConfig,
    JointPositions, OverheadCamera, WristCamera,
)
from so101_nexus.mujoco.pick_env import PickLiftEnv
from so101_nexus.mujoco.pick_and_place import PickAndPlaceEnv, PickAndPlaceV2Env
from so101_nexus.mujoco.stack_cube import StackCubeEnv
from so101_nexus.mujoco.touch_env import TouchEnv
from so101_nexus.mujoco.look_at_env import LookAtEnv
from so101_nexus.mujoco.move_env import MoveEnv
from .perception import unproject_pixel, locate_color

TASKS = {
    'Touch': (TouchConfig, TouchEnv), 'LookAt': (LookAtConfig, LookAtEnv),
    'Move': (MoveConfig, MoveEnv), 'PickLift': (PickConfig, PickLiftEnv),
    'PickAndPlace': (PickAndPlaceConfig, PickAndPlaceEnv),
    'PickAndPlace-v2': (PickAndPlaceV2Config, PickAndPlaceV2Env),
    'StackCube': (StackCubeConfig, StackCubeEnv),
}


def number(value, low, high, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{name} must be finite and in [{low}, {high}]')
    return float(value)


class VisualSimulation:
    def __init__(self, task, directory, seed=4, viewer=False, realtime=False, camera_viewer=False,
                 config_overrides=None):
        if camera_viewer and os.environ.get('MUJOCO_GL', 'glfw').lower() != 'glfw':
            raise ValueError('Use MUJOCO_GL=glfw with --camera-viewer')
        if task not in TASKS:
            raise ValueError(f'Unknown task: {task}')
        self.task, self.seed = task, seed
        self.directory = Path(directory).resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.episode_token = uuid.uuid4().hex[:16]
        self.frame_index = 0
        self.frame_id = ''
        self.frames = {}
        self.points = {}
        self.viewer = None
        self.camera_viewer = None
        self.realtime = realtime
        config_cls, env_cls = TASKS[task]
        config_options = dict(
            spawn_center=(0.20, 0) if task == 'LookAt' else (0, 0),
            spawn_half_size=0.03, spawn_min_radius=0.17, spawn_max_radius=0.23,
            spawn_angle_half_range_deg=35, robot_init_qpos_noise=0,
            robot=RobotConfig(rest_qpos_deg=(70, -85, 85, 30, 0, 70)),
            terminate_on_success=False, reset_settle_frames=50,
            obs_mode='visual', observations=[JointPositions(), OverheadCamera(),
                WristCamera(fov_deg_range=(60,60), pitch_deg_range=(-32.66,-32.66),
                            pos_x_noise=0, pos_y_center=0.055, pos_y_noise=0,
                            pos_z_center=-0.045, pos_z_noise=0)],
        )
        config_options.update(config_overrides or {})
        config = config_cls(**config_options)
        self.env = env_cls(config=config, robot_init_qpos_noise=0)
        self.env.reset(seed=seed)  # Upstream private info is intentionally discarded.
        self.model, self.data = self.env.model, self.env.data
        # Move resets its goal site after settling. Refresh derived transforms
        # before the first RGB-D frame, including when no viewer is running.
        mujoco.mj_forward(self.model, self.data)
        self.qadr, self.dadr = self.env._qpos_addrs, self.env._qvel_addrs
        self.low, self.high = self.env._target_low, self.env._target_high
        self.target = self.data.qpos[self.qadr].copy()
        self.site = self.model.site('gripperframe').id
        self.scratch = mujoco.MjData(self.model)
        if viewer:
            import mujoco.viewer as mjviewer
            self.viewer = mjviewer.launch_passive(self.model, self.data)
            self.viewer.cam.lookat[:] = [0.15, 0, 0.10]
            self.viewer.cam.distance = 0.8
            self.viewer.cam.azimuth = 145
            self.viewer.cam.elevation = -35
        try:
            if camera_viewer:
                from .camera_viewer import CameraViewer, ProcessCameraViewer
                # launch_passive owns a GLFW event loop in its render thread.
                # A second GLFW window must not poll events in this process.
                self.camera_viewer = ProcessCameraViewer() if viewer else CameraViewer()
            self.observe()
        except BaseException:
            self.close()
            raise

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()

    def close(self):
        if self.camera_viewer:
            self.camera_viewer.close()
        if self.viewer:
            self.viewer.close()
        self.env.close()

    def _camera(self, name):
        e = self.env
        if name == 'overhead':
            return e._overhead_obs_renderer, e._overhead_obs_cam
        if name == 'wrist':
            return e._wrist_renderer, e._wrist_cam_id
        raise ValueError('camera must be overhead or wrist')

    def capture_cameras(self):
        """Render without changing policy frame IDs, saved images or physics."""
        cameras = {}
        for name in ('overhead', 'wrist'):
            renderer, camera = self._camera(name)
            renderer.disable_depth_rendering()
            renderer.update_scene(self.data, camera=camera)
            rgb = renderer.render().copy()
            left, right = renderer.scene.camera
            # Mono rendering uses the average of stereo eye positions.
            eye = (left.pos.astype(float) + right.pos.astype(float))/2
            forward = np.array(left.forward, dtype=float)
            up = np.array(left.up, dtype=float)
            rotation = np.column_stack((np.cross(forward, up), up, -forward))
            height, width = rgb.shape[:2]
            fy = height*float(left.frustum_near)/(float(left.frustum_top)-float(left.frustum_bottom))
            calibration = {'width': width, 'height': height, 'fx':fy, 'fy':fy,
                           'cx':(width-1)/2, 'cy':(height-1)/2,
                           'position':eye.tolist(), 'rotation':rotation.tolist()}
            renderer.enable_depth_rendering()
            depth = renderer.render().copy()
            # A cleared depth buffer is a finite far-plane value, not a hit.
            depth[(depth >= float(left.frustum_far)*0.9999) |
                  (depth <= float(left.frustum_near))] = np.nan
            renderer.disable_depth_rendering()
            cameras[name] = (rgb, depth, calibration)
        return cameras

    def update_camera_viewer(self):
        if self.camera_viewer and self.camera_viewer.due(float(self.data.time)):
            self.camera_viewer.show(self.capture_cameras(), float(self.data.time))

    def observe(self):
        self.frame_index += 1
        self.frame_id = f'{self.episode_token}:frame-{self.frame_index}'
        captured = self.capture_cameras()
        cameras = {}
        for name, (rgb, depth, calibration) in captured.items():
            rgb_path = self.directory / f'{self.frame_index:04d}-{name}.png'
            depth_path = self.directory / f'{self.frame_index:04d}-{name}-depth.npy'
            Image.fromarray(rgb).save(rgb_path)
            np.save(depth_path, depth)
            self.frames[name] = (rgb, depth, calibration)
            cameras[name] = {'rgb':str(rgb_path), 'depth':str(depth_path), 'calibration':calibration}
        obs = {'task':self.task, 'instruction':self.env.task_description,
               'frame_id':self.frame_id, 'time':float(self.data.time),
               'robot': {'joints':self.data.qpos[self.qadr].tolist(),
                         'joint_velocities':self.data.qvel[self.dadr].tolist(),
                         'tcp':self.env._get_tcp_pose().tolist()}, 'cameras':cameras}
        (self.directory/'observation.json').write_text(json.dumps(obs, indent=2)+'\n')
        if self.camera_viewer and self.camera_viewer.poll():
            self.camera_viewer.show(captured, float(self.data.time))
        return obs

    def solve_ik(self, position, seed=None, mode='down'):
        position = np.asarray(position, dtype=float)
        if position.shape != (3,) or not np.isfinite(position).all() or np.linalg.norm(position) > 0.5 or position[2] < 0.005:
            raise ValueError('Target outside robot workspace')
        scratch, m = self.scratch, self.model
        scratch.qpos[:] = self.data.qpos
        seed = self.target[:5].copy() if seed is None else np.asarray(seed)
        def residual(q):
            scratch.qpos[self.qadr[:5]] = q
            mujoco.mj_forward(m, scratch)
            tcp = scratch.site_xpos[self.site]
            if mode == 'position':
                return np.r_[tcp-position, 0.0002*(q-seed)]
            z_axis = scratch.site_xmat[self.site].reshape(3,3)[:,2]
            return np.r_[tcp-position, 0.1*(z_axis-[0,0,-1]), 0.03*q[4]]
        best = None
        for guess in (seed, [0,-0.4,0.5,1.3,0], [0,-1,1,1.5,0]):
            sol = least_squares(residual, np.clip(guess,self.low[:5]+1e-6,self.high[:5]-1e-6),
                                bounds=(self.low[:5],self.high[:5]), max_nfev=120,
                                ftol=1e-10,xtol=1e-10,gtol=1e-10)
            err = np.linalg.norm(residual(sol.x))
            if best is None or err < best[0]:best=(err,sol.x.copy())
            if err < 0.0001:break
        if best[0] > 0.002:
            raise ValueError(f'IK cannot reach target with {mode} orientation; error={best[0]:.4f}')
        return best[1]

    def _advance(self, count):
        for _ in range(count):
            self.data.ctrl[self.env._actuator_ids] = self.target
            self.env._advance_physics()
            if np.any(self.data.warning.number) or not np.isfinite(self.data.qpos).all():
                raise RuntimeError('Unstable MuJoCo physics')
            if self.viewer:
                if not self.viewer.is_running():raise RuntimeError('Viewer closed')
                self.viewer.sync()
            if self.realtime and (self.viewer or self.camera_viewer):
                time.sleep(0.02)
            self.update_camera_viewer()
        mujoco.mj_forward(self.model,self.data)

    def _move_joints(self, q, seconds):
        start = self.target[:5].copy()
        count = max(1,round(seconds/0.02))
        for i in range(count):
            t=(i+1)/count
            self.target[:5] = start+(q-start)*(t*t*(3-2*t))
            self._advance(1)

    def execute(self, command):
        if not isinstance(command,dict):raise ValueError('Expected a JSON object')
        action = command.get('action')
        if action == 'observe':return self.observe()
        if action in ('localize','propose'):
            if command.get('frame_id') != self.frame_id:raise ValueError('Stale or missing frame_id')
            name = command.get('camera','overhead')
            if name not in self.frames:raise ValueError('Unknown camera')
            rgb,depth,cal = self.frames[name]
            if action == 'propose':
                return {'frame_id':self.frame_id, 'proposals':locate_color(rgb,depth,cal,command.get('color'))}
            point = unproject_pixel(depth,cal,command.get('pixel'))
            point_id = f'{self.episode_token}:point-{len(self.points)+1}'
            self.points[point_id] = point
            return {'frame_id':self.frame_id,'point_id':point_id,'surface_world':point.tolist(),
                    'source':{'camera':name,'pixel':command['pixel']}}
        if action in ('move','look_at'):
            seconds = number(command.get('seconds',2),0.2,8,'seconds')
            if 'point_id' in command:
                if command['point_id'] not in self.points:raise ValueError('Unknown visual point')
                position=self.points[command['point_id']]+np.asarray(command.get('offset',[0,0,0]),dtype=float)
            else:position=np.asarray(command.get('position'),dtype=float)
            if position.shape!=(3,) or not np.isfinite(position).all():raise ValueError('Expected finite XYZ')
            if action == 'look_at':
                seed=self.target[:5].copy();scratch=self.scratch;scratch.qpos[:]=self.data.qpos
                def residual(q):
                    scratch.qpos[self.qadr[:5]]=q;mujoco.mj_forward(self.model,scratch)
                    cam=self.env._wrist_cam_id
                    direction=position-scratch.cam_xpos[cam]
                    direction/=max(np.linalg.norm(direction),1e-9)
                    forward=-scratch.cam_xmat[cam].reshape(3,3)[:,2]
                    return np.r_[forward-direction,0.005*(q-seed)]
                q=least_squares(residual,np.clip(seed,self.low[:5],self.high[:5]),bounds=(self.low[:5],self.high[:5]),max_nfev=150).x
                self._move_joints(q,seconds)
            else:
                mode=command.get('orientation','down')
                if mode not in ('down','position'):raise ValueError('orientation must be down or position')
                end=self.solve_ik(position,mode=mode)
                # Optional straight TCP path, preflighted before any motion.
                if command.get('linear',False):
                    start=self.data.site_xpos[self.site].copy();seed=self.target[:5].copy();path=[]
                    for t in np.linspace(0,1,max(2,round(seconds/0.04)))[1:]:
                        seed=self.solve_ik(start+(position-start)*t,seed=seed,mode=mode);path.append(seed)
                    for q in path:self._move_joints(q,seconds/len(path))
                else:self._move_joints(end,seconds)
            self._advance(25)
        elif action == 'gripper':
            opening=number(command.get('opening'),0,1,'opening')
            seconds=number(command.get('seconds',1.5),0.2,5,'seconds')
            self.target[5]=self.low[5]+opening*(self.high[5]-self.low[5])
            self._advance(round(seconds/0.02))
        elif action == 'wait':
            seconds=number(command.get('seconds',1),0.02,5,'seconds')
            self._advance(round(seconds/0.02))
        else:raise ValueError(f'Unknown visual action: {action}')
        return self.observe()
