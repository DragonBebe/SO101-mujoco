"""Camera/proprioception boundary and bounded low-level robot controls."""
from contextlib import contextmanager, nullcontext
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
from .cameras import CameraSuite, WRIST
from .perception import intersect_pixel_with_plane, locate_color, unproject_pixel

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
                 config_overrides=None, cameras=None):
        if camera_viewer and os.environ.get('MUJOCO_GL', 'glfw').lower() != 'glfw':
            raise ValueError('Use MUJOCO_GL=glfw with --camera-viewer')
        if task not in TASKS:
            raise ValueError(f'Unknown task: {task}')
        self.cameras = cameras or CameraSuite()
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
        suite = self.cameras
        # Only the placement the run actually uses is built: an unused overhead
        # component would keep a renderer alive whose images nothing may read.
        observations = [JointPositions(),
            WristCamera(width=suite.width, height=suite.height,
                        fov_deg_range=(suite.wrist_fov_deg,)*2,
                        pitch_deg_range=(suite.wrist_pitch_deg,)*2,
                        pos_x_noise=0, pos_y_center=0.055, pos_y_noise=0,
                        pos_z_center=-0.045, pos_z_noise=0)]
        if suite.placement == 'overhead':
            observations.append(OverheadCamera(width=suite.width, height=suite.height,
                                               fov_deg=suite.overhead_fov_deg))
        config_options = dict(
            spawn_center=(0.20, 0) if task == 'LookAt' else (0, 0),
            spawn_half_size=0.03, spawn_min_radius=0.17, spawn_max_radius=0.23,
            spawn_angle_half_range_deg=35, robot_init_qpos_noise=0,
            robot=RobotConfig(rest_qpos_deg=(70, -85, 85, 30, 0, 70)),
            terminate_on_success=False, reset_settle_frames=50,
            obs_mode='visual', observations=observations,
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
        self.side_camera_params = None
        self._side_renderer = self._side_cam = None
        if suite.placement == 'side':
            self._build_side_camera()
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
                view = self.cameras.preview_view()
                self.camera_viewer = (ProcessCameraViewer(view) if viewer
                                      else CameraViewer(view))
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
        if self._side_renderer is not None:
            self._side_renderer.close()
        self.env.close()

    def _build_side_camera(self):
        """World-fixed bystander camera, framed from the scene's own scale.

        A free ``MjvCamera`` is fixed in world coordinates: its lookat,
        distance and orbit angles are resolved once, here, from the spawn
        configuration. Nothing re-aims it at an object afterwards.
        """
        config = self.env.config
        self.side_camera_params = self.cameras.side_camera(
            config.spawn_center, config.spawn_max_radius,
            config.spawn_angle_half_range_deg)
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.lookat[:] = self.side_camera_params['lookat']
        camera.distance = self.side_camera_params['distance']
        camera.azimuth = self.side_camera_params['azimuth']
        camera.elevation = self.side_camera_params['elevation']
        self._side_cam = camera
        self._side_renderer = mujoco.Renderer(self.model, height=self.cameras.height,
                                              width=self.cameras.width)

    @contextmanager
    def _render_fov(self, fov_deg):
        """Render a free camera at ``fov_deg`` instead of the model default.

        A free ``MjvCamera`` has no FOV of its own; ``mjv_updateCamera`` reads
        the model's global vertical FOV. Swapping it around ``update_scene``
        keeps each camera's own field of view without touching the other
        camera's framing. The viewer lock is held so a passive window never
        renders a frame with the borrowed value.
        """
        model = self.model
        previous = float(model.vis.global_.fovy)
        if fov_deg is None or abs(previous - float(fov_deg)) < 1e-9:
            yield
            return
        with self.viewer.lock() if self.viewer else nullcontext():
            model.vis.global_.fovy = float(fov_deg)
            try:
                yield
            finally:
                model.vis.global_.fovy = previous

    def _camera(self, name):
        """Return ``(renderer, camera, render_fov_deg)`` for a configured name."""
        e, suite = self.env, self.cameras
        if name == suite.environment_camera:
            if name == 'overhead':
                return e._overhead_obs_renderer, e._overhead_obs_cam, suite.overhead_fov_deg
            return self._side_renderer, self._side_cam, suite.side_fov_deg
        if name == WRIST:
            return e._wrist_renderer, e._wrist_cam_id, None
        raise ValueError(f'Unknown camera {name!r}; this run provides '
                         f'{" and ".join(suite.names)}')

    @staticmethod
    def _calibration(renderer, shape):
        """Pinhole intrinsics and world pose of the frame just rendered."""
        left, right = renderer.scene.camera
        # Mono rendering uses the average of stereo eye positions.
        eye = (left.pos.astype(float) + right.pos.astype(float))/2
        forward = np.array(left.forward, dtype=float)
        up = np.array(left.up, dtype=float)
        rotation = np.column_stack((np.cross(forward, up), up, -forward))
        height, width = shape
        fy = height*float(left.frustum_near)/(float(left.frustum_top)-float(left.frustum_bottom))
        return {'width': width, 'height': height, 'fx':fy, 'fy':fy,
                'cx':(width-1)/2, 'cy':(height-1)/2,
                'position':eye.tolist(), 'rotation':rotation.tolist()}

    def capture_cameras(self):
        """Render without changing policy frame IDs, saved images or physics.

        Depth is ``None`` in the RGB-only modality: no depth buffer is read
        and no placeholder array is fabricated.
        """
        cameras = {}
        for name in self.cameras.names:
            renderer, camera, fov = self._camera(name)
            with self._render_fov(fov):
                renderer.disable_depth_rendering()
                renderer.update_scene(self.data, camera=camera)
                rgb = renderer.render().copy()
                calibration = self._calibration(renderer, rgb.shape[:2])
                depth = None
                if self.cameras.depth_available:
                    left = renderer.scene.camera[0]
                    renderer.enable_depth_rendering()
                    depth = renderer.render().copy()
                    # A cleared depth buffer is a finite far-plane value, not a hit.
                    depth[(depth >= float(left.frustum_far)*0.9999) |
                          (depth <= float(left.frustum_near))] = np.nan
                    renderer.disable_depth_rendering()
            cameras[name] = (rgb, depth, calibration)
        return cameras

    def perception_report(self):
        """What this run can and cannot do with the images it returns."""
        suite = self.cameras
        plane = ("method 'plane' with an explicit plane_z: the pixel ray is intersected "
                 'with that horizontal plane. Depth-free and exact for content that '
                 'really lies on the plane; wrong by the height error otherwise')
        carve = ("pass object_height (and plane_z, default 0) to also estimate where an "
                 'upright object of that height stands on the plane, and a grasp_center '
                 'at half its height. Depth-free; assumes the object rests on the plane '
                 'and is not badly occluded')
        if suite.depth_available:
            localize = ('methods: depth (default) reprojects registered depth to the '
                        f'visible surface; {plane}')
            propose = f'rendered-colour regions with a depth-derived surface point; {carve}'
        else:
            localize = (f"method 'depth' is unavailable: no depth is rendered. {plane}. "
                        'This is the only 3D estimator in the rgb modality')
            propose = f'rendered-colour regions as pixels and boxes, no surface point; {carve}'
        return {'modality': suite.modality, 'depth_available': suite.depth_available,
                'environment_camera': suite.environment_camera,
                'cameras': list(suite.names),
                'resolution': {'width': suite.width, 'height': suite.height},
                'localize': localize, 'propose': propose,
                'sources': 'rendered images, camera calibration and robot forward kinematics'}

    def camera_report(self):
        """Configuration plus the resolved pose of the environment camera."""
        report = self.cameras.describe()
        if self.side_camera_params:
            resolved = dict(self.side_camera_params)
            for key in ('lookat', 'eye'):
                resolved[key] = np.asarray(resolved[key]).tolist()
            report['side_resolved'] = resolved
        return report

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
            Image.fromarray(rgb).save(rgb_path)
            entry = {'camera_id':name, 'placement':name, 'modality':self.cameras.modality,
                     'mounting':self.cameras.mounting(name),
                     'width':calibration['width'], 'height':calibration['height'],
                     'rgb':str(rgb_path), 'calibration':calibration}
            if depth is None:
                # Explicitly absent, never a stale path and never zeros.
                entry['depth'] = None
                entry['depth_note'] = 'no depth in the rgb modality; none is rendered or saved'
            else:
                depth_path = self.directory / f'{self.frame_index:04d}-{name}-depth.npy'
                np.save(depth_path, depth)
                entry['depth'] = str(depth_path)
            self.frames[name] = (rgb, depth, calibration)
            cameras[name] = entry
        obs = {'task':self.task, 'instruction':self.env.task_description,
               'frame_id':self.frame_id, 'time':float(self.data.time),
               'robot': {'joints':self.data.qpos[self.qadr].tolist(),
                         'joint_velocities':self.data.qvel[self.dadr].tolist(),
                         'tcp':self.env._get_tcp_pose().tolist()},
               'cameras':cameras, 'perception':self.perception_report()}
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
            name = command.get('camera',self.cameras.environment_camera)
            if name not in self.frames:
                raise ValueError(f'Unknown camera {name!r}; this run provides '
                                 f'{" and ".join(self.frames)}')
            rgb,depth,cal = self.frames[name]
            if action == 'propose':
                height = command.get('object_height')
                if height is not None:
                    height = number(height,0.002,0.3,'object_height')
                plane = command.get('plane_z')
                if plane is not None:
                    plane = number(plane,-0.05,0.5,'plane_z')
                if plane is not None and height is None:
                    raise ValueError('plane_z only applies with object_height')
                return {'frame_id':self.frame_id, 'camera':name,
                        'modality':self.cameras.modality,
                        'depth_available':depth is not None,
                        'proposals':locate_color(rgb,depth,cal,command.get('color'),
                                                 plane_z=plane,object_height=height)}
            method = command.get('method', 'depth' if depth is not None else 'plane')
            if method not in ('depth','plane'):
                raise ValueError("localize method must be 'depth' or 'plane'")
            point_id = f'{self.episode_token}:point-{len(self.points)+1}'
            source = {'camera':name,'pixel':command.get('pixel')}
            if method == 'depth':
                if depth is None:
                    # Never silently substitute: depth reprojection measures the
                    # surface, and this modality renders none.
                    raise ValueError(
                        "localize method 'depth' is not supported in the 'rgb' camera "
                        'modality: no depth is rendered. Use method "plane" with an '
                        'explicit plane_z, or restart with --camera-modality rgbd.')
                point = unproject_pixel(depth,cal,command.get('pixel'))
                source['method'] = 'depth_reprojection'
            else:
                # The plane is an assumption, so the caller has to state it:
                # a pixel on top of an object needs that object's own height.
                if 'plane_z' not in command:
                    raise ValueError(
                        'plane localization needs an explicit plane_z: use 0 for the '
                        'support surface, or the top height of whatever the pixel '
                        'shows. It is wrong wherever that assumption is wrong.')
                plane = number(command['plane_z'],-0.05,0.5,'plane_z')
                point = intersect_pixel_with_plane(cal,command.get('pixel'),plane)
                source['method'] = 'ray_plane_intersection'
                source['assumed_plane_z'] = plane
            self.points[point_id] = point
            return {'frame_id':self.frame_id,'point_id':point_id,
                    'surface_world':point.tolist(),'source':source}
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
