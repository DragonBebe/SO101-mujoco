"""Task lifecycle around a persistent world; the current Codex is the planner."""
from copy import deepcopy
from datetime import datetime, timezone
import json
from pathlib import Path
import uuid

from .cameras import suite_from_options
from .workspace import WorkspaceSimulation, scene_description

MOTION = {'move', 'look_at', 'gripper', 'wait', 'joints'}
ACTIVE = {'running', 'paused'}


def text_field(value, name, limit=8000):
    if not isinstance(value, str) or not value.strip() or len(value) > limit:
        raise ValueError(f'{name} must be nonempty text of at most {limit} characters')
    return value.strip()


def integer(value, name, low, high):
    if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
        raise ValueError(f'{name} must be an integer in [{low}, {high}]')
    return value


class LoopSession:
    def __init__(self, directory, task='workbench', seed=4, viewer=False,
                 realtime=False, camera_viewer=False, camera_modality=None,
                 environment_camera=None):
        if task != 'workbench':
            raise ValueError('Only the workbench scene is supported')
        integer(seed, 'seed', 0, 2**32-1)
        # Modality and placement are fixed for the life of the service, so
        # every frame, point and log row in this run shares one camera setup.
        self.cameras = suite_from_options(camera_modality, environment_camera)
        self.directory = Path(directory).resolve()
        self.directory.mkdir(parents=True, exist_ok=True)
        self.log = (self.directory / 'events.jsonl').open('x', encoding='utf-8')
        self.options = dict(viewer=viewer, realtime=realtime, camera_viewer=camera_viewer,
                            cameras=self.cameras)
        self.seed, self.epoch = seed, 1
        self.task = None
        self.running = True
        self.sequence = 0
        try:
            self.sim = WorkspaceSimulation(self.directory / 'world-001', seed, **self.options)
            self.save()
        except BaseException:
            self.log.close()
            raise

    def state(self):
        return {'task': deepcopy(self.task), 'scene': scene_description(self.cameras),
                'cameras': self.sim.camera_report(),
                'world': self.epoch, 'seed': self.seed, 'running': self.running,
                'frame_id': self.sim.frame_id, 'sequence': self.sequence}

    def save(self):
        temporary = self.directory / 'session.json.tmp'
        temporary.write_text(json.dumps(self.state(), ensure_ascii=False, indent=2,
                                        allow_nan=False) + '\n', encoding='utf-8')
        temporary.replace(self.directory / 'session.json')

    def observation(self):
        obs = self.sim.observe()
        obs['task'] = 'workbench'
        obs['instruction'] = self.task['instruction'] if self.task else '等待自然语言任务'
        return obs

    def require_task(self, command=None):
        if not self.task or self.task['status'] not in ACTIVE:
            raise ValueError('No active task; use begin')
        if command is not None and command.get('task_id') != self.task['id']:
            raise ValueError('Missing or stale task_id')

    def current_frame(self, command):
        if command.get('frame_id') != self.sim.frame_id:
            raise ValueError('Missing or stale frame_id; observe again')

    def dispatch(self, command):
        action = command.get('action')
        if action == 'status':
            return {}
        if action == 'observe':
            return {'observation': self.observation()}
        if action in ('localize', 'propose'):
            return {'result': self.sim.execute(command)}
        if action == 'begin':
            if self.task and self.task['status'] in ACTIVE:
                raise ValueError('Finish or cancel the active task before begin')
            instruction = text_field(command.get('instruction'), 'instruction')
            budget = integer(command.get('max_steps', 60), 'max_steps', 1, 500)
            self.task = {'id': uuid.uuid4().hex, 'instruction': instruction,
                         'status': 'running', 'steps': 0, 'max_steps': budget,
                         'failures': 0, 'verification': None}
            return {'observation': self.observation()}
        if action == 'step':
            self.require_task(command)
            self.current_frame(command)
            expected = integer(command.get('expected_step'), 'expected_step', 0, 500)
            if expected != self.task['steps']:
                raise ValueError('Stale expected_step; inspect status before retrying')
            if self.task['status'] != 'running':
                raise ValueError('Task is paused')
            if self.task['steps'] >= self.task['max_steps']:
                self.task['status'] = 'paused'
                raise ValueError('Step budget exhausted; complete or cancel this task')
            motion = command.get('command')
            if not isinstance(motion, dict) or motion.get('action') not in MOTION:
                raise ValueError('step.command must be move, look_at, gripper, wait or joints')
            self.task['steps'] += 1
            try:
                obs = self.sim.execute(motion)
            except (ValueError, TypeError, RuntimeError):
                self.task['failures'] += 1
                if self.task['failures'] >= 3:
                    self.task['status'] = 'paused'
                raise
            self.task['failures'] = 0
            obs['task'], obs['instruction'] = 'workbench', self.task['instruction']
            if self.task['steps'] >= self.task['max_steps']:
                self.task['status'] = 'paused'
            return {'observation': obs}
        if action in ('pause', 'resume', 'cancel'):
            self.require_task()
            if action == 'resume':
                if self.task['steps'] >= self.task['max_steps']:
                    raise ValueError('Step budget exhausted; complete or cancel this task')
                self.task['failures'] = 0
            self.task['status'] = {'pause': 'paused', 'resume': 'running', 'cancel': 'cancelled'}[action]
            return {}
        if action == 'complete':
            self.require_task(command)
            self.current_frame(command)
            outcome = command.get('outcome')
            if outcome not in ('succeeded', 'failed'):
                raise ValueError('outcome must be succeeded or failed')
            evidence = text_field(command.get('evidence'), 'evidence')
            self.task.update(status=outcome, evidence=evidence,
                             evidence_frame=self.sim.frame_id, verification='codex_visual')
            return {}
        if action == 'reset':
            if self.task and self.task['status'] in ACTIVE:
                raise ValueError('Cancel or complete the active task before reset')
            seed = integer(command.get('seed', self.seed), 'seed', 0, 2**32-1)
            self.sim.reset_world(self.directory / f'world-{self.epoch+1:03d}', seed)
            self.seed, self.epoch, self.task = seed, self.epoch+1, None
            return {'observation': self.observation()}
        if action == 'shutdown':
            self.running = False
            if self.task and self.task['status'] in ACTIVE:
                self.task['status'] = 'cancelled'
            return {}
        raise ValueError(f'Unknown action: {action}; motions must use step')

    def execute(self, command):
        self.sequence += 1
        try:
            if not isinstance(command, dict):
                raise ValueError('Expected JSON object')
            result = {'ok': True, **self.dispatch(command)}
        except (ValueError, TypeError, RuntimeError) as exc:
            result = {'ok': False, 'error': str(exc)}
            # A runtime failure can occur after partial movement. Refresh the
            # frame and pause so recovery never acts on a pre-motion image.
            if isinstance(exc, RuntimeError) and self.task and self.task['status'] in ACTIVE:
                self.task['status'] = 'paused'
            if isinstance(command, dict) and command.get('action') == 'step':
                try:
                    result['observation'] = self.observation()
                except Exception as capture_error:
                    result['capture_error'] = str(capture_error)
        result.update(self.state())
        self.save()
        self.log.write(json.dumps({'time': datetime.now(timezone.utc).isoformat(),
                                   'command': command, **result}, ensure_ascii=False,
                                  allow_nan=False) + '\n')
        self.log.flush()
        return result

    def close(self):
        self.sim.close()
        self.log.close()
