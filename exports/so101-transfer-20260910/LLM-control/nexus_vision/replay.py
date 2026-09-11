"""Replay a recorded Codex visual plan, re-reading RGB-D for every selection.

This is deterministic replay of a human/LLM-produced plan, not new LLM inference.
Pixels are meaningful for the case's recorded scene configuration and seed.
"""
from .bridge import VisualSession


def replay_case(case, directory, viewer=False, realtime=False, camera_viewer=False):
    commands = case.get('commands')
    allowed = {'observe','propose','localize','move','look_at','gripper','wait'}
    if (not isinstance(commands,list) or not commands
        or commands[-1] != {'action':'finish'}
        or any(not isinstance(c,dict) or c.get('action') not in allowed for c in commands[:-1])):
        raise ValueError('Replay requires a single episode with finish as the final command')
    session = VisualSession(directory, case['task'], case.get('seed',4), viewer, realtime, camera_viewer)
    points = {}
    final = None
    try:
        for original in commands:
            command = dict(original)
            save_as = command.pop('save_as',None)
            if command['action'] in ('localize','propose'):
                command['frame_id'] = session.sim.frame_id
            if 'point_id' in command:
                if command['point_id'] not in points:raise ValueError('Replay references an unknown visual point')
                command['point_id'] = points[command['point_id']]
            response = session.execute(command)
            if not response['ok']:raise RuntimeError(response['error'])
            if save_as:points[save_as] = response['result']['point_id']
            if command['action']=='finish':final=response['result']
        if final is None:raise ValueError('Replay must finish with terminal evaluation')
        return final
    finally:
        session.close()
