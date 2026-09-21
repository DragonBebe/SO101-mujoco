"""Supervised real joint jog for this session. No IK; one observed step at a time."""
import json
import math
import shutil
import sys
import time
from pathlib import Path
from rgbcal import robot

DIRECTORY = Path(__file__).resolve().parents[1]


def bounded_goal(delta, actual, limits):
    if not isinstance(delta, dict) or not delta:
        raise ValueError('nonempty named joint deltas required')
    result = {}
    for name, degrees in delta.items():
        if name not in robot.ARM_JOINTS or type(degrees) not in (float,int) or not math.isfinite(degrees) or abs(degrees) > 5:
            raise ValueError('only named arm joints, finite +/-5 degrees per step')
        motor = robot.JOINT_IDS[name]
        target = actual[motor] + round(degrees * robot.TICKS_PER_TURN / 360)
        low, high = limits[motor]
        if not low <= target <= high:
            raise ValueError(f'{name}: proposed {target} outside [{low},{high}]')
        result[motor] = target
    return result


def main():
    from realsim.arm_mapping import ArmMapping
    import numpy as np
    scene = json.loads((DIRECTORY.parents[1] / 'calib/realsim-model/scene.json').read_text())
    mapping = ArmMapping.from_scene(scene)
    kin = robot.Kinematics()  # forward kinematics only, auxiliary evidence
    log = open(DIRECTORY / 'actions.jsonl', 'x', buffering=1)
    frame = 0
    with robot.ServoBus() as bus:
        registers = bus.read_state()
        mapping.check_registers(registers)
        limits = {i:(s['min_position_limit'],s['max_position_limit']) for i,s in registers.items()}
        def observe(detail=None):
            nonlocal frame
            time.sleep(.7)
            frame += 1
            files = {}
            for name in ('env','wrist'):
                source = DIRECTORY / 'live' / f'{name}.png'
                if not source.exists() or time.time()-source.stat().st_mtime > 2:
                    raise RuntimeError(f'{name}: fresh camera unavailable')
                target = DIRECTORY / f'{frame:04d}-{name}.png'
                shutil.copyfile(source, target)
                files[name] = str(target)
            registers = bus.read_state()
            ticks = {i:v['present_position'] for i,v in registers.items()}
            angles, gripper = mapping.convert(ticks)
            T = kin.T_base_gripper(angles)
            result = {'frame':frame,'at':time.time(),'ticks':ticks,
                      'degrees':{n:float(np.degrees(q)) for n,q in angles.items()},
                      'tcp_fk_auxiliary_mm':(T[:3,3]*1000).tolist(),
                      'torque':{i:v['torque_enabled'] for i,v in registers.items()},
                      'images':files,'detail':detail}
            log.write(json.dumps(result)+'\n'); print(json.dumps(result),flush=True)
        observe('connected; no torque/motion command sent')
        for line in sys.stdin:
            try:
                command = json.loads(line)
                if command.get('frame') != frame:
                    raise ValueError(f'current frame is {frame}; stale commands refused')
                action = command['action']
                if action == 'observe': observe(); continue
                if action == 'quit': break
                # Require live visual feedback before any write.
                for name in ('env','wrist'):
                    if time.time()-(DIRECTORY/'live'/f'{name}.png').stat().st_mtime > 2:
                        raise RuntimeError('camera stale; no motion')
                alarms = bus.alarms()
                if alarms: raise RuntimeError(f'servo alarms: {alarms}')
                before = bus.read_positions()
                mapping.check_registers(bus.read_state())
                if action == 'hold':
                    result = bus.hold(list(robot.JOINT_IDS.values()))
                    observe({'command':command,'hold':result}); continue
                if not all(bus.torque_enabled(list(robot.JOINT_IDS.values())).values()):
                    raise RuntimeError('hold current pose before jogging')
                if action == 'joints':
                    target = bounded_goal(command['delta_deg'], before, limits)
                elif action == 'gripper':
                    value = command['ticks']
                    if type(value) is not int or abs(value-before[6])>100 or not limits[6][0]<=value<=limits[6][1]:
                        raise ValueError('gripper limited to 100 ticks per step and calibrated range')
                    target = {6:value}
                else: raise ValueError('unknown action')
                duration = max(1., max(abs(v-before[i]) for i,v in target.items()) / 80.)
                steps = max(1,int(duration*50))
                log.write(json.dumps({'command':command,'before':before,'target':target,'at':time.time()})+'\n')
                for k in range(1,steps+1):
                    fraction = k/steps
                    goals = {i:round(before[i]+(v-before[i])*fraction) for i,v in target.items()}
                    for i,value in goals.items(): bus._write(i,'Goal_Position',value)
                    time.sleep(duration/steps)
                    if k%5==0 or k==steps:
                        actual = bus.read_positions(list(target))
                        alarms=bus.alarms(list(target))
                        lag = {i:abs(actual[i]-goals[i]) for i in target}
                        if alarms or max(lag.values())>80:
                            for i,value in actual.items(): bus._write(i,'Goal_Position',value,tolerate_alarm=True)
                            raise RuntimeError(f'motion stopped: lag={lag}, alarms={alarms}')
                observe({'command':command,'target':target})
            except Exception as error:
                print(json.dumps({'error':str(error),'frame':frame,'instruction':'observe before another motion'}),flush=True)
                log.write(json.dumps({'error':str(error),'frame':frame,'at':time.time()})+'\n')
    log.close()


if __name__=='__main__': main()
