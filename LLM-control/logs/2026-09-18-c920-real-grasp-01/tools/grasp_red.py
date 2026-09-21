"""Supervised, staged grasp of the red cube on the real SO101.

Stages (one per invocation, state kept in --state):
  plan     camera estimate + IK, no motion; writes annotated preview
  hover    start pose -> 60 mm above the grasp centre, gripper opens there
  descend  straight down to the grasp centre (gripper stays open)
  grasp    close gripper, lift 60 mm
  home     back to the recorded start pose (keeps holding torque)
  release  torque off (SUPPORT THE ARM)
Any stage aborts by holding the present position if tracking error or an
alarm appears.  Speed is limited in software.
"""
import argparse, json, sys, time
from pathlib import Path
import cv2
import numpy as np
from scipy.optimize import least_squares

from rgbcal import cli, realcam, robot
from rgbcal.pointcheck import cube_candidates

LIFT_M = 0.060
MAX_DEG_PER_S = 15.0
RATE_HZ = 50
TRACK_LIMIT_TICKS = 180       # ~16 deg: something is blocking the arm
GRIP_OPEN = 2800
GRIP_CLOSE = 2260             # a clamped 25 mm cube read ~2307
IDS = robot.JOINT_IDS
ARM = robot.ARM_JOINTS
TO_RAD = 2 * np.pi / robot.TICKS_PER_TURN

cal = realcam.load()
ext = cal.extrinsics
manifest = json.loads((Path(ext['source_session']) / 'poses.json').read_text())
models = robot.models_from_dict(manifest['joint_models'])
conv = ext['joint_convention']
signs = {k: int(v) for k, v in conv['signs'].items()}
offsets = {k: float(v) for k, v in conv['zero_offsets_rad'].items()}
kin = robot.Kinematics()
lerobot = json.loads(Path(cli.LEROBOT_CALIBRATION).read_text())

# Grasp centre in the gripper frame: clamped grasps #1 and #3 of
# c920-point-check-02, the ones taken with the wrist orientation used here.
pc = json.loads((Path(__file__).resolve().parents[2] / 'c920-point-check-02/point_check.json').read_text())['records']
LOCAL = np.mean([pc[0]['grasp_in_gripper_frame_m'], pc[2]['grasp_in_gripper_frame_m']], axis=0)
APPROACH = np.array([0.0, 0.0, -1.0])
ROLL_REF = np.radians(4.3)


def q_from_ticks(t):
    return np.array([signs[n] * (t[IDS[n]] - models[n].zero_ticks) * TO_RAD + offsets.get(n, 0.0)
                     for n in ARM])


def ticks_from_q(q):
    return {IDS[n]: int(round(models[n].zero_ticks + (q[i] - offsets.get(n, 0.0)) / TO_RAD * signs[n]))
            for i, n in enumerate(ARM)}


def fk(q):
    return kin.T_base_gripper(dict(zip(ARM, q)))


def grasp_point(q):
    T = fk(q)
    return T[:3, :3] @ LOCAL + T[:3, 3], T


def limits():
    lo, hi = [], []
    for n in ARM:
        e = lerobot[n]
        lo.append((e['range_min'] + 40 - models[n].zero_ticks) * TO_RAD + offsets.get(n, 0.0))
        hi.append((e['range_max'] - 40 - models[n].zero_ticks) * TO_RAD + offsets.get(n, 0.0))
    return np.array(lo), np.array(hi)


def ik(target, q0):
    lo, hi = limits()
    def res(q):
        p, T = grasp_point(q)
        return np.concatenate([(p - target) * 1000.0,           # mm
                               (T[:3, 2] - APPROACH) * 60.0,     # ~1 mm per degree
                               [np.degrees(q[4] - ROLL_REF) * 0.2]])
    sol = least_squares(res, np.clip(q0, lo + 1e-3, hi - 1e-3), bounds=(lo, hi))
    p, T = grasp_point(sol.x)
    return sol.x, {'pos_err_mm': float(np.linalg.norm(p - target) * 1000),
                   'approach_err_deg': float(np.degrees(np.arccos(np.clip(T[:3, 2] @ APPROACH, -1, 1))))}


def table_z(xy):
    return realcam.table_to_base(cal, [0, 0, 0])[2] + 0.0159 * (xy[0] - realcam.table_to_base(cal, [0, 0, 0])[0]) \
        + 0.0288 * (xy[1] - realcam.table_to_base(cal, [0, 0, 0])[1])


# ---------------------------------------------------------------- motion
def move(bus, goal_ticks, label, grip=None):
    ids = list(goal_ticks) + ([IDS['gripper']] if grip is not None else [])
    goal = dict(goal_ticks)
    if grip is not None:
        goal[IDS['gripper']] = grip
    alarms = bus.alarms(ids)
    if alarms:
        raise RuntimeError(f'alarms before {label}: {alarms}')
    start = bus.read_positions(ids)
    if not all(bus.torque_enabled(ids).values()):
        bus.hold(ids)                          # goal = present, then torque on
    span = max(abs(goal[i] - start[i]) for i in ids) * 360 / robot.TICKS_PER_TURN
    steps = max(1, int(np.ceil(span / MAX_DEG_PER_S * RATE_HZ)))
    print(f'{label}: {span:.1f} deg max joint travel, {steps / RATE_HZ:.1f} s')
    for k in range(1, steps + 1):
        s = k / steps
        s = s * s * (3 - 2 * s)                # smooth start/stop
        for i in ids:
            bus._write(i, 'Goal_Position', int(round(start[i] + (goal[i] - start[i]) * s)))
        time.sleep(1 / RATE_HZ)
        if k % 5 == 0 or k == steps:
            now = bus.read_positions(ids)
            lag = {i: abs(now[i] - int(round(start[i] + (goal[i] - start[i]) * s))) for i in ids
                   if i != IDS['gripper']}
            if max(lag.values(), default=0) > TRACK_LIMIT_TICKS or bus.alarms(ids):
                stop(bus, ids)
                raise RuntimeError(f'ABORT {label}: tracking {lag}, alarms {bus.alarms(ids)}; '
                                   'arm held where it is')
    time.sleep(0.6)
    final = bus.read_positions(ids)
    return final


def stop(bus, ids):
    now = bus.read_positions(ids)
    for i in ids:
        bus._write(i, 'Goal_Position', now[i], tolerate_alarm=True)


def cartesian(bus, state, q_from, target_from, target_to, label, n=8):
    q = np.array(q_from)
    for k in range(1, n + 1):
        tgt = target_from + (target_to - target_from) * k / n
        q, info = ik(tgt, q)
        if info['pos_err_mm'] > 3 or info['approach_err_deg'] > 8:
            raise RuntimeError(f'{label}: IK failed at step {k}: {info}')
        want = ticks_from_q(q)
        got = move(bus, want, f'{label} {k}/{n}')
        # Servos sag under load: add the steady-state error once, bounded.
        fix = {i: want[i] + int(np.clip(want[i] - got[i], -40, 40)) for i in want}
        got = move(bus, fix, f'{label} {k}/{n} sag-fix')
        p, _ = grasp_point(q_from_ticks(got))
        print(f'   grasp centre FK {np.round(p * 1000, 1)} mm, wanted {np.round(tgt * 1000, 1)}')
    return q


# ---------------------------------------------------------------- stages
def detect():
    settings = cli._settings(argparse.Namespace(profile='c920'))
    with realcam.EnvironmentCamera(settings, cal) as cam:
        frames = [cam.frame() for _ in range(5)]
    found = []
    for f in frames:
        c = [x for x in cube_candidates(f, cal, 'red')
             if all(0.012 <= e <= 0.060 for e in x['footprint_m'])]
        if len(c) != 1:
            raise RuntimeError(f'need exactly one cube-sized red blob, got {len(c)}')
        found.append(c[0])
    g = np.array([c['grasp_centre_base'] for c in found])
    return g.mean(axis=0), float(g.std(axis=0).max() * 1000), frames[-1], found[-1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('stage', choices=['plan', 'hover', 'descend', 'grasp', 'home', 'release', 'goto', 'grip'])
    ap.add_argument('--d', default='0,0,0', help='goto: offset in mm (base x,y,z) from the last commanded point')
    ap.add_argument('--grip', type=int, default=None)
    ap.add_argument('--state', required=True)
    a = ap.parse_args()
    state_path = Path(a.state)
    state = json.loads(state_path.read_text()) if state_path.exists() else {}

    if a.stage == 'plan':
        with robot.ServoBus() as bus:
            start = bus.read_positions()
        target, jitter, frame, cand = detect()
        q_seed = q_from_ticks({int(k): v for k, v in pc[0]['servo_ticks'].items()})
        q_grasp, info_g = ik(target, q_seed)
        q_hover, info_h = ik(target + [0, 0, LIFT_M], q_grasp)
        # joint-space path start -> hover: lowest points of TCP and grasp centre
        q0 = q_from_ticks(start)
        lows = []
        for s in np.linspace(0, 1, 41):
            q = q0 + (q_hover - q0) * s
            p, T = grasp_point(q)
            lows.append(min(p[2], T[2, 3]) - table_z(p[:2]))
        view = frame['undistorted'].copy()
        K = frame['calibration']
        state = {'start_ticks': start, 'target_base_m': target.tolist(), 'jitter_mm': jitter,
                 'local_grasp_m': LOCAL.tolist(), 'q_grasp': q_grasp.tolist(),
                 'q_hover': q_hover.tolist(), 'ik_grasp': info_g, 'ik_hover': info_h,
                 'ticks_grasp': ticks_from_q(q_grasp), 'ticks_hover': ticks_from_q(q_hover),
                 'min_clearance_start_to_hover_mm': float(min(lows) * 1000),
                 'motion_status': realcam.motion_status(
                     cal, cli._settings(argparse.Namespace(profile='c920')))}
        x0, y0, x1, y1 = cand['bbox']
        cv2.rectangle(view, (x0, y0), (x1, y1), (60, 220, 60), 2)
        cv2.imwrite(str(state_path.with_suffix('.png')), view)
        state_path.write_text(json.dumps(state, indent=2, default=float))
        print(json.dumps({k: state[k] for k in state if k not in ('local_grasp_m',)},
                         indent=1, default=float))
        print('start deg', np.round(np.degrees(q0), 1), '\nhover deg', np.round(np.degrees(q_hover), 1),
              '\ngrasp deg', np.round(np.degrees(q_grasp), 1))
        return

    with robot.ServoBus() as bus:
        if a.stage == 'hover':
            move(bus, {int(k): v for k, v in state['ticks_hover'].items()}, 'to hover')
            move(bus, {}, 'open gripper', grip=GRIP_OPEN)
        elif a.stage == 'descend':
            t = np.array(state['target_base_m'])
            cartesian(bus, state, state['q_hover'], t + [0, 0, LIFT_M], t, 'descend')
        elif a.stage == 'grasp':
            move(bus, {}, 'close gripper', grip=GRIP_CLOSE)
            t = np.array(state['target_base_m'])
            cartesian(bus, state, state['q_grasp'], t, t + [0, 0, LIFT_M], 'lift')
        elif a.stage == 'home':
            start = {int(k): v for k, v in state['start_ticks'].items()}
            move(bus, {i: start[i] for i in (IDS[n] for n in ARM)}, 'home')
        elif a.stage == 'goto':
            # small relative move of the grasp centre, <= 5 mm per step
            cmd = np.array(state.get('cmd_m', np.array(state['target_base_m']) + [0, 0, LIFT_M]))
            d = np.array([float(v) for v in a.d.split(',')]) / 1000
            if np.linalg.norm(d) > 0.030:
                raise RuntimeError('refusing a single goto longer than 30 mm')
            new = cmd + d
            if new[2] - table_z(new[:2]) < 0.010:
                raise RuntimeError('refusing: grasp centre would be < 10 mm above the table')
            q0 = np.array(state.get('q_cmd', state['q_hover']))
            n = max(1, int(np.ceil(np.linalg.norm(d) / 0.005)))
            q = cartesian(bus, state, q0, cmd, new, 'goto', n=n)
            state['cmd_m'], state['q_cmd'] = new.tolist(), q.tolist()
            state.setdefault('log', []).append({'goto_mm': (d * 1000).tolist(), 'cmd_mm': (new * 1000).tolist()})
            state_path.write_text(json.dumps(state, indent=2, default=float))
        elif a.stage == 'grip':
            move(bus, {}, f'gripper -> {a.grip}', grip=a.grip)
        elif a.stage == 'release':
            print('released', bus.release(list(IDS.values())))
        now = bus.read_positions()
        p, T = grasp_point(q_from_ticks(now))
        print('now ticks', now, '\ngrasp centre (FK) mm', np.round(p * 1000, 1),
              'target mm', np.round(np.array(state['target_base_m']) * 1000, 1),
              '\nalarms', bus.alarms())


if __name__ == '__main__':
    main()
