"""Build data/events.json: key moments of the 2026-09-18 real grasp/stack test.

Measured servo ticks are copied from the run's console output; joint angles
and FK use the solved joint convention of calib/c920-handeye-03 (signs +1,
zero offsets lift +3.07 / elbow -3.56 / wrist_flex +10.46 deg) and the grasp
centre LOCAL = mean of point-check-02 #1/#3.  Where no ticks were recorded,
the commanded pose's IK solution is given instead (marked 'commanded').
Run from LLM-control/:  PYTHONPATH=. .venv-rgbcal/bin/python calib/c920-real-grasp-01/tools/build_events.py
"""
import json, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
sys.argv = sys.argv[:1]
import grasp_red as g

M = None  # measured ticks absent
E = [
 # id, what, measured ticks, cmd mm, camera cube mm (what the camera said), result
 ('E00', 'start pose (rest, torque off)', {1:1925,2:969,3:2962,4:2694,5:2153,6:2305}, None, None, 'rest'),
 ('E01', 'hover over red, gripper opened', {1:1966,2:1930,3:2139,4:2896,5:2102,6:2798}, [248.0,40.1,60.5], [248.0,40.1,0.5], 'user: needs +2 cm toward camera'),
 ('E02', 'after user correction +20 mm toward camera', {1:1994,2:2042,3:1992,4:2912,5:2096,6:2798}, [267.6,36.3,60.5], [248.0,40.1,0.5], 'wrist cam: cube still ahead'),
 ('E03', 'RED GRASP #1 closed (success)', {1:1998,2:2170,3:2040,4:2749,5:2096,6:2271}, [287.6,36.3,20.5], [248.0,40.1,0.5], 'held, gripper 2271'),
 ('E04', 'red lifted 30 mm', {1:1998,2:2167,3:1898,4:2892,5:2096,6:2269}, [287.6,36.3,50.5], None, 'cube off table'),
 ('E05', 'red lifted 60 mm', {1:1998,2:2215,3:1685,4:3057,5:2096,6:2269}, [287.6,36.3,80.5], None, ''),
 ('E06', 'first translate step toward green', {1:1939,2:2217,3:1692,4:3057,5:2096,6:2269}, [283.6,60.3,80.5], None, ''),
 ('E07', 'RED RELEASED ON GREEN (success)', {1:1729,2:2296,3:1709,4:2948,5:2098,6:2497}, [268.6,143.8,50.5], [234.8,145.2,3.3], 'green cam pos; red stacked, ok'),
 ('E08', 'red re-grasped from stack top', M, [268.6,143.8,47.5], None, 'gripper 2271'),
 ('E09', 'red placed back on table', M, [287.6,36.3,23.5], [262.4,40.8,0.7], 'camera afterwards saw red at this position'),
 ('E10', 'GREEN GRASP #1 (success, cube tilted in grip)', M, [268.6,149.8,20.5], [234.8,145.2,3.3], 'fixed jaw nudged cube ~5 mm -y first; gripper 2272'),
 ('E11', 'green released over red (FAIL: fell forward, red pushed -42 mm x)', M, [287.6,31.3,55.5], [262.4,40.8,0.7], 'red cam pos; green ~13 mm +x of red'),
 ('E12', 'far green attempt A (MISS)', M, [344.0,77.0,28.5], [314.2,75.0,2.5], 'gripper 2264 empty; jaws at cube-top height'),
 ('E13', 'far green attempt B (MISS)', M, [326.5,68.7,13.5], [306.7,60.8,2.0], 'gripper 2263 empty; then opening low knocked cube'),
 ('E14', 'GREEN GRASP #2 at known site (success)', M, [308.0,34.6,20.5], [268.4,38.4,0.7], 'gripper 2272'),
 ('E15', 'GREEN RELEASED ON RED (success)', M, [249.6,33.1,50.5], [218.0,39.9,0.0], 'red cam pos; after (-8,-3) mm visual correction'),
 ('E16', 'home (start pose), then torque released', {1:1927,2:974,3:2960,4:2701,5:2149,6:2497}, None, None, 'rest; drift <= 2 ticks'),
]
# FK measured at the moment (console output) where ticks were not recorded
FK_SEEN = {'E08': [265.1,142.2,39.1], 'E09': [283.8,37.2,16.4], 'E10': [265.2,148.1,14.3],
           'E11': [284.5,31.9,51.5], 'E12': [335.9,75.7,11.5], 'E13': [325.6,69.0,12.6],
           'E14': [302.7,35.2,12.3], 'E15': [247.3,32.7,46.0], 'E07': [266.0,143.7,45.1]}

out = []
for eid, what, ticks, cmd, cam, result in E:
    rec = {'id': eid, 'event': what, 'result': result}
    if ticks:
        q = g.q_from_ticks(ticks)
        p, T = g.grasp_point(q)
        rec['servo_ticks_measured'] = ticks
        rec['joint_deg_measured'] = dict(zip(g.ARM, np.round(np.degrees(q), 2).tolist()))
        rec['gripper_ticks'] = ticks[6]
        rec['fk_grasp_centre_mm'] = np.round(p * 1000, 1).tolist()
        rec['fk_tcp_mm'] = np.round(T[:3, 3] * 1000, 1).tolist()
    elif eid in FK_SEEN:
        rec['fk_grasp_centre_mm'] = FK_SEEN[eid]
    if cmd:
        rec['cmd_grasp_centre_mm'] = cmd
        qc, info = g.ik(np.array(cmd) / 1000, g.q_from_ticks({1:1998,2:2170,3:2040,4:2749,5:2096}))
        rec['joint_deg_commanded_ik'] = dict(zip(g.ARM, np.round(np.degrees(qc), 2).tolist()))
        rec['servo_ticks_commanded_ik'] = g.ticks_from_q(qc)
    if cam:
        rec['camera_cube_grasp_centre_mm'] = cam
        if cmd:
            rec['cmd_minus_camera_mm'] = np.round(np.array(cmd) - cam, 1).tolist()
        if 'fk_grasp_centre_mm' in rec:
            rec['fk_minus_camera_mm'] = np.round(np.array(rec['fk_grasp_centre_mm']) - cam, 1).tolist()
    if cmd and 'fk_grasp_centre_mm' in rec:
        rec['sag_fk_minus_cmd_mm'] = np.round(np.array(rec['fk_grasp_centre_mm']) - cmd, 1).tolist()
    out.append(rec)
path = Path(__file__).resolve().parent.parent / 'data/events.json'
path.write_text(json.dumps(out, indent=1, ensure_ascii=False) + '\n')
for r in out:
    print(r['id'], r['event'][:44].ljust(44), 'cmd', r.get('cmd_grasp_centre_mm'), 'fk', r.get('fk_grasp_centre_mm'),
          'cam', r.get('camera_cube_grasp_centre_mm'), 'cmd-cam', r.get('cmd_minus_camera_mm'),
          'fk-cam', r.get('fk_minus_camera_mm'), 'sag', r.get('sag_fk_minus_cmd_mm'))
    if 'joint_deg_measured' in r: print('     joints', r['joint_deg_measured'])
