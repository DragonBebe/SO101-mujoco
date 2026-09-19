"""Pixel offset between the held red cube and the point straight above the green cube at
the red cube's height; converted to base-frame mm via the local image Jacobian."""
import sys, json
import cv2, numpy as np
from pathlib import Path
S = sys.argv[1]; green = np.array([float(v) for v in sys.argv[2].split(',')]) / 1000
z_red = float(sys.argv[3]) / 1000
ext = json.loads(Path('calib/c920-handeye-03/extrinsics.json').read_text())
intr = json.loads(Path('calib/c920-intrinsics-01/intrinsics.json').read_text())
T = np.linalg.inv(np.array(ext['T_base_camera'])); K = np.array(intr['K']); D = np.array(intr['D'])
def proj(p):
    pc = T[:3, :3] @ p + T[:3, 3]
    return cv2.projectPoints(pc.reshape(1, 3), np.zeros(3), np.zeros(3), K, D)[0].ravel()
img = cv2.imread(f'{S}/live/env.png')
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
held = sys.argv[4] if len(sys.argv) > 4 else 'red'
if held == 'green':
    m = cv2.inRange(hsv, (40, 80, 40), (95, 255, 255))
    m[:int(img.shape[0] * 0.3)] = 0      # ignore floor / background above the table edge
else:
    m = cv2.inRange(hsv, (0, 120, 60), (8, 255, 255)) | cv2.inRange(hsv, (172, 120, 60), (180, 255, 255))
n, lab, st, cen = cv2.connectedComponentsWithStats(m)
k = 1 + np.argmax(st[1:, cv2.CC_STAT_AREA])
x, y, w, h = st[k, :4]
red_px = np.array([x + w / 2, y + h / 2])      # box centre ~ cube centre
target = np.array([green[0], green[1], z_red])
t_px = proj(target)
J = np.stack([proj(target + d) - t_px for d in np.eye(3)[:2] * 0.001], axis=1)  # px per mm (x,y)
d_mm = np.linalg.lstsq(J, red_px - t_px, rcond=None)[0]
print('red px', red_px.round(), 'above-green px', t_px.round(), 'bbox', (x, y, w, h))
print('red minus above-green, if at same x-depth: base dxy mm', d_mm.round(1), '(x poorly observable)')
print('pure-y estimate mm', ((red_px - t_px)[0] / J[0, 1]).round(1))
