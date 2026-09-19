"""Project reference points into the latest environment frame (raw, canonical).
Marks: cube grasp centre (green), the point straight above it at the gripper's
current FK height (yellow), FK grasp centre (magenta), FK TCP (cyan).  Reads
the servo bus (read-only)."""
import json, sys
from pathlib import Path
import cv2
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import grasp_red as g
from rgbcal import robot

S = Path(__file__).parent
state = json.loads((S / 'grasp1.json').read_text())
ext = json.loads(Path('calib/c920-handeye-03/extrinsics.json').read_text())
intr = json.loads(Path('calib/c920-intrinsics-01/intrinsics.json').read_text())
T_cb = np.linalg.inv(np.array(ext['T_base_camera']))
K, D = np.array(intr['K']), np.array(intr['D'])


def proj(p):
    pc = T_cb[:3, :3] @ np.asarray(p) + T_cb[:3, 3]
    uv, _ = cv2.projectPoints(pc.reshape(1, 3), np.zeros(3), np.zeros(3), K, D)
    return tuple(int(round(v)) for v in uv.ravel())


with robot.ServoBus() as bus:
    ticks = bus.read_positions()
p, T = g.grasp_point(g.q_from_ticks(ticks))
cube = np.array(state['target_base_m'])
above = np.array([cube[0], cube[1], p[2]])
img = cv2.imread(str(S / 'live/env.png'))
for pt, col, name in [(cube, (0, 200, 0), 'cube'), (above, (0, 220, 255), 'above cube'),
                      (p, (255, 0, 255), 'FK grasp'), (T[:3, 3], (255, 255, 0), 'FK tcp')]:
    u = proj(pt)
    cv2.drawMarker(img, u, col, cv2.MARKER_CROSS, 30, 3)
    cv2.putText(img, name, (u[0] + 12, u[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)
    print(name, np.round(np.asarray(pt) * 1000, 1), 'px', u)
x0, y0 = proj(above)
crop = img[max(0, y0 - 300):y0 + 300, max(0, x0 - 400):x0 + 400]
cv2.imwrite(str(S / 'overlay.png'), crop)
