"""One-shot, read-only localisation test: camera -> cubes in base frame. No arm access."""
import argparse, json, sys
from pathlib import Path
import cv2
import numpy as np
from rgbcal import cli, realcam
from rgbcal.pointcheck import cube_candidates

p = argparse.ArgumentParser()
p.add_argument('--out', required=True)
p.add_argument('--support-height', type=float, default=0.0)
p.add_argument('--colours', default='red,orange,yellow,green,blue')
a = p.parse_args()
settings = cli._settings(argparse.Namespace(profile='c920'))
cal = realcam.load()
status = realcam.motion_status(cal, settings)
results = []
with realcam.EnvironmentCamera(settings, cal) as cam:
    frames = [cam.frame() for _ in range(5)]
frame = frames[-1]
view = frame['undistorted'].copy()
for colour in a.colours.split(','):
    # Stability: re-run on 5 frames, report spread.
    per_frame = [cube_candidates(f, cal, colour, a.support_height) for f in frames]
    for c in per_frame[-1]:
        g = np.array(c['grasp_centre_base'])
        same = [np.array(o['grasp_centre_base']) for fc in per_frame for o in fc
                if np.linalg.norm(np.array(o['grasp_centre_base']) - g) < 0.02]
        spread = float(np.max(np.std(same, axis=0)) * 1000) if len(same) > 1 else None
        c.update(colour=colour, frames_seen=len(same), jitter_mm=spread)
        results.append(c)
        x0, y0, x1, y1 = c['bbox']
        col = (60, 220, 60) if c['plausible_cube'] else (60, 60, 220)
        cv2.rectangle(view, (x0, y0), (x1, y1), col, 2)
        cv2.putText(view, f"{colour} ({g[0]*1000:.0f},{g[1]*1000:.0f},{g[2]*1000:.0f})mm",
                    (x0, max(y0 - 8, 20)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2, cv2.LINE_AA)
out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
cv2.imwrite(str(out / 'annotated.png'), view)
cv2.imwrite(str(out / 'raw.png'), frame['raw'])
report = {'captured_at': frame['captured_at'], 'motion_status': status, 'candidates': results}
(out / 'result.json').write_text(json.dumps(report, indent=2))
print('motion:', status)
for c in results:
    g = np.array(c['grasp_centre_base']) * 1000
    print(f"{c['colour']:6s} cube={c['plausible_cube']!s:5s} px={c['pixel']} "
          f"footprint={np.round(np.array(c['footprint_m'])*1000,1)}mm "
          f"grasp_base=({g[0]:.1f},{g[1]:.1f},{g[2]:.1f})mm seen={c['frames_seen']}/5 "
          f"jitter={c['jitter_mm'] and round(c['jitter_mm'],2)}mm")
