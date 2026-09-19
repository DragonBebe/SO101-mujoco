"""Localise cubes in the latest saved env frame (raw canonical), undistorted with the calibration."""
import sys, json
import cv2, numpy as np
from rgbcal import realcam
from rgbcal.pointcheck import cube_candidates
S = sys.argv[1]; colour = sys.argv[2]
cal = realcam.load()
K_new, maps = cal.undistortion()
res = []
for k in range(3):
    raw = cv2.imread(f'{S}/live/env.png')
    frame = {'undistorted': cv2.remap(raw, *maps, cv2.INTER_LINEAR),
             'calibration': cal.nexus_calibration(K_new)}
    for c in cube_candidates(frame, cal, colour):
        res.append((c['pixel'], np.round(np.array(c['footprint_m']) * 1000), np.round(np.array(c['grasp_centre_base']) * 1000, 1)))
    import time; time.sleep(0.6)
for r in res: print(colour, *r)
