"""Live side-by-side view: environment C920 (calibrated profile) + wrist camera.
Also saves the latest frames every 0.5 s to --out (env.png, wrist.png), so
other processes can look without opening the cameras.  q / Esc quits."""
import argparse, time
from pathlib import Path
import cv2
import numpy as np
from rgbcal import cli
from rgbcal.capture import RealCamera

WRIST = '/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._USB2.0_CAM1_USB2.0_CAM1-video-index0'

ap = argparse.ArgumentParser()
ap.add_argument('--out', required=True)
a = ap.parse_args()
out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
env = RealCamera(cli._settings(argparse.Namespace(profile='c920')))
env.warm_up(10)
wrist = cv2.VideoCapture(WRIST, cv2.CAP_V4L2)
wrist.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
wrist.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
wrist.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cv2.namedWindow('SO101 cameras', cv2.WINDOW_NORMAL)
cv2.resizeWindow('SO101 cameras', 1600, 450)
last = 0.0
try:
    while True:
        e = env.canonical_frame()
        ok, w = wrist.read()
        if not ok or w is None:
            w = np.zeros((480, 640, 3), np.uint8)
            cv2.putText(w, 'wrist camera: no frame', (20, 240), cv2.FONT_HERSHEY_SIMPLEX,
                        1, (0, 0, 255), 2)
        h = 540
        es = cv2.resize(e, (int(e.shape[1] * h / e.shape[0]), h))
        ws = cv2.resize(w, (int(w.shape[1] * h / w.shape[0]), h))
        cv2.putText(es, 'environment C920', (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(ws, 'wrist', (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow('SO101 cameras', np.hstack([es, ws]))
        now = time.time()
        if now - last > 0.5:
            cv2.imwrite(str(out / 'env.tmp.png'), e); Path(out / 'env.tmp.png').replace(out / 'env.png')
            cv2.imwrite(str(out / 'wrist.tmp.png'), w); Path(out / 'wrist.tmp.png').replace(out / 'wrist.png')
            last = now
        if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
            break
finally:
    env.close(); wrist.release(); cv2.destroyAllWindows()
