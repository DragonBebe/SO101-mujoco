"""Stage 1: is the camera pointing at the right thing, from the right place?

Nothing here measures the camera.  This is the viewpoint check that has to
pass *before* any board images are worth collecting, because a calibration
is bound to a viewpoint: once the laptop moves, the extrinsics are gone.
The window shows what the camera really delivers, with the timing and
format facts overlaid, and it saves untouched frames on request so a
viewpoint candidate can be compared against another one later.
"""
from pathlib import Path
import json
import time

import cv2
import numpy as np

from . import boards as boards_module
from .capture import (RealCamera, exposure_report, save_frame, sharpness,
                      timestamp, transform_image)

HUD_BACKGROUND = (24, 28, 36)
KEY_HELP = ('s save raw frame | b board overlay | g framing guides | '
            'h hud | q quit')


def _put(canvas, lines, origin=(10, 10)):
    x, y = origin
    height = 18 * len(lines) + 10
    width = max(int(9.0 * len(line)) for line in lines) + 16
    overlay = canvas.copy()
    cv2.rectangle(overlay, (x, y), (x + width, y + height), HUD_BACKGROUND, -1)
    cv2.addWeighted(overlay, 0.65, canvas, 0.35, 0, canvas)
    for index, line in enumerate(lines):
        cv2.putText(canvas, line, (x + 8, y + 20 + 18 * index),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (235, 240, 250), 1, cv2.LINE_AA)


def framing_guides(canvas, margin=0.10):
    """Thirds plus a margin rectangle.

    The margin is the practical rule for stage 1: the working area has to
    sit inside it, so an object at the far edge of the workspace, or a
    raised gripper, does not leave the frame.
    """
    height, width = canvas.shape[:2]
    inset = (int(width * margin), int(height * margin))
    cv2.rectangle(canvas, inset, (width - inset[0], height - inset[1]), (90, 200, 90), 1)
    for fraction in (1 / 3, 2 / 3):
        cv2.line(canvas, (int(width * fraction), 0), (int(width * fraction), height),
                 (70, 70, 70), 1)
        cv2.line(canvas, (0, int(height * fraction)), (width, int(height * fraction)),
                 (70, 70, 70), 1)
    cv2.putText(canvas, f'{int(margin * 100)}% margin', (inset[0] + 6, inset[1] + 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (90, 200, 90), 1, cv2.LINE_AA)


class FrameRate:
    """Measured display-loop rate; the driver's advertised FPS is a claim."""

    def __init__(self, window=30):
        self.times = []
        self.window = window

    def tick(self):
        self.times.append(time.perf_counter())
        if len(self.times) > self.window:
            self.times.pop(0)
        if len(self.times) < 2:
            return float('nan')
        span = self.times[-1] - self.times[0]
        return (len(self.times) - 1) / span if span > 0 else float('nan')


def run(settings, directory, board=None, label='view', margin=0.10, guides=True,
        show_board=None, labels=None):
    """Open the live window.  Returns the session directory it wrote to.

    ``labels`` names the shots to take, in order.  One cube moved through
    five positions is the same evidence as five cubes, but only if each
    frame says which position it is; the window shows the next name so the
    record cannot drift from what actually happened.
    """
    directory = Path(directory)
    pending = list(labels) if labels else []
    raw_dir = directory / 'frames'
    annotated_dir = directory / 'annotated'
    saved = 0
    show_hud = True
    show_board = bool(board) if show_board is None else show_board
    rate = FrameRate()

    with RealCamera(settings) as camera:
        camera.warm_up()
        camera.measure_fps(20)
        configuration = camera.configuration()
        (directory).mkdir(parents=True, exist_ok=True)
        (directory / 'camera_settings.json').write_text(
            json.dumps(configuration, indent=2) + '\n')
        window = f'SO101 environment camera - {settings.device}'
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window, configuration['resolution'][0],
                         configuration['resolution'][1])
        print(f'preview: {window}')
        print(f'  resolution {configuration["resolution"]} '
              f'{configuration["pixel_format"]} driver fps {configuration["driver_fps"]:.1f} '
              f'measured {configuration["measured_fps"]:.1f}')
        print(f'  raw frames -> {raw_dir}')
        print(f'  keys: {KEY_HELP}')
        if pending:
            print('  shots to take, in order: ' + ', '.join(pending))

        while True:
            try:
                # ``frame`` is the archived evidence; ``view`` is the frame the
                # calibration will describe.  They differ only by the declared
                # transform, and only ``view`` is ever measured or displayed.
                frame = camera.frame()
            except RuntimeError as error:
                print(f'capture error: {error}')
                break
            view = transform_image(frame, settings)
            measured = rate.tick()
            detection = None
            if show_board and board is not None:
                detection = boards_module.detect(board, view)
                canvas = boards_module.draw(view, board, detection)
            else:
                canvas = view.copy()
            if guides:
                framing_guides(canvas, margin)
            if show_hud:
                exposure = exposure_report(view)
                lines = [
                    f'{settings.device}  {frame.shape[1]}x{frame.shape[0]}  '
                    f'{configuration["pixel_format"]}'
                    + ('  [un-mirrored for display]' if settings.hflip else ''),
                    f'fps driver {configuration["driver_fps"]:.1f} | live {measured:.1f}',
                    timestamp(),
                    f'sharpness {sharpness(view):7.1f}  mean {exposure["mean"]:5.1f}  '
                    f'clip hi {exposure["clipped_high"]*100:4.1f}% lo '
                    f'{exposure["clipped_low"]*100:4.1f}%',
                    (f'NEXT SHOT: {pending[0]}   ({len(pending)} left)' if pending
                     else f'saved {saved} raw frames  [{label}]'),
                ]
                if detection is not None:
                    lines.append('board: ' + (detection.note if detection.ok
                                              else f'NOT DETECTED ({detection.note})'))
                lines.append(KEY_HELP)
                _put(canvas, lines)
            cv2.imshow(window, canvas)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == ord('s'):
                saved += 1
                shot = pending.pop(0) if pending else label
                stem = (f'{shot}' if shot != label else f'{label}-{saved:03d}')
                if (raw_dir / f'{stem}.png').exists():
                    stem = f'{stem}-{saved:03d}'
                extra = {'label': shot, 'sequence_position': saved, 'sharpness': sharpness(view),
                         'exposure': exposure_report(view), 'stage': 'viewpoint',
                         'image_transform': camera.transform()}
                if detection is not None:
                    extra['board_detected'] = detection.ok
                    extra['board_note'] = detection.note
                path = save_frame(frame, raw_dir, stem, extra, configuration)
                annotated_dir.mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(annotated_dir / f'{stem}-annotated.png'), canvas)
                print(f'saved {path}')
            if key == ord('b') and board is not None:
                show_board = not show_board
            if key == ord('g'):
                guides = not guides
            if key == ord('h'):
                show_hud = not show_hud
        cv2.destroyAllWindows()
    return directory


def snap(settings, directory, count=1, label='snap', board=None, settle=15):
    """Headless capture: the same frames, without a window.

    Used to inspect the view over a remote or scripted session, and to
    record a viewpoint candidate without touching the live preview.
    """
    directory = Path(directory)
    results = []
    with RealCamera(settings) as camera:
        camera.warm_up(settle)
        camera.measure_fps(20)
        configuration = camera.configuration()
        directory.mkdir(parents=True, exist_ok=True)
        (directory / 'camera_settings.json').write_text(
            json.dumps(configuration, indent=2) + '\n')
        for index in range(count):
            frame = camera.frame()
            view = transform_image(frame, settings)
            stem = f'{label}-{index + 1:03d}'
            extra = {'label': label, 'sharpness': sharpness(view),
                     'exposure': exposure_report(view), 'stage': 'viewpoint',
                     'image_transform': camera.transform()}
            if board is not None:
                detection = boards_module.detect(board, view)
                extra['board_detected'] = detection.ok
                extra['board_note'] = detection.note
                extra['board_points'] = detection.count
                annotated = boards_module.draw(view, board, detection)
                (directory / 'annotated').mkdir(parents=True, exist_ok=True)
                cv2.imwrite(str(directory / 'annotated' / f'{stem}-annotated.png'), annotated)
            path = save_frame(frame, directory / 'frames', stem, extra, configuration)
            results.append({'path': str(path), **extra})
            if index + 1 < count:
                time.sleep(0.2)
    return configuration, results
