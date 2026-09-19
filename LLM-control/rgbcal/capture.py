"""Opening the real environment camera and recording exactly what it gave.

Every calibration number is only valid for one imaging configuration.  A
webcam silently renegotiates resolution, switches pixel format, refocuses
and rebalances exposure, and any of those changes can invalidate the
intrinsics that were measured earlier.  So this module never assumes a
setting took effect: it asks the driver back for the value it ended up
with, measures the frame rate it actually delivers, and writes both into
a sidecar next to every saved frame.

Frames are stored exactly as the driver produced them.  No rotation, no
mirroring, no cropping, no resizing: those operations change the
intrinsics, and a calibration bound to an undocumented transform is worse
than no calibration.
"""
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
import json
import time

import cv2
import numpy as np

#: Driver properties worth recording.  Read back after opening; a webcam
#: that ignores a request reports the value it kept.
PROPERTIES = {
    'frame_width': cv2.CAP_PROP_FRAME_WIDTH,
    'frame_height': cv2.CAP_PROP_FRAME_HEIGHT,
    'fps': cv2.CAP_PROP_FPS,
    'fourcc': cv2.CAP_PROP_FOURCC,
    'autofocus': cv2.CAP_PROP_AUTOFOCUS,
    'focus': cv2.CAP_PROP_FOCUS,
    'auto_exposure': cv2.CAP_PROP_AUTO_EXPOSURE,
    'exposure': cv2.CAP_PROP_EXPOSURE,
    'brightness': cv2.CAP_PROP_BRIGHTNESS,
    'contrast': cv2.CAP_PROP_CONTRAST,
    'saturation': cv2.CAP_PROP_SATURATION,
    'gain': cv2.CAP_PROP_GAIN,
    'gamma': cv2.CAP_PROP_GAMMA,
    'sharpness': cv2.CAP_PROP_SHARPNESS,
    'auto_wb': cv2.CAP_PROP_AUTO_WB,
    'wb_temperature': cv2.CAP_PROP_WB_TEMPERATURE,
    'zoom': cv2.CAP_PROP_ZOOM,
    'backend': cv2.CAP_PROP_BACKEND,
    'buffersize': cv2.CAP_PROP_BUFFERSIZE,
}


def fourcc_text(value):
    code = int(value)
    if code <= 0:
        return ''
    return ''.join(chr((code >> shift) & 0xFF) for shift in (0, 8, 16, 24))


def timestamp():
    return datetime.now(timezone.utc).astimezone().isoformat(timespec='milliseconds')


@dataclass
class CameraSettings:
    """What we ask the driver for.  What we get is measured, not assumed."""

    device: str = '/dev/video0'
    width: int = 1280
    height: int = 720
    fps: float = 30.0
    fourcc: str = 'MJPG'
    #: ``None`` leaves the driver's own automatic behaviour alone.  Autofocus
    #: and auto-exposure move the optical centre and the effective focal
    #: length between frames, so calibration runs should pin them down.
    autofocus: bool | None = None
    focus: float | None = None
    auto_exposure: bool | None = None
    exposure: float | None = None
    auto_wb: bool | None = None
    #: Some laptop panel cameras deliver a horizontally mirrored stream.  A
    #: mirror is not a rigid transform: it makes the camera frame left-handed,
    #: so ArUco codes stop decoding and ``solvePnP`` returns a pose that no
    #: rotation to the robot base can match.  Undoing it is therefore part of
    #: the imaging configuration, declared here and recorded with every frame
    #: and every calibration file, never applied silently.
    hflip: bool = False

    def as_dict(self):
        return {'device': self.device, 'width': self.width, 'height': self.height,
                'fps': self.fps, 'fourcc': self.fourcc, 'autofocus': self.autofocus,
                'focus': self.focus, 'auto_exposure': self.auto_exposure,
                'exposure': self.exposure, 'auto_wb': self.auto_wb,
                'hflip': self.hflip}


class RealCamera:
    """A V4L2 camera that reports its real configuration.

    ``frame()`` returns BGR exactly as delivered.  ``configuration()`` is
    the record that must travel with every calibration file: intrinsics
    measured at 1280x720 MJPG mean nothing at 640x480 YUYV.
    """

    def __init__(self, settings):
        self.settings = settings
        self.capture = cv2.VideoCapture(settings.device, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            raise RuntimeError(
                f'cannot open {settings.device}. Check the device list '
                f'(rgbcal devices) and that nothing else holds the camera.')
        self._apply(settings)
        self._measured_fps = None
        self.opened_at = timestamp()

    def _apply(self, settings):
        capture = self.capture
        if settings.fourcc:
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc(*settings.fourcc))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, settings.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, settings.height)
        if settings.fps:
            capture.set(cv2.CAP_PROP_FPS, settings.fps)
        # A short queue keeps the preview showing the present, not the past.
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if settings.autofocus is not None:
            capture.set(cv2.CAP_PROP_AUTOFOCUS, 1 if settings.autofocus else 0)
        if settings.focus is not None:
            capture.set(cv2.CAP_PROP_FOCUS, float(settings.focus))
        if settings.auto_exposure is not None:
            # V4L2 UVC: 3 = aperture priority (auto), 1 = manual.
            capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3 if settings.auto_exposure else 1)
        if settings.exposure is not None:
            capture.set(cv2.CAP_PROP_EXPOSURE, float(settings.exposure))
        if settings.auto_wb is not None:
            capture.set(cv2.CAP_PROP_AUTO_WB, 1 if settings.auto_wb else 0)

    def frame(self):
        """The frame exactly as the driver delivered it."""
        ok, image = self.capture.read()
        if not ok or image is None:
            raise RuntimeError('camera returned no frame')
        return image

    def canonical_frame(self):
        """The frame in the orientation every calibrated number refers to.

        Detection, calibration and localisation all use this one.  Raw
        samples are still archived as delivered, so a session can be audited
        without trusting this transform.
        """
        return transform_image(self.frame(), self.settings)

    def warm_up(self, frames=10):
        """Discard the first frames; auto-exposure needs a moment to settle."""
        for _ in range(frames):
            try:
                self.frame()
            except RuntimeError:
                time.sleep(0.05)

    def measure_fps(self, frames=30):
        start = time.perf_counter()
        for _ in range(frames):
            self.frame()
        elapsed = time.perf_counter() - start
        self._measured_fps = frames / elapsed if elapsed > 0 else float('nan')
        return self._measured_fps

    def actual(self):
        """Driver values read back after opening."""
        values = {}
        for name, prop in PROPERTIES.items():
            value = self.capture.get(prop)
            values[name] = value
        values['fourcc_text'] = fourcc_text(values['fourcc'])
        return values

    def configuration(self):
        """The full record that binds a calibration to an imaging setup."""
        actual = self.actual()
        return {
            'requested': self.settings.as_dict(),
            'actual': actual,
            'resolution': [int(actual['frame_width']), int(actual['frame_height'])],
            'pixel_format': actual['fourcc_text'],
            'driver_fps': actual['fps'],
            'measured_fps': self._measured_fps,
            'image_transform': self.transform(),
            'opened_at': self.opened_at,
        }

    def transform(self):
        """The declared pixel transform from delivered frame to canonical frame."""
        return image_transform(self.settings)

    def close(self):
        self.capture.release()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def image_transform(settings):
    """The one description of how a delivered frame becomes a canonical one."""
    return {'hflip': bool(settings.hflip), 'rotation_deg': 0, 'crop': None,
            'resized': False,
            'note': ('raw samples are archived exactly as delivered; '
                     'apply hflip to reach the frame the calibration describes')}


def transform_image(image, settings):
    """Apply the declared transform.  The only place a frame is ever flipped."""
    return cv2.flip(image, 1) if settings.hflip else image


def save_frame(image, directory, stem, extra=None, configuration=None):
    """Write a lossless PNG plus its sidecar, refusing to overwrite.

    Raw samples are evidence.  A calibration that silently re-used a
    filename cannot be audited afterwards, so an existing name is an error,
    not a warning.
    """
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    image_path = directory / f'{stem}.png'
    meta_path = directory / f'{stem}.json'
    if image_path.exists() or meta_path.exists():
        raise FileExistsError(f'{image_path} already exists; raw samples are never overwritten')
    if not cv2.imwrite(str(image_path), image):
        raise RuntimeError(f'failed to write {image_path}')
    meta = {'image': image_path.name, 'saved_at': timestamp(),
            'shape': list(image.shape), 'dtype': str(image.dtype)}
    if configuration is not None:
        meta['camera'] = configuration
    if extra:
        meta.update(extra)
    meta_path.write_text(json.dumps(meta, indent=2, ensure_ascii=False) + '\n')
    return image_path


def sharpness(image):
    """Variance of the Laplacian: a blur score for rejecting soft samples."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def exposure_report(image):
    """Clipping and brightness, so a washed-out sample is caught early."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    total = gray.size
    return {'mean': float(gray.mean()),
            'clipped_high': float(np.count_nonzero(gray >= 250) / total),
            'clipped_low': float(np.count_nonzero(gray <= 5) / total)}


#: Settings that change the imaging geometry.  Exposure and white balance do
#: not move a single pixel and are deliberately absent.
GEOMETRIC_SETTINGS = ('device', 'width', 'height', 'autofocus', 'focus', 'hflip')


def check_compatible(calibration, settings):
    """Refuse to pair intrinsics with a capture they do not describe.

    A mismatch here is not a warning.  Intrinsics measured at 1920x1080 and
    focus 0 are simply wrong for a 1280x720 frame or a refocused lens, and
    every 3D number built on them would be wrong without looking wrong.
    """
    recorded = calibration.get('imaging') or {}
    problems = []
    resolution = calibration.get('resolution', {})
    if [resolution.get('width'), resolution.get('height')] != [settings.width,
                                                                settings.height]:
        problems.append(f"resolution {resolution.get('width')}x{resolution.get('height')}"
                        f' calibrated, {settings.width}x{settings.height} requested')
    transform = calibration.get('image_transform', {})
    if bool(transform.get('hflip', False)) != bool(settings.hflip):
        problems.append('mirroring differs from the calibration')
    for key in ('device', 'autofocus', 'focus'):
        if key in recorded and recorded[key] != getattr(settings, key):
            problems.append(f'{key}: calibrated with {recorded[key]!r}, '
                            f'now {getattr(settings, key)!r}')
    if problems:
        raise RuntimeError('camera configuration does not match the intrinsics: '
                           + '; '.join(problems))
