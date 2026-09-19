"""Which physical camera is which, before anything is calibrated.

The laptop has more than one V4L2 node and more than one camera: the
built-in panel camera that becomes the *environment* camera, and a USB
camera that has been used as the *wrist* camera.  Calibrating the wrong
one silently produces intrinsics that look fine and are useless, so this
module reports identity from the kernel (device name, USB vendor/product,
bus path) instead of trusting an index, and it says which nodes actually
deliver frames and which are already open in another process.
"""
from pathlib import Path
import os
import re

V4L2_ROOT = Path('/sys/class/video4linux')
#: Names that a built-in laptop panel camera usually reports.
_BUILTIN_HINTS = re.compile(r'integrated|built[\s_-]?in|facetime|hd user facing', re.I)


def _read(path):
    try:
        return path.read_text().strip()
    except OSError:
        return ''


def _usb_identity(node):
    """Vendor/product/serial of the USB device behind a V4L2 node."""
    device = (node / 'device').resolve()
    for _ in range(6):
        vendor = device / 'idVendor'
        if vendor.exists():
            return {'usb_id': f'{_read(vendor)}:{_read(device / "idProduct")}',
                    'usb_product': _read(device / 'product'),
                    'usb_manufacturer': _read(device / 'manufacturer'),
                    'usb_serial': _read(device / 'serial'),
                    'usb_path': device.name}
        if device.parent == device:
            break
        device = device.parent
    return {'usb_id': '', 'usb_product': '', 'usb_manufacturer': '',
            'usb_serial': '', 'usb_path': ''}


def holders(path):
    """PIDs with ``path`` open, so a second program never fights for the device."""
    found = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            for fd in (entry / 'fd').iterdir():
                if os.readlink(fd) == str(path):
                    found.append({'pid': int(entry.name),
                                  'command': _read(entry / 'comm')})
                    break
        except (OSError, PermissionError):
            continue
    return found


def enumerate_nodes():
    """Every V4L2 node with its kernel identity, sorted by device path.

    ``role_hint`` is a guess from the reported name only.  It is a starting
    point for the operator to confirm, never an authority: the wrist camera
    of another SO101 could be named anything.
    """
    nodes = []
    for node in sorted(V4L2_ROOT.glob('video*'), key=lambda p: int(p.name[5:])):
        path = Path('/dev') / node.name
        name = _read(node / 'name')
        entry = {'device': str(path), 'node': node.name,
                 'name': name, 'index': _read(node / 'index'),
                 'role_hint': ('laptop_builtin' if _BUILTIN_HINTS.search(name)
                               else 'external_usb'),
                 'busy': holders(path)}
        entry.update(_usb_identity(node))
        nodes.append(entry)
    return nodes


def probe_capture(device, backend=None):
    """Can this node actually hand over an image? Metadata nodes cannot.

    Each UVC camera exposes a capture node and a metadata node; both are
    ``/dev/videoN`` and only one of them returns frames.
    """
    import cv2

    api = cv2.CAP_V4L2 if backend is None else backend
    capture = cv2.VideoCapture(str(device), api)
    try:
        if not capture.isOpened():
            return {'opens': False, 'reads': False, 'shape': None}
        ok, frame = capture.read()
        shape = None if not ok or frame is None else list(frame.shape)
        return {'opens': True, 'reads': bool(ok and frame is not None), 'shape': shape}
    finally:
        capture.release()


def survey(probe=True):
    """Full device survey; ``probe`` opens each node briefly to test it."""
    nodes = enumerate_nodes()
    for entry in nodes:
        if probe and not entry['busy']:
            entry['capture'] = probe_capture(entry['device'])
        elif entry['busy']:
            entry['capture'] = {'opens': None, 'reads': None, 'shape': None,
                                'note': 'skipped: device is open in another process'}
    return nodes
