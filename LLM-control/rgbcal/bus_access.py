"""Linux serial ownership and transaction locking; no servo register writes."""
from functools import wraps
from pathlib import Path
import fcntl
import hashlib
import os
import termios


def serialized(method):
    @wraps(method)
    def wrapped(self, *args, **kwargs):
        with self._io_lock:
            return method(self, *args, **kwargs)
    return wrapped


class PortOwnership:
    def __init__(self, port):
        path = Path(port).resolve()
        # Catch existing non-cooperating SDK/LeRobot processes before any open.
        for process in Path('/proc').glob('[0-9]*'):
            try:
                for fd in (process / 'fd').iterdir():
                    try:
                        if fd.resolve() == path:
                            raise RuntimeError(f'{path} already open by PID {process.name}; '
                                               'publish feedback from its owner instead')
                    except (FileNotFoundError, PermissionError):
                        pass
            except (FileNotFoundError, PermissionError):
                pass
        directory = Path('/tmp') / f'so101-serial-{os.getuid()}'
        directory.mkdir(mode=0o700, exist_ok=True)
        name = hashlib.sha256(str(path).encode()).hexdigest()
        self.lock = open(directory / name, 'a')
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            self.lock.close()
            raise RuntimeError(f'{path} already reserved by another SO101 process')
        self.serial = None

    def attach(self, serial):
        self.serial = serial
        # Kernel rejects subsequent opens, including non-cooperating processes.
        fcntl.ioctl(serial.fileno(), termios.TIOCEXCL)

    def close(self):
        if self.serial is not None and self.serial.is_open:
            fcntl.ioctl(self.serial.fileno(), termios.TIOCNXCL)
        self.lock.close()
