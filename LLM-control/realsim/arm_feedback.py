"""Latest-only local feedback publication; never commands a servo.

Can run inside the existing ServoBus owner. SDK transactions are serialized
by ServoBus, so this worker does not open a second port.
"""
from pathlib import Path
import json
import os
import threading
import time
import uuid

SCHEMA = 'realsim/arm-feedback/1'


def clock_id():
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + f'.{os.getpid()}.{threading.get_ident()}.tmp')
    try:
        temporary.write_text(json.dumps(value, allow_nan=False) + '\n')
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


class FeedbackPublisher:
    def __init__(self, bus, path, rate=30., log=None):
        if not 0 < rate <= 200:
            raise ValueError('sample rate must be in (0, 200] Hz')
        self.bus, self.path, self.rate = bus, Path(path), rate
        self.log = None
        self.stop_event = threading.Event()
        self.stream_id = str(uuid.uuid4())
        self.clock = clock_id()
        self.sequence = 0
        self.last = None
        # Single publisher per output, including separate control processes.
        import fcntl
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.owner = open(str(self.path) + '.lock', 'a')
        try:
            fcntl.flock(self.owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
            if log:
                Path(log).parent.mkdir(parents=True, exist_ok=True)
                self.log = open(log, 'x', buffering=1)
        except Exception:
            self.owner.close()
            raise
        self.thread = threading.Thread(target=self.run, name='arm-feedback', daemon=True)

    def start(self):
        self.thread.start()
        return self

    def emit(self, record):
        atomic_json(self.path, record)
        if self.log:
            self.log.write(json.dumps(record, allow_nan=False) + '\n')
        self.last = record

    def run(self):
        while not self.stop_event.is_set():
            start = time.monotonic()
            self.sequence += 1
            record = {'schema': SCHEMA, 'stream_id': self.stream_id,
                      'sequence': self.sequence, 'clock_id': self.clock,
                      'sampled_at': time.time(), 'sampled_monotonic': start,
                      'source': 'ServoBus Present_Position (actual feedback)', 'valid': False}
            try:
                # Include registers in every sample: recalibration must not go unnoticed.
                registers = self.bus.read_state()
                record.update(valid=True, registers=registers,
                              ticks={str(k): v['present_position'] for k, v in registers.items()})
            except Exception as error:
                record.update(error=str(error), status='disconnected')
            record.update(received_at=time.time(), received_monotonic=time.monotonic(),
                          sampling_span_s=time.monotonic() - start)
            try:
                self.emit(record)
            except (OSError, ValueError) as error:
                # Publication is optional; its failure cannot abort or command the controller.
                print(f'arm-feedback publication failed: {error}', flush=True)
            self.stop_event.wait(max(0, 1 / self.rate - (time.monotonic() - start)))

    def close(self):
        self.stop_event.set()
        self.thread.join()
        try:
            self.sequence += 1
            self.emit({'schema': SCHEMA, 'stream_id': self.stream_id,
                       'sequence': self.sequence, 'clock_id': self.clock,
                       'valid': False, 'status': 'disconnected', 'error': 'publisher closed',
                       'sampled_at': time.time(), 'sampled_monotonic': time.monotonic(),
                       'received_at': time.time(), 'received_monotonic': time.monotonic()})
        finally:
            if self.log:
                self.log.close()
            self.owner.close()
