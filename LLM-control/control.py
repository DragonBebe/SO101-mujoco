#!/usr/bin/env python3
"""Persistent local MuJoCo bridge for commands issued by the current Codex."""
import argparse
from contextlib import ExitStack
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import socket
import socketserver
import sys
import time

from simulation import Simulation

ROOT = Path(__file__).resolve().parent
DEFAULT_SOCKET = ROOT / '.runtime' / 'simulation.sock'


class Session:
    def __init__(self, record, viewer=False, realtime=False, capture=False):
        self.record = Path(record).resolve()
        self.record.mkdir(parents=True, exist_ok=True)
        self.stack = ExitStack()
        self.viewer = None
        self.realtime = realtime
        self.capture = capture
        self.running = True
        self.sequence = 0
        self.sim = Simulation(on_step=self.on_step)
        if viewer:
            import mujoco.viewer
            self.viewer = self.stack.enter_context(mujoco.viewer.launch_passive(self.sim.model, self.sim.data))
            self.viewer.cam.lookat[:] = [1.08, 0.65, 0.88]
            self.viewer.cam.distance = 0.85
            self.viewer.cam.azimuth = 135
            self.viewer.cam.elevation = -30
        self.log = self.stack.enter_context((self.record / 'actions.jsonl').open('x', encoding='utf-8'))
        self.save_state()

    def on_step(self, sim):
        if self.viewer:
            if not self.viewer.is_running():
                self.running = False
                raise RuntimeError('Viewer was closed during the command')
            self.viewer.sync()
            if self.realtime:
                time.sleep(sim.model.opt.timestep * 20)

    def save_state(self):
        state = self.sim.observe()
        (self.record / 'state.json').write_text(json.dumps(state, indent=2, allow_nan=False) + '\n')

    def execute(self, command):
        self.sequence += 1
        try:
            if not isinstance(command, dict):
                raise ValueError('Command must be a JSON object')
            if command.get('action') == 'shutdown':
                self.running = False
                observation = self.sim.observe()
            elif command.get('action') == 'snapshot':
                observation = self.sim.observe()
            else:
                observation = self.sim.execute(command)
            response = {'ok': True, 'sequence': self.sequence, 'observation': observation}
            if command.get('action') == 'snapshot' or (self.capture and command.get('action') not in ('observe', 'shutdown')):
                path = self.record / f'{self.sequence:03d}-{command["action"]}.png'
                try:
                    response['image'] = self.sim.snapshot(path)
                except Exception as exc:
                    # Rendering is optional: the physics command already ran.
                    response['capture_error'] = str(exc)
        except (ValueError, TypeError, RuntimeError) as exc:
            response = {'ok': False, 'sequence': self.sequence, 'error': str(exc),
                        'observation': self.sim.observe()}
        self.save_state()
        entry = {'wall_time': datetime.now(timezone.utc).isoformat(), 'command': command, **response}
        self.log.write(json.dumps(entry, ensure_ascii=False, allow_nan=False) + '\n')
        self.log.flush()
        return response

    def close(self):
        self.stack.close()


def strict_json(payload):
    def bad_constant(value):
        raise ValueError(f'Nonfinite JSON number: {value}')
    def finite_float(value):
        result = float(value)
        if not math.isfinite(result):
            raise ValueError('JSON floating point value is out of range')
        return result
    return json.loads(payload, parse_constant=bad_constant, parse_float=finite_float)


def serve(args):
    address = Path(args.socket).resolve()
    address.parent.mkdir(parents=True, exist_ok=True)
    if address.exists():
        raise ValueError(f'Socket already exists: {address}. Use the running server or another --socket path.')
    session = Session(args.record, args.viewer, args.realtime, args.capture)

    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            self.connection.settimeout(5)
            try:
                raw = self.rfile.readline(65537)
                if len(raw) > 65536:
                    raise ValueError('Command exceeds 64 KiB')
                command = strict_json(raw)
                response = session.execute(command)
            except (ValueError, UnicodeError, socket.timeout) as exc:
                response = {'ok': False, 'error': str(exc)}
            try:
                self.wfile.write(json.dumps(response, allow_nan=False).encode() + b'\n')
            except (BrokenPipeError, ConnectionResetError):
                pass

    try:
        with socketserver.UnixStreamServer(str(address), Handler) as server:
            address.chmod(0o600)
            server.timeout = 0.05
            print(json.dumps({'ready': True, 'socket': str(address), 'record': str(session.record)}), flush=True)
            while session.running:
                server.handle_request()
                if session.viewer:
                    if not session.viewer.is_running():
                        break
                    session.viewer.sync()
    finally:
        session.close()
        address.unlink(missing_ok=True)


def send(args):
    payload = strict_json(args.json)
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(120)
        connection.connect(str(Path(args.socket).resolve()))
        connection.sendall(json.dumps(payload, allow_nan=False).encode() + b'\n')
        with connection.makefile('rb') as stream:
            result = strict_json(stream.readline(1024*1024))
    print(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False))
    return 0 if result.get('ok') else 1


def replay(args):
    entries = [strict_json(line) for line in Path(args.trace).read_text().splitlines() if line.strip()]
    session = Session(args.record, args.viewer, args.realtime, args.capture)
    try:
        for entry in entries:
            command = entry['command']
            if not entry.get('ok') or command['action'] in ('observe', 'snapshot', 'shutdown'):
                continue
            result = session.execute(command)
            print(json.dumps({'command': command, **result}, ensure_ascii=False), flush=True)
            if not result['ok']:
                return 1
        return 0 if session.sim.observe()['success'] else 1
    finally:
        session.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='mode', required=True)
    for name in ('serve', 'replay'):
        cmd = commands.add_parser(name)
        cmd.add_argument('--viewer', action='store_true', help='Open the existing MuJoCo viewer')
        cmd.add_argument('--realtime', action='store_true', help='Play commands near real time with the viewer')
        cmd.add_argument('--capture', action='store_true', help='Save a PNG after each action (requires OpenGL)')
        cmd.add_argument('--record', default=str(ROOT / 'runs' / datetime.now().strftime('%Y%m%d-%H%M%S-%f')))
        if name == 'serve':
            cmd.add_argument('--socket', default=str(DEFAULT_SOCKET))
        else:
            cmd.add_argument('trace', help='Saved actions.jsonl; replay makes no LLM calls')
    command = commands.add_parser('command')
    command.add_argument('json', help='One JSON command')
    command.add_argument('--socket', default=str(DEFAULT_SOCKET))
    args = parser.parse_args()
    try:
        if args.mode == 'serve':
            serve(args)
            return 0
        return send(args) if args.mode == 'command' else replay(args)
    except (OSError, ValueError, RuntimeError) as exc:
        print(f'Error: {exc}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == '__main__':
    raise SystemExit(main())
