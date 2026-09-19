#!/usr/bin/env python3
"""Persistent SO101 workbench controlled by the current Codex conversation."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
# Resolve the vendored package locally, even after moving the repository.
sys.path.insert(0, str(ROOT / 'third_party/so101-nexus/src'))


def main():
    from nexus_vision.cameras import MODALITIES, PLACEMENTS
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='mode', required=True)
    server = sub.add_parser('serve', help='Keep one multi-object world alive')
    server.add_argument('--seed', type=int, default=4)
    server.add_argument('--viewer', action='store_true')
    server.add_argument('--realtime', action='store_true')
    server.add_argument('--camera-viewer', action='store_true')
    # Modality and environment placement are independent and fixed per run;
    # the defaults reproduce the original RGB-D overhead setup exactly.
    server.add_argument('--camera-modality', choices=MODALITIES, default='rgbd',
                        help='rgb: colour only from both cameras; rgbd: colour plus depth')
    server.add_argument('--environment-camera', choices=PLACEMENTS, default='overhead',
                        help='side: world-fixed oblique desk view; overhead: original top-down view; '
                             'calibrated: the measured real C920 camera (rgb modality only)')
    server.add_argument('--record', default=str(ROOT / 'runs' / datetime.now().strftime('loop-%Y%m%d-%H%M%S-%f')))
    server.set_defaults(task='workbench')
    client = sub.add_parser('command', help='Send one JSON command; use - for stdin')
    client.add_argument('json')
    status = sub.add_parser('status', help='Read the current task without advancing physics')
    for item in (server, client, status):
        item.add_argument('--socket', default=str(ROOT / '.runtime/loop.sock'))
    args = parser.parse_args()
    try:
        from nexus_vision.bridge import loads, send, serve
        if args.mode == 'serve':
            from nexus_vision.loop_session import LoopSession
            serve(args, session_factory=LoopSession)
            return 0
        if args.mode == 'status':
            command = {'action': 'status'}
        else:
            payload = sys.stdin.read(65537) if args.json == '-' else args.json
            if len(payload.encode('utf-8')) > 65536:
                raise ValueError('Command exceeds 64 KiB')
            command = loads(payload)
        result = send(Path(args.socket).resolve(), command)
        print(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False))
        return 0 if result['ok'] else 1
    except (OSError, ValueError, RuntimeError, ImportError) as exc:
        print(f'Error: {exc}', file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130


if __name__ == '__main__':
    raise SystemExit(main())
