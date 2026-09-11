#!/usr/bin/env python3
"""Run Nexus RGB-D tasks controlled by commands from the current Codex."""
import argparse
from datetime import datetime
import json
from pathlib import Path
import sys
from nexus_vision.bridge import serve,send,loads
from nexus_vision.simulation import TASKS

ROOT=Path(__file__).resolve().parent


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='mode',required=True)
    server=sub.add_parser('serve')
    server.add_argument('--task',choices=TASKS,default='Touch')
    server.add_argument('--seed',type=int,default=4)
    server.add_argument('--record',default=str(ROOT/'runs'/datetime.now().strftime('vision-%Y%m%d-%H%M%S-%f')))
    server.add_argument('--viewer',action='store_true');server.add_argument('--realtime',action='store_true')
    client=sub.add_parser('command');client.add_argument('json')
    replay=sub.add_parser('replay',help='Replay recorded visual plans; no new LLM inference')
    replay.add_argument('--task',choices=['all',*TASKS],default='all')
    replay.add_argument('--record',default=str(ROOT/'runs'/datetime.now().strftime('vision-replay-%Y%m%d-%H%M%S-%f')))
    replay.add_argument('--viewer',action='store_true');replay.add_argument('--realtime',action='store_true')
    for p in (server,replay):
        p.add_argument('--camera-viewer',action='store_true',help='Show live overhead/wrist RGB and depth (MUJOCO_GL=glfw)')
    for p in (server,client):p.add_argument('--socket',default=str(ROOT/'.runtime/vision.sock'))
    args=parser.parse_args()
    try:
        if args.mode=='serve':serve(args);return 0
        if args.mode=='replay':
            from nexus_vision.replay import replay_case
            tasks=list(TASKS) if args.task=='all' else [args.task]
            results=[]
            for task in tasks:
                case=loads((ROOT/'examples/vision'/f'{task}.json').read_text())
                result=replay_case(case,Path(args.record)/task,args.viewer,args.realtime,args.camera_viewer)
                results.append(result)
                print(json.dumps(result,ensure_ascii=False),flush=True)
            (Path(args.record)/'summary.json').write_text(json.dumps(results,indent=2)+'\n')
            return 0 if all(r['success'] for r in results) else 1
        result=send(args.socket,loads(args.json))
        print(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False))
        return 0 if result['ok'] else 1
    except (OSError,ValueError,RuntimeError) as exc:
        print(f'Error: {exc}',file=sys.stderr);return 1
    except KeyboardInterrupt:return 130


if __name__=='__main__':raise SystemExit(main())
