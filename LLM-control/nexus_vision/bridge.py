"""Local visual-only command service. finish seals an episode before scoring."""
import json
import math
from pathlib import Path
import socket
import socketserver
from datetime import datetime, timezone
from .simulation import VisualSimulation, TASKS
from .evaluate import evaluate


def loads(payload):
    def finite(value):
        parsed=float(value)
        if not math.isfinite(parsed):raise ValueError('JSON contains a nonfinite number')
        return parsed
    return json.loads(payload,parse_constant=finite,parse_float=finite)


class VisualSession:
    def __init__(self, directory, task='Touch', seed=4, viewer=False, realtime=False, camera_viewer=False):
        self.directory=Path(directory).resolve();self.directory.mkdir(parents=True,exist_ok=True)
        self.log=(self.directory/'actions.jsonl').open('x')
        self.viewer,self.realtime=viewer,realtime
        self.camera_viewer=camera_viewer
        self.sim=None;self.episode=0;self.sealed=False;self.running=True
        self.start(task,seed)

    def start(self,task,seed):
        if task not in TASKS:raise ValueError('Unknown task')
        if isinstance(seed,bool) or not isinstance(seed,int) or not 0<=seed<2**32:raise ValueError('Invalid seed')
        if self.sim:self.sim.close()
        self.episode+=1
        self.sim=VisualSimulation(task,self.directory/f'{self.episode:02d}-{task}',seed,self.viewer,self.realtime,self.camera_viewer)
        self.sealed=False
        return self.sim.observe()

    def execute(self,command):
        if not isinstance(command,dict):raise ValueError('Expected JSON object')
        try:
            action=command.get('action')
            if action=='start':result=self.start(command.get('task'),command.get('seed',4))
            elif action=='shutdown':self.running=False;result={'closed':True}
            elif action=='finish':
                self.sealed=True
                result=evaluate(self.sim)
            elif self.sealed and action!='observe':raise ValueError('Episode is sealed after evaluation; start a new episode')
            else:result=self.sim.execute(command)
            response={'ok':True,'result':result}
        except (ValueError,TypeError,RuntimeError) as exc:
            response={'ok':False,'error':str(exc)}
        row={'time':datetime.now(timezone.utc).isoformat(),'episode':self.episode,'command':command,**response}
        self.log.write(json.dumps(row,allow_nan=False)+'\n');self.log.flush()
        return response

    def close(self):
        if self.sim:self.sim.close()
        self.log.close()


def serve(args, session_factory=VisualSession):
    path=Path(args.socket).resolve();path.parent.mkdir(parents=True,exist_ok=True)
    if path.exists():raise ValueError(f'Socket already exists: {path}')
    session=session_factory(args.record,args.task,args.seed,args.viewer,args.realtime,args.camera_viewer)
    class Handler(socketserver.StreamRequestHandler):
        def handle(self):
            self.connection.settimeout(5)
            try:
                raw=self.rfile.readline(65537)
                if len(raw)>65536:raise ValueError('Command too large')
                response=session.execute(loads(raw))
            except (ValueError,UnicodeError,socket.timeout) as exc:response={'ok':False,'error':str(exc)}
            try:self.wfile.write(json.dumps(response,allow_nan=False).encode()+b'\n')
            except (BrokenPipeError,ConnectionResetError):pass
    try:
        with socketserver.UnixStreamServer(str(path),Handler) as server:
            path.chmod(0o600);server.timeout=0.05
            print(json.dumps({'ready':True,'socket':str(path),'record':str(session.directory)}),flush=True)
            while session.running:
                server.handle_request()
                session.sim.update_camera_viewer()
                if session.sim.viewer:
                    if not session.sim.viewer.is_running():break
                    session.sim.viewer.sync()
    finally:
        session.close();path.unlink(missing_ok=True)


def send(address,command):
    with socket.socket(socket.AF_UNIX,socket.SOCK_STREAM) as connection:
        connection.settimeout(120);connection.connect(str(address))
        connection.sendall(json.dumps(command,allow_nan=False).encode()+b'\n')
        with connection.makefile('rb') as stream:return loads(stream.readline(1024*1024))
