"""Offline archive only: never sends robot commands. Run from repository root."""
import json, shutil, hashlib, subprocess
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo
ROOT=Path('LLM-control')
RUN=ROOT/'runs/real-only-20260922-204451'
OUT=ROOT/'logs/2026-09-22-real-only-cancel-return'
SESSION=Path('/home/dragon/.codex/sessions/2026/09/22/rollout-2026-09-22T20-15-58-01a0ca54-ee62-7c01-9b59-26b8fa9d1e0f.jsonl')
OUT.mkdir(exist_ok=True)
for d in ['frames','source']: (OUT/d).mkdir(exist_ok=True)
def dump(p,v): (OUT/p).write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n')
rows=[dict(json.loads(l),_seq=i) for i,l in enumerate((RUN/'actions.jsonl').read_text().splitlines(),1)]
frames={r['frame']:r for r in rows if 'ticks' in r}
last=max(frames)
phases=[('setup',1,3,'连接、保持'),('lift-red',4,28,'抬起红色方块'),('stack-cancelled',29,58,'红块放桌、绿块堆叠（用户终止）')]
if last>58: phases.append(('return',59,last,'回到启动时姿态'))
allmetrics=[]
for name,lo,hi,label in phases:
 selected=[]
 for r in rows:
  f=r.get('frame')
  if 'command' in r and 'ticks' not in r: f=r['command']['frame']+1
  if f is not None and lo<=f<=hi: selected.append(r)
 file='events-setup.jsonl' if name=='setup' else f'task-{name}.jsonl'
 (OUT/file).write_text(''.join(json.dumps(r,ensure_ascii=False)+'\n' for r in selected))
 observations=[r for r in selected if 'ticks' in r]
 allmetrics.append(dict(id=name,what=label,first_frame=lo,last_frame=hi,event_first=selected[0]['_seq'],event_last=selected[-1]['_seq'],observed_actions=sum(bool(isinstance(r.get('detail'),dict) and r['detail'].get('command')) for r in observations),errors=sum('error' in r for r in selected),recorded_interval_s=round(observations[-1]['at']-observations[0]['at'],3),service_begin_to_complete_s=None,simulation_time_s=None))
 table=['# '+label,'','TCP 列为辅助 FK 推导毫米值，不是实测 TCP。完整目标和发令前反馈保留于 JSONL。','', '| 步/帧 | 事件 | 指令 | 实测 ticks（电机1–6） | 辅助 FK mm | 新帧 | 说明 |','|---|---|---|---|---|---|---|']
 for r in observations:
  detail=r.get('detail'); cmd=detail.get('command') if isinstance(detail,dict) else detail
  table.append('| '+ ' | '.join([str(r['frame']),str(r['_seq']),json.dumps(cmd,ensure_ascii=False),str(list(r['ticks'].values())),str([round(v,2) for v in r['tcp_fk_auxiliary_mm']]),f"{r['frame']:04d}-env/wrist.png",'仅关键帧入档，其余原图保留于本地 runs'])+' |')
 (OUT/f'ACTIONS-{name}.md').write_text('\n'.join(table)+'\n')
keys=[1,27,28,37,39,50,51,58]
if last>58: keys.append(last)
for f in keys:
 for camera in ['env','wrist']:
  if camera=='wrist' and f not in [27,51,58,last]: continue
  fn=f'{f:04d}-{camera}.png';shutil.copyfile(RUN/fn,OUT/'frames'/fn)
shutil.copyfile(RUN/'tools/joint_control.py',OUT/'source/joint_control.py')
if (RUN/'lift-red-result.json').exists(): shutil.copyfile(RUN/'lift-red-result.json',OUT/'lift-red-result.json')
session=[json.loads(l) for l in SESSION.open()]
models=[{'timestamp':r['timestamp'],'model':r['payload'].get('model')} for r in session if r['type']=='turn_context']
dump('model-source.json',models)
users=[{'timestamp':r['timestamp'],'text':r['payload']['message']} for r in session if r['type']=='event_msg' and r['payload'].get('type')=='user_message']
cut=next(r['timestamp'] for r in users if '终止当前的任务' in r['text'])
start=next(r['timestamp'] for r in users if r['text'].strip()=='抬起红色方块')
dump('task-instructions.json',[r for r in users if start<=r['timestamp']<=cut])
seen=set(); usage=[]
for r in session:
 if r['type']!='token_usage_record' or not start<=r['timestamp']<cut: continue
 p=r['payload']; rid=p['response_id']
 if rid in seen:continue
 seen.add(rid); usage.append({'timestamp':r['timestamp'],**p})
(OUT/'usage-source.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in usage))
tot={k:sum(r['usage'].get(k,0) for r in usage) for k in ['input_tokens','cached_input_tokens','cache_write_input_tokens','output_tokens']}
checks=[]
for tid in {r['turn_id'] for r in usage}:
 rs=[r for r in usage if r['turn_id']==tid]
 sums={k:sum(r['usage'].get(k,0) for r in rs) for k in tot}
 checks.append({'turn_id':tid,'sum':sums,'last_turn_token_usage':rs[-1].get('turn_token_usage'),'matches':all(sums[k]==rs[-1]['turn_token_usage'].get(k,0) for k in sums)})
dump('metrics.json',{'schema':'so101-log-metrics/1','session':SESSION.stem,'timezone':'CEST (UTC+02:00)','model_source':'model-source.json','cutoff_seq':len(rows),'cutoff_frame':last,'phases':allmetrics,'task_usage':{'start':start,'end_exclusive':cut,'responses':len(usage),'input':tot['input_tokens']-tot['cached_input_tokens'],'cache_read':tot['cached_input_tokens'],'cache_write':tot['cache_write_input_tokens'],'output':tot['output_tokens'],'image_tokens':None,'checks':checks},'exclusions':'当前终止/归档/回位轮未结束，token未计入；对话结束时间及服务begin/complete不存在，不推算。'})
initial=frames[1]['ticks']; final=frames[last]['ticks']
dump('summary.json',{'stack_status':'cancelled_by_user','red_on_table':'visually_confirmed','green_stacked':False,'last_frame':last,'cutoff_seq':len(rows),'initial_ticks':initial,'latest_ticks':final,'return_requested':True,'return_status':'pending' if last<=58 else 'see RETURN.md and fresh visual verification','initial_to_latest_delta_ticks':{k:final[k]-v for k,v in initial.items()},'source_run':str(RUN),'code_base_commit':'d9749fe90271bb5bd6a0d84346788b0fd139c342'})
(OUT/'source/git-status.txt').write_text(subprocess.check_output(['git','status','--short','--branch'],text=True))
manifest=[]
for p in sorted(OUT.rglob('*')):
 if p.is_file() and p.name!='SHA256SUMS':manifest.append(hashlib.sha256(p.read_bytes()).hexdigest()+'  '+str(p.relative_to(OUT)))
(OUT/'SHA256SUMS').write_text('\n'.join(manifest)+'\n')
print(json.dumps({'last_frame':last,'events':len(rows),'phases':allmetrics,'usage':tot,'checks':[r['matches'] for r in checks]},ensure_ascii=False))
