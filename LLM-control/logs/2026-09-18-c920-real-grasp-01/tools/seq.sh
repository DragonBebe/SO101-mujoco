# usage: seq.sh "goto:dx,dy,dz" "grip:ticks" ...   stops at the first error
cd /home/dragon/SO101-mujoco/LLM-control
S=/tmp/claude-1000/-home-dragon-SO101-mujoco/2e6b2439-901e-4cbc-90d7-98e1f195987c/scratchpad
for step in "$@"; do
  kind=${step%%:*}; arg=${step#*:}
  if [ "$kind" = goto ]; then out=$(PYTHONPATH=. .venv-rgbcal/bin/python $S/grasp_red.py goto --d=$arg --state $S/grasp1.json 2>&1)
  else out=$(PYTHONPATH=. .venv-rgbcal/bin/python $S/grasp_red.py grip --grip $arg --state $S/grasp1.json 2>&1); fi
  echo "[$step] $(echo "$out" | grep -E 'grasp centre FK' | tail -1) $(echo "$out" | grep -E '^now ticks' | grep -oE '6: [0-9]+')"
  if echo "$out" | grep -qE "Error|ABORT|refus"; then echo "$out" | tail -4; exit 1; fi
done
PYTHONPATH=. .venv-rgbcal/bin/python -c "import json;s=json.load(open('$S/grasp1.json'));print('cmd',[round(v*1000,1) for v in s['cmd_m']])"
sleep 1.5
