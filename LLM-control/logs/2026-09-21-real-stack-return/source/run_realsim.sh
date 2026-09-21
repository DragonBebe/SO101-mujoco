#!/usr/bin/env bash
# Real tabletop -> MuJoCo scene mapping.
#
# The two halves of this pipeline need different Python environments and this
# script picks the right one per command, so neither locked environment has to
# grow a dependency for the other's sake:
#
#   real side  (check, drift, scale-check, colours, measure, observe, live, arm, arm-stream)
#              -> .venv-rgbcal  (OpenCV, camera, serial SDK)
#   sim  side  (mirror, snapshot, export, replay, selftest, sync, arm-replay)
#              -> .venv-loop    (MuJoCo, so101_nexus)
#
# The list below must match the `side` set on each subparser in realsim/cli.py;
# tests/test_colour.py asserts that it does, because a command missing from it
# silently runs in the environment that lacks its dependencies.
#
# They meet at one file, the scene state written by `observe`/`live` and read
# by `mirror`/`snapshot`/`replay`. Joint feedback has a separate latest JSON
# and raw JSONL log, so camera processing cannot slow down joint mirroring.
set -euo pipefail
# Keep where the user typed the command so relative file arguments still work.
export REALSIM_CWD="$PWD"
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

command="${1:-}"
case "$command" in
    mirror|snapshot|export|replay|selftest|sync|arm-replay) side=sim ;;
    *)                               side=real ;;
esac

if [[ "$side" == sim ]]; then
    export PYTHONPATH="$PWD:$PWD/third_party/so101-nexus/src${PYTHONPATH:+:$PYTHONPATH}"
    export MUJOCO_GL="${MUJOCO_GL:-egl}"
    python="${SO101_PYTHON:-.venv-loop/bin/python}"
    hint='bash LLM-control/setup_loop.sh'
else
    # nexus_vision.perception and the rgbcal integration layer are reused as they are.
    export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
    python="${RGBCAL_PYTHON:-.venv-rgbcal/bin/python}"
    hint='see LLM-control/RGBCAL_README.md for creating .venv-rgbcal'
fi

if [[ ! -x "$python" ]]; then
    echo "Missing $python for the $side side of realsim. $hint" >&2
    exit 1
fi
exec "$python" -m realsim.cli "$@"
