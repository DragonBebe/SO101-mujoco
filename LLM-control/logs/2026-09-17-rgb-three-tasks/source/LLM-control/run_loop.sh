#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
export PYTHONPATH="$PWD/third_party/so101-nexus/src${PYTHONPATH:+:$PYTHONPATH}"
if [[ -n "${SO101_PYTHON:-}" ]]; then
    exec "$SO101_PYTHON" loop.py "$@"
fi
if [[ ! -x .venv-loop/bin/python ]]; then
    echo 'Missing .venv-loop. Run: bash LLM-control/setup_loop.sh' >&2
    echo 'Or set SO101_PYTHON to an existing Python with MuJoCo and the required dependencies.' >&2
    exit 1
fi
exec .venv-loop/bin/python loop.py "$@"
