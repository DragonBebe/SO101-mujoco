#!/usr/bin/env bash
# Real environment-camera calibration entry point. Uses its own OpenCV
# virtualenv so the locked .venv-loop simulation dependencies stay untouched.
set -euo pipefail
# Keep where the user typed the command, so relative file arguments still work
# after this script changes directory.
export RGBCAL_CWD="$PWD"
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
# nexus_vision.perception is reused unmodified by the real-camera integration.
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
if [[ -n "${RGBCAL_PYTHON:-}" ]]; then
    exec "$RGBCAL_PYTHON" -m rgbcal.cli "$@"
fi
if [[ ! -x .venv-rgbcal/bin/python ]]; then
    echo 'Missing .venv-rgbcal. Create it with:' >&2
    echo '  uv venv --python 3.12 LLM-control/.venv-rgbcal' >&2
    echo '  uv pip install --python LLM-control/.venv-rgbcal/bin/python opencv-contrib-python "numpy<3" scipy mujoco feetech-servo-sdk pyserial pytest' >&2
    exit 1
fi
exec .venv-rgbcal/bin/python -m rgbcal.cli "$@"
