"""Real tabletop -> MuJoCo scene mapping for the calibrated SO101 setup.

The pipeline is the one SIMPACT's rigid real-to-sim document describes --
perceive, estimate a pose, transform into the simulator's frame, build a
physical scene -- with one substitution forced by the hardware here: there
is no depth camera.  The third dimension comes from two *stated* conditions
instead, the measured table plane and the block's known height, and every
result says which of its numbers were seen and which were assumed.

The package is split so that the two virtual environments this project keeps
can each run the half they are able to:

``geometry``, ``blocks``, ``estimate``, ``track``, ``scene``
    Pure NumPy/SciPy.  They import ``nexus_vision.perception`` for the colour
    mask and nothing else, so they run under ``.venv-rgbcal`` (OpenCV, no
    gymnasium) and under ``.venv-loop`` (MuJoCo, no OpenCV) alike.

``observe``, ``overlay``
    The real side.  Needs OpenCV and ``rgbcal`` -> ``.venv-rgbcal``.

``mirror``
    The simulation side.  Needs MuJoCo and ``so101_nexus`` -> ``.venv-loop``.

They exchange one artefact: a scene-state JSON/JSONL file whose frame is the
measured table frame, which is also the simulation world.  Nothing in
``rgbcal`` or ``nexus_vision`` is modified.
"""

SCENE_SCHEMA = 'realsim/scene/1'
#: Anchor displacement the calibration documents as "re-measure the extrinsic".
#: It lives here rather than in :mod:`realsim.drift` because the command line
#: needs it as a default on both sides, and ``drift`` imports OpenCV, which the
#: simulation environment does not have.
DISPLACEMENT_LIMIT_PX = 3.0
