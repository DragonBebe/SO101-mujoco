# Nexus RGB-D + current Codex implementation

User authorization: migrate the provided local Nexus environments, use RGB + depth (explicitly confirmed), and current Codex as the reasoning controller for all shipped task cases.

## Design and boundaries

Vendor the Apache-2.0 package source/assets from `/home/dragon/Downloads/so101-nexus-main` into `third_party/so101-nexus` with its license. Use an isolated `.venv-vision` Python 3.12 environment. Preserve the existing control.py/state-based demonstration.

New `nexus_vision/` modules: `perception.py` accepts only RGB/depth/calibration arrays and unprojects pixel selections; `simulation.py` loads Nexus tasks, exposes only images/calibration/proprioception, executes Cartesian/jaw/look-at commands; `bridge.py` persists the simulator behind a local command socket; `evaluate.py` computes private upstream task results after actions, never returns target/object coordinates to the policy. Record observation frames, pixel selections, executed commands, and final reports. No object coordinates or scene-object segmentation IDs in the policy API. Only robot model/FK is used for IK. During manipulation, all objects remain free simulated bodies.

Task scope: Touch, LookAt, Move, PickLift, PickAndPlace v1/v2, StackCube. Use reproducible configurations with the upstream task definitions and success criteria. RGB identifies targets; registered depth and calibrated camera pose provide visible surface coordinates. A supplied pixel comes from Codex viewing the image. Optional RGB color proposals may aid selecting objects, but cannot use simulator labels or poses. Proposals are not substitutes for the Codex action loop.

## Execution ledger

- [x] Migrate package and install isolated runtime; load upstream tasks and inspect visual outputs.
- [x] Implement and test pure RGB-D unprojection and RGB-only region proposals (independent subtask).
- [x] Implement model-based motion and local visual-only observation/action service.
- [x] Run current Codex with visual observations to complete all seven registered task cases; retain evidence and actual upstream success results.
- [x] Test input/provenance boundaries, independent review, reproducible visual replay, documentation.

Ruling: work directly in the user's shared workspace with all new work under LLM-control; the existing workspace contains required uncommitted model work, so creating a clean checkout would discard necessary context. No commits or changes outside the new implementation are needed.

Ruling: allow known object dimensions/task language and proprioception, but never task-object state or target positions in policy observations. Private evaluator reads simulation truth only after control, and writes results separately. Any reduced spawn region or model adaptation must be stated explicitly in the final documentation.

## Results and adjustments

Current Codex completed all seven tasks with RGB image inspection and pixel selections; results are in `examples/vision/recorded_results.json`. A fresh all-task replay also passed, recorded in `examples/vision/verified_replay_results.json` and `runs/nexus-release-verification`. Tests: 52 passed, including existing state-control tests. Dependency compatibility and shell syntax checks passed.

The original task models, physics and success criteria are unchanged. Demonstrations use seed 4, fixed robot/camera initialization and a reachable visible spawn region, documented in `VISION_README.md`. An initially occluded LookAt attempt was abandoned, then completed with a visible square spawn center. No arbitrary-seed success rate is claimed.

Independent reviews drove regression fixes for cross-episode frame/point aliasing, finite far-plane depth mistaken for a hit, replay accepting another episode after finish, and launcher failure to retry interrupted installation. A real headless replay regression found that upstream Move resets its model goal site after settling without refreshing derived transforms; the wrapper now runs `mj_forward` before its first image. All reproductions failed before their corresponding fixes and passed afterward. Final independent review found no remaining material issue.

Entrypoints: `run_vision.sh serve`, `command`, and `replay`. Replay reprojects saved pixel selections against newly rendered depth; it is explicitly labeled as recorded Codex planning, not fresh LLM inference. No extra model service is required for interaction with the current Codex session.
