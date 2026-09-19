# SO101 MuJoCo project

Preserve existing uncommitted experiments/assets. Current natural-language control
entry: `LLM-control/run_loop.sh`; read `LLM-control/LOOP_README.md` for the protocol.

When the user asks to operate the simulated robot, use the current Codex conversation
as the planner. Keep the model selected by the user in VS Code. Do not introduce
another API model, a keyword-only language parser, or substitute a canned replay.

1. Run `bash LLM-control/run_loop.sh status`. If no service is running, start
   `bash LLM-control/run_loop.sh serve --camera-modality rgb --environment-camera side
   --viewer --realtime --camera-viewer` in a persistent terminal on a desktop; use
   `MUJOCO_GL=egl` and omit GUI flags headlessly. Omit both camera flags, or pass
   `--camera-modality rgbd --environment-camera overhead`, for the original setup.
   Do not delete an existing socket without checking its owning process.
2. Read current task first. Continue a matching active task; otherwise complete or
   cancel it as appropriate to the user's instruction. `begin` records the user's
   natural-language instruction; it does not reset the world.
3. Open returned RGB images using the image-viewing tool. Read the observation's
   `perception` block: it names the current modality, the environment camera and
   what `localize`/`propose` can do. Under `rgbd`, use the current `frame_id` with
   `localize`/`propose` to reproject depth onto visible surfaces. Under `rgb` there
   is no depth: get 3D from the plane assumption instead, either `localize` with
   `method:"plane"` and an explicit `plane_z`, or `propose` with `object_height`
   for an upright object's `grasp_center`. State the plane; never guess it for a
   stacked or unknown-height object, and check `extent` before trusting a blob.
   Refine from the wrist camera once the arm is close. Do not infer object
   positions from saved demonstration pixels or private simulator state.
4. Send one `step` at a time with current `task_id`, `frame_id`, `expected_step`
   and a bounded low-level command. Inspect actual returned pose and new images,
   then decide the next step. Re-localize moved objects; saved points do not track.
   Use raised waypoints and inspect clutter; IK is not a collision-free planner.
5. On error/timeout, query status and observe before retrying. Never blindly resend
   a physical command. Respect pause/cancel, task budgets, and user corrections.
6. For placement, open the gripper, retreat, wait for settling, then inspect fresh
   images. `complete` needs current task/frame IDs, outcome and specific evidence.
   `codex_visual` is a model judgment, not an automatic arbitrary-task evaluator.
   Report uncertainty/failure honestly. Keep the service for the next instruction.

The service alone does not generate model decisions. User text is submitted here
in the VS Code conversation. All motion must go through actuators and physics;
never teleport/weld objects to manufacture task success. No real hardware control.

Tests (from `LLM-control`):
`PYTHONPATH=third_party/so101-nexus/src MUJOCO_GL=egl .venv-loop/bin/python -m pytest tests -q --confcutdir=. --rootdir=. --import-mode=importlib`
