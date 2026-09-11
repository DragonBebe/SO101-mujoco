# Codex → MuJoCo pick and place

Goal: 当前 Codex 会话通过本地 JSON 命令读取状态、控制末端和夹爪，完成真实接触抓取与放置。

Design: 复用 `scene_table_cubes.xml`，仅在新入口的内存模型中修正基座焊接偏移和可达放置区。使用 MuJoCo 模型求运动学，关节位置执行器推进物理仿真。物体只在 reset 初始化，不使用物体焊接、瞬移或运动中修改物体 qpos。控制入口只接受有界命令，不执行传入代码。仿真在等待 LLM 时暂停，命令执行时连续推进。

Files: `simulation.py` 负责模型、IK、运动与成功判定；`control.py` 提供持久仿真服务及 JSON CLI；`tests/test_control.py` 验证真实物理及错误输入；`README.md` 说明运行与限制。

Execution: inline in the user's current workspace; all additions under the previously empty `LLM-control` directory, preserving uncommitted work.

- [x] Add failing tests for settled scene, invalid command rejection, unreachable IK with no mutation, and no false success before lifting.
- [x] Implement simulation initialization and bounded Cartesian/jaw commands; inspect contacts and tune grasp using physical observations.
- [x] Implement local persistent JSON control service, state feedback, action trace and screenshots.
- [x] Let current Codex issue observe → approach → close → lift → transfer → release → retreat commands, using feedback at every stage.
- [x] Verify lift, released object position, low velocity and table support; retain evidence and a replay entry point. Run tests and document exact commands.

Verified: original finger mesh convex hulls occupied the grasp gap. In the new entry point only, replaced their collision hulls with two fingertip boxes, corrected the grasp site and contact tuning. Physical grasp regression failed before this fix and passed afterward. Formal current-Codex run: max lift 0.0474272 m; final XY error 0.00466891 m; stable after release, retreat and 2 s of additional simulation. Independent review findings for screenshot failures and JSON exponent overflow were reproduced with failing tests and fixed; 11 tests pass.

Acceptance: object rises at least 4 cm while contacting the gripper, is transported at least 8 cm, then remains within 2 cm XY and 1 cm Z of the goal after release and settling. Success must come from simulation measurements, not LLM claims. A replay is explicitly labeled replay, not a new LLM inference.
