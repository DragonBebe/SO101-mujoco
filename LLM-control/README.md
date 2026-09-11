# 当前 Codex 控制 SO101 抓取与放置

**持续自然语言操控的新入口：请先看 [LOOP_README.md](LOOP_README.md)。**
支持多物体桌面、同一场景连续任务、当前 Codex 的 RGB-D 反馈循环，以及 VS Code 启动任务。

**RGB＋深度视觉版本已实现：请使用 [视觉控制说明](VISION_README.md)。** 新入口迁移了你提供的 so101-nexus 环境，支持当前 Codex 看图控制七个任务案例及独立回放。下文为最初使用仿真状态反馈的演示。

已经在本项目的 MuJoCo 3.3.2 环境实测完成：当前 Codex 会话通过本地命令逐步读取仿真反馈，将红色方块从 `[1.20, 0.60, 0.755]` 抓起并放到橙色目标区 `[1.16, 0.79, 0.755]`。坐标使用 MuJoCo 世界坐标，单位为米。

本次实测最大抬升 **47.43 mm**，最终水平误差 **4.67 mm**，释放、退开并继续仿真 2 秒后方块稳定。原始记录位于 `runs/codex-pick-place/actions.jsonl`，最终状态为 `runs/codex-pick-place/state.json`，截图位于同一目录。`examples/pick-place.jsonl` 保留了此次操作的可回放命令。

## 运行

在项目根目录执行（本机验证使用 `/home/dragon/anaconda3/bin/python`）：

```bash
cd /home/dragon/SO101-mujoco
python LLM-control/control.py serve --viewer --realtime --capture
```

保持该终端运行，然后告诉当前 Codex：

> 使用 LLM-control/control.py 的本地控制服务，读取当前状态，把红色方块抓起并放到 goal；每一步检查返回状态，最后验证 success。

Codex 使用终端工具逐步发出下面这样的指令：

```bash
python LLM-control/control.py command '{"action":"observe"}'
python LLM-control/control.py command '{"action":"move","position":[1.2,0.6,0.805],"seconds":2}'
python LLM-control/control.py command '{"action":"snapshot"}'
```

本入口不需要额外 API Key。LLM 就是当前会话中的 Codex；它下发末端目标和夹爪命令，Python 控制器求 IK、插值关节目标并运行物理仿真。等待下一条命令时暂停仿真时间，窗口仍然可以查看。LLM 的决策间隔不等于电机控制频率。

关闭 Codex 后不会自行产生新的模型决策。要重新观看已完成的动作，可以独立回放：

```bash
python LLM-control/control.py replay LLM-control/examples/pick-place.jsonl --viewer --realtime
```

回放使用保存的动作，不调用新的 LLM；仿真测量未达到成功条件时返回非零退出码。回放结束窗口关闭。再次交互时使用 `serve`。

## 命令

| JSON | 行为 |
| --- | --- |
| `{"action":"observe"}` | 返回末端、关节、物体位置与速度、夹指接触、目标及成功状态 |
| `{"action":"move","position":[x,y,z],"seconds":2}` | 夹爪朝下、腕部滚转固定的末端目标；使用平滑关节插值，路径不保证为直线 |
| `{"action":"gripper","opening":0}` | 闭合；`opening:1` 张开，可指定 0.2–5 秒执行时长 |
| `{"action":"wait","seconds":2}` | 保持当前指令并推进仿真，允许 0.01–5 秒 |
| `{"action":"snapshot"}` | 在本次运行目录保存截图 |
| `{"action":"reset"}` | 重新初始化本演示的基座、方块和机械臂姿态 |
| `{"action":"shutdown"}` | 关闭服务和窗口 |

默认使用 `LLM-control/.runtime/simulation.sock`，仅本机当前用户可访问。另起实例时，服务和命令两端均指定相同的 `--socket /tmp/another-so101.sock`。默认每次创建独立的 `runs/时间戳/` 目录；`--record` 指定目录时不能覆盖已有 `actions.jsonl`。

## 对现有场景的适配

复用 `manipulator_grasp/assets/SO101/scene_table_cubes.xml` 及其机器人、桌面和三个方块。以下调整只存在于新入口的内存模型中，原 XML 及原有未提交修改保持不变：

- 修正原基座焊接约束的偏移，把基座安置在 `[1.0, 0.6, 0.71]`。
- reset 时将现有三个自由方块摆放在桌面，避免原来从高空掉落的方块撞到机械臂；把原桌外放置标记调整为可达的橙色目标区。
- 原固定指/活动指的整体凸包碰撞会填充夹持空隙，替换为对应指尖位置的两个盒形碰撞体；保留视觉网格、其他连杆碰撞与原惯量。
- 把原来位于固定指边缘的末端 site 校准到 3 cm 方块的夹持中心；调整位置控制增益、阻尼和接触刚度，夹爪扭矩限制为 ±0.5 N·m。

运动期间物体只由接触、摩擦和重力驱动。没有物体焊接，没有运行中改写物体 `qpos`，没有使用实际机械臂串口。物体坐标来自 MuJoCo 状态读取；本实现没有图像识别模块。

当前成功判定针对此红色方块任务：有夹爪接触时抬升至少 4 cm、水平运输超过 8 cm、释放后水平误差小于 2 cm、高度误差小于 1 cm、线速度小于 0.01 m/s、无夹指接触且已发出张开指令。正式验证还会退开并等待 2 秒。`ok:true` 表示命令执行完成，`observation.success:true` 才表示任务条件成立；LLM 应检查实际坐标及接触状态。

这是已验证的固定桌面抓取任务，不是任意障碍环境的通用规划器。目标会先经过工作范围和带姿态约束的 IK 检查，但路径没有完整的碰撞规划。当前校准针对 3 cm 方块；调整模型、布局或尺寸后应重新验证。

## 验证与依赖

```bash
python -m pytest LLM-control/tests -q
python LLM-control/control.py replay LLM-control/examples/pick-place.jsonl
```

测试覆盖真实接触抓取、放置与静置、错误输入不推进状态、不可达目标、持久服务、非有限 JSON 数值，以及截图失败不丢失已执行动作的日志。依赖见 `requirements.txt`；本机已有所需依赖，没有更改你的 Python 环境。

无显示环境下省略 `--viewer --capture` 可以执行物理验证。本机截图使用现有 X11/GLFW；本机 `MUJOCO_GL=egl` 因已有 PyOpenGL 的 `EGLDeviceEXT` 缺失而不可用。

参考：[MuJoCo 仿真函数](https://mujoco.readthedocs.io/en/3.3.2/APIreference/APIfunctions.html)、[Codex 本地终端能力](https://learn.chatgpt.com/docs/codex/cli)。
