# 在当前 Codex 对话中持续操控 SO101

这是新的持续任务入口。你在 VS Code 对话中使用自己选择的 Codex-astra 发出中文
指令；当前会话读取相机画面、决定动作、检查反馈，并继续下一步。本地 Python
负责 MuJoCo、RGB-D、IK、任务状态及日志，不调用额外 LLM，不需要新的 API Key。

## 启动

本机已建立 `LLM-control/.venv-loop`。在项目根目录运行：

```bash
bash LLM-control/run_loop.sh serve --viewer --realtime --camera-viewer
```

也可以在 VS Code 的“终端 → 运行任务”中选择 **SO101: 启动自然语言控制环境**。
打开 MuJoCo 视窗和双相机 RGB/深度窗口后，在当前 Codex 对话中直接说：

> 观察桌面，把红色方块抓起并放到蓝色区域，每一步检查画面。

或者先验证简单操作：

> 把夹爪向上移动三厘米。
> 张开夹爪，然后保持姿态。
> 继续把绿色方块移到红色方块旁边。

这些是交给当前 Codex 的任务文本，不是已验证成功的通用技能列表。`begin` 只记录
文本；模型在对话中完成理解和规划。根目录 `AGENTS.md` 提供持续操作约定，采用
[官方支持的项目指令机制](https://learn.chatgpt.com/docs/agent-configuration/agents-md)。
已有对话若没有读取新约定，可以说“读取 AGENTS.md，使用 loop 服务执行任务”。

服务等待指令时暂停物理时间，窗口仍可查看。完成一项任务后可以继续下一项，物体
位置和机械臂姿态保留。关闭对话后，服务不会自行产生新的 LLM 决策。`--realtime`
只控制动作播放节奏。相机图像在动作后反馈，控制器每 20 ms 推进动作。
双窗口模式将相机预览放在独立进程中，避免与 MuJoCo 主窗口争用 GLFW 事件循环；
预览队列满时丢弃预览帧，不阻塞控制命令，也不影响动作后的正式 RGB-D 观测。

## 场景及当前范围

默认 `workbench` 有红色 25 mm 方块、绿色 25 mm 方块、黄色 25 mm 圆柱、紫色
35 mm 方块、蓝色目标圆区和支撑平面。物体参与碰撞、摩擦与重力；支持 `--seed`
改变初始布局。目标物坐标通过 RGB 选点与深度反投影获取，场景说明仅提供物体
种类和尺寸。原有七任务 `vision.py` 入口及回放保持独立。

第一版搭好持续闭环的代码基础。控制器检查数值、工作范围、IK 和关节限位，尚无
通用避障或任意任务自动评分；随机布局的抓取成功率未测量。紫色大方块也可作为
需要避开的物体。动作成功返回 `ok:true`，任务完成状态则是 Codex 根据反馈记录
的 `codex_visual` 判断，两者不能混同。旧七任务的物理评分仍使用旧入口。

## 控制协议

```bash
bash LLM-control/run_loop.sh status
bash LLM-control/run_loop.sh command '{"action":"begin","instruction":"把夹爪向上移动三厘米","max_steps":60}'
bash LLM-control/run_loop.sh command '{"action":"observe"}'
```

`begin` / `observe` 返回任务 ID、最新 `frame_id`、双相机图像与深度路径、TCP 与
关节状态。坐标单位米，世界坐标 z=0 是支撑面。每次 `observe` 都产生新帧 ID。
客户端须真正打开 RGB 文件，再选择像素；JSON 中的路径不会自动显示为模型图像。

| action | 参数与行为 |
| --- | --- |
| `status` | 当前任务、步数、世界编号；不采图、不推进物理 |
| `begin` | `instruction`，可选 `max_steps:60`，创建任务并观察；保留世界 |
| `observe` | 获取新 RGB-D 帧与本体状态 |
| `localize` | `frame_id`、`pixel:[u,v]`，可选 `camera`；返回表面世界点及 `point_id` |
| `propose` | `frame_id`、`color`，可选 `camera`；基于颜色的候选辅助，需看图核实 |
| `step` | `task_id`、`frame_id`、`expected_step`、`command`；只执行一个低层动作 |
| `pause` / `resume` | 暂停/继续当前任务；预算耗尽后不能继续 |
| `complete` | `task_id`、`frame_id`、`outcome:"succeeded"/"failed"`、`evidence` |
| `cancel` | 取消当前任务，保留现场 |
| `reset` | 可选 `seed`；仅在任务结束/取消后重建场景，旧帧/点失效 |
| `shutdown` | 关闭服务及窗口 |

低层 `command` 支持 `move`、`look_at`、`gripper`、`wait`，其参数同
[视觉接口](VISION_README.md#命令协议)。例如下面的 JSON **必须换成刚返回的 ID、
步数和当前可达坐标** 后才能发出：

```json
{
  "action": "step",
  "task_id": "返回的任务ID",
  "frame_id": "返回的最新帧ID",
  "expected_step": 0,
  "command": {
    "action": "move",
    "position": [0.20, 0.0, 0.15],
    "orientation": "position",
    "seconds": 2
  }
}
```

`orientation:"position"` 只约束位置；抓取通常用默认朝下姿态。`point_id` 可配合
`offset:[dx,dy,dz]`。定位点是表面点，需根据物体尺寸与夹爪几何规划抓取中心。
保存的点不会跟踪移动物体。运动后必须用新反馈决定下一步。

有效动作尝试消耗步数，包括 IK 失败；连续三次动作失败会暂停，运行期异常也会
暂停。旧 `expected_step` 拒绝重复运动。网络超时后先查状态，不能直接重发。
预算默认 60、最多 500 步，达到预算后仍可观察、完成或取消。暂停命令在当前
单步结束后处理（移动参数最长 8 秒，另有稳定时间），不是硬实时急停。

完成示例：

```json
{"action":"complete","task_id":"当前任务ID","frame_id":"最新帧ID","outcome":"succeeded","evidence":"已张开夹爪并退开，静置后新画面显示方块留在目标区域。"}
```

记录按运行目录保存：`events.jsonl` 是所有请求/反馈，`session.json` 原子更新当前
任务，`world-*/` 保存图像及深度。日志支持检查和复盘，不是进程崩溃后的物理
检查点；重启服务将创建新世界。`--record` 建议用绝对路径，已有日志不会被覆盖。

默认 socket：`LLM-control/.runtime/loop.sock`，权限 0600。多实例在客户端和服务
端使用相同的自定义 `--socket`。JSON 可用 `command -` 从标准输入传入，避免
shell 转义问题。没有配置公网端口，也没有连接真机串口。

## 新机器安装与测试

Python 3.12 或以上：

```bash
bash LLM-control/setup_loop.sh
```

安装器优先使用已安装的 uv，否则使用 `python3 -m venv` 与 pip。可通过
`SO101_BASE_PYTHON=/path/to/python3.12` 指定创建环境的 Python。已有兼容环境可
用 `SO101_PYTHON=/path/to/python bash LLM-control/run_loop.sh ...`。依赖清单是
`requirements-loop.txt`，本机实际版本快照为 `requirements-loop.lock`。

无图形桌面时：

```bash
MUJOCO_GL=egl bash LLM-control/run_loop.sh serve
```

本机已验证 EGL 离屏渲染；GUI 使用 `MUJOCO_GL=glfw`。测试：

```bash
cd LLM-control
PYTHONPATH=third_party/so101-nexus/src MUJOCO_GL=egl .venv-loop/bin/python -m pytest tests -q --confcutdir=. --rootdir=. --import-mode=importlib
```

2026-09-10 本机验证：GLFW 全套 **67 项通过**；EGL 新闭环测试 **9 项通过、
1 项因需要桌面跳过**。真实 socket 测试包括连续两轮任务、TCP 移动、双窗口、
重置与关闭；手动请求上移 30 mm，实际上移约 29.85 mm。
