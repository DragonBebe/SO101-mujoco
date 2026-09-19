# 在当前 Codex 对话中持续操控 SO101

这是新的持续任务入口。你在 VS Code 对话中使用自己选择的 Codex-astra 发出中文
指令；当前会话读取相机画面、决定动作、检查反馈，并继续下一步。本地 Python
负责 MuJoCo、RGB-D、IK、任务状态及日志，不调用额外 LLM，不需要新的 API Key。

## 启动

本机已建立 `LLM-control/.venv-loop`。在项目根目录运行推荐入口（双 RGB + 侧视）：

```bash
bash LLM-control/run_loop.sh serve --camera-modality rgb --environment-camera side \
     --viewer --realtime --camera-viewer
```

其他组合：

```bash
# 双 RGB + 真机标定机位：环境相机复现实测 C920 的内参与位姿（仅 rgb 模态）
bash LLM-control/run_loop.sh serve --camera-modality rgb --environment-camera calibrated \
     --viewer --realtime --camera-viewer
# RGB-D + 侧视：侧面机位，同时提供深度与三维定位
bash LLM-control/run_loop.sh serve --camera-modality rgbd --environment-camera side \
     --viewer --realtime --camera-viewer
# RGB-D + 俯视：原有默认，不加相机参数时等价
bash LLM-control/run_loop.sh serve --viewer --realtime --camera-viewer
```

也可以在 VS Code 的“终端 → 运行任务”中选择 **SO101: 启动双 RGB + 侧视环境（推荐）**、
**SO101: 启动 RGB-D + 侧视环境** 或 **SO101: 启动 RGB-D + 俯视环境（原有）**。
打开 MuJoCo 视窗和双相机预览窗口后，在当前 Codex 对话中直接说：

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
预览队列满时丢弃预览帧，不阻塞控制命令，也不影响动作后的正式观测。预览帧不产生
新的 `frame_id`，因此刷新预览不会让已取得的观测变成陈旧帧。

## 相机模态与机位

两个选项互相独立，服务启动时确定，任务执行中不热切换：

| 参数 | 取值 | 含义 |
| --- | --- | --- |
| `--camera-modality` | `rgb` | 两路相机只输出彩色图；不渲染、不保存深度 |
| | `rgbd` | 保留原有彩色 + 光学 Z 深度（默认） |
| `--environment-camera` | `side` | 新增世界固定侧视机位，从桌面右前方斜向下观察 |
| | `overhead` | 原有正俯视机位（默认），用于回归与对照 |
| | `calibrated` | 复现真机标定的 C920 环境相机；**只能与 `rgb` 搭配**（标定只覆盖 RGB） |

环境相机的名称就是机位名：`side` 模式下两路相机是 `side` 与 `wrist`，`overhead`
模式下是 `overhead` 与 `wrist`。`localize` / `propose` 的 `camera` 默认取环境相机，
指定另一种机位的名称会明确报错，不会读到上一次运行的图像。

侧视机位固定在世界坐标系中，不跟踪任何物体。位置、朝向、视场角和分辨率集中配置在
`nexus_vision/cameras.py` 的 `CameraSuite`：方位角 130°、俯角 −32°、垂直视场 50°，
距离由场景自身的生成半径与角度范围拟合而成（`fit_orbit_distance`），使整个可达工作区
留在画面内。当前本机解析结果为相机位于 `[0.531, -0.430, 0.475]`、注视点
`[0.17, 0, 0.125]`。**这是仿真初始配置，并未标定复现任何一台真实笔记本摄像头。**
腕部相机仍是机器人模型里的 `wrist_cam`，固定连接在 `camera_mount` 刚体上随臂运动。

### 真机标定机位 `calibrated`

`calibrated` 把真机 C920 的标定结果搬进仿真（见 `RGBCAL_SUMMARY.md`）。数据流：

```bash
# 标定更新后重新导出（使用 .venv-rgbcal，需要 OpenCV）
bash LLM-control/run_rgbcal.sh sim-camera
# -> LLM-control/calib/c920-sim-camera/sim_camera.json
```

- **内参**：用真机管线送给 perception 的**去畸变**内参 `K_new`（alpha=0），不是原始 `K/D`；
  仿真针孔图像本就无畸变，对应的是去畸变后的真机画面。fx≠fy 和偏离中心的主点都按原值
  渲染（直接设置 MuJoCo GL 相机的非对称视锥）。视场约 68.7° × 42.0°，16:9。
- **分辨率**：默认按 0.5 缩放为 960×540（`CameraSuite.calibrated_scale`，视场不变）；腕部相机
  仍为 640×480。预览窗口按各自宽高比显示，不拉伸。
- **位姿**：仿真世界 = 真机的**桌面坐标系**（z 为实测桌面法向、z=0 为桌面，原点在底座原点
  正下方的桌面上，x 为底座 x 在桌面上的投影），相机位于 `T_table_camera`：约
  `[0.767, -0.045, 0.461]` m，在机器人正前方约 0.77 m、朝向机器人、俯角约 35.7°。
  因此 `plane_z = 0`、`object_height` 在仿真和真机中含义相同。
- **已知差异**：标定解出的底座系相对桌面高 17 mm、倾斜 1.9°（主要是外参与关节零位的补偿），
  仿真中底座直立于原点，所以画面里机械臂相对真机有约这个量级的偏差；桌面、物体的透视一致。
  仿真光照、纹理、背景与真机不同；生成的物体布局仍是仿真自己的。
- 不产生深度，也不改变 `rgbd` 模态；选择 `rgbd` + `calibrated` 会直接报错。

`observe` 的返回里每路相机都带 `camera_id`、`placement`、`mounting`、`modality`、
`width`/`height`、`rgb` 路径与 `calibration`；顶层 `perception` 字段说明当前模态、
可用相机与 `localize`/`propose` 的能力。`status` / `session.json` / `events.jsonl`
每一行都记录 `cameras` 配置块，便于对照复现。

### RGB 模态下的三维：平面假设

没有深度并不等于没有三维。`rgb` 模态提供两个**不读深度、不读仿真坐标**的估计器，
它们用相机标定把像素射线和一个**由调用方说明的水平面**求交。假设说出来了，错的时候
也就知道错在哪。

**1. `localize` + `method:"plane"`** —— 单像素射线与平面 `z = plane_z` 求交：

```json
{"action":"localize","frame_id":"当前帧ID","pixel":[318,348],"method":"plane","plane_z":0}
```

`plane_z` **必须显式给出**，没有默认值：桌面用 `0`，物体顶面用该物体的高度。服务不会替你
猜——猜错的代价正是"未知高度的堆叠物体"那类错误。`rgbd` 模态下也可用，便于和深度互校。

**2. `propose` + `object_height`** —— 已知尺寸物体的站位估计：

```json
{"action":"propose","frame_id":"当前帧ID","color":"red","object_height":0.025}
```

每个候选多出 `support`：`center`（物体在桌面上的中心）、`grasp_center`（半高处的抓取中心）、
`extent`（估出的底面尺寸）、`cells`。原理是双平面雕刻：一个桌面格点只有在它本身**和**它正
上方 `object_height` 处**都**投影到颜色掩码内时，才可能属于物体底面。单张图的轮廓给出底面的
外包围，对直立凸物体这个包围很紧——地面投影朝远离相机方向拉长，抬高后的投影朝相机方向拉长，
两者相交就收回到底面附近。

`extent` 是调用方的自检：25 mm 方块应得到约 0.028–0.036 m；若明显偏大（例如黄色掩码把黄色
机械臂也框进去）或偏小（被遮挡），就不要用这个候选。

### RGB 模态的范围与限制

- 可用：两路真实渲染 RGB、相机标定、正运动学与本体状态、颜色区域候选、上面两个平面估计器，
  以及 `move` / `look_at` / `gripper` / `wait`。环境相机和腕部相机都能跑平面估计，先用环境
  相机粗定位、靠近后再用腕部相机校正。
- 不可用：`method:"depth"` 明确报错，不会静默回退，也不会用零深度顶替；`depth` 字段为 `null`
  并附 `depth_note`，磁盘上不生成 `.npy`。
- 平面假设失效的场景：堆叠或悬空物体（高度未知）、非水平支撑面、掩码被严重遮挡或与同色背景
  （如黄色机械臂）粘连。这些情况下请改用 `rgbd`。
- 本次没有建立双目重建或单目深度网络；`rgb` 模态的三维完全来自上述平面假设。

## 场景及当前范围

默认 `workbench` 有红色 25 mm 方块、绿色 25 mm 方块、黄色 25 mm 圆柱、紫色
35 mm 方块、蓝色目标圆区和支撑平面。物体参与碰撞、摩擦与重力；支持 `--seed`
改变初始布局。在 `rgbd` 模态下，目标物坐标通过 RGB 选点与深度反投影获取；`rgb`
模态没有这一步，场景说明仍只提供物体种类和尺寸。原有七任务 `vision.py` 入口及回放
保持独立，其相机行为未改变。

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

`begin` / `observe` 返回任务 ID、最新 `frame_id`、双相机图像（`rgbd` 模态另有深度
路径，`rgb` 模态为 `null`）、相机标定、TCP 与关节状态。坐标单位米，世界坐标 z=0 是支撑面。每次 `observe` 都产生新帧 ID。
客户端须真正打开 RGB 文件，再选择像素；JSON 中的路径不会自动显示为模型图像。

| action | 参数与行为 |
| --- | --- |
| `status` | 当前任务、步数、世界编号；不采图、不推进物理 |
| `begin` | `instruction`，可选 `max_steps:60`，创建任务并观察；保留世界 |
| `observe` | 获取新 RGB-D 帧与本体状态 |
| `localize` | `frame_id`、`pixel:[u,v]`，可选 `camera`、`method:"depth"/"plane"`、`plane_z` |
| `propose` | `frame_id`、`color`，可选 `camera`、`object_height`、`plane_z`；颜色候选 |
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
`offset:[dx,dy,dz]`。`localize` 给的是表面点或平面点，需根据物体尺寸与夹爪几何规划抓取
中心；`propose` 的 `grasp_center` 已经是半高处的中心。保存的点不会跟踪移动物体。运动后
必须用新反馈决定下一步。

朝下姿态在较大伸展半径处存在运动学上限：本机在半径约 0.23 m 处，`orientation:"down"`
最高只能到 z≈0.09，再高 IK 会失败。悬停高度应从大到小逐档重试，不要把这类失败当成定位错误。

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

2026-09-17 相机改造后本机验证：EGL **63 项通过、3 项跳过**，GLFW **66 项全部通过**。
四种模态/机位组合都通过真实 MuJoCo 渲染取图；`rgb` + `side` 真实 socket 跑通
`begin → observe → move → gripper → observe`（请求 `[0.18,-0.10,0.14]`，实际 TCP
`[0.180,-0.100,0.1399]`，运行目录内无任何 `.npy`）；`rgbd` + `side` 用侧视相机
`localize` 红色方块后带 8 cm 抬升移动，TCP 落在 0.2 mm 内。截图见
`logs/screenshots-2026-09-17/`。深度预览沿用固定的 0–1 m 配色，侧视机位下 1 m 以外
的地面会整体显示为红色，属于显示标尺，不影响 `.npy` 中的实际深度值。

2026-09-17 平面估计验证：`rgb` + `side` / `overhead` 在 5 个随机种子上估计红、绿方块中心，
横向误差多数在 0.5 mm 以内（俯视被机械臂遮挡的两例为 1.7 mm 和 6.0 mm，`cells` 与 `extent`
同时明显偏小，可被调用方识别）。腕部相机近距离复算把误差从 0.15–0.29 mm 降到 0.05–0.10 mm。
纯 RGB 抓取（估计 → 悬停 → 腕部校正 → 下降 → 夹紧 → 抬起）在 6 个随机种子上 **6/6 成功**，
方块被抬起约 70 mm，全程未渲染深度、运行目录无 `.npy`。
