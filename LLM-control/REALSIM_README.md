# 真实相机采集 → MuJoCo 场景建模（realsim）

用已标定的固定 RGB 相机观测真实桌面上的方块，自动估计每个方块的**身份、位置和朝向**，
在 MuJoCo 里建立与之对应的物理场景。入口：[run_realsim.sh](run_realsim.sh)，代码：[realsim/](realsim/)。

> **当前状态（2026-09-21）**：软件链路已实现并通过合成与真实图像验证，但**绝对位置暂不可用于抓取**。
> 独立尺度检验不通过（+4%），并已确认相机相对标定时的机位移动了约 55 px。
> 场景的相对排布、物理试验、动作预测**可以用**；发给机械臂的绝对坐标**不可以**。
> 上述为既有视觉验证记录，本轮未重新验证。新增关节同步、命令与限制见 [第 10 节](#10-真实关节反馈同步本轮新增)。

## 本轮代码核查（同步实现前）

现有 `observe` 是单帧初始化，`export` 是一次性离线 MJCF 导出；`live` 持续写物体观测 JSON/JSONL，`mirror --follow` 轮询文件更新物体。`live` 当前不采集机械臂状态；`observe --with-arm` / `arm` 只读取一次。既有夹爪 fraction 到模型角度的关系未验证，不能当实测角度。导出已包含 Menagerie SO101 模型，但底座尚未使用场景里的桌面变换。下文末尾记录本轮新增状态同步的最终行为和验证结果。

## 1. 实现目标

**要做到的**

1. 相机看到真实的方块后，输出每块的身份、尺寸、中心位置和可观测的朝向；
2. MuJoCo 中对应方块的尺寸、位置和朝向与真实场景一致；
3. 先单帧初始化，再支持连续观测下的位置更新；
4. 结果可以三种方式使用：**镜像**（跟随真实）、**快照**（可独立步进，供动作预测）、
   **导出**（独立 MJCF 文件，任何 MuJoCo 查看器直接打开）；
5. 每个数字都能追溯：哪个标定版本、哪张原始图像、测的还是假设的、当前还是过期的。

**明确不做的**

- 不控制真实机械臂：真实侧只读相机和舵机，从不下发运动指令；
- 复用既有代码：视觉与标定算法保持；本轮只扩展 `rgbcal.robot.ServoBus` 的互斥和可选反馈发布，不修改真实抓取脚本；
- 不猜物理参数：无法确认的量（方块尺寸、质量、摩擦）带出处标注，写 `guess` 会被拒绝加载；
- 不用"画面对得上"证明位置是对的：重投影误差小不等于真实坐标准，见 [第 6 节](#6-独立验证关卡)。

**首版适用条件**：已知尺寸、平放在已知桌面上的刚性长方体。抬起、倾倒、堆叠（未声明支撑高度）、
严重遮挡超出适用条件时，标记为**无效**，而不是继续输出看似可信的桌面位姿。

## 2. 总体思路

思路来自 SIMPACT 的刚体 real-to-sim 流水线（[仓库](https://github.com/ShaoxiongYao/simpact)、
[RIGID_PIPELINE](https://github.com/ShaoxiongYao/simpact/blob/main/docs/RIGID_PIPELINE.md)）：
**感知 → 位姿 → 坐标转换 → 物理场景**。原流程依赖 RGB-D；这里没有深度传感器，
用"标定 RGB 相机 + 已知物体尺寸 + 桌面几何约束"替代深度。

```
真实相机原始帧
   │  去畸变（实测 K/D → K_new）
   ▼
颜色分割 ──► 每块的颜色掩码        perception.locate_color 的思路，按方块自己的色带
   │
   ▼
双平面雕刻（perception.plane_support_estimate）
   │  给出轮廓的外包络：作为初值 + 独立的尺寸筛选
   ▼
方块模型轮廓拟合   x, y, yaw       8 角点投影的凸包 = 轮廓；与掩码求 IoU；粗扫 + Nelder-Mead
   │
   ▼
质量检查   iou / area_ratio / height_check   不通过 → 标记无效并写明原因
   │
   ▼
关联 + 滤波 + 状态   detected / stale / lost    最后一次有效位姿带时间戳，不冒充当前测量
   │
   ▼
场景状态 JSON/JSONL   ← 唯一的交接文件（桌面系 = 仿真世界）
   │
   ├─► mirror    镜像：设置状态，只 mj_forward，从不 mj_step
   ├─► snapshot  快照：同一状态取一次，此后独立步进 → 动作预测
   └─► export    独立 MJCF：含机器人、方块、实测内参的相机
```

## 3. 实现方法

### 3.1 测量的量 vs 已知条件约束的量

这是整个方法最重要的区分，每条结果都带 `measured` 和 `assumed` 两个字段照抄下表：

| 量 | 来源 |
| --- | --- |
| **x、y、yaw**（桌面系） | **相机测量**：标定射线 + 方块轮廓拟合 |
| 中心高度 z | **不是测量**：实测桌面高度 + 已知边长的一半 |
| roll、pitch | **不是测量**：假设方块平放在平面上 |
| 尺寸、质量、摩擦 | 配置文件人工填写，每项带 `source`；质量/摩擦是仿真默认值，**从未辨识过** |

平放方块的中心高度是"已知几何的推论"，不能称为相机独立测得的深度。三个方块的 z 一模一样
（15.0 mm），正因为它不是测出来的。

### 3.2 单帧位姿估计

只有一张 RGB 图、没有深度时，一个已知尺寸的方块平放在已知平面上，只剩 `(x, y, yaw)` 三个自由度，
而它的像素轮廓恰好依赖这三个量——所以不需要深度传感器。

1. **颜色分割**：按方块自己的色带取连通域（[3.3](#33-颜色带)）。
2. **双平面雕刻做初值和筛选**：复用 `perception.plane_support_estimate`。它给出的是轮廓能保证的
   **外包络**（30 mm 方块转个角度会报出远大于边长的 extent），所以只作初值和尺寸筛选——
   一个大到不可能是方块的色块（背景椅子、人手）在**拟合之前**就被短路拒绝，不付拟合的代价。
3. **模型轮廓拟合**：方块是凸体，8 个角点投影的凸包**就是**它的轮廓。候选 `(x, y, yaw)` 用
   轮廓与颜色掩码的 IoU 打分，先粗扫再 Nelder-Mead 细化。打分用扫描线 + 掩码行前缀和，
   按亚像素插值，使分数是位姿的**连续函数**（像素量化的分数是台阶，局部优化器下不去）。
4. **平面映射考虑顶面高差**：射线与平面求交时用的是方块的真实几何，不会把顶面像素直接
   投影到桌面。
5. **对称性**：方块轮廓在自身对称步长下不变（正方形底面 90°，其他 180°）。yaw 一律折叠进
   `[0, step)`，误差按对称取模，滤波用该圆周上的环形平均——**不会出现无依据的 90° 跳变**。

### 3.3 颜色带

`perception` 的色带是给**渲染出来的**饱和色画的（纯绿 120°、纯蓝 240°），真实漆面不在那里。
本桌面的青绿色方块实测 hue 160–172，而 green 带在 170° 截止——暗面越界被判成蓝色，掩码缺一角。

所以每个方块可以在目录里带**自己的色带**（`hue_deg`、`min_saturation`、`min_value`），不写就退回
`perception` 的默认带。`run_realsim.sh colours` 实测带边是否正在切某个方块。不同颜色的带
**重叠会直接拒绝加载**——没有阈值能把它们分开。

仿真里的方块**按各自色带的颜色渲染**，而不是仿真调色板里的罐头色。否则渲染图上的自检会
"通过"一个真实方块根本不满足的色带（这个 bug 真的发生过，被 `selftest` 抓到）。

### 3.4 什么时候拒绝

平面假设失效时错误是**无界的**，所以三道检查不通过就判定无效并写明原因，而不是给出桌面位姿：

| 检查 | 含义 |
| --- | --- |
| `iou` | 这个尺寸的平放方块能不能解释这块掩码 |
| `area_ratio` | 观测掩码 / 模型轮廓面积。遮挡时偏小，色块粘连时偏大 |
| `height_check` | 重新拟合 x、y、yaw，找最能解释该轮廓的离地高度；只有比假设平面好出 `min_height_margin` 才允许拒绝 |

**能力边界是实测的，不是声称的**：`selftest` 每次重测并写进报告。当前机位下 ≥ 20 mm 的抬起被拒绝，
10 mm 不会——方块面正对相机时轮廓接近矩形，抬高的方块几乎能复现同一轮廓，这是几何退化。
另外**尺寸偏差和抬高互相补偿**（方块比目录大，拟合会用"抬高一点"去补），两者从这个视角
无法区分，所以拒绝理由里同时报出 `area_ratio`，不替你猜是哪一种。

颜色阈值会把边缘部分覆盖的像素也收进来，掩码带约半像素光晕：腐蚀 1 像素，反查边长下降 0.75 mm。
默认**不腐蚀**，偏差通过 `area_ratio` 如实报出；`measure` 给出腐蚀前后两个端点作为区间。

### 3.5 时序：关联、滤波、状态

- **关联**：同色多块按最近的上一位姿在门限内匹配；首次出现的同色同尺寸方块**无法区分**，
  按位置给一个确定的约定顺序，并在 `association.note` 里写明"这是约定不是测量"；
- **滤波**：位置指数滤波，yaw 在对称圆周上滤波；跳变超过门限就重置而不是拖出尾迹；
  **原始值和滤波值同时记录**，滤波可追溯；
- **状态**：`detected`（新鲜）→ `stale`（超过 0.5 s）→ `lost`（超过 5 s）。过期位姿始终带着**它被
  测到的时间戳**和年龄，绝不重新盖成当前时间；镜像默认拒绝把 `stale`/`lost` 的方块放进场景，
  而是把它**移出世界**。

### 3.6 坐标系

记法 `T_a_b`：把 `b` 系坐标变到 `a` 系，与 [rgbcal/transforms.py](rgbcal/transforms.py) 一致。

```
T_table_object = T_table_camera × T_camera_object
T_base_object  = T_base_table  × T_table_object
```

| 系 | 定义 |
| --- | --- |
| `table` | **参考系，也是 MuJoCo 世界**。z = 0 是实测桌面，原点是底座原点在桌面上的垂足，x 是底座 x 投影到桌面 |
| `base` | 手眼外参解出的机器人底座系。结果里同时给出 `pose_base` |
| `camera` | OpenCV 轴（x 右、y 下、z 前），内参与外参所在的系 |
| `object` | 方块自身：轴对齐长方体，**原点在几何中心**，也是镜像它的 MuJoCo free joint 的原点 |

单位米、弧度；四元数 `[w, x, y, z]`（MuJoCo 顺序）。仿真世界就是桌面系（`rgbcal sim-camera`
早就采用的约定），所以观测到的位姿**原样**写进仿真，不需要再转换。

> 本轮修复：`export`、含桌面变换的 `mirror` / `snapshot` 和新增 `sync` 按 `inverse(T_base_table)` 放置基座，
> 不再忽略 17 mm 高度和 1.88° 倾角。数学对齐不等于真实空间精度已经通过验收。

### 3.7 写入 MuJoCo：镜像、快照、导出

三种用法故意分开——混为一谈正是"数字孪生"开始自说自话的方式。

- **镜像**：虚拟方块由观测**设置**，只调 `mj_forward`，速度清零，**从不 `mj_step`**，仿真里没有任何
  东西能把方块带离相机看到的位置。模型只编译一次，更新只写 free joint 的 `qpos`，不重建场景。
- **快照**：同一状态取一次，此后**自己步进**，用来预测动作结果；**从不写回真实侧**。
- **导出**：写成独立 MJCF，任何 MuJoCo 查看器直接打开。

导出文件里有：

| 元素 | 说明 |
| --- | --- |
| 机器人 | 内嵌已有 Menagerie 模型元素，以场景桌面变换对齐 base；上游文件不修改 |
| `<compiler meshdir>` | **必须写在内嵌模型之后**：上游模型声明 `meshdir="assets"`，MuJoCo 相对**主文件**解析，不覆盖就一个网格都找不到 |
| 方块 | 每块一个 `<body>` + `<freejoint>`，位姿写在 body 上——**打开就是采集到的状态**，不用先载入关键帧 |
| `calibrated` 相机 | 用实测内参而非 fovy：`resolution` + `focalpixel` + `principalpixel`，切到它就是真实相机的视角 |
| 关键帧 | 机械臂姿态，**名字写明是否观测到**：`arm_as_read`（夹爪未知时加 `_gripper_NOT_observed`）或 `arm_rest_NOT_observed` |
| 文件头注释 | 采集时间、世界系定义、每块位姿、测量/假设的区分、标定文件及 sha256 |

`principalpixel` 的两个轴都**与图像轴反向**。这是把四种符号组合逐一渲染、从 GL 视锥回读内参
测出来的，不是从文档猜的；搞错任一个轴视角会偏两倍主点偏移量，而画面看上去依然正常。
`tests/test_export.py` 用回读内参把它锁住。

## 4. 代码结构

**两个环境、一个文件。** 真实侧要 OpenCV（`.venv-rgbcal`），仿真侧要 MuJoCo 和 `so101_nexus`
（`.venv-loop`）；采集侧需要 `scservo_sdk`、pyserial 和 NumPy，显示侧不需要串口 SDK。所以映射不是一次函数调用，而是一个文件——
`observe`/`live` 写、`mirror`/`snapshot`/`export`/`replay` 读的场景状态。这与 `rgbcal sim-camera`
导出 `sim_camera.json` 给仿真读是同一种做法。`run_realsim.sh` 按子命令自动选解释器。

| 模块 | 运行环境 | 作用 |
| --- | --- | --- |
| [geometry.py](realsim/geometry.py) | 两者皆可（纯 NumPy） | 投影、凸包、扫描线打分、yaw 与对称性 |
| [blocks.py](realsim/blocks.py) | 两者皆可 | 方块目录：尺寸/质量/摩擦及出处、色带；拒绝"猜的"尺寸 |
| [estimate.py](realsim/estimate.py) | 两者皆可 | 单帧位姿拟合、质量检查、`measure_size`、`colour_report` |
| [track.py](realsim/track.py) | 两者皆可 | 关联、滤波、`detected`/`stale`/`lost` |
| [scene.py](realsim/scene.py) | 两者皆可 | 场景状态 schema、JSON/JSONL 读写、可用位姿筛选、`contact_check` |
| [observe.py](realsim/observe.py)、[overlay.py](realsim/overlay.py) | `.venv-rgbcal` | 取图、去畸变、叠加、警告、只读舵机 |
| [drift.py](realsim/drift.py) | `.venv-rgbcal` | 相机相对锁定机位动了多少 |
| [mirror.py](realsim/mirror.py) | `.venv-loop` | 镜像、快照、重投影自检 |
| [export.py](realsim/export.py) | `.venv-loop` | 写独立 MJCF |
| [selftest.py](realsim/selftest.py) | `.venv-loop` | 渲染已知位姿 → 估计 → 量误差 |
| [arm_feedback.py](realsim/arm_feedback.py)、[bus_access.py](rgbcal/bus_access.py) | `.venv-rgbcal` / ServoBus 所有者 | 原子最新值、原始日志、进程内发布和串口/SDK 互斥 |
| [arm_mapping.py](realsim/arm_mapping.py) | 两者 | 具名标定转换、寄存器与范围校验、可选实测夹爪标定 |
| [arm_sync.py](realsim/arm_sync.py) | `.venv-loop` | 基座对齐、关节镜像、状态时效、审计日志与回放 |
| [cli.py](realsim/cli.py) | 两者 | 命令行；模块顶层不 import cv2 或 MuJoCo，因为两个环境都要构建同一个解析器 |

**复用**：`nexus_vision.perception`（颜色阈值、平面求交、双平面雕刻）、
`rgbcal.realcam`（去畸变、桌面系、标定加载与配置校验）、`rgbcal.pointcheck.solved_arm`（关节约定）、
`nexus_vision.cameras`（`calibrated` 相机）、`so101_nexus` 的 free joint 槽位机制。

## 5. 使用

### 配置

物体目录配置：[realsim/config/blocks.json](realsim/config/blocks.json)。关节同步复用场景关联的标定，夹爪可选实测配置见第 10 节。

| 字段 | 说明 |
| --- | --- |
| `id` / `colour` | 唯一标识；颜色名取 red/orange/yellow/green/blue |
| `hue_deg`（可选） | 这个方块自己的色相带，用 `colours` 实测后填；不同颜色的带不能重叠 |
| `min_saturation` / `min_value`（可选） | 阈值下限，暗面偏暗时调低；默认 0.35 / 0.20 |
| `edge_mm` 或 `size_mm` | **必填**。整条链路的尺度，所有位置随它线性缩放。当前三块均为 30 mm（用户提供） |
| `size_source` | 数字的出处，逐字写。`guess`/`unknown` 直接拒绝加载 |
| `size_measured` | 有明确依据才写 `true`；`false` 时每条记录带警告 |
| `physics.mass_kg` / `friction` | 仿真默认值，`*_source` 如实写 |
| `tolerances` | 验收阈值（位置 5 mm、yaw 10°），依据写在文件里：夹爪对 30 mm 方块的容差 |

内参、外参、桌面、机位全部复用 `rgbcal` 已有的标定文件，**不需要重填**。

### 命令

```bash
# ── 体检：先跑这三条 ─────────────────────────────────────────────────────
bash LLM-control/run_realsim.sh check                 # 有哪些标定、哪些方块、哪些尺寸没量过
bash LLM-control/run_realsim.sh drift                 # 相机相对锁定机位动了没有（失败退出码 2）
bash LLM-control/run_realsim.sh scale-check --pair green_cube blue_cube   # 独立尺度检验（失败退出码 2）
#   scale-check 要先把两块方块正对贴紧放在工作区中央

# ── 采集 ─────────────────────────────────────────────────────────────────
bash LLM-control/run_realsim.sh observe --session my-run              # 实时取一帧
bash LLM-control/run_realsim.sh observe --image <raw.png> --session my-run   # 或离线读一张原始帧
bash LLM-control/run_realsim.sh observe --stacked red_cube=0.030 ...  # 堆叠必须声明支撑高度
bash LLM-control/run_realsim.sh live --duration 60 --no-height-check --session my-run  # 连续 → JSONL

# ── 进入 MuJoCo ──────────────────────────────────────────────────────────
bash LLM-control/run_realsim.sh export   --scene LLM-control/calib/my-run/scene.json \
     --output LLM-control/calib/my-run/scene.xml
LLM-control/.venv-loop/bin/python -m mujoco.viewer --mjcf=$PWD/LLM-control/calib/my-run/scene.xml
bash LLM-control/run_realsim.sh mirror   --scene LLM-control/calib/my-run/scene.json --follow --viewer
bash LLM-control/run_realsim.sh snapshot --scene LLM-control/calib/my-run/scene.json --settle 1.5
bash LLM-control/run_realsim.sh replay   --log LLM-control/calib/my-run/scene.jsonl

# ── 换方块 / 换灯光 / 复核尺寸 ───────────────────────────────────────────
bash LLM-control/run_realsim.sh colours --image <raw.png>   # 色带是不是在切某个方块
bash LLM-control/run_realsim.sh measure --image <raw.png>   # 反查尺寸（给区间，不能代替卡尺）

# ── 不碰真机的自检 / 只读舵机 ────────────────────────────────────────────
bash LLM-control/run_realsim.sh selftest --grid 9
bash LLM-control/run_realsim.sh arm
```

场景状态记录了：时间戳、物体 ID、位姿（原始与滤波）、尺寸、坐标系、估计方法、质量指标、
有效状态与年龄、标定文件及 sha256、对应原始图像路径、警告。

### 测试

```bash
cd LLM-control
PYTHONPATH=. .venv-rgbcal/bin/python -m pytest realsim/tests -q                    # 真实侧：56 passed, 3 skipped
PYTHONPATH=third_party/so101-nexus/src:. MUJOCO_GL=egl .venv-loop/bin/python \
    -m pytest realsim/tests -q                                                    # 仿真侧：57 passed, 2 skipped
```

以上测试数量是既有文档的历史记录；本轮最终测试数量见第 10.6 节链接的验证记录。两个环境跑同一套测试，各自跳过缺依赖的几个。测试覆盖：变换方向与四元数、顶面高度补偿、
对称与 yaw 环形平均、失效状态（抬起/堆叠/遮挡/空桌面）、`stale`/`lost` 与镜像的关系、
导出文件的相机内参回读、启动器路由表与解析器一致。

## 6. 独立验证关卡

自洽指标（IoU、抖动、重投影）**看不见系统性偏差**：一个估计可以稳定地错。所以有三道关卡，
每一道都不依赖被检验的那条标定链路：

| 关卡 | 检验什么 | 依据 |
| --- | --- | --- |
| `selftest` | 软件链路本身（投影约定、桌面系、平面假设、yaw 对称、拒绝路径） | 渲染已知位姿，真值本来就是输入。**看不见**真实颜色、光照和标定错误 |
| `drift` | 相机相对锁定机位有没有动 | 特征匹配 + 实测内参。判据是位移（3 px，标定文档规定），旋转角只作量级 |
| `scale-check` | 场景尺度 | 两块贴紧的方块中心距必须等于边长——**关于方块的事实，不来自标定**，不需要量具 |

`scale-check` 把中心距分解为沿接触面法向和侧向：法向分量才是尺度，侧向大说明没贴紧，
是摆放问题而不是管线问题，不会被算成误差。

## 7. 验证状态

完整记录与证据：[logs/2026-09-21-realsim-mapping/README.md](logs/2026-09-21-realsim-mapping/README.md)。
三类严格分开，**合成结果不能当实测**：

| 类别 | 结果 |
| --- | --- |
| **合成**（渲染已知位姿，9 机位） | 9/9 检出；位置误差中位 **0.15 mm**、最大 0.29 mm；yaw 最大 **0.73°**；20/40/80 mm 抬起拒绝，10 mm 不拒绝 |
| **真实图像**（在线，三块同时） | 三块全检出，IoU 0.92–0.96；镜像重投影 0.7–1.3 px |
| **真实设备**（在线 30 s，只读） | 12/12 帧三块全检出；0.39 Hz、单帧 2.3 s（关 `height_check`）；静止抖动 σ 0.04–0.07 mm |
| **MJCF 导出** | 任意目录可编译；三块在静止状态即位于采集位姿；相机回读 fx/fy/cx/cy/位置/旋转与标定完全一致 |
| **独立尺度检验** | ❌ **未通过**：应为 30.00 mm，实测 **31.2 mm（+4%）**，σ 0.06 mm，侧向错位 −0.34 mm（确实齐平） |
| **相机是否移动** | ❌ **已移动**：匹配点位移中位 **55 px**，判据 3 px，量级约 2.2° |
| **真实位置精度** | 未测量；在上两项通过之前没有意义 |

**重复性不是精度。** σ 0.06 mm 说明估计稳定，独立检验一测就发现它稳定地偏了 4%。
两者同时成立，因为偏差来自输入的外参，不来自估计代码。

据此的**误差预算**（用于判断这次建模能干什么）：场景内部相对尺度 +4.4%；朝向一致性 1.8°；
相对底座的绝对位置约 **25 mm 量级**（把实测的 2.2° 旋转施加到标定位姿，同一像素的桌面交点移动 25.3–25.9 mm）。

- **可以用**：场景的相对排布、物理试验、把快照接给对话模型做动作预测；
- **不可以用**：任何要发给机械臂的绝对抓取坐标（25 mm 远大于夹爪容差）。

## 8. 已知限制

- 只支持**平放的刚性长方体**；`so101_nexus` 原语只有正方体，非正方体会被**明确拒绝**，不会四舍五入成正方体；
- 抬起 < 20 mm、堆叠未声明支撑高度、严重遮挡时 `height_check` 可能判不出来；
- 同色同尺寸方块首次出现时无法区分（见 3.5）；
- **不是实时**：1920×1080 三个方块开 `height_check` 约 11 s/帧，关掉约 2.3 s/帧；
- 颜色阈值对光照敏感，换灯光后重跑 `colours`，`area_ratio` 会先报警；
- 机械臂姿态：只有读了舵机才有；未读时导出文件里是标注为 `arm_rest_NOT_observed` 的关键帧。
  本轮新增持续关节反馈镜像，已做合成端到端验证；真机只读尝试未收到状态包，真实姿态一致性尚未验证，见第 10 节；
- `usable_for_motion` 仍为 `false`：手眼外参没过监督运动检查，见 [RGBCAL_SUMMARY.md](RGBCAL_SUMMARY.md) §5。

## 9. 后续

**先重新标定**（`drift` 与 `scale-check` 都已指向同一个原因）：

```
run_rgbcal.sh lock                       # 锁定当前机位
run_rgbcal.sh handeye-collect / handeye-solve / handeye-verify
run_rgbcal.sh table-plane
run_rgbcal.sh sim-camera                 # 重新导出给仿真
run_realsim.sh drift  &&  run_realsim.sh scale-check   # 两者都通过才继续
```

**再做真实精度验证**（需要人在现场）：

1. 用卡尺量每个方块三边，核对 `blocks.json`；
2. 工作区**中心和边缘**至少 5 个位置摆方块，用直尺/卡尺相对底座独立量出 x、y（不要用这台相机），记下朝向；
3. 每个位置 `observe`，对比 `pose_base.position_m`；
4. 再测不同朝向（0/15/30/45°）、移动后重新检出、部分遮挡、抬起 10/20/40 mm 是否被拒；
5. 记录位置误差、按对称取模的朝向误差、静止抖动、有效更新率、端到端延迟，把验收阈值写回 `tolerances`。

**接到 LLM 动作试验上**：

1. `observe` 或 `live` 得到当前场景状态 → `snapshot`（或 `export`）；
2. 对话里的模型按 [LOOP_README](LOOP_README.md) 的协议提出一个动作；
3. 在快照里执行并步进，读回方块位姿判断结果——这是**预测**；
4. 只有预测通过，才考虑在真机上执行（需 `usable_for_motion` 为 true 且用户明确授权）；
5. 真机执行后**重新观测**，绝不用快照的状态冒充真实状态。

仿真闭环控制本身仍走既有的 `run_loop.sh`；本包只负责把真实场景搬进去。

## 10. 真实关节反馈同步（本轮新增）

本轮目标是把真实工作区中的**物体和机械臂实际状态**放到同一 MuJoCo 世界，供观察和后续仿真试验使用。
`sync` 已实现持续读取实际关节反馈、具名映射、镜像刷新、原始录制和离线回放。
**五个臂关节复用既有手眼标定；夹爪模型角度仍缺实物标定，默认只报告 ticks，不刷新夹爪角度。**
没有新增 IK，也不发送真机运动指令。物体仍取自启动时的 XML；本命令不持续重建物体。

### 10.1 状态来源、串口和只读保证

现有抓取入口 `calib/c920-real-grasp-01/tools/grasp_red.py` 的 `move()` 使用
`rgbcal.robot.ServoBus`：写 `Goal_Position` 推进动作，另外读 `Present_Position` 检查跟踪。
同步读取的是后者，**从不把目标位置当反馈**，也不读取控制脚本里的规划数组。

新增可选发布器在**持有 ServoBus 的同一进程内**采集，通过同一个 `RLock` 串行执行总线事务；
每次采集六个电机的 `read_state()`，包括实际位置、Homing Offset、范围寄存器和扭矩状态。
采用进程内后台线程是为了控制脚本即使睡眠，也能持续发布反馈。它会增加总线占用，需现场测量其对
原控制循环的影响；`SO101_FEEDBACK_HZ` 可以降低频率，目标频率不保证能够达到。

检查了当前安装的 `scservo_sdk/port_handler.py` 和 `robot.py`：打开连接只打开串口、设置波特率、
清空输入缓冲；退出只关闭串口。发布器只调用 `read_state()`，不调用 `_write`、`hold`、`release`，
不改变扭矩、模式、Homing Offset 或目标。控制流程自己原有的写操作保持原语义。

Linux 串口互斥：打开前检查 `/proc/*/fd` 已有持有者，以 `flock` 防止两个本项目进程同时初始化，
连接后 `TIOCEXCL` 拒绝其他普通进程再次打开。遇到“already open”时使用设备所有者的发布接口；
不要启动第二个直接采集器。已启动的控制进程无法事后注入环境变量，需要在其**下一次正常启动**时启用发布。
不使用本项目 `ServoBus` 的外部控制程序（例如直接使用 LeRobot）尚未自动接入，必须在该程序内适配实际反馈，
不能同时启动 `arm-stream` 抢串口。

发布文件为原子替换的单份最新 JSON，无历史队列；消费者刷新慢时跳过中间帧。原始 JSONL 则保留每次采样。
发布路径另有单写者锁。所有录制文件使用新建模式，存在时拒绝覆盖；控制进程每次启动应使用新的日志路径。

### 10.2 映射与标定来源

启动时从 `scene.json.calibration.sources.extrinsics` 加载手眼标定，并核对场景记录的 SHA256；
再读取该外参的 `source_session/poses.json`。以下是当前 `c920-handeye-03` 的实际值：

`q_rad = sign × (Present_Position_ticks − zero_ticks) × 2π/4095 + calibration_offset_rad + model_offset_rad`

下表的 offset 是原手眼约定偏移；当前场景另含腕滚转模型偏移 −π/2，见表后说明。

`Present_Position` 已包含舵机内 Homing Offset 的作用，**不能再次减去 homing_offset**。
没有使用 LeRobot 的归一化值。原始反馈须为 0–4095 的整数；转换值还必须位于 MJCF joint range 内，
不截断、不静默环绕，任一必要关节出错时整帧不应用。

| 真实名称 / MuJoCo joint（同名） | 电机 ID | zero_ticks | sign | offset_rad | 模型范围 rad |
| --- | --- | --- | --- | --- | --- |
| shoulder_pan | 1 | 2101 | +1 | 0（固定为舵机零点） | −1.91986…1.91986 |
| shoulder_lift | 2 | 2087 | +1 | 0.0535294564 | −1.7453293…1.7453293 |
| elbow_flex | 3 | 1984 | +1 | −0.0620798201 | −1.69…1.69 |
| wrist_flex | 4 | 1956 | +1 | 0.1824787813 | −1.658063…1.658063 |
| wrist_roll | 5 | 2047.5 | +1 | 0（固定为舵机零点） | −2.7438473…2.7438473 |
| gripper | 6 | **没有确认的模型零点** | **未确认** | **未确认** | −0.174533…1.7453292 |

运行时映射表、标定哈希及实际模型范围会打印到终端，并写入 `--record` 日志的 metadata。
五轴方向和三个偏移是既有标定求解结果，不代表本轮重新做过物理验证。
肩部旋转和腕部旋转的零点在手眼问题中不可单独辨识。没有独立修正时使用固定零点约定，
并明确显示 `unverified_pinned`，不能把外参 `solved: true` 当作所有轴零点已验证。
**2026-09-21 现场补充：已观察到末端本体朝向不一致（用户观察约 90°）。**
`wrist_roll` 的 2047.5 ticks 零点来自全编码器范围中点，源文件仍标为未对照运动学模型验证；
手眼求解不能独立确认该零点。当前映射不应视作已通过真机末端朝向验证。
检查证据与回位记录见 [EE_CHECK_AND_RETURN.md](logs/2026-09-21-real-joint-grasp/EE_CHECK_AND_RETURN.md)。

**当前修正：** `calib/realsim-model/scene.json` 的 `model_joint_offsets.wrist_roll`
配置 `radians: -1.5707963267948966`、`status: "visual_estimate"` 和现场对比来源 `source`。
这是一组真实照片/同反馈渲染支持的约 −90° 显示修正，精确零点和误差范围尚未测量。
修正作用于实体 joint qpos，保留原手眼标定、基座变换和真机控制。
`sync` 和 `arm-replay` 自动读取该字段，原有启动命令继续有效；无需重新导出 XML。
静态 `mujoco.viewer --mjcf` 不读取该 JSON，不能用于验证实时反馈映射。
其他场景不会自动继承该配置；重新生成 scene.json 时需显式保留本机的修正及来源。
每个修正须指定已知关节名、有限弧度值（绝对值 ≤ π）、非空 `source`，以及
`visual_estimate` 或 `measured` 状态。只添加一次，不修改编码器值，不自动扩大关节范围。
日志分别记录 `calibration_angles_rad` 与 `converted`，metadata 中保存全部修正参数；
窗口和状态 JSON 分开显示反馈新鲜度与零点验证状态。
具体根因、前后图、实测频率及限制见 [腕部映射修正记录](logs/2026-09-21-real-joint-grasp/ee-mapping-fix/README.md)。
采样中的六个电机 Homing Offset 和范围寄存器必须与手眼采集时一致；重新做过舵机标定会拒绝同步，
需要更新相应标定与场景。

**夹爪是单独的 hinge**：当前 Menagerie 模型只有一个可动钳口关节，没有第二个联动关节或 equality 约束。
LeRobot 保存的 `[2020, 3466]` 是舵机标定行程，不是模型 `[-10°, 100°]` 的角度标定；抓取脚本的
开合目标同样不能证明实际钳口角度。本轮去掉了旧 `mirror` / `export` 把 fraction 当已知模型角度的行为。
没有夹爪标定时保留当前仿真钳口、显示 `uncalibrated`，日志仍记录电机 6 的原始 ticks。

补齐实测关系后，可通过 `--gripper-map /绝对路径/gripper-measured.json` 启用两点线性转换。
JSON 包含 `ticks: [低tick, 高tick]`、`radians: [对应模型角度, 对应模型角度]` 和非空 `source`。
两个角度可以递减，以表达负方向；只在两点覆盖区间内使用，区间外判无效。
必须先测量两个实际姿态相对模型钳口零点的角度，并用第三个中间姿态验证线性关系；
不能用最大行程、发送目标值或测试夹具文件充当实物测量。

### 10.3 空间对齐与显示

世界系仍是桌面系。物体已经使用 `pose_table`，机器人则设置为：

`T_world_baseframe = T_table_base = inverse(scene.calibration.T_base_table)`。

`align_base()` 同时处理 `baseframe` 在模型 base body 内的局部变换，检查该 base 是世界的直接子节点，
避免重复应用变换。当前记录对应基座原点距桌面约 **17.15 mm** 和约 **1.88°** 的倾角。
`export` 现在内嵌已有机器人 XML 元素并设置 base 位姿（不修改上游模型），网格仍使用绝对路径。
`sync` 也会按场景记录重新设置 base，因此旧版未对齐的 XML 也能使用。
`mirror` / `snapshot` 遇到含桌面变换的场景记录时同样对齐，快照会保存并恢复基座位姿。

场景 JSON 必须与 XML 属于同一次导出，尤其是物体、相机和桌面配置；不会自动比较或修复配错的 XML。
旧合成记录若没有桌面变换，原导出仍可用，但 `sync` 要求真实场景的完整标定字段。
坐标数学对齐已测试；相机漂移和外参误差不会因此消失，机械臂与真实物体的物理相对位置仍需现场验收。

模型只加载一次。通过 `model.joint(name).qposadr` 更新关节 `qpos`，清零速度并调用 `mj_forward`。
不调用 `mj_step`，不依赖 actuator 目标、重力或接触积分，所以这是**反馈镜像显示**，不能作为动力学结果。
常规 `python -m mujoco.viewer` 只是查看导出的静态模型，不读取真实状态；持续镜像必须用 `sync --viewer`。

### 10.4 实际启动、录制和回放命令

以下均在仓库根目录执行；原有导出命令保留。重新导出会覆盖指定 XML，已有实验 XML 如需保留请换输出名。

```bash
bash LLM-control/run_realsim.sh export \
  --scene LLM-control/calib/realsim-model/scene.json \
  --output LLM-control/calib/realsim-model/scene.xml

LLM-control/.venv-loop/bin/python -m mujoco.viewer \
  --mjcf="$PWD/LLM-control/calib/realsim-model/scene.xml"
```

**优先方案：随原有控制进程发布。** 在启动原有人工操作或控制流程的终端中设置下面变量，再执行你原有的控制命令。
下面命令本身不启动控制、不触发动作；不用改抓取脚本，也不要运行另一个 `arm-stream`。

```bash
mkdir -p LLM-control/runs/arm-sync
export SO101_FEEDBACK_PATH="$PWD/LLM-control/.runtime/arm-feedback.json"
export SO101_FEEDBACK_HZ=30
export SO101_FEEDBACK_LOG="$PWD/LLM-control/runs/arm-sync/raw-$(date +%Y%m%d-%H%M%S).jsonl"
# 在此终端按原有流程启动使用 rgbcal.robot.ServoBus 的程序。
# 这些变量在 ServoBus 创建时生效；不启用变量则没有后台采集线程。
```

**独占只读采集（设备空闲、没有控制进程时）：** 例如由人按原有流程操作机械臂；采集模块不启用或关闭扭矩。
在新的终端中运行，可 Ctrl-C 退出：

```bash
mkdir -p LLM-control/runs/arm-sync
unset SO101_FEEDBACK_PATH SO101_FEEDBACK_LOG SO101_FEEDBACK_HZ
bash LLM-control/run_realsim.sh arm-stream \
  --port /dev/ttyACM0 --rate 30 \
  --state LLM-control/.runtime/arm-feedback.json \
  --log "LLM-control/runs/arm-sync/raw-$(date +%Y%m%d-%H%M%S).jsonl"
```

**另一个终端：实时显示并录制原始输入、转换结果、状态变化。**

```bash
bash LLM-control/run_realsim.sh sync \
  --scene LLM-control/calib/realsim-model/scene.json \
  --xml LLM-control/calib/realsim-model/scene.xml \
  --state LLM-control/.runtime/arm-feedback.json \
  --display-rate 60 --stale-after 0.5 --disconnect-after 2 \
  --status-output LLM-control/.runtime/arm-sync-status.json \
  --record "LLM-control/runs/arm-sync/converted-$(date +%Y%m%d-%H%M%S).jsonl" \
  --viewer
```

无图形桌面时去掉 `--viewer`，可加 `--duration 10` 限时运行（0 表示直到退出）；终端与 status JSON 显示状态。
采集、显示频率相互独立。raw 日志记录所有发布样本；converted 日志只记录显示进程实际收到的样本，
原始反馈在每条 `raw` 中，弧度在 `converted` 中，另有 status_event。

**离线回放原始日志**（不打开串口，按记录的采样间隔播放，空档保留）：

```bash
RAW_LOG=$(ls -t LLM-control/runs/arm-sync/raw-*.jsonl | head -n 1)
bash LLM-control/run_realsim.sh arm-replay \
  --scene LLM-control/calib/realsim-model/scene.json \
  --xml LLM-control/calib/realsim-model/scene.xml \
  --log "$RAW_LOG" --speed 1 --display-rate 60 \
  --record "LLM-control/runs/arm-sync/replay-$(date +%Y%m%d-%H%M%S).jsonl" \
  --viewer
```

raw 日志与 converted 审计日志格式不同；`arm-replay --log` 接受前者。
`--speed` 可调整播放速度；调度、数据年龄和超时统一使用虚拟回放时钟，按记录的接收时刻发布并保留采样耗时。慢渲染只显示最新到期样本，不会为了逐帧补播积压延迟。
窗口标明 `REPLAY`，日志保留原始时间；结束前短暂保留末姿态，随后自动退出。
如录制时使用过夹爪实测映射，回放时也提供相同 `--gripper-map`。录制不会改变场景中的物体位置。

### 10.5 时间、失效行为与限制

- 原始记录包含 `sampled_at`（Unix 秒）、`sampled_monotonic`、同机启动标识 `clock_id`、
  `received_at`（六电机读完时间）、采样跨度 `sampling_span_s`、stream ID、sequence、valid、原始 ticks 和寄存器。
  六电机是**依次读取**，不是硬件同时采样；采样时间是这一批开始的主机时间，不是舵机硬件时间戳。
- 显示端另外记录接收时间、处理耗时、转换结果和最后有效采样时间。live 数据年龄以同一机器的单调时钟计算，
  不受系统墙钟校时影响；跨主机/重启遗留记录拒绝作为 live 使用，应用回放命令。
- 默认 >0.5 s 无有效数据为 `stale`，>2 s 为 `disconnected`；生产者明确报错或退出可立即显示 disconnected。
  malformed / 缺关节 / NaN / 范围越界 / 标定寄存器不一致为 invalid，不修改最后有效姿态。
  恢复有效数据后自动继续。窗口文字、终端和 status JSON 显示状态、年龄和最后有效时间；从未收到反馈时为 disconnected。
- 同步端退出记录 mirror stopped，并把 status JSON 标为 disconnected，保留最后姿态与时间，然后关闭窗口与日志。发布器退出先停止采样线程，写 disconnected，再关闭串口；**不释放扭矩**，
  不改变现有控制进程对机械臂的管理。发布路径或录制文件初始化失败会明确报错。
- 暂时读失败后收到有效数据会恢复；USB 拔出重插需要原串口所有者按原流程重新连接，发布器不擅自重建控制连接。
- 当前实现面向本机 Linux；未适配 Windows 串口互斥、远程时钟同步或外部控制框架。
- `refresh_hz` 是主机刷新循环/`viewer.sync` 调用率，不能当作显示器扫描输出 FPS。
  `processing_s` 不包括相机曝光、真实机械运动、显示器扫描输出；本轮无法测得真实运动到画面的端到端延迟。

### 10.6 本轮验证与剩余现场步骤

最终回归：`.venv-loop` **175 passed, 6 skipped**；`.venv-rgbcal` **109 passed, 5 skipped**。
详细证据：[logs/2026-09-21-arm-sync/README.md](logs/2026-09-21-arm-sync/README.md)。
合成程序：[realsim/tests/synthetic_sync.py](realsim/tests/synthetic_sync.py)。它只写测试反馈，不访问真机：

```bash
PYTHONPATH=LLM-control LLM-control/.venv-loop/bin/python \
  LLM-control/realsim/tests/synthetic_sync.py \
  --output "LLM-control/runs/arm-sync/synthetic-$(date +%Y%m%d-%H%M%S)"
```

已做软件验证：具名映射、零点/单位、正向增量、缺失与非有限值、夹爪两点转换与超范围拒绝、标定变化拒绝、
错误后恢复、过期/未来时间、串口已占用拒绝、后台采样与控制读取互斥、连接/退出不写舵机、
MuJoCo 中物体位置不变及 baseframe 与桌面逆变换一致。夹爪测试数据是**合成夹具**，不作为真实标定。

本次无头合成集成验证：有效应用 93 帧，刷新 59.7 Hz；内部处理平均 0.19 ms、最大 0.26 ms，
数据年龄平均 14.48 ms、最大 17.09 ms。含人为停止发布 2.4 s、自动恢复及按原间隔回放；这些数字依赖本机负载。
采集阶段配置 30 Hz，统计全程接收率约 13.3 Hz（含中断、启动和结束等待），不是正常采样率降低的测量。

最初设备检查（历史记录）：发现空闲 `/dev/ttyACM0`，尝试了约 3 s 只读采样，没有收到电机 1 的状态包，零个有效反馈。
可能电机未通电或总线连接异常，**无法由超时确定具体原因**。未发送任何运动、扭矩或校准指令；
没有完成真实静止姿态、单关节方向/幅度、夹爪同步、物体相对位置或真实有效采样率验证。

现场继续验证：

后续同日已在供电后取得六个电机的有效实测反馈，并在用户授权的逐步关节控制过程中持续发布给 MuJoCo。
该现场操作发现了上述末端朝向问题，已中断抓取并回到启动姿态；不能据此宣称姿态映射验证通过。

1. 由原有人工流程检查供电与连线，并确认只有一个串口持有者；启动所有者发布或独占采集，先检查六电机 ticks 与寄存器有效。
2. 三个以上静止姿态逐轴对照模型；按原有人工/控制流程单独改变各关节，检查正负方向和角度幅度。同步程序保持只读。
3. 实测夹爪两个角度及 ticks，填写有出处的映射，再用中间角度验证；未通过时保持 uncalibrated。
4. 复查相机漂移、桌面外参以及底座/模型定义；独立测量机器人与重建物体的相对位置。当前旧标定偏差不能忽略。
5. 暂停发布超过 0.5 s / 2 s，确认状态与最后姿态保留；恢复发布；关闭显示/发布进程，确认没有新增扭矩或位置写入。
6. 从 raw 日志计算有效采样间隔、采样跨度，从 converted 日志计算数据年龄和处理耗时，记录开启采样前后原控制周期。
   如需真实端到端延迟，用共同时间参考/高速摄影同时观测真机与屏幕，不能用 Python 内部计时代替。
