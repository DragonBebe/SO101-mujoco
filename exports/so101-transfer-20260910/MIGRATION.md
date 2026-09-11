# SO101 闭环控制迁移说明

可以迁移。这个目录是当前已验证工作流的独立副本，不依赖原仓库中的 LeRobot
实验、旧控制脚本或原机器的绝对项目路径。建议先在另一台 Linux 桌面电脑验证；
原生 Windows/macOS 尚未验证，现有 Bash 启动器、Unix socket 和图形后端需要适配。

## 必要模块

|模块|文件/目录|用途|
|---|---|---|
|当前对话中的规划器|另一台电脑上的 VS Code Codex 会话|理解自然语言、查看图像、逐步选择动作；沿用用户选择的 Codex-astra。Python 没有另外调用 LLM|
|命令入口|`LLM-control/loop.py`|serve / command / status，定位本目录的 Nexus 包|
|本地通信|`nexus_vision/bridge.py`|Unix socket、严格 JSON、持久服务；其导入依赖 `evaluate.py`，迁移时保留|
|任务管理|`nexus_vision/loop_session.py`|任务原文、步数、帧和任务校验、暂停/取消/完成、事件日志|
|多物体场景|`nexus_vision/workspace.py`|红绿方块、紫色大方块、黄色圆柱、蓝色区域；原地重置|
|机械臂控制与相机|`nexus_vision/simulation.py`|MuJoCo 物理、本体反馈、IK、动作插值和 RGB-D 渲染|
|视觉定位|`nexus_vision/perception.py`|像素/深度反投影、颜色候选；不需要下载视觉模型|
|相机窗口|`nexus_vision/camera_viewer.py`|RGB/深度预览；双窗口时使用独立进程。无窗口运行时可不启用|
|机器人与物体资产|`LLM-control/third_party/so101-nexus/`|环境源码、SO101 XML/STL、物体定义、相机与动力学支持、许可证|
|Python 依赖|`requirements-loop.txt` / `requirements-loop.lock`|兼容范围 / 本次安装版本快照|
|启动与对话约定|`run_loop.sh`、`setup_loop.sh`、根目录 `AGENTS.md`|安装、启动，以及 Codex 的逐步观察执行约定|
|VS Code 菜单（可选）|`.vscode/tasks.json`|启动、状态、暂停、关闭快捷任务|

上述 nexus_vision 文件均位于 `LLM-control/nexus_vision/`。迁移包保留整个
`nexus_vision` 和第三方源码目录，以满足当前导入关系和资产引用；不建议只复制
一个 XML 或几个 Python 文件。第三方许可证随源码保留。

依赖包括 Python 3.12+、MuJoCo 3.3.2、NumPy、SciPy、Pillow、Gymnasium、GLFW、
PyOpenGL，以及 Nexus 导入所需的 tyro、huggingface-hub、trimesh。pytest 用于测试。
本工作流不依赖训练框架、单独的 LLM API Key、真机电机驱动或串口。
图形模式需要可用桌面与 OpenGL 驱动；无头渲染需可用 EGL。MuJoCo 在 CPU
推进物理；这里没有 CUDA/训练 GPU 的代码要求。

## 不需要迁移的内容

- 原机器 `.venv-loop` / `.venv-vision`：在新机器重建，不能直接搬解释器软链接。
- `.runtime/*.sock`：每次服务启动时重新创建。
- 根目录的旧 IK 实验、`lerobot/`、`manipulator_grasp/`、`SO101-envs/`。
- Codex 的登录文件或模型权重：这个包不包含它们，也不应复制登录凭据。
- 旧 `control.py` / `vision.py` / 示例回放：三次任务走的是 `loop.py`。

## 新电脑安装与启动

1. 解压迁移包，用 VS Code 打开最外层 `so101-transfer-20260910` 文件夹。
2. 确认 Python 3.12+，在该文件夹终端执行：

```bash
bash LLM-control/setup_loop.sh
```

安装器优先使用 uv；没有 uv 时使用 python3 的 venv 和 pip。要指定解释器：

```bash
SO101_BASE_PYTHON=/path/to/python3.12 bash LLM-control/setup_loop.sh
```

首次安装需要访问 Python 依赖源。若需要尽量对齐本次版本，环境建好后可执行：

```bash
LLM-control/.venv-loop/bin/python -m pip install -r LLM-control/requirements-loop.lock
```

如果 uv 建立的环境没有 pip，可改用：

```bash
uv pip install --python LLM-control/.venv-loop/bin/python -r LLM-control/requirements-loop.lock
```

版本快照不是跨操作系统、跨架构通用的离线 wheel 包；本包不含依赖安装包。

3. 启动桌面服务：

```bash
MUJOCO_GL=glfw bash LLM-control/run_loop.sh serve --viewer --realtime --camera-viewer
```

或在 VS Code 中运行任务“SO101: 启动自然语言控制环境”。保持服务终端运行，
在另一终端执行：

```bash
bash LLM-control/run_loop.sh status
```

4. 在新的 Codex 对话中说：

> 读取 AGENTS.md，使用 loop 服务观察桌面，把绿色方块放到红色方块上。

当前会话需要能执行本地命令并查看生成的图片。服务只接收动作，不会自行读取
VS Code 聊天内容，也不会在 Codex 不运行时独立产生决策。模型选择在你的
Codex 客户端完成，这个迁移包没有硬编码另一个模型。

## 迁移自检

从迁移包根目录执行：

```bash
cd LLM-control
PYTHONPATH=third_party/so101-nexus/src MUJOCO_GL=egl .venv-loop/bin/python -m pytest tests -q --confcutdir=. --rootdir=. --import-mode=importlib
```

这组随包测试覆盖视觉定位、任务状态、服务、真实 TCP 移动、重置与启动脚本。
桌面上把 `MUJOCO_GL=egl` 换成 `glfw`，会再运行窗口测试。无头启动可省略所有
窗口参数：`MUJOCO_GL=egl bash LLM-control/run_loop.sh serve`。

## 三次任务记录

- `task-records/TASK_LOG.md`：中文过程、失败/调整、逐步 JSON、时间和最终依据。
- `task-records/summary.json`：三次任务统计。
- `task-records/session/events.jsonl`：逐字节保留的原始事件快照。
- `task-records/session/events.portable.jsonl`：图片路径改为相对 session 目录的副本。
- `task-records/session/world-001/`：原始 RGB PNG、深度 NPY 和最新观测。

记录是已完成任务的归档，不是可直接继续仿真的检查点。`session.json` 中
`running:true` 描述保存时原服务的状态，不表示新电脑已有运行进程。原始事件的
绝对路径指向旧机器；查看时使用相对路径副本或 TASK_LOG 中的图片链接。

这三次任务依赖同一世界的连续状态。新服务从初始场景开始，旧点 ID/帧 ID
不能直接使用；现有 loop 入口尚未实现此归档格式的一键回放。跨机器物理结果
仍应重新验证，本次成功不能推导为随机布局的通用成功保证。

## 低层动作参数

每次只将一个低层动作放到 `step.command` 中，外层需要当前 `task_id`、
`frame_id`、`expected_step`。所有位置/偏移以米为单位。

|action|参数|
|---|---|
|`move`|`position:[x,y,z]` 或 `point_id` + `offset:[dx,dy,dz]`；`seconds` 0.2–8；`orientation:"down"`（默认）或 `"position"`；`linear:true` 预检查直线 TCP 路径|
|`look_at`|`position` 或 `point_id` + `offset`；让腕部相机指向该点；`seconds` 0.2–8|
|`gripper`|`opening:0` 闭合、`opening:1` 张开；`seconds` 0.2–5|
|`wait`|保持控制目标并推进物理，`seconds` 0.02–5|

`localize` 是外层独立 action，使用最新 `frame_id`、`pixel:[列,行]`、
`camera:"overhead"` 或 `"wrist"`。返回的是可见表面点；点不会跟踪移动物体。
动作后必须检查新图像和本体状态。IK 可达不等于全路径无碰撞。
