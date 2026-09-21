# LLM-control

主入口是 [run_loop.sh](run_loop.sh)，完整操作协议见 [LOOP_README.md](LOOP_README.md)。项目让当前 Codex 对话读取相机图像、理解自然语言、规划单步动作并检查结果；本地服务本身不会生成模型决策。

## 实现链路

| 文件 / 模块 | 作用 |
| --- | --- |
| `setup_loop.sh`、`requirements-loop.txt` | 建立 Python 3.12+ 主环境 `.venv-loop`；`requirements-loop.lock` 保存版本快照 |
| `run_loop.sh`、`loop.py` | 设置本地 Nexus 导入路径，提供服务、状态和 JSON 命令入口 |
| `nexus_vision/bridge.py` | 本机 Unix socket、严格 JSON 解析、请求响应及窗口刷新 |
| `nexus_vision/loop_session.py` | 持续任务、步数预算、暂停与恢复、帧校验和重复动作防护、日志 |
| `nexus_vision/workspace.py` | 在 Nexus 物理环境上建立多物体桌面；任务间保留世界 |
| `nexus_vision/cameras.py` | 相机模态（`rgb`/`rgbd`）与环境机位（`side`/`overhead`）的唯一配置源，含侧视机位的取景计算 |
| `nexus_vision/simulation.py` | 双相机取图（深度按模态可选）、机器人自身状态、带边界检查的 IK 和执行器动作 |
| `nexus_vision/perception.py` | 深度反投影、像素射线与平面求交、已知高度物体的双平面雕刻定位、颜色候选 |
| `nexus_vision/camera_viewer.py` | 按模态排版的 RGB/深度预览；双窗口模式使用独立进程 |
| `third_party/so101-nexus` | 上游仿真环境、机器人资产和物理实现，保留许可证及包配置 |
| `vision.py`、`nexus_vision/replay.py`、`evaluate.py` | 七个固定视觉任务的独立回放和物理评分 |
| `run_realsim.sh`、`realsim/` | 真实桌面 → MuJoCo 场景映射：标定相机观测方块的位置和朝向，镜像进仿真或存成可步进快照，见 [REALSIM_README.md](REALSIM_README.md) |
| `tests`、`examples/vision` | 闭环、感知、服务、视觉任务回归测试与可复现案例 |

一次控制循环是：观察图像 → 用当前帧定位（`rgbd` 走深度反投影，`rgb` 走显式平面假设）→ 规划有界低层动作 → 经 IK、执行器和物理仿真执行 → 获取新图像与实际姿态。物体运动来自接触、摩擦和重力。IK 不保证路径无碰撞；`ok:true` 只说明动作执行成功，闭环任务的 `codex_visual` 是模型根据反馈作出的判断。

## 安装与运行

在项目根目录执行：

```bash
bash LLM-control/setup_loop.sh
bash LLM-control/run_loop.sh serve --camera-modality rgb --environment-camera side \
     --viewer --realtime --camera-viewer
bash LLM-control/run_loop.sh status
```

无头环境：`MUJOCO_GL=egl bash LLM-control/run_loop.sh serve`。
已有兼容解释器可通过 `SO101_PYTHON` 指定。依赖不包括额外 LLM SDK、PyTorch 或真机串口驱动；`gymnasium`、`tyro`、`huggingface-hub` 和 `trimesh` 是上游包及其导入链的依赖，不能只根据主入口的 import 删掉。

## 视觉验证与测试

[视觉协议](VISION_README.md)和七任务回放仍可用。直接复用主环境，无需建立第二个环境：

```bash
cd LLM-control
PYTHONPATH=third_party/so101-nexus/src MUJOCO_GL=egl .venv-loop/bin/python vision.py replay --task all
PYTHONPATH=third_party/so101-nexus/src MUJOCO_GL=egl .venv-loop/bin/python -m pytest tests -q --confcutdir=. --rootdir=. --import-mode=importlib
```

真机方向另有两条链路，都不改动上面的仿真闭环：相机标定见 [RGBCAL_README.md](RGBCAL_README.md)
（状态与结论在 [RGBCAL_SUMMARY.md](RGBCAL_SUMMARY.md)），把标定好的真实桌面映射成 MuJoCo 场景见
[REALSIM_README.md](REALSIM_README.md)（验证记录 [logs/2026-09-21-realsim-mapping](logs/2026-09-21-realsim-mapping/README.md)）。
`realsim` 真实侧跑 `.venv-rgbcal`、仿真侧跑 `.venv-loop`，物体通过场景 JSON 交接；新增 `arm-stream` / `sync` / `arm-replay` 通过独立最新关节反馈 JSON 和原始 JSONL 持续镜像、录制和回放。
已有 `ServoBus` 控制进程可选开启只读后台发布；夹爪未知标定会明确标记，启动与验证见 realsim 文档第 10 节。

`run_vision.sh` 是可选的独立环境入口，首次使用需要 uv，并会按 `requirements-vision.lock` 重建 `.venv-vision`。主流程只需 `.venv-loop`。

`logs/screenshots-2026-09-17/` 是相机改造后的实拍截图。`runs/` 保存运行时图像、深度（仅 `rgbd`）与事件，`.runtime/` 保存 socket，均不是源码。保留的连续任务实验报告见 [logs/2026-09-11-continuous-tasks.md](logs/2026-09-11-continuous-tasks.md)；纯 RGB 操作归档见 [logs/2026-09-17-rgb-three-tasks](logs/2026-09-17-rgb-three-tasks/README.md) 和 [logs/2026-09-17-rgb-stack-swap](logs/2026-09-17-rgb-stack-swap/README.md)（后者含工具调用、推理过程、时长与 token 统计）。清理依据及删除清单见 [CLEANUP.md](CLEANUP.md)。
