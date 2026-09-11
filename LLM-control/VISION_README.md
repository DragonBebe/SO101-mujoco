# Codex 通过 RGB-D 控制 SO101

已将你提供的 `/home/dragon/Downloads/so101-nexus-main` 的环境、机械臂和资产迁入本项目。视觉入口是 `vision.py`，使用独立 Python 3.12 环境；原来的 `control.py` 是读取仿真状态的旧演示。

**本方案的 LLM 就是当前 Codex 会话。** Codex 查看俯视/腕部 RGB 图像，选择目标像素，读取该像素的深度投影结果，再下发末端移动、夹爪和观察命令。Python 控制器负责 IK、关节插值与 MuJoCo 物理步进。无须另配模型 API Key；本机启动控制服务不代表模型在本机离线推理。

## 启动与使用

在仓库根目录运行：

```bash
cd /home/dragon/SO101-mujoco
./LLM-control/run_vision.sh serve --task PickAndPlace-v2 --viewer --realtime
```

服务启动后，可以向当前 Codex 提出任务，例如：

> 使用 LLM-control/vision.py 的视觉服务，观察 RGB 图像，用 localize 读取所选像素的深度，把红色方块抓起并放到蓝色圆形目标区。逐步查看动作后的图像，完成后调用 finish。

服务在等待指令时暂停物理时间，窗口仍可查看。每条运动命令完成后自动返回新的双相机图像、深度文件、标定和机器人自身状态。`--realtime` 配合 `--viewer` 让动作按近似真实时间播放；省略二者可以快速运行。本机已安装所需环境；新环境首次启动需要 `uv`，脚本会安装 Python 3.12 和锁定的依赖。需要可用的 MuJoCo OpenGL 渲染后端。

另一个终端可以操作正在运行的服务：

```bash
./LLM-control/run_vision.sh command '{"action":"observe"}'
./LLM-control/run_vision.sh command '{"action":"start","task":"StackCube","seed":4}'
./LLM-control/run_vision.sh command '{"action":"shutdown"}'
```

默认使用本机 Unix socket `LLM-control/.runtime/vision.sock`，权限为当前用户可读写。同一个 socket 只运行一个服务；已有服务时直接发送命令。需要独立实例时，在 `serve` 和 `command` 两端指定相同的 `--socket /tmp/so101-second.sock`。

## 同时查看两路 RGB 和深度

在 `serve` 或 `replay` 命令后加 `--camera-viewer`，会额外打开一个四格相机窗口：上排为俯视 RGB/深度，下排为腕部 RGB/深度。动作执行时约每 0.1 秒刷新一次（实际速度受渲染负载影响）；等待命令时仿真暂停，窗口保留最近画面并响应缩放、关闭。

本机已验证环境的启动命令：

```bash
cd /home/dragon4090/Documents/SO101-mujoco/LLM-control
MUJOCO_GL=glfw /home/dragon4090/Documents/so101-functional-check-2026-09-09/vision-env/bin/python vision.py serve --task PickAndPlace-v2 --viewer --realtime --camera-viewer
```

如旧服务仍在运行，先在其终端按 Ctrl+C，再用新命令启动。已启动的旧进程不会自动加载新窗口功能。`serve` 依然等待交互命令，不会自动抓取。

直接观看带相机窗口的抓取回放：

```bash
MUJOCO_GL=glfw /home/dragon4090/Documents/so101-functional-check-2026-09-09/vision-env/bin/python vision.py replay --task PickAndPlace-v2 --viewer --realtime --camera-viewer
```

深度显示为相机光轴方向的距离，单位米，固定 0–1 m 色标：近处蓝、远处红，超过 1 m 饱和为红色；黑色表示无效深度/未命中表面。色彩用于显示，原始米制深度数组保持不变。RGB 与深度均来自同一仿真状态。

关闭相机窗口不会停止控制服务；关闭 MuJoCo 主窗口仍会结束当前服务。可以省略 `--viewer`，只显示相机窗口。相机窗口需要本地图形桌面以及 `MUJOCO_GL=glfw`；无头运行请省略 `--camera-viewer`。

预览不会改变用于 localize 的 frame_id、不会持续保存图片，也不会把额外仿真状态加入策略观测。原来的动作后图片及日志仍正常保存。

## 独立回放七个案例

```bash
./LLM-control/run_vision.sh replay --task all --viewer --realtime
```

只看抓取放置：

```bash
./LLM-control/run_vision.sh replay --task PickAndPlace-v2 --viewer --realtime
```

回放采用本次 Codex 看图后保存的像素选择与动作计划，在新仿真中重新渲染、读取深度、执行物理动作；**回放不产生新的 LLM 决策**。它不需要启动 socket 服务，结束后关闭窗口。所有案例通过才返回退出码 0。关闭 Codex 后可以回放；新场景的视觉判断需要再次由 Codex 会话执行。

案例文件在 [examples/vision](examples/vision)，每个文件注明任务、种子、动作与来源。像素位置绑定本次相机和场景配置，不能直接用于任意随机布局。

## 视觉数据与控制边界

| 输入/接口 | 内容 |
| --- | --- |
| RGB | 俯视和腕部相机的 640 × 480 PNG |
| 深度 | 与 RGB 对齐的 NPY，单位米，沿相机光轴的深度；无有效表面处为 NaN |
| 标定 | 内参 `fx, fy, cx, cy`，相机在世界坐标中的位置与旋转 |
| 本体状态 | 6 个关节的位置、速度和通过机器人 FK 得到的 TCP 位姿 |
| 任务文本 | 原项目任务描述，例如向上移动 0.10 m |
| 终局判定 | `finish` 封存本轮后调用原项目判定，保存成功与标量指标 |

目标物体的位置来自 **RGB 选点 + 深度反投影**。控制接口没有物体/目标的仿真坐标、分割 ID、接触状态、奖励或在线成功信号。相机深度和标定由仿真传感器产生；机器人模型用于 IK。`finish` 之后不能继续移动本轮机械臂，防止把评分接口当成在线定位反馈。

`localize` 返回的是可见表面坐标；本次方块已知边长 25 mm，抓取计划据此偏移到抓取中心，并使用校准的 TCP 与夹爪间隙偏移。移动通过关节执行器与原始接触物理实现；动作执行期间没有直接改写物体位姿或绑定物体。

## 命令协议

以下命令通过 `run_vision.sh command '<JSON>'` 发送。坐标为世界坐标，单位米；像素顺序为 `[列, 行]`。每次 `observe` 或运动后都生成新的 `frame_id`。

| action | 参数与结果 |
| --- | --- |
| `observe` | 返回任务、图像/深度文件路径、标定及本体状态 |
| `localize` | `pixel:[u,v]`、最新 `frame_id`；可选 `camera:"overhead"` / `"wrist"`。返回 `point_id`、`surface_world` |
| `propose` | `color:"red"` 等、最新 `frame_id`、可选 `camera`。纯 RGB 颜色连通域候选，供选点辅助 |
| `move` | `point_id` 加 `offset:[dx,dy,dz]`，或 `position:[x,y,z]`；`seconds` 默认 2。默认夹爪朝下；`orientation:"position"` 仅约束位置；`linear:true` 使用预检查的直线 TCP 路径 |
| `look_at` | `point_id` 或 `position`，让腕部相机朝向该点 |
| `gripper` | `opening:0` 闭合、`opening:1` 张开；`seconds` 默认 1.5 |
| `wait` | `seconds`，保持动作并推进仿真 |
| `finish` | 封存本轮并输出原项目终局指标 |
| `start` | `task`、`seed`，创建新一轮，旧帧与旧点 ID 失效 |
| `shutdown` | 关闭服务 |

调用流程示例（替换返回的 ID）：

```json
{"action":"localize","camera":"overhead","pixel":[431,156],"frame_id":"本轮最新帧 ID"}
{"action":"move","point_id":"上一步返回的点 ID","offset":[0.0062,-0.0043,0.055],"seconds":2}
```

应先查看图像再选点。保存的点不会自动跟踪移动后的物体；遮挡、滑移或目标移动后，需要重新观察并定位。控制器检查数值、关节限位和 IK 可达性，但没有通用避障规划；当前验证覆盖下面的桌面案例。

## 已验证范围与记录

本次当前 Codex 会话完成了七个注册任务案例，之后又用保存的视觉计划进行独立回放。成功判定使用原项目的实现。当前 Codex 实测结果见 [recorded_results.json](examples/vision/recorded_results.json)，最终独立回放见 [verified_replay_results.json](examples/vision/verified_replay_results.json)，完整帧与日志在 `runs/nexus-release-verification`。

| 案例 | 本次 Codex 实测结果 |
| --- | --- |
| Touch | 成功；TCP 到物体距离 14.61 mm，符合原项目接近阈值 |
| LookAt | 成功；朝向误差约 4.79° |
| Move | 成功；向上移动 98.51 mm，目标 100 mm |
| PickLift | 成功；保持抓取并抬升 68.35 mm |
| PickAndPlace | 成功；目标误差 1.12 mm，释放后稳定 |
| PickAndPlace-v2 | 成功；目标误差 1.12 mm，桌面支撑、机械臂脱离，持续稳定 4.15 秒 |
| StackCube | 成功；堆叠目标误差 0.74 mm，释放后稳定 |

这是 **seed=4、固定标定、可达采样范围内的案例验证**，没有测量任意随机场景成功率。为确保物体可见且可达，演示配置将径向采样限制在 0.17–0.23 m、±35°；LookAt 使用中心 `(0.20, 0)`、半宽 0.03 m 的方形采样。初始关节为 `(70, -85, 85, 30, 0, 70)` 度，关闭初始关节和腕部相机噪声，腕部 FOV 固定为 60°。原来的任务物理模型和判定条件保留。尚未验证随机外观、干扰物、其他物体资产和任意抓取姿态。

每次运行的 `runs/…/` 包含 `actions.jsonl`、每轮 RGB/深度、最新 `observation.json` 和终局 `evaluation.json`。本次交互原始记录在 `runs/nexus-visual-development` 与 `runs/nexus-visual-cases`；前者包含一次因初始遮挡中止的 LookAt 调试尝试，成功的 LookAt 在后者。运行记录和虚拟环境均被 Git 忽略，可复现案例 JSON 与结果摘要保存在源码目录。

测试命令：

```bash
cd LLM-control
.venv-vision/bin/python -m pytest tests -q --confcutdir=. --rootdir=. --import-mode=importlib
```

本次 **52 项测试通过**。`--confcutdir` 和 `--import-mode` 避免仓库根目录已有的 `__init__.py` 导入旧实验代码。测试覆盖 RGB-D 投影、无效深度、陈旧帧/跨轮点拒绝、策略输入边界、终局封存、真实 Move 回放、非法回放预检查、安装中断恢复与原有控制功能。

## 项目结构与来源

- `vision.py` / `run_vision.sh`：服务、命令、回放入口。
- `nexus_vision/perception.py`：不依赖 MuJoCo 的 RGB-D 感知。
- `nexus_vision/simulation.py`：图像采集、本体观测、IK 和动作执行。
- `nexus_vision/bridge.py`：本地会话、日志、命令服务。
- `nexus_vision/evaluate.py`：终局评估；`replay.py`：视觉计划回放。
- `third_party/so101-nexus`：提供的 Nexus 0.5.4 源码及资产，保留 [Apache-2.0 许可证](third_party/so101-nexus/LICENSE.md) 和资产目录中的原许可证。
- `requirements-vision.lock`：本机验证使用的依赖，MuJoCo 3.3.2。

迁移后的环境不依赖 Downloads 中的原目录。新视觉实现均在 `LLM-control` 内，原参考项目保留。
