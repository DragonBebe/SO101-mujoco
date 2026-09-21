# SO101 MuJoCo 自然语言控制

项目目标是在 MuJoCo 中，由当前 Codex 对话根据相机反馈持续操控 SO101 机械臂，完成桌面抓取、移动和放置。当前对话负责规划，本地 Python 负责感知接口、逆运动学、物理仿真和任务记录，不另外调用模型 API。

相机有两个独立选项：模态 `rgb` / `rgbd`，环境机位 `side` / `overhead`（腕部相机始终装在机械臂上）。`rgb` 模态不渲染深度，三维来自显式的平面假设：像素射线与给定水平面求交，或给 `propose` 传入已知物体高度来估计它在桌面上的站位与抓取中心。该假设对放在桌面上的已知尺寸物体成立，对未知高度的堆叠物体不成立。

核心代码位于 [LLM-control](LLM-control/README.md)，操作协议见 [LOOP_README](LLM-control/LOOP_README.md)。

```bash
bash LLM-control/setup_loop.sh
# 推荐：双 RGB + 侧视
bash LLM-control/run_loop.sh serve --camera-modality rgb --environment-camera side \
     --viewer --realtime --camera-viewer
# RGB-D + 侧视
bash LLM-control/run_loop.sh serve --camera-modality rgbd --environment-camera side \
     --viewer --realtime --camera-viewer
# 原有 RGB-D + 俯视（默认）
bash LLM-control/run_loop.sh serve --viewer --realtime --camera-viewer
```

真机方向：环境相机标定见 [RGBCAL_SUMMARY](LLM-control/RGBCAL_SUMMARY.md)；把标定好的真实桌面
（方块的位置与朝向）映射成 MuJoCo 场景，做镜像或可步进的仿真快照，见
[REALSIM_README](LLM-control/REALSIM_README.md)。映射只读相机和舵机，不下发任何运动指令。
新增 `arm-stream` / `sync` / `arm-replay` 提供关节实际反馈镜像、录制与回放；已有控制进程可通过 `SO101_FEEDBACK_PATH` 发布状态。夹爪角度需补实测标定，详见该文档第 10 节。

```bash
bash LLM-control/run_realsim.sh check                      # 有哪些标定和方块
bash LLM-control/run_realsim.sh observe --session my-run   # 一帧 -> 一份场景状态
bash LLM-control/run_realsim.sh mirror --scene LLM-control/calib/my-run/scene.json --viewer
```

无图形桌面时使用 `MUJOCO_GL=egl bash LLM-control/run_loop.sh serve`。服务启动后，在当前 Codex 对话中提交任务。VS Code 的任务入口在 `.vscode/tasks.json`。

清理范围、依赖判断和删除清单见 [CLEANUP.md](LLM-control/CLEANUP.md)。
