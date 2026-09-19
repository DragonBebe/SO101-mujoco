# 清理记录（2026-09-17）

清理前 Git 工作区无未提交改动。以当前持续自然语言控制为主入口，保留七任务视觉验证能力。

## 判断依据

- 当前链路：`run_loop.sh → loop.py → nexus_vision.bridge / loop_session → workspace → simulation → so101_nexus`。
- 根目录旧 Python 演示及 `lerobot`、`lerobot-IK`、`manipulator_grasp`、`SO101-envs`、`so100_6dof` 只服务旧演示；保留的运行代码不引用它们。
- 第一代 `control.py` 和同层 `simulation.py` 使用旧桌面模型与直接物体状态反馈，其专属测试、依赖和回放一起删除；保留 `nexus_vision/simulation.py`。
- `exports` 是项目迁移副本及压缩包；旧计划文件已被运行说明取代。
- `.venv-loop` 是主环境，保留。`.venv-vision` 是可重建的独立视觉环境，移除其安装目录，保留启动脚本和锁文件。`run_vision.sh` 会通过 uv 重建，也可按 README 使用主环境直接运行视觉入口。
- 历史运行产物多数删除；保留 `logs/2026-09-11-continuous-tasks.md` 及其引用的 `runs/loop-20260911-143004-815664`，避免丢失连续任务实验的图像证据。
- 完整保留上游 Nexus 源码、资产、元数据和许可证。其初始化导入、场景构建及安装配置有跨模块依赖，不按当前单场景的表面使用情况裁剪第三方包。
- `examples/vision` 保留可复现计划和历史结果摘要；摘要内原始运行路径属于历史出处，不代表清理后文件仍存在。

## 删除清单

- `2_so100_feetech_mujoco_control_realTosim.py`
- `main_so101.py`
- `5-IK.py`
- `arm-origin-IK.py`
- `Simple-IK.py`
- `3-IK-mujoco.py`
- `p_servo.py`
- `ET.py`
- `so101GraspEnv.py`
- `__init__.py`
- `4-Simple-IK-orig.py`
- `mujoco-so101-control_IK.py`
- `IK.py`
- `1_so100_feetech_mujoco_control_sim.py`
- `mujoco-so101-import-envs.py`
- `main-test-IK.py`
- `params.py`
- `IK_SO101.py`
- `kinematics.py`
- `lerobot`
- `lerobot-IK`
- `manipulator_grasp`
- `SO101-envs`
- `so100_6dof`
- `exports`
- `LLM-control/control.py`
- `LLM-control/simulation.py`
- `LLM-control/tests/test_control.py`
- `LLM-control/examples/pick-place.jsonl`
- `LLM-control/requirements.txt`
- `LLM-control/PLAN.md`
- `LLM-control/VISION_PLAN.md`
- `LLM-control/LOOP_PLAN.md`
- `LLM-control/.venv-vision`
- `LLM-control/runs/loop-20260910-175535-123993`
- `LLM-control/runs/nexus-final-replay`
- `LLM-control/runs/nexus-visual-development`
- `LLM-control/runs/nexus-release-verification`
- `LLM-control/runs/verified-replay`
- `LLM-control/runs/codex-pick-place`
- `LLM-control/runs/loop-build-check`
- `LLM-control/runs/nexus-verified-replay`
- `LLM-control/runs/loop-20260910-174435-883947`
- `LLM-control/runs/vision-replay-20260909-003632-532713`
- `LLM-control/runs/nexus-visual-cases`
- `LLM-control/runs/loop-20260910-174306-005247`
- `LLM-control/runs/vision-20260909-155805-056594`
- `__pycache__`
- `.pytest_cache`
- `LLM-control/__pycache__`
- `LLM-control/.pytest_cache`
- `LLM-control/nexus_vision/__pycache__`
- `LLM-control/tests/__pycache__`
- `LLM-control/third_party/so101-nexus/src/so101_nexus/__pycache__`
- `LLM-control/third_party/so101-nexus/src/so101_nexus/mujoco/__pycache__`

## 验证结果

- 清理前：64 passed，3 skipped（EGL）。
- 清理后：53 passed，3 skipped（EGL）；减少的 11 项是已删除第一代演示的专属测试，现有闭环及视觉测试全部保留。
- 七任务独立物理回放：7/7 success，结果位于 `/tmp/so101-cleanup-verification-20260917/summary.json`。这是固定案例回归，不代表任意自然语言任务的成功率。
- 三个 shell 脚本通过 `bash -n`；主入口 `--help` 正常；`git diff --check` 通过；自有说明文档及连续任务报告无失效本地 Markdown 链接。
- 3 项跳过需要 GLFW 图形桌面，此次没有重新验证 GUI。
- 清理前工作目录（不含 `.git`）约 1.98 GiB，清理后约 0.52 GiB，释放约 1.46 GiB；保留主虚拟环境和连续任务实验数据。
