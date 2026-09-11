# 持续自然语言控制：设计与实施计划

目标：由当前 VS Code Codex-astra 会话理解自然语言，在持久 MuJoCo 世界中执行
观察、定位、动作、再观察的循环，完成后接收下一条任务。Python 不启动第二个 LLM。

采用现有 RGB-D 和 IK 实现，加独立 `loop.py` 入口和 `nexus_vision/workspace.py`、
`loop_session.py`。相比另建 API 聊天界面，此方案复用当前对话和权限；相比仅回放，
每次动作都由当前模型根据实际反馈决定。保留旧入口及其七任务评分语义。

1. 先写真实 MuJoCo 集成测试，覆盖同场景连续任务、暂停、陈旧帧、重复步骤、
   错误输入、步数预算和日志；执行测试确认缺少新模块。
2. 将可选配置覆盖加入 VisualSimulation；workspace 预设加入红/绿方块、黄色
   圆柱、紫色大方块与蓝色目标区。使用已有物理资产，不下载额外模型。
3. 实现独立 LoopSession：begin/status/observe/localize/propose/step/pause/resume/
   complete/cancel/reset/shutdown。step 绑定 task_id、frame_id、expected_step；
   每次尝试消耗预算，运动后观察，三次连续失败暂停；complete 记录最新帧与
   Codex 的视觉结论，不冒充自动物理评分。任务之间保留场景。
4. 复用 Unix socket 传输，增加独立 CLI、可移植 launcher、依赖清单、VS Code
   启动任务和仓库 AGENTS.md 操作约定。启动器允许 SO101_PYTHON 指定解释器。
5. 执行新测试及旧测试，在真实服务上验证两轮任务与图像反馈，检查场景截图。
   保存中文使用说明，明确不具备通用避障、后台独立模型推理或任意任务保证。

所有修改限于新增文件和已有 LLM-control 接口的向后兼容扩展；当前大量未提交
资产和实验代码保留。直接在现有工作区构建，因为 LLM-control 尚未跟踪，另建
worktree 无法获得用户当前代码。按用户“先完成代码搭建”的授权直接实施。

## 实施结果（2026-09-10）

以上五项已实现。图形联调补充修复了双窗口共享 GLFW 事件循环的问题：主窗口
与相机窗口使用不同进程，相机数据通过有界非阻塞队列传递。重置复用既有模型
和窗口；预览的新帧身份不依赖仿真时间，防止同时间重置留下旧画面。

验证：项目内 `.venv-loop`，MuJoCo 3.3.2。GLFW 全套 **67 passed**；EGL
新闭环测试 **9 passed, 1 skipped**（跳过需要桌面的双窗口测试）。双窗口测试
覆盖启动、两轮任务、真实末端运动、重置、陈旧帧拒绝及正常关闭。
手动 socket 实测 TCP 上升 29.85 mm（请求 30 mm），图像已查看；原始记录在
`runs/loop-build-check`。独立代码审查发现的启动解释器覆盖和预览刷新问题均已修复。
新多物体布局的抓取成功率与通用避障未验证，不在此次代码搭建完成声明之内。
