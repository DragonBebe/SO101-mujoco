# 真机抓取前检查 — 2026-09-21

用户目标：真实 SO101 执行基础抓取，MuJoCo 同步显示实测关节状态。

已完成只读检查，没有发送运动、扭矩或目标位置指令，没有中断现有 arm-stream/sync。

- 现有抓取脚本：calib/c920-real-grasp-01/tools/grasp_red.py，分阶段执行。
- 当前反馈：六轴 Present_Position 有效；读取时六轴 torque_enabled 均为 false。
- 现有 arm-stream 占用串口；执行控制前必须由同一个 ServoBus 控制进程发布反馈，不能另开串口。
- 新采集场景：scene.json，原始图像 frames/frame-0001.png；三块识别结果仅供检查，不是已经通过验收的动作坐标。
- 当前 drift：111 对匹配，58 内点；中位位移 55.681 px，阈值 3 px；判定机位变化。旋转约2.4度仅为估计量级。
- 当前图像与 setup-03/reference.png 人工复核：桌沿、基座和背景投影均有偏移。
- 旧抓取代码经 rgbcal.pointcheck.cube_candidates 使用25mm方块；realsim目录为30mm。不能把两者定位参数直接混用。
- 旧经验偏移来自9月18日特定机位与工作区，不能直接用于这次变化后的机位。

下一步需要现场恢复相机原固定机位并重新检查，或重新采集外参/桌面标定；然后统一方块尺寸、生成新的只读抓取计划，检查接近路径，才继续分阶段实际动作。不能通过只重新保存 reference 或设置 usable_for_motion=true 来代替校准。
