# 腕部映射零点修正（2026-09-21）

## 根因与证据边界

`rgbcal.robot.joint_models_from_lerobot()` 把电机 5 的全行程 `[0,4095]`
中点 2047.5 当作初始零点，明确标记为未验证。`c920-handeye-03/extrinsics.json`
虽然有 `solved: true`，但 `offsets_fitted` 只有肩抬、肘、腕俯仰三轴；
`wrist_roll` 的零点与未知标定板安装变换无法独立辨识，因此固定为 0。
旧 `ArmMapping.from_scene()` 未区分这种人为固定与真正求解的零点，直接把同一角度用于
Menagerie 夹爪实体的姿态。相机标定求解成功不能证明这个实体零点正确。

照片 0001/0065（初始姿态）、0030（接近目标姿态）均可见实际相机支架在夹爪前侧。
同组实测 ticks、同一模型相机渲染，原映射的支架在侧面；仿真 wrist_roll 增加/减少
90° 的对照支持负方向修正，−90° 时固定钳口及支架的整体朝向更接近照片。
这是**视觉支持的约四分之一圈修正**，不是带测量不确定度的精密零点标定。
历史相机机位漂移、两者夹爪开合不同仍影响图像比较；不能声称残差已降到某个角度。

已排除的混淆：模型 `gripperframe` site 的本地 quaternion 曾修改过 90°，
但它只定义 TCP 坐标轴，不旋转夹爪实体。此次实际修正在 wrist_roll qpos 上，
没有修改 site、模型资产、手眼外参、真实寄存器或抓取控制器。

## 当前配置与效果

`calib/realsim-model/scene.json.model_joint_offsets.wrist_roll` 保存：

```json
{"radians": -1.5707963267948966, "status": "visual_estimate", "source": "有来源的现场照片对比，详见实际 scene.json"}
```

转换为 `q_model = (ticks - 2047.5) * 2π/4095 - π/2`；
当前实际 2151 ticks：标定约定角 +9.098901° → 模型角 −80.901099°。
这是仿真显示的修正，**没有向真机发送 −90° 动作**。
原始场景完整备份在 `scene-before.json`；删除当前场景的 `model_joint_offsets`
字段可恢复旧映射。复制到其他机器人/更换模型前需重新核实，不能视为 SO101 通用常数。
重新采集生成新 scene.json 时不会自动复制这个本机修正，需要显式保留该字段。

同步和离线关节回放都从场景读取同一修正。录制 metadata 保存各轴原标定偏移、模型偏移、
来源与验证状态；每帧保存原始 ticks、`calibration_angles_rad` 和 `converted`。
HUD/状态 JSON 区分反馈 `live` 与零点 `visual_estimate`，同时提示基座轴零点
`unverified_pinned`。没有配置的场景仍使用原映射，但不再把固定零点标成已验证。

## 验证

- 新增 7 个合成回归案例，覆盖偏移方向/单位、只应用一次、未知关节、无来源/无效偏移、
  验证状态，以及实体旋转矩阵。实体矩阵确认实际沿局部 z 旋转 −90°；基座、物体及
  相机支架的转轴原点不移动，仿真时间仍为 0。该测试不证明真实零点数值。
- 仿真环境：`realsim/tests tests` **182 passed, 6 skipped**。
- 真实侧环境：`realsim/tests rgbcal/tests` **116 passed, 5 skipped**。
- 起初把 rgbcal 测试也放到 `.venv-loop`，三个测试模块因没有 OpenCV 收集失败：
  `test_handeye_synthetic.py`、`test_realcam_integration.py`、`test_simcam.py`。
  随后按项目两套依赖环境分别运行，以上是最终结果，未为此修改依赖。
- 保留真实控制器（串口唯一所有者 PID 66354），只重启 MuJoCo 镜像。
  新窗口实测读取反馈并应用姿态，启动约 29 秒的快照约 **29.4 Hz** 收样、
  **58.7 Hz** 刷新。848 条有效转换样本的数据年龄中位 **17.4 ms**、P95 **24.5 ms**；
  Python 读取/转换/应用段耗时中位 **0.265 ms**、P95 **0.340 ms**。
  这些都不是“真实运动到显示器出光”的端到端延迟；该指标未测量。
  详情 `live-metrics.json`、持续写入的 `live-converted.jsonl`。
- 本次修正未移动真机；它仍在已回到的初始姿态。使用现有照片及 ticks 做了多姿态离线
  对比（0001、0030、0065），没有新增独立腕滚转的真机运动验证。

## 对比与复现

- [初始姿态对比](0065-comparison.jpg)：左真实照片，中旧映射，右修正映射。
- [接近目标姿态对比](0030-comparison.jpg)。
- `comparison.json`：每组原始 ticks、修正前后各关节角度。
- `render_evidence.py`：纯离线渲染脚本，运行说明在文件顶部。仿真夹爪保持未标定的默认角度，
  图像标题明确说明这一点；没有把估计开合度混入验证。

仓库根目录启动（当前已有该窗口，无需再开第二个）：

```bash
DISPLAY=:0 MUJOCO_GL=glfw bash LLM-control/run_realsim.sh sync \
  --scene LLM-control/calib/realsim-model/scene.json \
  --xml LLM-control/calib/realsim-model/scene.xml \
  --state /tmp/so101-arm.json --display-rate 60 --viewer
```

相同场景用于 `arm-replay` 就会应用同一修正。静态 `mujoco.viewer --mjcf=...`
只加载 XML，不消费反馈/场景 JSON，因此不会加载这个运行时修正。

## 尚未完成的物理验证

精确腕零点与误差范围仍未测量；需在可测几何基准下核对夹爪/支架朝向，再以数个独立
腕滚转姿态验证。当前约 −90° 修正只解决已观察到的明显方向差。
夹爪电机 6 到活动钳口角的标定仍缺失，显示仍标注 uncalibrated；现有相机漂移导致的
桌面/物体绝对空间误差也仍存在。校正 wrist_roll 会改变偏离滚转轴的 TCP 点位置，
后续若使用该 TCP 做控制或迁移标定板变换，必须显式处理新旧坐标约定；旧抓取 FK 未改。
模型关节范围仍照常检查，不扩大、不静默绕圈；修正后超范围数据会保留上一姿态并报无效。

## 用户现场复核

用户随后确认：“现在观察到正常的仿真和 real robot 的映射了”，并要求继续抓取任务。
这补充了现场视觉验收；配置仍保留 visual_estimate，不把主观对齐改写成精密角度测量。
45 条真实反馈的离线 CLI 回放也已正常结束，保留姿态并按时间进入 stale/disconnected。
