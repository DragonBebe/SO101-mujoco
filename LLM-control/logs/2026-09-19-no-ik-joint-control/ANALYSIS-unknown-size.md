# 分析：RGB 下的物体尺寸从哪来，能否抓取未知尺寸的物体

本次会话在两个任务之后的两轮问答，整理成文。模型：Claude Sonnet 5（`claude-sonnet-5`）；用时和 token 见 [README.md](./README.md) §1 和 [metrics.json](./metrics.json)（T2、T3）。
§3 中关于"关节不动"的部分按事后复现结果更新（见 README §6.2）。

## 1. 问：方块的尺寸，VLM 是观察到的还是从仿真文件里读的？

**都不是。** 尺寸来自服务在每次 `begin` / `observe` / `status` 时返回的场景说明：

```json
"scene": {
  "objects": ["red cube (25 mm)", "green cube (25 mm)",
              "yellow cylinder (diameter/height 25 mm)", "purple cube (35 mm)"],
  "regions": ["blue goal disc (radius 35 mm)"]
}
```

它是 [workspace.py](../../nexus_vision/workspace.py) 里写死的 `SCENE_DESCRIPTION`，作为**给定先验**告诉调用方；执行任务时模型没有打开 XML 或 Python 源码去读尺寸，也没有从像素测尺寸。

| 信息 | 来源 | 性质 |
| --- | --- | --- |
| 尺寸（25 / 35 mm） | `scene.objects` 文本 | 给定先验，照抄进 `propose(object_height=0.025)` |
| 位置（x, y） | `propose` 双平面雕刻、`localize` 平面求交 | 从图像估计，依赖上面的高度假设 |
| 尺寸假设是否合理 | `propose` 返回的 `extent` | 视觉自检：25 mm 方块的外包围应在约 28–36 mm，超出就不采信 |

这样设计的原因：纯 RGB 没有深度，单张图不能把像素大小换算成物理尺寸，缺少尺度基准。项目的约定是**尺寸作为先验给出，位置必须观察**
（`LOOP_README.md`："场景说明仍只提供物体种类和尺寸"）；`AGENTS.md` 禁止用未验证的假设代替观察，指的是位置，对未知高度的物体还明确要求"不能猜"。

## 2. 问：RGB 下能否抓取尺寸未知的物体？

**结论：用现有代码不能；有三条不需要深度传感器的办法可以做到，其中接触测高的底层信号已经存在。**

### 2.1 根本困难：单目射线上的尺度歧义

`propose` 与 `localize method:"plane"` 的三维都来自"像素射线 ∩ 指定平面"：

- 平面高度正确时，水平位置解得准，与物体多高无关。任务 A 定位方块、圆盘用的就是这一点。
- 物体顶面像素的位置同时取决于物体多高和物体在哪，同一个像素可以是矮物体离相机近，也可以是高物体离相机远，单张图分不开。
  所以 `propose` 必须由调用方给 `object_height`，没有默认值（[perception.py](../../nexus_vision/perception.py) `locate_color`）。
  高度未知时，RGB 模态**直接拒绝**给出三维估计，而不是给一个不准的结果。
- 对斜视相机（`side` / `calibrated`，俯角约 32–36°），高度假设错一点，水平中心就偏得多；接近正俯视的 `overhead` 相机对高度误差最不敏感。

### 2.2 三条可行路线

**(a) 只用物体与桌面接触的底边像素。** 色块质心混合了顶面和侧面，所以需要高度去雕刻；但轮廓最靠桌面的那圈像素本来就在 `z=0` 上，
直接做 `plane_z=0` 求交就是准的，与高度无关。限制：斜视时物体会挡住背对相机一侧的底边，只能看到一部分。
`locate_color` / `plane_support_estimate` 目前取整个色块，没有"只取底边"模式。

**(b) 腕部相机多视角三角化。** 腕部相机随手臂移动，每个位姿下的相机外参由正运动学精确给出。在两个位姿各拍一张，
用对极几何三角化即可恢复物体三维形状和高度，相当于用手臂自身运动当立体基线。原理最扎实，但仓库没有任何多视图代码，工作量最大。

**(c) 接触测高。** 用 `joints` 缓慢下降，碰到物体顶面时停下，此时 TCP 高度减去桌面高度就是物体高度，再回填给 `propose(object_height=…)` 求水平中心。
任务 B 其实已经多次"意外"做到了这一点：事后复现证实，那些指令下发但关节不动的情况就是夹爪压到了桌面或方块（README §6.2），只是当时只能靠"位置纹丝不动"间接判断。
干净的信号在底层已经存在：

```
third_party/so101-nexus/src/so101_nexus/observations.py
  GripperContactForce  — 两指所受接触合力（世界系，牛顿）
  ActuatorForce        — 各关节执行器广义力
third_party/so101-nexus/src/so101_nexus/mujoco/base_env.py
  抓取判定：两侧手指法向力达到 grasp_force_threshold 且方向相对
```

但 `nexus_vision.simulation.VisualSimulation.observe()` 返回的 `robot` 块只有 `joints` / `joint_velocities` / `tcp`（本次新增了 `joints_deg` 等），没有透出力和接触。
把 `GripperContactForce` 或"目标已下发但实际角不跟随"这一事件加进 `observe()`，就能把接触测高写成显式流程。真机上没有力传感器，可以用舵机负载/电流或同样的"跟随误差"判断。

### 2.3 对比

| 方式 | 需要新代码 | 仍是纯 RGB | 适用性 |
| --- | --- | --- | --- |
| 现状（`object_height` 必须给出） | 无 | 是 | 高度已知时准，未知时拒绝 |
| (a) 只取底边像素 | `perception.py` 加模式 | 是 | 依赖底边可见比例；俯视好，斜视差 |
| (b) 多视角三角化 | 新模块，量大 | 是（用了运动） | 最完整 |
| (c) 接触测高 | 小：`observe()` 透出接触力 | 是 | 只得到高度，配合现有水平定位已够抓取 |

### 2.4 建议

- 不改代码：RGB 模式只抓尺寸已知的物体（场景给出，或 `extent` 自检通过）。本次任务 A/B 就守在这个边界内。
- 要支持真正未知尺寸：优先做 (c)，其次 (a)，(b) 是完整方案但成本最高。
- 或者切换到项目本来就支持的 `rgbd` 模态，`LOOP_README.md` 对这类情况的建议就是"请改用 rgbd"。
