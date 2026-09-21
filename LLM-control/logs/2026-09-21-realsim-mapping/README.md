# 真实桌面 → 仿真映射：实现与验证记录（2026-09-21）

本归档记录 `LLM-control/realsim/` 的一次实现与验证，**不是模型驱动机器人的任务归档**，
所以不套用 [LOG_FORMAT.md](../LOG_FORMAT.md) 的事件/token 章节；但它的要求照做：
自包含、写清证据边界、失败单列、**事后更正单列**。

工具说明见 [REALSIM_README.md](../../REALSIM_README.md)。

## 0. 一页结论

1. **合成验证通过**：渲染已知位姿，9/9 检出，位置误差中位 0.15 mm、最大 0.29 mm，yaw 最大 0.73°。
   这只证明几何链路（投影约定、桌面系、平面假设、yaw 对称、拒绝路径）是对的。
2. **真实设备验证通过（自洽层面）**：在线 30 s、三个方块，**12/12 帧三块全部检出**，
   0.39 Hz；静止抖动 0.04–0.07 mm、yaw RMS 0.05–0.53°。
3. **独立尺度检验不通过**：两块正对贴紧的 30 mm 方块，中心距应为 30.00 mm，实测
   **31.2 mm（+4.0%）**，重复性 σ 0.06 mm。**相机位姿已与标定不一致**（同一个方块在
   09-18 和今天反查差 2.7%）。在重新标定前，这条链路的绝对位置不可用。
4. **方块是 30 mm**（用户 2026-09-21 告知），日志里的标称 25 mm 是错的。按 30 mm 反查得
   29.2–30.0 mm，**与 30 mm 一致**。
5. **青绿色方块必须有自己的色相带**：它 hue 160–172，`perception` 的 green 带 170° 截止。
6. **相机是否被碰动仍无法确认**：`check-setup` 因场景变动太大给不出一致结论。

## 1. 条件与证据边界

| 项 | 值 |
| --- | --- |
| 日期 | 2026-09-21（CEST，UTC+02:00） |
| 相机 | Logitech C920，`--profile c920`，1920×1080 MJPG，focus 0 |
| 标定 | `calib/c920-intrinsics-01`、`calib/c920-handeye-03`、`calib/c920-table`（全部复用，未重做） |
| 方块 | 三个立方体（红 / 青绿 / 蓝），**边长 30 mm，用户 2026-09-21 告知** |
| 在线图像 | 本次现场抓帧两张：三块贴合（`frames/three-cubes-touching.jpg`）与三块分开（`frames/three-cubes-detected.jpg`） |
| 归档图像 | `calib/c920-point-check-02/frames/estimate-00{1..6}.png`（2026-09-18） |
| 机械臂 | **未供电**，`run_realsim.sh arm` 报 "no status packet"；关节同步只有单元测试覆盖 |
| 下发过的运动指令 | **零**。全程只读相机；舵机连读都没读到 |
| 三维来源 | 实测桌面平面 + 方块已知高度的平面假设；**没有深度传感器** |
| 从未读取 | 仿真的物体真值（合成自检除外，那里真值本来就是输入）、分割 ID、任何私有模拟器状态 |

## 2. 合成验证（渲染，不是照片）

`bash LLM-control/run_realsim.sh selftest --grid 9`，完整报告 `data/selftest.json`。

在 MuJoCo 里用 `--environment-camera calibrated`（`rgbcal sim-camera` 导出的实测机位）
渲染已知位姿，再把渲染图送进同一套估计器。

| 指标 | 结果 |
| --- | --- |
| 检出 | **9 / 9** |
| 位置误差 | 中位 **0.15 mm**，p90 0.21 mm，最大 **0.29 mm** |
| yaw 误差（按 90° 对称取模） | 中位 0.34°，最大 **0.73°** |
| 中心高度误差 | 0（按定义：它是假设算出来的，不是测量） |
| 抬起 20 / 40 / 80 mm | 全部拒绝 |
| 抬起 5 / 10 mm | **未拒绝**，作为 `lift_detection_floor_mm` 明写 |
| 双色场景 | 两块都检出，误差 0.17 / 0.26 mm |

证据帧：`frames/synthetic-placement-01.jpg`、`frames/synthetic-two-blocks.jpg`。

**不能外推到真机**：渲染图和估计器共用同一个世界模型，它不检验真实的颜色、光照、模糊，
也不检验标定有没有把桌面放对地方。

### 2.1 被实测推翻的一个阈值

`height_check` 最初判"最佳高度偏离假设平面 > 8 mm 就拒绝"。9 个机位有 1 个（方块面正对
相机）被误拒。逐点测量：

| 情况 | best_elevation | margin（最佳 IoU − 平面处 IoU） |
| --- | --- | --- |
| 平放，yaw 20–80° | 0 ~ +5 mm | 0.000 – 0.001 |
| 平放，yaw ≈ 0°/90°（面正对相机） | +10 mm | **0.020** |
| 抬起 10 mm | +10 mm | 0.021 – 0.022 |
| 抬起 20 mm | +20 mm | 0.048 – 0.054 |

这是**几何退化**——面正对相机时轮廓接近矩形，抬高的方块几乎能复现同一轮廓。改成"必须比
假设平面好出 `min_height_margin = 0.025`"，代价是 10 mm 抬起检不出来，并把这个能力边界
写进 `selftest` 报告，每次重测。

### 2.2 被 selftest 抓到的一个真 bug

给青绿色方块配了实测色带 `[145, 182]` 之后，**合成场景里的绿块检不出来了**——因为 MuJoCo
把它渲染成纯绿（hue 120°），不在这个带里。

这不是测试的问题，是**镜像的问题**：仿真里的方块必须用方块自己的颜色渲染，否则在渲染图上
做自检会"通过"一个真实方块根本不满足的色带。改法：`MirrorSimulation.paint()` 按每个方块
色带的中点颜色覆写 geom rgba。对照图 `frames/three-cubes-real-vs-mirror.jpg` 里仿真方块
是青绿/蓝/红，不是罐头色。

## 3. 真实图像与真实设备验证

### 3.1 颜色带：青绿色方块压在 green/blue 分界上

`run_realsim.sh colours` 的实测输出（`perception` 默认带）：

```
green_cube  band [[75, 170]]   hue 160-168   margins [(76.7, 0.0)]   neighbours 289 px @ hue 172
blue_cube   band [[170, 270]]  hue 200-208   margins [(27.8, 40.8)]  neighbours 0 px
```

`margins` 的高位是 **0.0°**，紧邻还有 289 个饱和像素落在 hue 172——带边正在切这个方块，
被切掉的部分跑到蓝色那边。按实测改成 green `[145, 182]`、blue `[186, 265]` 后：
绿块掩码 6356 → 7396 px（+16%），越界像素 289 → 13。

改之前绿块被 `area_ratio 0.73` 判为无效——**质量检查起了作用，没有给出一个缩水的位姿**。

### 3.2 三块分开（工作区内）

`frames/three-cubes-detected.jpg`，记录 `data/three-cubes-scene.jsonl`。

| 方块 | 桌面系 x / y (mm) | yaw | IoU | 镜像重投影 |
| --- | --- | --- | --- | --- |
| red_cube | 356.5 / 105.4 | 75.7° | 0.964 | 1.05 px |
| blue_cube | 352.8 / 46.8 | 87.4° | 0.921 | 1.05 px |
| green_cube | 357.0 / −16.4 | 1.9° | 0.926 | 0.87 px |

相邻中心距 58.6 / 62.8 mm，与照片上"约一个方块宽的间隙"一致。

### 3.3 在线连续观测（30 s，只读）

`bash LLM-control/run_realsim.sh live --duration 30 --no-height-check`，`data/three-cubes-live.jsonl`。

| 指标 | 结果 |
| --- | --- |
| 检出 | **12 / 12 帧，三块全部** |
| 有效更新率 | **0.39 Hz**（三个方块；单个方块约 0.8–1.0 Hz） |
| 端到端延迟（取图 → 状态写盘） | 中位 **2.33 s**；有上一帧种子后降到 2.1 s |
| 静止抖动（12 帧） | 位置 σ **0.04 / 0.07 / 0.04 mm**，yaw RMS 0.05 / 0.53 / 0.14° |

### 3.4 尺寸交叉校验

`run_realsim.sh measure`（把尺寸当自由参数一起拟合），在线三块分开那一帧：

| 方块 | 目录 | 拟合 | 区间（腐蚀 0 / 1 像素） | IoU 目录 → 拟合 |
| --- | --- | --- | --- | --- |
| red_cube | 30.0 | 29.81 | [29.22, 29.81] | 0.964 → 0.965 |
| green_cube | 30.0 | 30.00 | [29.22, 30.00] | 0.926 → 0.926 |
| blue_cube | 30.0 | 29.24 | [28.69, 29.24] | 0.921 → 0.943 |

放开尺寸后 IoU 几乎不变，说明 **30 mm 已经能解释图像**。

### 3.5 独立尺度检验：不通过

用户把绿、蓝两块正对贴紧放在**工作区中央**（`frames/scale-check-touching.jpg`）。
两块贴紧的立方体中心距必须等于边长——这是关于方块的事实，不来自标定链路。

`bash LLM-control/run_realsim.sh scale-check --pair green_cube blue_cube`，
记录 `data/scale-check.jsonl`、`data/scale-check.json`：

| 项 | 值 |
| --- | --- |
| 应为 | **30.00 mm** |
| 实测（沿接触面法向，14 帧） | **31.17 mm**（另一次 8 帧：31.24） |
| 误差 | **+1.17 mm（+3.9%）** |
| 重复性 | σ = **0.064 mm** |
| 侧向错位 | **−0.36 mm** → 两块确实齐平对正，不是摆放问题 |
| 相对偏航 | 0.18° |

σ 0.06 mm 说明这不是噪声，是**稳定地偏**。自洽指标（IoU 0.96、抖动 0.06 mm）完全看不见它。

### 3.6 定位原因：不是尺度，是相机几何

同一帧里两个量给出**相反符号**：

| 测的是什么 | 结果 | 相对 30 mm |
| --- | --- | --- |
| 两块贴紧的中心距 | 31.17 mm | **+3.9%** |
| 单块尺寸反查（含孤立的红块 29.68） | 29.3 – 29.7 mm | **−1.5%** |

单个尺度因子做不到这件事，方块尺寸错了也做不到。跨会话对比给出原因——**同一个红方块、
同一套标定文件、同一段代码**：

| 会话 | 反查边长 | 中位 |
| --- | --- | --- |
| 2026-09-18 归档 6 帧（x 158–365 mm） | 30.45 / 30.48 / 30.29 / 30.52 / 31.29 / 31.42 | **30.50 mm** |
| 2026-09-21 今天（x 341 mm） | 29.68 | **29.68 mm** |

相差 2.7%。位置可比（09-18 在 x=365 处是 30.52，今天在 x=341 处是 29.68），所以不是
会话内的位置依赖。

### 3.6b 直接确认：相机移动了 55 px

`check-setup` 第二次运行仍报 "no large group of anchors agrees"（57 个锚点 5 个一致）。
但它的原始输出里有一个明显的簇：约 20 个锚点落在 dx ≈ −55、dy ≈ +25 附近，离散 ±15 px。
**离散本身就是证据**——纯平移不会有这种空间变化的位移场，旋转才会。`check-setup` 只按
单一平移投票，所以只能报"判不出来"。

用实测内参对 `calib/setup-03/reference.png` 与当前帧做特征匹配 + 本质矩阵
（新命令 `run_realsim.sh drift`）：

| 项 | 值 |
| --- | --- |
| 匹配点位移中位 | **55 px**（−51, +25） |
| 判据阈值（标定文档） | **3 px** |
| 旋转量级 | 约 **2.2°** |
| 内点 | 54 / 107 |
| 单应内点比 | 0.30 → 不是纯旋转，还有平移 |

超阈值 18 倍。**相机确实动了**，这解释了 3.5 和 3.6 的全部现象。

旋转角只作量级用：合成测试显示 0.1–0.6 px 特征噪声能让 2° 的估计漂 0.6°（基线相对景深
很小时本质矩阵条件差）。判据用位移。另外，完全不动时本质矩阵退化，`recoverPose` 会给出
任意角度（实测 179.99°），所以位移低于阈值时 `drift` **不报旋转**，而不是报一个数。

### 3.7 被排除的两个解释

| 假设 | 检验 | 结果 |
| --- | --- | --- |
| 接触处的暗线把两块掩码各截去一段，把中心推开 | 实测接缝宽 1.2 px、value 0.176（低于 0.20 阈值）。把 `min_value` 从 0.20 降到 0.06 让两块各自认领接缝 | 距离只从 31.17 降到 31.03 mm，**只解释 12%**。假设基本不成立 |
| 掩码光晕 | 把掩码腐蚀 1 px（对称，不应移动中心） | 距离 31.15 mm，**几乎无变化** |

## 4. 事后更正

| 原说法（2026-09-21 早些时候） | 新证据 | 更正后 |
| --- | --- | --- |
| "红方块约 30.5 mm（6 帧反查 30.2–31.4）" | 把掩码腐蚀 1 像素，反查值在**每一帧都整齐下降 0.75 mm**；在线工作区内的帧给 29.2–30.0 | **多读了约半像素的颜色光晕**。用户给的 30 mm 是对的，我先前的 30.5 把光晕算进了方块 |
| "归档帧 6 被拒是因为尺寸偏小被高度补偿"（标为未验证） | 30 mm 下归档帧 5、6 仍被拒（+12 / +24 mm，margin 0.027 / 0.054），且 `area_ratio` 1.06 / 1.08 | **仍未验证**。已在拒绝理由里同时报出 `area_ratio`，因为"方块更大"和"方块被抬高"从这个视角**无法区分**，说哪一个都是猜 |
| 首版按 25 mm 配置 | 每一帧 `area_ratio` ≈ 1.35，全部判无效 | 25 mm 来自 `calib/c920-real-grasp-01/LOG.md` 的标称值，是错的 |

## 5. 失败与未验证项

| 现象 | 数据 | 结论 |
| --- | --- | --- |
| 合成 9 个机位曾有 1 个误拒 | margin 0.020 vs 抬起 10 mm 的 0.021 | **已验证**是几何退化；阈值改为 0.025，能力边界写进报告 |
| 合成绿块检不出 | 带 [145,182] vs 渲染 hue 120 | **已验证**：镜像必须按方块自身颜色渲染。已修 |
| 归档帧 5、6 被拒 | +12 / +24 mm，`area_ratio` 1.06 / 1.08 | **未验证**：尺寸偏差与抬高不可分辨。理由里已并列报出两种解释 |
| 三块贴合帧中红块被拒 | `carved footprint 4 x 12 mm`，只剩顶面可见 | **已验证**：被另外两块遮挡，拒绝正确 |
| 蓝色带同时匹配背景椅子 | 10 万像素区域，`footprint 794 x 576 mm` | **已验证**：尺寸筛选在拟合前短路，正确拒绝 |
| 独立尺度检验 +3.9% | 31.17 ± 0.06 mm vs 应为 30.00；同帧尺寸反查 −1.5%；同一方块跨会话差 2.7% | **已验证**：不是尺度因子、不是方块尺寸、不是接缝、不是光晕。**相机位姿与标定不一致** |
| `check-setup` 两次都无结论 | 57 锚点中 5 个一致；但原始位移里有约 20 个锚点聚在 (−55, +25) px | **已解释**：它只按单一平移投票，相机旋转产生的空间变化位移场拼不出共识 |
| 相机移动 | `drift`：位移中位 55 px vs 阈值 3 px，旋转约 2.2°，单应内点比 0.30 | **已验证**：相机确实动了，且不是纯旋转 |
| 关节同步未在真机跑过 | 舵机无响应，机械臂未供电 | 只有单元测试覆盖（`test_mirror.py`） |
| 真实位置精度 | —— | **没做**。缺独立参照，步骤见 REALSIM_README 第 10 节 |

## 6. 映射到仿真

- `mirror`：三块同时应用，`reprojection_check` 0.87–1.05 px。这只说明映射链路自洽——
  位姿没贴错物体、四元数顺序没错、坐标系没搞混——**不是精度**。
  对照图 `frames/three-cubes-real-vs-mirror.jpg`；仿真机械臂姿态与真实不同，因为该帧
  没有关节读数（机械臂未供电）。
- `snapshot`：`data/snapshot.json`。载入后步进 1 s，方块沉降 0.045 mm 且不漂移。
  质量和摩擦是**默认值**，文件里 `physics_parameters.source` 如实写着 "defaults, not identified"。
- `replay`：被拒绝的帧会**把方块移出世界**，而不是留在上一帧位置冒充当前状态。

## 6b. 在已知漂移下的一次完整建模（用户要求）

用户判断相机未被移动，并要求先做一次建模。特征匹配的证据与此不符——**104 个匹配里
没有一个位移小于 10 px**，包括钉在建筑上的门框（74 px）和远景地面（56 px）：

| 图像区域 | 匹配数 | 位移中位 |
| --- | --- | --- |
| 远景顶部（桌子以外） | 17 | 55.6 px |
| 左三分之一上半（门框／窗） | 13 | 74.1 px |
| 机械臂区域 | 46 | 55.4 px |
| 桌面与物体 | 12 | 211 px（物体本来就换过位置） |

但用户的另一个判断是对的：**55 px 对"重建场景并观察"够用**。据此做了完整一轮，
证据 `frames/model-run-*.jpg`、`data/model-run-*.json`。

观测（桌面系 = 仿真世界，三块全部检出）：

| 方块 | x / y (mm) | yaw | IoU | 镜像重投影 |
| --- | --- | --- | --- | --- |
| red_cube | 340.5 / 103.1 | 83.4° | 0.948 | 1.08 px |
| blue_cube | 336.1 / 37.8 | 85.0° | 0.943 | 1.26 px |
| green_cube | 333.1 / 6.6 | 85.2° | 0.958 | 0.69 px |

快照载入后独立步进 1.5 s，三块各沉降 0.045 mm、无漂移。

### 误差预算（这次建模能用来做什么）

| 量 | 值 | 依据 |
| --- | --- | --- |
| 场景内部相对尺度 | **+4.4%** | 绿蓝贴紧实测 31.33 mm，真值 30.00 |
| 朝向一致性 | 三块 yaw 83.4–85.2°（照片里三块同向） | 离散 1.8° |
| 共线性 | 绿→蓝 与 蓝→红 夹角 1.5° | 照片里三块基本共线 |
| **相对底座的绝对位置** | **约 25 mm 量级** | 把实测的 2.2° 相机旋转施加到标定位姿，同一像素在桌面上的交点移动 25.3–25.9 mm |

**可用**：场景的相对排布、物理试验、把快照接给 LLM 做动作预测。
**不可用**：任何要送给机械臂的绝对抓取坐标。

### 导出成独立 MJCF

用户要求能在 MuJoCo 里直接打开这个场景，于是加了 `export`：

```bash
bash LLM-control/run_realsim.sh export --scene <scene.json> --output scene.xml
LLM-control/.venv-loop/bin/python -m mujoco.viewer --mjcf=<绝对路径>/scene.xml
```

两处需要实测才能做对的地方：

1. **`meshdir`**：上游机器人模型声明 `meshdir="assets"`，MuJoCo 是相对**主文件**解析的，
   所以写在别处的场景文件一个网格都加载不到（实测报错
   `Error opening file '.../waveshare_mounting_plate_so101_v2.stl'`）。把
   `<compiler meshdir="<绝对路径>">` 放在 `<include>` **之后**可以覆盖，实测 nmesh 18 正常。
2. **`principalpixel` 的符号**：两个轴都与图像轴反向。四种符号组合逐一渲染并从 GL 视锥
   回读内参：

   | sx | sy | 回读 cx | 回读 cy | 与目标之差 |
   | --- | --- | --- | --- | --- |
   | +1 | +1 | 496.983 | 283.503 | 34.966 / 28.005 |
   | +1 | −1 | 496.983 | 255.497 | 34.966 / 0 |
   | −1 | +1 | 462.017 | 283.503 | 0 / 28.005 |
   | **−1** | **−1** | **462.017** | **255.497** | **0 / 0** |

   搞错任一个轴，视角会偏两倍主点偏移量，而画面看上去依然正常——所以
   `tests/test_export.py` 用回读内参把这个约定锁住，不靠肉眼。

导出文件的验收：从任意目录打开都能编译（nmesh > 0）；三块在**不载入关键帧**的静止状态就位于
采集位姿；`calibrated` 相机回读的 fx/fy/cx/cy/位置/旋转与标定值完全一致；用标定解析投影得到的
像素落在对应颜色的方块上。机械臂姿态写成关键帧，名字直接说明是否观测到
（本次为 `arm_rest_NOT_observed`，因为机械臂未通电）。

### 顺带修掉的两个真 bug

1. `drift` 命令把 `DISPLACEMENT_LIMIT_PX` 当作 argparse 的默认值，而该常量所在模块 import 了
   cv2——于是**仿真侧（`.venv-loop` 无 cv2）连 `--help` 都构建不出解析器**。常量移到
   `realsim/__init__.py`，并加了一条在两个环境都跑的测试，断言解析器能构建、子命令齐全、
   带默认值的参数能解析。这条测试在仿真侧能复现该 bug。
2. `export` 加进 CLI 后忘了加进 `run_realsim.sh` 的环境路由表，于是它跑到了没有
   `so101_nexus` 的真实侧。现在有一条测试解析 shell 里的 `case` 分支，和解析器里每个子命令
   声明的 `side` 逐一比对——两份清单不能再分家。

## 7. 代码改动与测试

新增，不改动 `rgbcal` 与 `nexus_vision`：

```
LLM-control/run_realsim.sh
LLM-control/REALSIM_README.md
LLM-control/realsim/{__init__,geometry,blocks,estimate,track,scene,observe,overlay,drift,mirror,export,selftest,cli}.py
LLM-control/realsim/config/blocks.json
LLM-control/realsim/tests/{test_geometry,test_estimate,test_mask,test_colour,test_track_scene,test_drift,test_mirror,test_export,test_observe}.py
```

```bash
cd LLM-control
PYTHONPATH=. .venv-rgbcal/bin/python -m pytest realsim/tests -q          # 42 passed, 2 skipped
PYTHONPATH=third_party/so101-nexus/src:. MUJOCO_GL=egl .venv-loop/bin/python \
    -m pytest realsim/tests -q                                          # 45 passed, 1 skipped
PYTHONPATH=. .venv-rgbcal/bin/python -m pytest rgbcal/tests -q           # 23 passed
PYTHONPATH=third_party/so101-nexus/src MUJOCO_GL=egl .venv-loop/bin/python \
    -m pytest tests -q --confcutdir=. --rootdir=. --import-mode=importlib  # 86 passed, 3 skipped
```

两套测试在各自环境里跳过缺依赖的那一个：真实侧跳过 MuJoCo 镜像测试，仿真侧跳过 OpenCV 测试。

## 8. 复现

```bash
# 合成自检
bash LLM-control/run_realsim.sh selftest --grid 9 --session repro-selftest
# 颜色带体检 + 尺寸反查（换方块或换灯光后必做）
bash LLM-control/run_realsim.sh colours --session repro
bash LLM-control/run_realsim.sh measure --session repro
# 在线单帧 -> 镜像
bash LLM-control/run_realsim.sh observe --session repro
bash LLM-control/run_realsim.sh mirror --scene LLM-control/calib/repro/scene.json --session repro
```

## 9. 关键帧索引

| 文件 | 说明 |
| --- | --- |
| `frames/three-cubes-detected.jpg` | 三块分开、全部检出：绿色为拟合轮廓、方块自身坐标轴、位置/yaw/IoU |
| `frames/three-cubes-touching.jpg` | 三块贴合：红块只剩顶面，被正确拒绝；用于尺度检验 |
| `frames/three-cubes-real-vs-mirror.jpg` | 左：真实帧叠加；右：同一观测镜像出的 MuJoCo 场景（方块按自身颜色渲染） |
| `frames/real-overlay-estimate-004.jpg` | 2026-09-18 归档帧（远端 x = 365 mm） |
| `frames/real-overlay-estimate-003.jpg` | 2026-09-18 归档帧（近端 x = 158 mm） |
| `frames/real-vs-mirror.jpg` | 归档帧的真实/镜像对照 |
| `frames/synthetic-placement-01.jpg` | 合成自检第 1 个机位 |
| `frames/synthetic-two-blocks.jpg` | 合成双色场景 |
| `data/selftest.json` | 合成自检完整报告（含抬起拒绝下限） |
| `data/three-cubes-scene.jsonl` | 在线三块的完整场景状态 |
| `data/three-cubes-live.jsonl` | 在线连续观测的最后 8 帧（原 12 帧，含逐帧耗时与抖动） |
| `data/real-frames-scene.jsonl` | 2026-09-18 归档 6 帧（按 30.5 mm 跑的历史记录，见第 4 节更正） |
| `data/live-rate-scene.jsonl` | 单方块在线记录的最后 6 帧，速率与抖动 |
| `data/snapshot.json` | 由真实观测初始化的可步进仿真快照 |
| `data/blocks-as-configured.json` | 本次使用的方块目录（30 mm，含实测色相带） |
| `data/scale-check.json` | 独立尺度检验的结论（不通过，+3.9%） |
| `data/scale-check.jsonl` | 尺度检验的最后 8 帧场景状态 |
| `frames/scale-check-touching.jpg` | 绿、蓝两块正对贴紧放在工作区中央 |
| `data/drift.json` | 相机运动估计（相对 setup-03 锁定参考帧） |
| `frames/drift-reference-setup-03.jpg` | 2026-09-18 锁定机位的参考帧 |
| `frames/drift-now.jpg` | 2026-09-21 同一相机的当前帧，位移中位 55 px |
| `frames/model-run-detected.jpg` | 已知漂移下的建模：三块检出 |
| `frames/model-run-real-vs-mirror.jpg` | 同一次的真实／镜像对照 |
| `data/model-run-scene.json` | 该次的完整场景状态 |
| `data/model-run-snapshot.json` | 该次的可步进仿真快照 |
