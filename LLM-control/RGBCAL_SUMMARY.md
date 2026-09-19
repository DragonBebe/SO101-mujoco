# SO101 真机环境相机标定：过程、结果与交接（2026-09-17 ~ 09-18）

本文档总结一次完整的标定工作会话，供新对话直接接续。工具使用说明见
[RGBCAL_README.md](RGBCAL_README.md)，其中的「本机实测结论」一节是**笔记本摄像头**的
历史结果；**当前使用的环境相机是 Logitech C920**，以本文为准。

## 1. 目标

项目已有的 MuJoCo 双 RGB 闭环控制（`run_loop.sh`、`nexus_vision`）让对话中的模型
（VLM / LLM 作为规划者）看图、用**平面假设**把像素变成三维坐标
（`localize method:"plane"` + `plane_z`，`propose` + `object_height`，即
`rgb_two_plane_carve`），再下发有界的机械臂动作。仿真里完成了抓取、放置和堆叠。

本次工作要把这条链路搬到**真机 SO101**：给环境相机测出真实的内参、相对机器人底座的外参和
桌面平面，使「像素 → 底座坐标」在真机上成立。约束（用户原始要求）：

- 相机参数必须实测，不能复制 MuJoCo 的焦距、位置、旋转或桌面高度；
- 先确定视角再标定；标定板相对镜头的位姿 ≠ 机器人坐标；
- 不依赖腕部相机的手眼标定；不默认夹爪边角就是 TCP；不用未验证的 FK 冒充测量；
- 标定缺失或配置不符时可以预览，但**不能静默输出可用于运动的三维坐标**；
- 不自动执行未验证的抓取；真实运动验证由用户配合、先低速悬停。

## 2. 设备与软件

| 项 | 值 |
| --- | --- |
| 环境相机（当前） | Logitech **HD Pro Webcam C920**，USB `046d:082d`，序列号 `B222F89F` |
| 稳定设备路径 | `/dev/v4l/by-id/usb-046d_HD_Pro_Webcam_C920_B222F89F-video-index0`（`/dev/videoN` 编号插拔会变） |
| 成像配置 | 1920×1080 MJPG，实测 30.1 fps，**自动对焦关闭、focus = 0**，zoom 100，无镜像 |
| 配置文件 | `calib/profiles/c920.json`，所有命令用 `--profile c920` |
| 腕部相机 | `USB2.0_CAM1`（`05a3:9230`），本次**未使用** |
| 笔记本自带摄像头 | `Integrated Camera`（`5986:118a`），第一轮用过，已弃用 |
| 机械臂总线 | `/dev/ttyACM0`，1 Mbps，Feetech STS3215 ×6（ID 1–6：pan、lift、elbow、wrist_flex、wrist_roll、gripper） |
| 舵机标定 | lerobot `~/.cache/huggingface/lerobot/calibration/robots/so101_follower/follower.json` |
| 运动学模型 | `third_party/so101-nexus/.../SO101_menagerie/so101.xml`，TCP = `gripperframe` site |
| 工具 | `LLM-control/rgbcal/`，入口 `bash LLM-control/run_rgbcal.sh <命令>` |
| Python 环境 | `LLM-control/.venv-rgbcal`（OpenCV 5.0 含 aruco/Qt、scipy、mujoco、feetech-servo-sdk、pytest），**不改动** `.venv-loop` |

串口权限：用户已加入 `dialout` 组（重新登录后生效）；未重新登录时用
`sudo chmod a+rw /dev/ttyACM0`（插拔后失效）。

## 3. 流程与结果

### 3.1 第一阶段：视角（先定视角，再标定）

**笔记本摄像头（第一轮，已弃用）**：经过 5 轮调整（抬高、后退、调屏幕角度、移动工作区），
达到方块 67–101 px、四周留 10% 余量、抬升时夹爪在画面内；锁定为 `setup-01`。

**切换到 C920（当前）**：
- 自动对焦会搜索（1080p 下一帧清晰度只有 6），用 `focus-sweep` 实测选定 **focus = 0**；
- 调整后（`c920-view-03`）：近端方块距下边缘 115 px、左右 254 / 188 px（10% 线分别为 108 / 192 px），
  方块约 82–122 px，抬到最高时夹爪完整入画（上臂顶端距上边约 50 px，可接受）；
- 锁定 `setup-02`，之后重新锁定为 **`setup-03`**（见 3.3 的漂移）。

### 3.2 第二阶段：内参

**C920（当前）**：`calib/c920-intrinsics-01/intrinsics.json`，普通棋盘格 9×6、25 mm，36 标定 + 9 验证。

```
fx 1379.97  fy 1377.83  cx 925.33  cy 514.58     @1920x1080
D [0.1328, -0.26156, -0.00369, 0.00017, 0.11882]  (k1 k2 p1 p2 k3)
视场 69.65° x 42.80°
```

| 检验 | 结果 |
| --- | --- |
| **留出验证 RMS（9 张，拍摄时即分配）** | **0.762 px**（训练 0.915 px，无过拟合） |
| 标定板直角（与打印尺寸无关） | 中位 0.43°，最大 0.83° |
| 边缘/中心误差比 | 0.46 |
| 覆盖半径 | 86%，工作区全部在已验证范围内 |
| 模型选择 | standard 与 rational 留出误差相同，用 standard |

约 11 帧拍摄时板子被拿弯（平滑曲面解释 76–86% 残差）。按预定规则剔除后，固定留出集
**没有改善**（0.762 → 0.768 px），所以保留全部帧——没有为训练误差好看而挑数据。
C920 畸变很小：整幅画面上原始/去畸变像素相差 ≤ 2.4 px（若混用约 1–3 mm）。

**笔记本摄像头（历史）**：`intrinsics-01` 纸面手持弯曲，RMS 2.545 px（失败，保留作记录）；
贴硬板后 `intrinsics-02` 留出 RMS 0.244 px。

### 3.3 第三阶段：外参（眼在手外手眼标定）

方法：大号 ChArUco（7×9、20 mm、DICT_4X4_50）**刚性固定在夹爪/腕部固定件上**，
相机固定；每个姿态记录图像（PnP 得 `T_camera_board`）和 6 个舵机读数（FK 得
`T_base_gripper`），联合求解 `T_base_camera`、`T_gripper_board` 以及关节约定。

**当前结果**：`calib/c920-handeye-03/extrinsics.json`（31 个锁力矩姿态，24 求解 + 7 留出）

| 项 | 值 |
| --- | --- |
| 相机位置（底座系） | **(0.760, −0.058, 0.455) m**，光轴朝向机器人、向下 |
| 关节符号 | **全 +1**（区分度 3.2 倍） |
| 零位偏移（模型零位 − lerobot 行程中点） | lift +3.07°，elbow −3.56°，wrist_flex +10.46°；pan、wrist_roll 钉为 0（不可辨识，见下） |
| 拟合 | Huber 损失，姿态残差中位 9.1 px，无离群 |
| **留出姿态板子位置误差** | **中位 4.3 mm，最大 12.0 mm** |
| Bootstrap（30 次重采样） | 相机位置标准差 (6.5, 15.7, 19.8) mm；旋转 95% 范围 4.0° |
| 物理合理性 | 在底座平面上方、朝下、面向机器人：全部通过 |

外参与关节零位之间存在补偿：在数据覆盖的空间里「相机 ↔ 机械臂」组合关系准确，外参本身
单独看误差更大，离开覆盖区域误差会增大。**运行时必须使用同一套关节约定（符号 + 偏移）**。

**过程中确认的事实与修正**（新对话务必知道）：

1. **lerobot 约定**（读源码确认）：`Homing_Offset` 写在舵机 EEPROM 里，`Present_Position`
   已经减过；角度零位是**标定行程中点** `(range_min+range_max)/2`，不是 2048；换算 `360/4095` 度/tick；
   `Goal_Position` 与 `Present_Position` 不做符号编码。
2. **OpenCV 5 的 Python 绑定没有 `calibrateHandEye`**（常量在、函数不在）→ 自己实现 Park 闭式解做初值，合成测试证明方向正确（返回 `T_base_camera` 而非其逆）。
3. **镜像歧义**：只比较旋转角无法区分整条臂符号全反 → 改用螺旋不变量（转角 + 沿轴平移）。
4. **不可辨识性**：pan 的零位 ≡ 相机绕底座 z 轴旋转；wrist_roll 的零位 ≡ 板子安装姿态 → 只拟合 lift / elbow / wrist_flex 的零位。
5. **肘部翻转镜像解**：lift/elbow/wrist 三个符号全反 + elbow、wrist 各加约 180° 几乎复现同样的运动，
   无约束时优化器选中了它（相机被算到机器人背后）。依据 lerobot 与 `so101_new_calib` 的零位定义，
   零位偏移限制在 ±30°，并加物理合理性检查。
6. **手扶会压弯打印连杆**：力矩关闭时手必须扶着手臂拍照，逐关节误差 1–3°；
   改成**锁力矩拍照**（先写 目标位置=当前位置，再开力矩，手臂不动）后，lift/elbow 误差降到 0.2–0.3°。
7. **舵机过载**：wrist_roll（ID 5）长时间保持板子重量触发 Overload（状态字节 32），舵机自行卸力。
   释放力矩改为容忍告警并回读确认；锁定前检查告警；采集中断时从每帧 sidecar 恢复清单。

**失败/实验记录（保留不删）**：
- `c920-handeye-01`：板子被手捏着 → 实验解把相机算到桌面下 18 cm、朝上看，定位落在机器人背后 0.4–0.5 m（`extrinsics-experimental.json`，标 `EXPERIMENTAL-UNVERIFIED`，被接入层拒绝）；
- `c920-handeye-02`：板子固定但手扶无力矩，留出误差中位 20.8 mm；
- `handeye-01`（笔记本摄像头）：板子松动，RMS 48.9 px。

**相机漂移**：锁定 `setup-02` 后、手眼标定前，相机缓慢下沉约 3–4 px（约 0.17°），之后稳定在 ±1 px；
标定与后续检验在同一状态，影响可忽略。已重新锁定 **`setup-03`**。漂移检测改为网格分区锚点 + 最大一致簇投票
（原先的锚点被走动的人和机器人本身带偏）。

### 3.4 桌面平面

`calib/c920-table/table.json`：棋盘格纸直接平放，4 个位置、216 个角点拟合为**一个平面**，
残差 RMS 0.39 mm。底座系中 `z = 0.0159·x + 0.0288·y − 0.0172`（m），倾斜 **1.88°**，
在工作区高度约 −10 ~ −15 mm。倾斜在各处一致，是解出的底座系相对桌面的整体倾斜，外参与机械臂仍自洽。
覆盖区域集中在 y ≈ −0.03 ~ +0.07 m，两侧为外推。

### 3.5 接入现有 RGB 定位

`rgbcal/realcam.py`，**不修改** `nexus_vision.perception`：

- 画面用实测 K/D **去畸变**到新内参 `K_new`（alpha=0）；感知只用「去畸变图 + `K_new`」，不混用；
- 旋转从 OpenCV 轴（x 右、y 下、z 前）**显式转换**为 perception 需要的 OpenGL 轴；
- 在**桌面坐标系**中定位：z 轴 = 实测桌面法向，`plane_z = 0` 严格等于桌面，`object_height` 垂直于桌面
  （堆叠时 `plane_z = 0.025`）；结果经 `T_base_table` 转回底座系；
- 标定与设备序列号、分辨率、自动对焦/focus、镜像绑定，配置不符直接拒绝；
- 每个结果带 `usable_for_motion`，只有通过监督运动检查（`calib/c920-motion-check/motion_check.json`，
  尚不存在）才为 true；实验性外参无法加载。

测试：`rgbcal/tests` 22 项全部通过（手眼合成闭环、端到端经未修改的 perception 回到原点 < 1.5 mm、
坐标轴约定、整幅画面、图像去畸变与点去畸变一致、堆叠高度、配置不符拒绝、漂移检测）；
原有仿真测试 69 通过 / 3 跳过。

### 3.6 第六阶段：端到端点检查（不自动驱动机械臂）

协议：方块单独放桌上 → 相机估计抓取中心；用户手动把夹爪合在方块上 → 锁定 → FK 算 TCP → 比较。

- `c920-point-check-01`（手动合爪）：偏差 8–54 mm。把 FK 的 TCP 反投影到图像上，
  它在 4 个不同姿态下都落在夹爪的**同一个物理部位（钳口根部）**——外参与 FK 一致，
  主要差距是模型 TCP 与实际抓取点不同，再加上手动合爪位置不一；
- `c920-point-check-02`（夹紧后连夹爪一起锁定，5 次）：
  - **沿接近方向的抓取中心偏移一致：模型 TCP 之外 26.4 ± 3.7 mm**（实测，不再假设）；
  - 同一手腕朝向的 3 次（#1–#3）：横向留一误差 5.8 / 15.4 / 11.4 mm；
  - **#4、#5 方块在同一位置、夹法相同，只有手腕朝向不同，FK 给出的抓取点相差约 40 mm**
    → 误差来自**机械臂模型的手腕部分**，不是相机。与手眼阶段逐关节数据吻合
    （wrist_flex 板子/舵机比 0.927，wrist_roll 0.958）；
  - 全部 5 次留一端到端误差中位 36 mm。**目前不能用于运动。**

### 3.7 映射到仿真（2026-09-19）

`bash LLM-control/run_rgbcal.sh sim-camera` 把去畸变内参 `K_new` 与桌面系相机位姿
`T_table_camera` 导出到 `calib/c920-sim-camera/sim_camera.json`；仿真用
`--camera-modality rgb --environment-camera calibrated` 加载（仅 RGB）。验证：渲染的
GL 相机回读内参与文件一致；仿真深度反投影地面像素中位误差 0.28 mm；与真机去畸变画面
叠加时底座、桌面透视重合。**重新标定（外参 / 桌面 / 内参）后需重新导出。**
详见 `LOOP_README.md`「真机标定机位」。

## 4. 状态

| 项 | 状态 |
| --- | --- |
| 设备识别、成像配置固定（C920 profile，锁焦） | ✅ |
| 视角确认与锁定（`setup-03`） | ✅ |
| 内参（留出 0.76 px） | ✅ |
| 相机相对标定板位姿（PnP） | ✅ |
| 相机相对底座外参（留出 4.3 mm 中位，板子姿态） | ✅ 已求解并独立验证；未经运动验证 |
| 桌面平面（0.39 mm 残差） | ✅ |
| 接入层 + 测试 | ✅ |
| 抓取中心（TCP）偏移 | ⚠ 接近方向已测（26.4 mm）；横向受手腕模型误差影响 |
| **端到端抓取点精度** | ❌ 留一中位 36 mm，手腕朝向改变时约 40 mm 不一致 |
| 监督下的低速悬停测试 | ❌ 未做（`usable_for_motion` 仍为 false） |
| 打印尺寸实测 | ❌ 用户未量；格长按标称 25 / 20 mm（只影响米制尺度，可用单个标量修正） |
| 腕部相机 | 不在本次范围 |

## 5. 下一步（按优先级）

> 2026-09-18 已做第一次真机抓取和堆叠实验，完整记录见
> [calib/c920-real-grasp-01/LOG.md](calib/c920-real-grasp-01/LOG.md)。要点：固定竖直接近时，
> 所有成功操作满足 FK − cam = (32.1 ± 2.3, −2.4 ± 3.3, +11.9) mm；需要按实测 FK 闭环来消除舵机下垂；
> 放置时要带上夹起时记录的夹持偏移。下次真机操作前先读它的第 0 节和第 7 节。

1. **解决手腕运动学不一致**（当前主要误差源）：
   - 锁力矩下做 wrist_flex、wrist_roll 的单关节测试，确认刻度（0.927 / 0.958 是否真实）及是否有关节回差/柔性；
   - 把 wrist_roll 零位、wrist 刻度等纳入模型，或者把运行时抓取限制为固定手腕朝向（例如竖直向下、固定 roll），
     在该朝向下单独标定抓取中心；
   - 用夹紧抓取点检查（`point-check` + `point-report`）重新评估，目标：留一横向误差 < 5–8 mm。
2. 需要时补采一批**覆盖左右两侧**的手眼姿态与桌面测量（目前集中在 y ≈ 0 附近）。
3. 精度达标后做**监督下的低速悬停测试**（需用户明确授权驱动机械臂；先停在目标上方数厘米，测实际偏差），
   通过后写 `calib/c920-motion-check/motion_check.json`，才允许 `usable_for_motion = true`。
4. 真机闭环接入：真机 IK / 关节下发必须使用 `extrinsics.json` 中的关节约定（符号 + 零位偏移）和实测抓取中心偏移；
   不沿用仿真的夹爪偏移补偿与定位阈值；颜色候选要排除人手/手臂（皮肤会被识别为红色，已用方块尺寸过滤）。

## 6. 常用命令（在仓库根目录）

```bash
# 相机是否被碰动（位移 > 3 px 必须重测外参）
bash LLM-control/run_rgbcal.sh check-setup --profile c920 --session setup-03
# 实时预览（带棋盘格检测）
bash LLM-control/run_rgbcal.sh preview --profile c920 --board chessboard --session <name>
# 读舵机（只读）/ 释放力矩（先托住手臂）
bash LLM-control/run_rgbcal.sh arm
bash LLM-control/run_rgbcal.sh arm-release
# 手眼：锁力矩采集 / 逐关节检查 / 求解（bootstrap）/ 留出验证
bash LLM-control/run_rgbcal.sh handeye-collect --profile c920 --board charuco-large \
     --intrinsics LLM-control/calib/c920-intrinsics-01/intrinsics.json --session <name> --hold
bash LLM-control/run_rgbcal.sh handeye-joints --session <name> --detail
bash LLM-control/run_rgbcal.sh handeye-solve  --session <name> --bootstrap 30
bash LLM-control/run_rgbcal.sh handeye-verify --session <name>
# 桌面平面 / 端到端点检查 / 抓取中心报告
bash LLM-control/run_rgbcal.sh table-plane --profile c920 --board chessboard \
     --intrinsics LLM-control/calib/c920-intrinsics-01/intrinsics.json \
     --extrinsics LLM-control/calib/c920-handeye-03/extrinsics.json --thickness 0.0001 --session <name>
bash LLM-control/run_rgbcal.sh point-check  --profile c920 --session <name> [--support-height 0.025]
bash LLM-control/run_rgbcal.sh point-report --session <name>
# 导出给仿真的相机（标定更新后重跑）
bash LLM-control/run_rgbcal.sh sim-camera
# 测试
cd LLM-control && PYTHONPATH=. .venv-rgbcal/bin/python -m pytest rgbcal/tests -q
```

采集界面：`s` 拍照，`l` 锁全部关节，`1`–`5` 只放开一个关节，`r` 全部释放（**先托住手臂**），`q` 结束。
结束时手臂若仍锁定，工具不会自动释放。

## 7. 什么变化需要重新标定

| 变化 | 内参 | 外参 / 桌面 / 抓取中心 |
| --- | --- | --- |
| 相机被碰动（`check-setup` 位移 > 3 px）、机械臂底座或桌面移动 | 不需要 | **重测** |
| 改分辨率、像素格式、对焦、镜像、换相机 | **重做** | 重测 |
| 重新做 lerobot 舵机标定 | 不需要 | **重测**（关节零位会变） |
| 更换夹爪或改变夹持方式 | 不需要 | 重测抓取中心 |
| 只是光照、曝光、白平衡变化 | 不需要 | 不需要 |

## 8. 经验教训

- 标定板必须刚性平整；纸面弯曲会被算成镜头畸变（笔记本摄像头第一次 RMS 2.5 px）；
- 手眼标定的板子必须和夹爪**固定件**成为一个刚体：手捏或夹在自由的钳口之间都会让结果失效；
- 单关节运动下「板子转角 = 关节转角」是与任何约定无关的现场检验，采集工具实时显示，发现问题比事后求解快得多；
- 残差小不等于物理正确：镜像解残差更小，却把相机放到机器人背后；
- 自动选取的特征点会集中在机器人这种高对比度、会移动的物体上；
- 力矩关闭时拍照需要手扶，会引入 1–3° 的连杆形变；锁力矩拍照可以消除，但要防舵机过载。
