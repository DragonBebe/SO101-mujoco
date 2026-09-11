# SO101 三次自然语言任务执行日志

日期：2026-09-10。以下从原始 events.jsonl 整理，保留失败与调整。墙钟时间显示为 Europe/Paris（UTC+02:00）；仿真时间单位秒。三次任务在同一 workbench、seed=4、world=1 中连续执行，没有重置；任务之间等待时物理时间暂停。

控制方式：当前 VS Code Codex-astra 会话读取 RGB-D 和本体状态，逐条发出有界动作。物体由执行器、接触、摩擦和重力驱动。完成判据为 codex_visual，即图像/深度核验，不是任意任务的独立自动评分。

原始逐事件记录：[events.jsonl](session/events.jsonl)。相对路径副本：[events.portable.jsonl](session/events.portable.jsonl)。原始文件字节保持不变；其中绝对路径记录的是原机器，跨机器查看可用相对路径副本（图像路径相对 session 目录）。

|任务|动作尝试|接口成功 / 拒绝|仿真时长|最终堆叠（自下而上）|
|---|---:|---:|---:|---|
|1. 把绿色方块放到红色上面|13|10 / 3|24.26 s|红—绿|
|2. 将红色方块放在绿色方块上面|18|18 / 0|44.98 s|绿—红|
|3. 将红色方块和绿色方块放在紫色方块上面|35|34 / 1|82.66 s|紫—红—绿|

## 任务 1：把绿色方块放到红色上面

- 任务 ID：`1bdee0d61b0247ccbf0d2e47d5e9ccaa`
- 墙钟：2026-09-10T17:56:28+02:00 至 2026-09-10T18:00:23+02:00
- 仿真时间：1.00 → 25.26 s
- 过程：通过俯视 RGB-D 定位绿块与红块；降低不可达的接近高度；抓起绿块，运到红块上方，下降、释放并退开；静置 3 秒复查。第 1、9、11 步被 IK 拒绝，未推进物理状态，随后用较低的可达高度完成。
- 最终证据：绿色25mm方块已释放并堆叠在红色25mm方块上，夹爪张开并退开。俯视RGB可见绿色在红色顶部；RGB-D测得绿块顶面z=0.049896m，约为两块方块总高度50mm，顶面采样XY=[0.219850,-0.062228]，与放置前红块顶面中心[0.219538,-0.062371]相差约0.34mm。退开后额外静置3秒，再次观察和同像素深度定位结果不变。依据为相机图像、深度及机器人本体反馈。
- [最终俯视图](session/world-001/0015-overhead.png)；对应原始 RGB、深度 NPY 与标定位于 `session/world-001/`。

### 逐步执行

|步骤|原始序号|仿真时间|动作 JSON|结果|
|---:|---:|---:|---|---|
|1|5|1.00|`{"action":"move","point_id":"d24809b1cdad4842:point-1","offset":[0.0056,-0.005,0.065],"seconds":2}`|IK cannot reach target with down orientation; error=0.0059|
|2|6|3.50|`{"action":"move","point_id":"d24809b1cdad4842:point-1","offset":[0.0056,-0.005,0.04],"seconds":2}`|执行完成|
|3|7|5.96|`{"action":"move","point_id":"d24809b1cdad4842:point-1","offset":[0.0056,-0.005,-0.0125],"linear":true,"seconds":2}`|执行完成|
|4|8|7.46|`{"action":"gripper","opening":0,"seconds":1.5}`|执行完成|
|5|9|9.92|`{"action":"move","point_id":"d24809b1cdad4842:point-1","offset":[0.0056,-0.005,0.04],"linear":true,"seconds":2}`|执行完成|
|6|11|13.38|`{"action":"move","point_id":"d24809b1cdad4842:point-3","offset":[0.0072,-0.0021,0.055],"linear":true,"seconds":3}`|执行完成|
|7|12|15.84|`{"action":"move","point_id":"d24809b1cdad4842:point-3","offset":[0.0072,-0.0021,0.0135],"linear":true,"seconds":2}`|执行完成|
|8|13|17.34|`{"action":"gripper","opening":0.75,"seconds":1.5}`|执行完成|
|9|14|17.34|`{"action":"move","point_id":"d24809b1cdad4842:point-3","offset":[0.0072,-0.0021,0.07],"linear":true,"seconds":2}`|IK cannot reach target with down orientation; error=0.0034|
|10|15|19.80|`{"action":"move","point_id":"d24809b1cdad4842:point-3","offset":[0.0072,-0.0021,0.055],"linear":true,"seconds":2}`|执行完成|
|11|16|19.80|`{"action":"move","position":[0.14,-0.14,0.12],"linear":true,"seconds":2}`|IK cannot reach target with down orientation; error=0.0171|
|12|17|22.26|`{"action":"move","position":[0.14,-0.14,0.08],"linear":true,"seconds":2}`|执行完成|
|13|19|25.26|`{"action":"wait","seconds":3}`|执行完成|

### 视觉定位记录

|原始序号|点 ID|相机 / 像素|表面世界 XYZ（米）|
|---:|---|---|---|
|3|`d24809b1cdad4842:point-1`|overhead / [468, 198]|[0.187494, -0.169948, 0.024955]|
|4|`d24809b1cdad4842:point-2`|overhead / [374, 170]|[0.219538, -0.062371, 0.024955]|
|10|`d24809b1cdad4842:point-3`|overhead / [374, 170]|[0.219538, -0.062371, 0.024955]|
|18|`d24809b1cdad4842:point-4`|overhead / [376, 167]|[0.21985, -0.062228, 0.049896]|
|20|`d24809b1cdad4842:point-5`|overhead / [376, 167]|[0.21985, -0.062228, 0.049896]|

## 任务 2：将红色方块放在绿色方块上面

- 任务 ID：`4967fe0b679f41d9ad195a2c5a3ce722`
- 墙钟：2026-09-10T18:00:50+02:00 至 2026-09-10T18:05:59+02:00
- 仿真时间：25.26 → 70.24 s
- 过程：先从红块上抓走绿块，放到旁边空桌面；重新定位两块方块；抓起红块并放到绿块上；松爪退开，静置 3 秒复查。场景保持连续，没有 reset。
- 最终证据：先将顶部绿色方块抓起放在旁边桌面，再将红色方块抓起堆叠到绿色方块上。夹爪已张开并退开，俯视图可见红色位于绿色之上。RGB-D测得红块顶面z=0.049854m，符合两块25mm方块叠放高度；退开后额外静置3秒，同一可见顶面像素测得XYZ=[0.214348,0.054522,0.049854]保持不变。完成依据为相机图像与深度反馈。
- [最终俯视图](session/world-001/0034-overhead.png)；对应原始 RGB、深度 NPY 与标定位于 `session/world-001/`。

### 逐步执行

|步骤|原始序号|仿真时间|动作 JSON|结果|
|---:|---:|---:|---|---|
|1|26|27.72|`{"action":"move","point_id":"d24809b1cdad4842:point-6","offset":[0.0072,-0.0021,0.03],"linear":true,"seconds":2}`|执行完成|
|2|27|30.18|`{"action":"move","point_id":"d24809b1cdad4842:point-6","offset":[0.0072,-0.0021,-0.0125],"linear":true,"seconds":2}`|执行完成|
|3|28|31.68|`{"action":"gripper","opening":0,"seconds":1.5}`|执行完成|
|4|29|34.14|`{"action":"move","point_id":"d24809b1cdad4842:point-6","offset":[0.0072,-0.0021,0.03],"linear":true,"seconds":2}`|执行完成|
|5|30|37.60|`{"action":"move","point_id":"d24809b1cdad4842:point-7","offset":[0.0072,0.002,0.08],"linear":true,"seconds":3}`|执行完成|
|6|31|40.06|`{"action":"move","point_id":"d24809b1cdad4842:point-7","offset":[0.0072,0.002,0.0135],"linear":true,"seconds":2}`|执行完成|
|7|32|41.56|`{"action":"gripper","opening":0.75,"seconds":1.5}`|执行完成|
|8|33|44.02|`{"action":"move","point_id":"d24809b1cdad4842:point-7","offset":[0.0072,0.002,0.08],"linear":true,"seconds":2}`|执行完成|
|9|35|47.48|`{"action":"move","point_id":"d24809b1cdad4842:point-8","offset":[0.0072,-0.0021,0.055],"linear":true,"seconds":3}`|执行完成|
|10|37|49.94|`{"action":"move","point_id":"d24809b1cdad4842:point-8","offset":[0.0072,-0.0021,-0.0125],"linear":true,"seconds":2}`|执行完成|
|11|38|51.44|`{"action":"gripper","opening":0,"seconds":1.5}`|执行完成|
|12|39|53.90|`{"action":"move","point_id":"d24809b1cdad4842:point-8","offset":[0.0072,-0.0021,0.055],"linear":true,"seconds":2}`|执行完成|
|13|40|57.36|`{"action":"move","point_id":"d24809b1cdad4842:point-9","offset":[0.0073,0.0019,0.055],"linear":true,"seconds":3}`|执行完成|
|14|41|59.82|`{"action":"move","point_id":"d24809b1cdad4842:point-9","offset":[0.0073,0.0019,0.0135],"linear":true,"seconds":2}`|执行完成|
|15|42|61.32|`{"action":"gripper","opening":0.75,"seconds":1.5}`|执行完成|
|16|43|63.78|`{"action":"move","point_id":"d24809b1cdad4842:point-9","offset":[0.0073,0.0019,0.055],"linear":true,"seconds":2}`|执行完成|
|17|44|67.24|`{"action":"move","position":[0.14,-0.14,0.08],"linear":true,"seconds":3}`|执行完成|
|18|46|70.24|`{"action":"wait","seconds":3}`|执行完成|

### 视觉定位记录

|原始序号|点 ID|相机 / 像素|表面世界 XYZ（米）|
|---:|---|---|---|
|24|`d24809b1cdad4842:point-6`|overhead / [376, 167]|[0.21985, -0.062228, 0.049896]|
|25|`d24809b1cdad4842:point-7`|overhead / [270, 177]|[0.214219, 0.058781, 0.0]|
|34|`d24809b1cdad4842:point-8`|overhead / [374, 170]|[0.219538, -0.062371, 0.024955]|
|36|`d24809b1cdad4842:point-9`|overhead / [270, 175]|[0.213816, 0.056649, 0.024955]|
|45|`d24809b1cdad4842:point-10`|overhead / [270, 172]|[0.214348, 0.054522, 0.049854]|
|47|`d24809b1cdad4842:point-11`|overhead / [270, 172]|[0.214348, 0.054522, 0.049854]|

## 任务 3：将红色方块和绿色方块放在紫色方块上面

- 任务 ID：`eff3965d556b4259a1ad6229c49f2c1c`
- 墙钟：2026-09-10T22:42:18+02:00 至 2026-09-10T22:51:59+02:00
- 仿真时间：70.24 → 152.90 s
- 过程：紫块原先位于较远位置。第 1–5 步首次夹取未抬起紫块：接口返回 ok 仅代表动作执行，不代表抓取成功。查看图像后松爪退开并重新定位；第 8–16 步调整较大紫块的抓取偏移，成功搬到底座位置。随后把红块从绿块上取下，放到紫块上，再把绿块叠到红块上。第 29 步较高的目标被 IK 拒绝，降低 5 mm 后成功。最后松爪退开，静置 4 秒复查。最终顺序为紫—红—绿（自下而上）；不是红绿并排，也不是两者都直接接触紫块。
- 最终证据：紫色35mm底座已搬到可达前方空位，红色25mm方块置于紫色上，绿色25mm方块置于红色上，形成自下而上紫、红、绿三层。夹爪已张开退开。俯视RGB可见三色层次，RGB-D测得顶面z=0.084742m，接近35+25+25=85mm。额外静置4秒后同顶面像素XYZ=[0.198830,-0.058830,0.084742]不变，图像中堆叠稳定。结论基于图像与深度观测。
- [最终俯视图](session/world-001/0070-overhead.png)；对应原始 RGB、深度 NPY 与标定位于 `session/world-001/`。

### 逐步执行

|步骤|原始序号|仿真时间|动作 JSON|结果|
|---:|---:|---:|---|---|
|1|53|73.74|`{"action":"move","point_id":"d24809b1cdad4842:point-13","offset":[0.0043,0.0062,0.015],"seconds":3}`|执行完成|
|2|54|74.74|`{"action":"gripper","opening":1,"seconds":1}`|执行完成|
|3|55|77.20|`{"action":"move","point_id":"d24809b1cdad4842:point-13","offset":[0.0043,0.0062,-0.0175],"linear":true,"seconds":2}`|执行完成|
|4|56|78.70|`{"action":"gripper","opening":0,"seconds":1.5}`|执行完成|
|5|57|81.16|`{"action":"move","point_id":"d24809b1cdad4842:point-13","offset":[0.0043,0.0062,0.015],"linear":true,"seconds":2}`|执行完成；图像显示紫块未随夹爪抬升（抓取未成功）|
|6|59|82.16|`{"action":"gripper","opening":1,"seconds":1}`|执行完成|
|7|60|85.66|`{"action":"move","position":[0.19,-0.12,0.08],"seconds":3}`|执行完成|
|8|62|89.16|`{"action":"move","point_id":"d24809b1cdad4842:point-16","offset":[-0.0025,-0.0037,0.015],"seconds":3}`|执行完成|
|9|63|91.62|`{"action":"move","point_id":"d24809b1cdad4842:point-16","offset":[-0.0025,-0.0037,-0.012],"linear":true,"seconds":2}`|执行完成|
|10|64|93.12|`{"action":"gripper","opening":0,"seconds":1.5}`|执行完成|
|11|65|95.58|`{"action":"move","point_id":"d24809b1cdad4842:point-16","offset":[-0.0025,-0.0037,0.015],"linear":true,"seconds":2}`|执行完成|
|12|66|98.04|`{"action":"move","position":[0.19,0.15,0.075],"linear":true,"seconds":2}`|执行完成|
|13|67|101.50|`{"action":"move","position":[0.2,-0.06,0.08],"linear":true,"seconds":3}`|执行完成|
|14|68|103.96|`{"action":"move","position":[0.2,-0.06,0.024],"linear":true,"seconds":2}`|执行完成|
|15|69|105.46|`{"action":"gripper","opening":1,"seconds":1.5}`|执行完成|
|16|70|107.92|`{"action":"move","position":[0.2,-0.06,0.08],"linear":true,"seconds":2}`|执行完成|
|17|72|110.38|`{"action":"move","point_id":"d24809b1cdad4842:point-12","offset":[0.0073,0.0019,0.03],"linear":true,"seconds":2}`|执行完成|
|18|74|112.84|`{"action":"move","point_id":"d24809b1cdad4842:point-12","offset":[0.0073,0.0019,-0.0125],"linear":true,"seconds":2}`|执行完成|
|19|75|114.34|`{"action":"gripper","opening":0,"seconds":1.5}`|执行完成|
|20|76|116.80|`{"action":"move","point_id":"d24809b1cdad4842:point-12","offset":[0.0073,0.0019,0.03],"linear":true,"seconds":2}`|执行完成|
|21|77|120.26|`{"action":"move","point_id":"d24809b1cdad4842:point-18","offset":[0.0072,-0.0022,0.045],"linear":true,"seconds":3}`|执行完成|
|22|79|122.72|`{"action":"move","point_id":"d24809b1cdad4842:point-18","offset":[0.0072,-0.0022,0.0135],"linear":true,"seconds":2}`|执行完成|
|23|80|124.22|`{"action":"gripper","opening":0.75,"seconds":1.5}`|执行完成|
|24|81|126.68|`{"action":"move","point_id":"d24809b1cdad4842:point-18","offset":[0.0072,-0.0022,0.055],"linear":true,"seconds":2}`|执行完成|
|25|82|130.14|`{"action":"move","point_id":"d24809b1cdad4842:point-19","offset":[0.0073,0.0019,0.055],"linear":true,"seconds":3}`|执行完成|
|26|84|132.60|`{"action":"move","point_id":"d24809b1cdad4842:point-19","offset":[0.0073,0.0019,-0.0125],"linear":true,"seconds":2}`|执行完成|
|27|85|134.10|`{"action":"gripper","opening":0,"seconds":1.5}`|执行完成|
|28|86|136.56|`{"action":"move","point_id":"d24809b1cdad4842:point-19","offset":[0.0073,0.0019,0.06],"linear":true,"seconds":2}`|执行完成|
|29|87|136.56|`{"action":"move","point_id":"d24809b1cdad4842:point-20","offset":[0.0072,-0.0022,0.035],"linear":true,"seconds":3}`|IK cannot reach target with down orientation; error=0.0026|
|30|88|140.02|`{"action":"move","point_id":"d24809b1cdad4842:point-20","offset":[0.0072,-0.0022,0.03],"linear":true,"seconds":3}`|执行完成|
|31|89|142.48|`{"action":"move","point_id":"d24809b1cdad4842:point-20","offset":[0.0072,-0.0022,0.0135],"linear":true,"seconds":2}`|执行完成|
|32|90|143.98|`{"action":"gripper","opening":0.85,"seconds":1.5}`|执行完成|
|33|91|146.44|`{"action":"move","point_id":"d24809b1cdad4842:point-20","offset":[0.0072,-0.0022,0.031],"linear":true,"seconds":2}`|执行完成|
|34|92|148.90|`{"action":"move","position":[0.14,-0.14,0.09],"linear":true,"seconds":2}`|执行完成|
|35|94|152.90|`{"action":"wait","seconds":4}`|执行完成|

### 视觉定位记录

|原始序号|点 ID|相机 / 像素|表面世界 XYZ（米）|
|---:|---|---|---|
|50|`d24809b1cdad4842:point-12`|overhead / [270, 172]|[0.214348, 0.054522, 0.049854]|
|51|`d24809b1cdad4842:point-13`|overhead / [120, 226]|[0.155217, 0.224871, 0.034955]|
|52|`d24809b1cdad4842:point-14`|overhead / [374, 170]|[0.222531, -0.064719, 0.0]|
|58|`d24809b1cdad4842:point-15`|wrist / [365, 170]|[0.140821, 0.239031, 0.021013]|
|61|`d24809b1cdad4842:point-16`|overhead / [114, 225]|[0.156344, 0.231634, 0.034955]|
|71|`d24809b1cdad4842:point-17`|wrist / [351, 140]|[0.20337, -0.04282, 0.023693]|
|73|`d24809b1cdad4842:point-18`|overhead / [372, 189]|[0.196922, -0.059177, 0.034955]|
|78|`d24809b1cdad4842:point-19`|overhead / [270, 175]|[0.213816, 0.056649, 0.024955]|
|83|`d24809b1cdad4842:point-20`|overhead / [374, 186]|[0.198004, -0.059089, 0.059856]|
|93|`d24809b1cdad4842:point-21`|overhead / [376, 183]|[0.19883, -0.05883, 0.084742]|
|95|`d24809b1cdad4842:point-22`|overhead / [376, 183]|[0.19883, -0.05883, 0.084742]|

## 复现边界

这些是三次已执行轨迹的归档，不是跨布局通用策略。point_id / frame_id 绑定原运行，不能直接向新服务重放；新任务应重新看图定位。旧 vision.py 的 replay 接口也不能直接读取此 loop 事件格式。迁移包不包含模型权重、登录凭据或运行中物理检查点；新进程会创建新世界。
