# 回位阶段（当前受相机断连阻塞）

任务已终止并推送原分支，提交1b5012a。用户要求的回位尚未执行成功。

## 目标与顺序

以本次启动第1帧实测六电机ticks `[2013,979,2958,2698,2151,2495]` 为初始姿态目标。先抬离方块，再按≤10°的关节增量逐步回收，最后恢复夹爪开口，观察并对比编码器；不盲目反放抓取动作。

## 当前证据

复位前发送 `{"action":"observe","frame":58}`，控制器返回 `env: fresh camera unavailable`，内部frame推进至59，但没有生成第59帧照片或新编码器记录。错误保留在 [task-return.jsonl](task-return.jsonl)。因此不能把最新实测frame=58误写成当前控制协议frame=58。

相机进程已退出，报错 `VIDIOC_REQBUFS: errno=19 (No such device)` 和 `RuntimeError: camera returned no frame`。检查 `/dev/v4l/by-id/` 仅余笔记本内置相机，两路外接环境/腕部相机均不在列表。断连事实已确认，拔线、集线器或电源等具体原因未验证。已请求用户重新连接。

控制器进程仍运行，复位阶段没有发送任何运动指令；最后已知姿态为第58帧，torque均为true。不以旧图绕过实时图像检查。相机恢复后重新启动采集，从frame59发送observe取得新帧，再逐步复位。

## 结果

堆叠 cancelled_by_user；回位 blocked_camera_disconnected。没有宣称机械臂已经回到初始姿态。
