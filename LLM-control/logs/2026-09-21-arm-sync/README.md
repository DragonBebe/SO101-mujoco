# 2026-09-21 关节反馈镜像验证

本轮先更新已有 REALSIM_README，再实现反馈通道与同步。保留原有未提交的场景、模型和抓取实验；
没有覆盖 `calib/realsim-model/scene.xml`。这里的导出 XML 和夹爪映射属于验证产物。

## 真实设备只读尝试

证据：[hardware-read-attempt.jsonl](hardware-read-attempt.jsonl)。
打开前检查 `/dev/ttyACM0` 未被占用，进行了 3.018 秒、88 次读取尝试，**0 个有效反馈**。
电机 1 的 Present_Position 未返回状态包，每次错误等待约 34.17 ms。
不能从这个错误判断是否未通电、线缆问题或设备身份不匹配；没有继续猜测或发送任何寄存器写入。
日志末尾是发布器关闭的 disconnected 状态，退出后设备不再被持有。

因此没有完成：多静止姿态一致性、真实各轴方向/幅度、真实夹爪开合、真实物体相对位置、
真机有效采样率与真实端到端延迟验证。后续现场步骤见 [REALSIM_README 第 10 节](../../REALSIM_README.md#10-真实关节反馈同步本轮新增)。

## 合成端到端验证

最终证据：[synthetic-verified/synthetic-summary.json](synthetic-verified/synthetic-summary.json)。
输入显式标为 `SYNTHETIC TEST ONLY`，由脚本生成 30 Hz 原始编码器值；不打开串口。
在启动时的已导出场景中更新五臂轴和**合成两点夹爪标定**，中断发布 2.4 秒再恢复。
断言完整的 `live → stale → disconnected → live` 顺序，同时验证 raw JSONL 的按时回放。
审计末尾记录 mirror stopped，保留最后有效姿态及时间。

| 指标 | 最终无头运行 |
| --- | --- |
| 有效应用样本 | 93 |
| 主机刷新循环 | 59.72 Hz |
| 处理耗时平均 / 最大 | 0.191 / 0.261 ms |
| 主机观测数据年龄平均 / 最大 | 14.48 / 17.09 ms |
| 全程接收频率（含启动、暂停、结束等待） | 13.26 Hz |

年龄受到采集/显示循环相位与系统调度影响。此处所有延迟都是软件内部测量，**不是运动到显示器的端到端延迟**。
采样、转换、回放产物：

- `synthetic-verified/synthetic-feedback.jsonl`：原始合成 ticks、寄存器夹具、采样/接收时间。
- `synthetic-verified/sync-audit.jsonl`：显示进程收到的 raw 与 converted 弧度、状态变化、关闭状态。
- `synthetic-verified/replay-audit.jsonl`：按原始接收时刻回放的转换结果。
- `synthetic-verified/scene.xml`：本次新导出的基座对齐场景，物体保持静态。
- `synthetic-verified/synthetic-gripper.json`：**只供测试，禁止当作真机夹爪标定**。
- `synthetic/` 和 `synthetic-final/`：实现迭代期间的先前验证，最终结果以 `synthetic-verified/` 为准。

虚拟桌面烟雾测试使用 `xvfb-run -a ... arm-replay --speed 2 --viewer`，正常退出，
应用 91 个样本，viewer 同步循环 57.96 Hz，最终反馈状态 disconnected。
记录：[viewer-replay-console.jsonl](viewer-replay-console.jsonl)、[viewer-replay-audit.jsonl](viewer-replay-audit.jsonl)。
证明窗口、文字状态和 MuJoCo viewer 刷新调用可运行，不证明真实桌面屏幕上的物理刷新率。

## 回归与审查

最终测试输出存于 [pytest-loop.txt](pytest-loop.txt)、[pytest-rgbcal.txt](pytest-rgbcal.txt)。
`.venv-loop`：**175 passed, 6 skipped**（57.12 s）；`.venv-rgbcal`：**109 passed, 5 skipped**（39.26 s）。
跳过项是环境缺少对应依赖（如 OpenCV、so101_nexus、串口包）的测试；没有测试失败。
启动脚本 `bash -n`、命令帮助与 `git diff --check` 均通过。

```bash
cd LLM-control
PYTHONPATH=.:third_party/so101-nexus/src MUJOCO_GL=egl .venv-loop/bin/python \
  -m pytest realsim/tests tests -q --confcutdir=. --rootdir=. --import-mode=importlib
PYTHONPATH=. MUJOCO_GL=egl .venv-rgbcal/bin/python \
  -m pytest realsim/tests rgbcal/tests -q --confcutdir=. --rootdir=. --import-mode=importlib
```

专门测试覆盖：

- 五个轴各自正负 ticks 变化、标定零点补偿、异常/缺失值、标定寄存器变化拒绝。
- 明确两点夹爪映射及区间外拒绝；未标定 fraction 不应用。
- 具名关节地址和整组范围检查；错误不改变最后 qpos，物体不移动、仿真时间不推进。
- baseframe = inverse(T_base_table)；导出、镜像、快照保存/恢复保持对齐。
- 出错、超时、恢复、未来或旧反馈、 malformed 结构。
- 已有串口描述符时拒绝另开；真实 PTY 加模拟 SDK 检查并发事务不重叠、生命周期无寄存器写入。
- 0.1×、1×、10× 回放统一虚拟时钟，保留采集耗时；采集已经过期的样本不在回放中伪装新鲜。

一次独立审查发现了倍速回放时钟、采集耗时和快照基座保存三个问题，均新增可复现失败的测试后修复；
审查者独立复核相关 4 项回归通过。未发现新增真机扭矩/目标写入。
