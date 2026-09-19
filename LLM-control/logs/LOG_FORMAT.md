# 运行日志书写规范（LLM-control/logs）

适用于对话规划（VLM/LLM）驱动 SO101 仿真或真机、需要事后复核的每一次任务归档。
规范来自已有归档的共同做法，以及它们暴露的缺口：

| 归档 | 做得好的地方 | 缺的地方 |
| --- | --- | --- |
| `2026-09-11-continuous-tasks.md` | 任务总览、逐步动作附录、"记录边界"一节 | 单文件、无帧和原始事件，模型/token/耗时不全 |
| `2026-09-17-rgb-three-tasks/` | 原始事件快照、截止序号、`summary.json`、逐事件流程、模型与 token 口径说明 | 失败与推断没有单独成节 |
| `2026-09-17-rgb-stack-swap/` | 元信息表（模型 ID、时长、token）、证据边界、"关键推理"与"需要注意的地方"单列、复现方法 | 无机器可读的用量文件 |
| `2026-09-19-no-ik-joint-control/` | 按本规范撰写；失败分析与"事后更正"单列 | —— |

## 1. 目录结构

`runs/` 在 `.gitignore` 里，**归档必须自包含**，不能只写一个运行目录路径。

```
logs/YYYY-MM-DD-<短名>/
├── README.md              # 主日志（必需），结构见第 2 节
├── metrics.json           # 模型、时长、token 的机器可读版本（必需），见第 4 节
├── task-<X>-<短名>.jsonl  # 每个任务的服务事件，逐行照抄 events.jsonl，加 _seq（必需）
├── events-setup.jsonl     # 任务外的事件：场景勘查、失败的启动、重启（有则必需）
├── session.json           # 运行目录的 session.json 快照（建议）
├── frames/                # 关键帧 PNG，只挑证据帧，不整目录复制（必需）
└── ANALYSIS-<主题>.md     # 与任务相关的独立分析或问答（可选）
```

- 事件序号 `_seq` = 该行在原运行目录 `events.jsonl` 里的行号（从 1 开始）；跨两个运行目录时再加 `_run`。
- 在 README 开头写明**截止序号**：归档是快照，之后服务再有动作不改变归档。
- 帧文件名保持原名（`0035-calibrated.png`），这样表格里的"新帧"列可以直接对上；任务外的帧加前缀（如 `survey-`）。
- 体积：单个归档目标 ≤ 3 MB（已有归档 1–3 MB）。

## 2. README.md 章节（按顺序）

1. **标题 + 结论摘要**：日期、主题；每个任务一行结果（成功/失败 + 一个关键数字）。
2. **元信息**表：用户原话（逐字）、规划模型 **ID**（从会话记录读，不凭记忆写；中途切换模型要写明哪一段用哪个）、
   Python 侧职责、服务启动参数、相机模态/机位/分辨率、seed/world、运行目录、截止序号、时区约定、代码版本（commit）。
3. **时长**表：分"对话侧"（用户消息 → 最后一条助手回复）、"服务侧"（`begin` → `complete` 事件时间差）、
   "仿真物理时间"三种，并写清楚各自含义；它们不能互相换算。
4. **Token**表：见第 3 节口径。
5. **运行条件与证据边界**：用了哪些先验（例如场景说明里的尺寸）、哪些东西**从未**读取（物体仿真位姿、分割 ID）、
   三维从哪来（平面假设/深度），结果由谁判定（`codex_visual` 是模型判断，不是自动评分）。
6. **汇总**表：任务 / 步数 / 预算 / 被拒请求 / 结果 / task_id。
7. **每个任务一节**，固定小节：
   - 用户原话、服务任务文本、事件范围、对应 jsonl 链接；
   - 定位记录表（事件、相机、方法与假设、结果、`extent` 自检）；
   - 动作与反馈表：`步 | 事件 | 指令 | 实际关节 | 实际 TCP | 新帧 | 说明`，**由脚本从 jsonl 生成**，说明列人工补；
   - 关键判断：只写改变了后续动作的判断及其依据；
   - 完成证据：`complete` 的证据文本 + 1–2 张最终帧。
8. **失败分析**（有失败或异常就必须有）：`现象 → 数据 → 假设 → 如何验证 → 结论（已验证/未验证）→ 教训`。
   未验证的推测必须标"未验证"，不能写成事实。
9. **事后更正**：运行中写进服务记录（如 `complete.evidence`）或初版日志的结论，事后被推翻的，列出原说法、
   新证据、更正后的结论。服务端的原记录不改，只在这里更正。
10. **关键帧索引**：帧文件 → 说明。
11. **代码改动与测试**（有改动时）：文件清单、测试命令与结果、对应 commit。
12. **复现方法**：启动命令、seed、从哪个状态开始。

## 3. 时间与 token 口径

- 时间：表里用本地时间（写明 CEST/UTC+02:00），原始事件保留 UTC。
- Claude Code：数据在 `~/.claude/projects/<项目>/<session>.jsonl`。按 `message.id` 去重后，对每条助手消息的
  `message.usage` 求和；按用户消息时间戳切分轮次，按服务事件时间戳切分任务阶段。
  字段：`input_tokens`（未命中缓存的输入）、`cache_creation_input_tokens`（缓存写入）、
  `cache_read_input_tokens`（缓存读取）、`output_tokens`（含思考）。缓存读取远大于其他项，是每一轮重读上下文前缀，
  按缓存价计费，**不能**与新增 token 相加成"总量"来比较。
- Codex：用 `token_usage_record` 的逐响应 `usage` 求和，并与 `turn_token_usage` 校验（见 `2026-09-17-rgb-three-tasks/FLOW_AND_USAGE.md`）。
- 撰写归档这一轮本身的用量在写文件时还没结束，写明"未计入"，不要估。
- 只报告测得的数：没有字段的（例如图像 token）写 null/"无"，不推算。

```python
# 按时间窗口统计 Claude Code 用量（窗口取服务事件或用户消息的时间戳）
import json
from datetime import datetime
ts = lambda s: datetime.fromisoformat(s.replace('Z', '+00:00'))
seen, tot = set(), dict(input_tokens=0, cache_creation_input_tokens=0,
                        cache_read_input_tokens=0, output_tokens=0)
for line in open(SESSION_JSONL):
    r = json.loads(line)
    if r.get('type') != 'assistant' or not (START <= ts(r['timestamp']) < END):
        continue
    m = r['message']
    if m['id'] in seen:
        continue
    seen.add(m['id'])
    for k in tot:
        tot[k] += m['usage'].get(k) or 0
```

## 4. metrics.json

```json
{
  "schema": "so101-log-metrics/1",
  "session": "claude-code <session-id>",
  "source": "数据文件及统计方法",
  "timezone": "CEST (UTC+02:00)",
  "phases": [
    {"id": "T1-P2", "what": "任务 A 执行", "model": "claude-sonnet-5",
     "start": "12:51:37", "end": "12:58:38", "minutes": 7.0,
     "service_begin_to_complete_s": 426.4, "responses": 55,
     "input": 110, "cache_write": 48822, "cache_read": 11639636, "output": 23757,
     "tools": {"Bash": 35, "Read": 24}}
  ]
}
```

## 5. 书写规则

- **数字来源**：步骤表、坐标、关节角、时间一律从归档的 jsonl 生成或复制，不凭记忆转写。
- **区分三类量**：实测（服务返回的 TCP、关节角）、估计（`propose`/`localize` 的结果，写明假设）、推测（写"推测/未验证"）。
- **失败如实**：失败任务照样完整记录；不删掉不好看的步骤；被拒的请求也进步骤表。
- **不要把表面现象当原因**：例如"关节不动"可能是接触、限位、目标累积或代码错误，写原因前先给出区分它们的证据。
- **不导出原始推理**：决策说明是事后整理的可复核依据，不是逐字思考过程。
- **不含敏感信息**：不写令牌、密码、个人信息；设备序列号只在标定需要时出现。
- **链接用相对路径**，图片用 `![说明](./frames/xxx.png)`。
- 语言：正文中文，代码、字段名、命令保持原样。
