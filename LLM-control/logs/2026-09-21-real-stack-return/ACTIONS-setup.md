# 连接及首次抓取暂停：脚本生成动作与反馈表

| 步 | 事件 | 指令 | 实际关节（编码器 ticks，ID 1..6） | 实际 TCP | 新帧 | 说明 |
|---|---:|---|---|---|---|---|
| 1 | 1 | `"connected; no torque/motion command sent"` | 2019, 973, 2961, 2701, 2149, 2497 | 未直接测量；旧约定FK 166.07,16.99,32.26 mm（辅助） | [0001-env.png](frames/0001-env.png) | 实测反馈；观察后再决策 |
| 2 | 2 | `{"action": "hold", "frame": 1}` | 2019, 973, 2961, 2701, 2149, 2497 | 未直接测量；旧约定FK 166.07,16.99,32.26 mm（辅助） | 0002-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 3 | 3 | `{"action": "joints", "delta_deg": {"elbow_flex": -5}, "frame": 2}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 4 | 4 | `{"action": "joints", "delta_deg": {"elbow_flex": -5}, "frame": 2}` | 2019, 973, 2928, 2701, 2149, 2497 | 未直接测量；旧约定FK 170.96,17.61,43.14 mm（辅助） | 0003-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 5 | 5 | `{"action": "joints", "delta_deg": {"elbow_flex": -5, "shoulder_pan": -5}, "frame": 3}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 6 | 6 | `{"action": "joints", "delta_deg": {"elbow_flex": -5, "shoulder_pan": -5}, "frame": 3}` | 1964, 973, 2898, 2701, 2149, 2497 | 未直接测量；旧约定FK 172.91,29.51,53.25 mm（辅助） | 0004-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 7 | 7 | `{"action": "joints", "delta_deg": {"wrist_flex": -5, "shoulder_pan": -5}, "frame": 4}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 8 | 8 | `{"action": "joints", "delta_deg": {"wrist_flex": -5, "shoulder_pan": -5}, "frame": 4}` | 1907, 973, 2898, 2652, 2149, 2497 | 未直接测量；旧约定FK 178.79,43.86,60.73 mm（辅助） | 0005-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 9 | 9 | `{"action": "joints", "delta_deg": {"wrist_flex": -5, "shoulder_lift": 5}, "frame": 5}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 10 | 10 | `{"action": "joints", "delta_deg": {"wrist_flex": -5, "shoulder_lift": 5}, "frame": 5}` | 1907, 1028, 2898, 2601, 2149, 2497 | 未直接测量；旧约定FK 183.29,45.24,58.84 mm（辅助） | 0006-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 11 | 11 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 6}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 12 | 12 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 6}` | 1907, 1083, 2871, 2552, 2149, 2497 | 未直接测量；旧约定FK 190.98,47.60,66.00 mm（辅助） | 0007-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 13 | 13 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 7}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 14 | 14 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 7}` | 1907, 1138, 2843, 2505, 2149, 2497 | 未直接测量；旧约定FK 198.87,50.02,73.00 mm（辅助） | 0008-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 15 | 15 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 8}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 16 | 16 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 8}` | 1907, 1194, 2815, 2457, 2149, 2497 | 未直接测量；旧约定FK 207.28,52.60,79.69 mm（辅助） | 0009-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 17 | 17 | `{"action": "observe"}` | 1906, 1194, 2815, 2457, 2149, 2497 | 未直接测量；旧约定FK 207.20,52.86,79.69 mm（辅助） | 0010-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 18 | 18 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5, "shoulder_pan": -3}, "frame": 10}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 19 | 19 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5, "shoulder_pan": -3}, "frame": 10}` | 1876, 1250, 2790, 2410, 2149, 2497 | 未直接测量；旧约定FK 212.67,63.46,84.69 mm（辅助） | 0011-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 20 | 20 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5, "shoulder_pan": -3}, "frame": 11}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 21 | 21 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5, "shoulder_pan": -3}, "frame": 11}` | 1842, 1306, 2765, 2362, 2149, 2497 | 未直接测量；旧约定FK 217.51,75.96,89.45 mm（辅助） | 0012-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 22 | 22 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5, "shoulder_pan": -2}, "frame": 12}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 23 | 23 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5, "shoulder_pan": -2}, "frame": 12}` | 1822, 1362, 2741, 2315, 2149, 2497 | 未直接测量；旧约定FK 223.34,85.17,93.15 mm（辅助） | 0013-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 24 | 24 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 13}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 25 | 25 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 13}` | 1821, 1420, 2716, 2269, 2149, 2497 | 未直接测量；旧约定FK 231.61,89.30,95.96 mm（辅助） | 0014-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 26 | 26 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 14}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 27 | 27 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 14}` | 1821, 1477, 2691, 2221, 2149, 2497 | 未直接测量；旧约定FK 240.51,93.37,98.83 mm（辅助） | 0015-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 28 | 28 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 15}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 29 | 29 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 15}` | 1821, 1533, 2667, 2173, 2149, 2497 | 未直接测量；旧约定FK 249.38,97.44,100.98 mm（辅助） | 0016-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 30 | 30 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 16}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 31 | 31 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 16}` | 1821, 1592, 2643, 2126, 2149, 2497 | 未直接测量；旧约定FK 258.11,101.44,101.34 mm（辅助） | 0017-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 32 | 32 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 17}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 33 | 33 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 17}` | 1821, 1649, 2618, 2078, 2149, 2497 | 未直接测量；旧约定FK 267.18,105.59,102.25 mm（辅助） | 0018-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 34 | 34 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 18}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 35 | 35 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 18}` | 1821, 1706, 2594, 2031, 2149, 2497 | 未直接测量；旧约定FK 275.89,109.58,101.85 mm（辅助） | 0019-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 36 | 36 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5, "shoulder_pan": -3}, "frame": 19}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 37 | 37 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5, "shoulder_pan": -3}, "frame": 19}` | 1791, 1765, 2569, 1982, 2149, 2497 | 未直接测量；旧约定FK 279.34,124.88,100.83 mm（辅助） | 0020-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 38 | 38 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 20}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 39 | 39 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 20}` | 1787, 1824, 2543, 1934, 2149, 2497 | 未直接测量；旧约定FK 287.16,130.85,99.24 mm（辅助） | 0021-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 40 | 40 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 21}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 41 | 41 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 21}` | 1786, 1884, 2519, 1887, 2149, 2497 | 未直接测量；旧约定FK 294.78,135.33,95.47 mm（辅助） | 0022-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 42 | 42 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 22}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 43 | 43 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 22}` | 1786, 1944, 2495, 1842, 2149, 2497 | 未直接测量；旧约定FK 302.05,139.15,90.49 mm（辅助） | 0023-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 44 | 44 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 23}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 45 | 45 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 23}` | 1786, 2003, 2470, 1795, 2149, 2497 | 未直接测量；旧约定FK 309.35,142.98,86.00 mm（辅助） | 0024-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 46 | 46 | `{"action": "joints", "delta_deg": {"shoulder_lift": 3, "wrist_flex": -3}, "frame": 24}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 47 | 47 | `{"action": "joints", "delta_deg": {"shoulder_lift": 3, "wrist_flex": -3}, "frame": 24}` | 1786, 2040, 2466, 1771, 2149, 2497 | 未直接测量；旧约定FK 310.78,143.73,77.09 mm（辅助） | 0025-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 48 | 48 | `{"action": "joints", "delta_deg": {"shoulder_lift": 3, "wrist_flex": -3}, "frame": 25}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 49 | 49 | `{"action": "joints", "delta_deg": {"shoulder_lift": 3, "wrist_flex": -3}, "frame": 25}` | 1786, 2079, 2465, 1748, 2149, 2497 | 未直接测量；旧约定FK 310.97,143.83,65.84 mm（辅助） | 0026-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 50 | 50 | `{"action": "joints", "delta_deg": {"shoulder_lift": 3, "wrist_flex": -3}, "frame": 26}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 51 | 51 | `{"action": "joints", "delta_deg": {"shoulder_lift": 3, "wrist_flex": -3}, "frame": 26}` | 1786, 2116, 2465, 1725, 2151, 2497 | 未直接测量；旧约定FK 310.59,143.67,54.99 mm（辅助） | 0027-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 52 | 52 | `{"action": "joints", "delta_deg": {"shoulder_lift": 3, "wrist_flex": -3}, "frame": 27}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 53 | 53 | `{"action": "joints", "delta_deg": {"shoulder_lift": 3, "wrist_flex": -3}, "frame": 27}` | 1786, 2154, 2465, 1701, 2151, 2497 | 未直接测量；旧约定FK 309.79,143.25,43.92 mm（辅助） | 0028-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 54 | 54 | `{"action": "joints", "delta_deg": {"shoulder_lift": 2, "wrist_flex": -3}, "frame": 28}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 55 | 55 | `{"action": "joints", "delta_deg": {"shoulder_lift": 2, "wrist_flex": -3}, "frame": 28}` | 1786, 2180, 2465, 1677, 2151, 2497 | 未直接测量；旧约定FK 309.84,143.28,37.91 mm（辅助） | 0029-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 56 | 56 | `{"action": "observe"}` | 1786, 2180, 2465, 1677, 2151, 2497 | 未直接测量；旧约定FK 309.84,143.28,37.91 mm（辅助） | [0030-env.png](frames/0030-env.png) | 实测反馈；观察后再决策 |
