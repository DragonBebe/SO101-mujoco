# 用户确认后回到初始位置：脚本生成动作与反馈表

| 步 | 事件 | 指令 | 实际关节（编码器 ticks，ID 1..6） | 实际 TCP | 新帧 | 说明 |
|---|---:|---|---|---|---|---|
| 1 | 256 | `{"action": "observe"}` | 2003, 1829, 2471, 1992, 2151, 2359 | 未直接测量；旧约定FK 322.93,43.98,114.25 mm（辅助） | 0132-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 2 | 257 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 1, "wrist_flex": 4}, "frame": 132}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 3 | 258 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 1, "wrist_flex": 4}, "frame": 132}` | 2003, 1800, 2485, 2037, 2151, 2359 | 未直接测量；旧约定FK 315.09,42.79,110.26 mm（辅助） | 0133-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 4 | 259 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 1, "wrist_flex": 4}, "frame": 133}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 5 | 260 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 1, "wrist_flex": 4}, "frame": 133}` | 2003, 1770, 2499, 2083, 2151, 2359 | 未直接测量；旧约定FK 306.80,41.54,106.40 mm（辅助） | 0134-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 6 | 261 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 1, "wrist_flex": 4}, "frame": 134}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 7 | 262 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 1, "wrist_flex": 4}, "frame": 134}` | 2003, 1742, 2512, 2129, 2151, 2359 | 未直接测量；旧约定FK 298.36,40.26,102.20 mm（辅助） | 0135-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 8 | 263 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 1, "wrist_flex": 3}, "frame": 135}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 9 | 264 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 1, "wrist_flex": 3}, "frame": 135}` | 2003, 1709, 2526, 2163, 2151, 2359 | 未直接测量；旧约定FK 291.37,39.20,101.64 mm（辅助） | 0136-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 10 | 265 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 1.5, "wrist_flex": 3}, "frame": 136}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 11 | 266 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 1.5, "wrist_flex": 3}, "frame": 136}` | 2003, 1675, 2546, 2196, 2151, 2359 | 未直接测量；旧约定FK 283.32,37.98,99.03 mm（辅助） | 0137-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 12 | 267 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 1.5, "wrist_flex": 3}, "frame": 137}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 13 | 268 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 1.5, "wrist_flex": 3}, "frame": 137}` | 2003, 1640, 2567, 2229, 2151, 2359 | 未直接测量；旧约定FK 275.05,36.72,96.16 mm（辅助） | 0138-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 14 | 269 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 2, "wrist_flex": 3}, "frame": 138}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 15 | 270 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 2, "wrist_flex": 3}, "frame": 138}` | 2003, 1604, 2595, 2261, 2151, 2359 | 未直接测量；旧约定FK 265.56,35.29,90.89 mm（辅助） | 0139-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 16 | 271 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 2, "wrist_flex": 3}, "frame": 139}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 17 | 272 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 2, "wrist_flex": 3}, "frame": 139}` | 2003, 1568, 2621, 2295, 2151, 2359 | 未直接测量；旧约定FK 256.11,33.85,85.85 mm（辅助） | 0140-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 18 | 273 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 2, "wrist_flex": 3}, "frame": 140}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 19 | 274 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 2, "wrist_flex": 3}, "frame": 140}` | 2003, 1527, 2648, 2329, 2151, 2359 | 未直接测量；旧约定FK 246.72,32.43,81.68 mm（辅助） | 0141-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 20 | 275 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 2, "wrist_flex": 3}, "frame": 141}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 21 | 276 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 2, "wrist_flex": 3}, "frame": 141}` | 2003, 1485, 2673, 2362, 2151, 2359 | 未直接测量；旧约定FK 238.05,31.12,78.40 mm（辅助） | 0142-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 22 | 277 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 2, "wrist_flex": 3}, "frame": 142}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 23 | 278 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 2, "wrist_flex": 3}, "frame": 142}` | 2003, 1440, 2701, 2395, 2151, 2359 | 未直接测量；旧约定FK 229.09,29.76,74.46 mm（辅助） | 0143-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 24 | 279 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 2, "wrist_flex": 3}, "frame": 143}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 25 | 280 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 2, "wrist_flex": 3}, "frame": 143}` | 2003, 1395, 2727, 2428, 2151, 2359 | 未直接测量；旧约定FK 220.73,28.49,70.89 mm（辅助） | 0144-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 26 | 281 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 2, "wrist_flex": 3}, "frame": 144}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 27 | 282 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 2, "wrist_flex": 3}, "frame": 144}` | 2003, 1348, 2754, 2461, 2151, 2359 | 未直接测量；旧约定FK 212.55,27.25,67.08 mm（辅助） | 0145-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 28 | 283 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 2, "wrist_flex": 3}, "frame": 145}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 29 | 284 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 2, "wrist_flex": 3}, "frame": 145}` | 2003, 1301, 2781, 2493, 2151, 2359 | 未直接测量；旧约定FK 204.84,26.09,63.06 mm（辅助） | 0146-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 30 | 285 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 2, "wrist_flex": 3}, "frame": 146}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 31 | 286 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 2, "wrist_flex": 3}, "frame": 146}` | 2003, 1252, 2806, 2526, 2151, 2359 | 未直接测量；旧约定FK 197.80,25.02,59.66 mm（辅助） | 0147-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 32 | 287 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 2, "wrist_flex": 3}, "frame": 147}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 33 | 288 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 2, "wrist_flex": 3}, "frame": 147}` | 2003, 1201, 2832, 2559, 2151, 2359 | 未直接测量；旧约定FK 191.10,24.00,55.95 mm（辅助） | 0148-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 34 | 289 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 2, "wrist_flex": 3}, "frame": 148}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 35 | 290 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 2, "wrist_flex": 3}, "frame": 148}` | 2003, 1149, 2858, 2592, 2151, 2359 | 未直接测量；旧约定FK 184.90,23.06,52.07 mm（辅助） | 0149-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 36 | 291 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 2.5, "wrist_flex": 3}, "frame": 149}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 37 | 292 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 2.5, "wrist_flex": 3}, "frame": 149}` | 2003, 1097, 2890, 2625, 2151, 2359 | 未直接测量；旧约定FK 178.14,22.04,45.84 mm（辅助） | 0150-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 38 | 293 | `{"action": "joints", "delta_deg": {"shoulder_pan": 1.4, "shoulder_lift": -5, "elbow_flex": 2.5, "wrist_flex": 3}, "frame": 150}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 39 | 294 | `{"action": "joints", "delta_deg": {"shoulder_pan": 1.4, "shoulder_lift": -5, "elbow_flex": 2.5, "wrist_flex": 3}, "frame": 150}` | 2013, 1045, 2922, 2658, 2151, 2359 | 未直接测量；旧约定FK 172.23,19.05,39.39 mm（辅助） | 0151-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 40 | 295 | `{"action": "joints", "delta_deg": {"shoulder_pan": 0.5, "shoulder_lift": -5, "elbow_flex": 2.5, "wrist_flex": 3}, "frame": 151}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 41 | 296 | `{"action": "joints", "delta_deg": {"shoulder_pan": 0.5, "shoulder_lift": -5, "elbow_flex": 2.5, "wrist_flex": 3}, "frame": 151}` | 2013, 991, 2952, 2690, 2151, 2359 | 未直接测量；旧约定FK 167.32,18.39,33.86 mm（辅助） | 0152-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 42 | 297 | `{"action": "joints", "delta_deg": {"shoulder_pan": 0.53, "shoulder_lift": -1.58, "elbow_flex": 0.79, "wrist_flex": 0.97, "wrist_roll": -0.18}, "frame": 152}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 43 | 298 | `{"action": "joints", "delta_deg": {"shoulder_pan": 0.53, "shoulder_lift": -1.58, "elbow_flex": 0.79, "wrist_flex": 0.97, "wrist_roll": -0.18}, "frame": 152}` | 2013, 979, 2958, 2698, 2151, 2359 | 未直接测量；旧约定FK 166.23,18.24,32.71 mm（辅助） | 0153-env.png（未选入关键帧） | 具名目标已回到初始编码器，反馈有小偏差 |
| 44 | 299 | `{"action": "gripper", "ticks": 2440, "frame": 153}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 45 | 300 | `{"action": "gripper", "ticks": 2440, "frame": 153}` | 2013, 979, 2958, 2698, 2151, 2437 | 未直接测量；旧约定FK 166.23,18.24,32.71 mm（辅助） | 0154-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 46 | 301 | `{"action": "gripper", "ticks": 2497, "frame": 154}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 47 | 302 | `{"action": "gripper", "ticks": 2497, "frame": 154}` | 2013, 979, 2958, 2698, 2151, 2494 | 未直接测量；旧约定FK 166.23,18.24,32.71 mm（辅助） | 0155-env.png（未选入关键帧） | 恢复初始夹爪目标2497 |
| 48 | 303 | `{"action": "observe"}` | 2013, 979, 2958, 2698, 2151, 2494 | 未直接测量；旧约定FK 166.23,18.24,32.71 mm（辅助） | [0156-env.png](frames/0156-env.png) | 静置复查，最大偏差6ticks |
