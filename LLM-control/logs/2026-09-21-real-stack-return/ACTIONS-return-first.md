# 朝向检查后首次回位：脚本生成动作与反馈表

| 步 | 事件 | 指令 | 实际关节（编码器 ticks，ID 1..6） | 实际 TCP | 新帧 | 说明 |
|---|---:|---|---|---|---|---|
| 1 | 57 | `{"action": "joints", "delta_deg": {"shoulder_lift": -3, "wrist_flex": 2}, "frame": 30}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 2 | 58 | `{"action": "joints", "delta_deg": {"shoulder_lift": -3, "wrist_flex": 2}, "frame": 30}` | 1786, 2175, 2465, 1699, 2151, 2497 | 未直接测量；旧约定FK 307.83,142.22,35.47 mm（辅助） | 0031-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 3 | 59 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5}, "frame": 31}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 4 | 60 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5}, "frame": 31}` | 1786, 2150, 2461, 1699, 2151, 2497 | 未直接测量；旧约定FK 311.36,144.08,47.46 mm（辅助） | 0032-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 5 | 61 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5}, "frame": 32}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 6 | 62 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5}, "frame": 32}` | 1786, 2124, 2461, 1699, 2151, 2497 | 未直接测量；旧约定FK 313.61,145.26,58.60 mm（辅助） | 0033-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 7 | 63 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5}, "frame": 33}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 8 | 64 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5}, "frame": 33}` | 1785, 2098, 2461, 1699, 2151, 2497 | 未直接测量；旧约定FK 315.24,146.65,69.83 mm（辅助） | 0034-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 9 | 65 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 3, "elbow_flex": 1, "shoulder_pan": 3}, "frame": 34}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 10 | 66 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 3, "elbow_flex": 1, "shoulder_pan": 3}, "frame": 34}` | 1812, 2075, 2475, 1732, 2151, 2497 | 未直接测量；旧约定FK 316.20,132.76,67.22 mm（辅助） | 0035-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 11 | 67 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 1, "elbow_flex": 1, "shoulder_pan": 3}, "frame": 35}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 12 | 68 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 1, "elbow_flex": 1, "shoulder_pan": 3}, "frame": 35}` | 1842, 2051, 2490, 1742, 2151, 2497 | 未直接测量；旧约定FK 319.19,118.67,69.55 mm（辅助） | 0036-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 13 | 69 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 1}, "frame": 36}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 14 | 70 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 1}, "frame": 36}` | 1842, 2027, 2490, 1752, 2151, 2497 | 未直接测量；旧约定FK 319.60,118.85,77.48 mm（辅助） | 0037-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 15 | 71 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 1, "elbow_flex": 1, "shoulder_pan": 3}, "frame": 37}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 16 | 72 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 1, "elbow_flex": 1, "shoulder_pan": 3}, "frame": 37}` | 1871, 2002, 2503, 1762, 2151, 2497 | 未直接测量；旧约定FK 322.09,105.31,80.75 mm（辅助） | 0038-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 17 | 73 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 2, "elbow_flex": 1, "shoulder_pan": 3}, "frame": 38}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 18 | 74 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 2, "elbow_flex": 1, "shoulder_pan": 3}, "frame": 38}` | 1899, 1977, 2518, 1785, 2151, 2497 | 未直接测量；旧约定FK 321.90,91.62,80.28 mm（辅助） | 0039-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 19 | 75 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 2, "elbow_flex": 1, "shoulder_pan": 3}, "frame": 39}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 20 | 76 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 2, "elbow_flex": 1, "shoulder_pan": 3}, "frame": 39}` | 1927, 1951, 2532, 1808, 2151, 2497 | 未直接测量；旧约定FK 321.24,78.20,80.47 mm（辅助） | 0040-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 21 | 77 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 2, "elbow_flex": 1, "shoulder_pan": 3}, "frame": 40}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 22 | 78 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 2, "elbow_flex": 1, "shoulder_pan": 3}, "frame": 40}` | 1956, 1923, 2546, 1829, 2151, 2497 | 未直接测量；旧约定FK 320.31,64.62,81.74 mm（辅助） | 0041-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 23 | 79 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 2, "elbow_flex": 1, "shoulder_pan": 3}, "frame": 41}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 24 | 80 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 2, "elbow_flex": 1, "shoulder_pan": 3}, "frame": 41}` | 1986, 1900, 2561, 1851, 2151, 2497 | 未直接测量；旧约定FK 318.13,50.74,80.29 mm（辅助） | 0042-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 25 | 81 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 2, "elbow_flex": 1, "shoulder_pan": 2.9}, "frame": 42}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 26 | 82 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 2, "elbow_flex": 1, "shoulder_pan": 2.9}, "frame": 42}` | 2014, 1870, 2577, 1873, 2151, 2497 | 未直接测量；旧约定FK 315.23,38.05,81.03 mm（辅助） | 0043-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 27 | 83 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 3, "elbow_flex": 1}, "frame": 43}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 28 | 84 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 3, "elbow_flex": 1}, "frame": 43}` | 2014, 1840, 2590, 1906, 2151, 2497 | 未直接测量；旧约定FK 309.61,37.29,80.46 mm（辅助） | 0044-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 29 | 85 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 3, "elbow_flex": 1}, "frame": 44}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 30 | 86 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 3, "elbow_flex": 1}, "frame": 44}` | 2014, 1808, 2604, 1940, 2151, 2497 | 未直接测量；旧约定FK 303.59,36.49,79.88 mm（辅助） | 0045-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 31 | 87 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 3, "elbow_flex": 1}, "frame": 45}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 32 | 88 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 3, "elbow_flex": 1}, "frame": 45}` | 2014, 1773, 2618, 1972, 2151, 2497 | 未直接测量；旧约定FK 297.88,35.72,80.58 mm（辅助） | 0046-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 33 | 89 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 3, "elbow_flex": 1}, "frame": 46}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 34 | 90 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 3, "elbow_flex": 1}, "frame": 46}` | 2014, 1738, 2633, 2004, 2151, 2497 | 未直接测量；旧约定FK 291.85,34.91,80.66 mm（辅助） | 0047-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 35 | 91 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 3, "elbow_flex": 1}, "frame": 47}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 36 | 92 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 3, "elbow_flex": 1}, "frame": 47}` | 2014, 1702, 2648, 2037, 2151, 2497 | 未直接测量；旧约定FK 285.65,34.08,80.63 mm（辅助） | 0048-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 37 | 93 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 3, "elbow_flex": 1}, "frame": 48}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 38 | 94 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 3, "elbow_flex": 1}, "frame": 48}` | 2014, 1663, 2661, 2069, 2151, 2497 | 未直接测量；旧约定FK 280.12,33.33,82.30 mm（辅助） | 0049-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 39 | 95 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 3, "elbow_flex": 1.5}, "frame": 49}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 40 | 96 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 3, "elbow_flex": 1.5}, "frame": 49}` | 2014, 1623, 2683, 2102, 2151, 2497 | 未直接测量；旧约定FK 272.50,32.31,80.29 mm（辅助） | 0050-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 41 | 97 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 3, "elbow_flex": 1.5}, "frame": 50}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 42 | 98 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 3, "elbow_flex": 1.5}, "frame": 50}` | 2014, 1582, 2704, 2135, 2151, 2497 | 未直接测量；旧约定FK 265.16,31.33,78.65 mm（辅助） | 0051-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 43 | 99 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 3, "elbow_flex": 1.5}, "frame": 51}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 44 | 100 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 3, "elbow_flex": 1.5}, "frame": 51}` | 2014, 1541, 2724, 2167, 2151, 2497 | 未直接测量；旧约定FK 258.24,30.40,77.25 mm（辅助） | 0052-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 45 | 101 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 3, "elbow_flex": 1.5}, "frame": 52}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 46 | 102 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 3, "elbow_flex": 1.5}, "frame": 52}` | 2014, 1497, 2746, 2199, 2151, 2497 | 未直接测量；旧约定FK 251.16,29.44,75.60 mm（辅助） | 0053-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 47 | 103 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 4, "elbow_flex": 1.5}, "frame": 53}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 48 | 104 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 4, "elbow_flex": 1.5}, "frame": 53}` | 2014, 1452, 2767, 2244, 2151, 2497 | 未直接测量；旧约定FK 242.29,28.25,71.92 mm（辅助） | 0054-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 49 | 105 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 4, "elbow_flex": 1.5}, "frame": 54}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 50 | 106 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 4, "elbow_flex": 1.5}, "frame": 54}` | 2014, 1406, 2787, 2289, 2151, 2497 | 未直接测量；旧约定FK 233.75,27.11,68.59 mm（辅助） | 0055-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 51 | 107 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 4, "elbow_flex": 1.5}, "frame": 55}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 52 | 108 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 4, "elbow_flex": 1.5}, "frame": 55}` | 2014, 1359, 2807, 2334, 2151, 2497 | 未直接测量；旧约定FK 225.37,25.98,65.20 mm（辅助） | 0056-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 53 | 109 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 4, "elbow_flex": 1.5}, "frame": 56}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 54 | 110 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 4, "elbow_flex": 1.5}, "frame": 56}` | 2014, 1311, 2828, 2378, 2151, 2497 | 未直接测量；旧约定FK 217.19,24.88,61.53 mm（辅助） | 0057-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 55 | 111 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 4, "elbow_flex": 1.5}, "frame": 57}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 56 | 112 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 4, "elbow_flex": 1.5}, "frame": 57}` | 2014, 1261, 2848, 2422, 2151, 2497 | 未直接测量；旧约定FK 209.56,23.86,58.37 mm（辅助） | 0058-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 57 | 113 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 4, "elbow_flex": 1.5}, "frame": 58}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 58 | 114 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 4, "elbow_flex": 1.5}, "frame": 58}` | 2014, 1212, 2869, 2467, 2151, 2497 | 未直接测量；旧约定FK 201.69,22.80,54.16 mm（辅助） | 0059-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 59 | 115 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 4, "elbow_flex": 1.3}, "frame": 59}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 60 | 116 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 4, "elbow_flex": 1.3}, "frame": 59}` | 2014, 1161, 2886, 2512, 2151, 2497 | 未直接测量；旧约定FK 194.98,21.90,51.51 mm（辅助） | 0060-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 61 | 117 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 4, "elbow_flex": 1.3}, "frame": 60}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 62 | 118 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 4, "elbow_flex": 1.3}, "frame": 60}` | 2014, 1110, 2904, 2558, 2151, 2497 | 未直接测量；旧约定FK 188.15,20.98,48.06 mm（辅助） | 0061-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 63 | 119 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 4, "elbow_flex": 1.3}, "frame": 61}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 64 | 120 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 4, "elbow_flex": 1.3}, "frame": 61}` | 2014, 1057, 2922, 2603, 2151, 2497 | 未直接测量；旧约定FK 182.01,20.16,44.84 mm（辅助） | 0062-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 65 | 121 | `{"action": "joints", "delta_deg": {"shoulder_lift": -4, "wrist_flex": 4, "elbow_flex": 1.3}, "frame": 62}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 66 | 122 | `{"action": "joints", "delta_deg": {"shoulder_lift": -4, "wrist_flex": 4, "elbow_flex": 1.3}, "frame": 62}` | 2014, 1015, 2939, 2648, 2151, 2497 | 未直接测量；旧约定FK 175.09,19.23,39.89 mm（辅助） | 0063-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 67 | 123 | `{"action": "joints", "delta_deg": {"shoulder_lift": -3.69, "wrist_flex": 4.66, "elbow_flex": 1.67, "shoulder_pan": 0.44, "wrist_roll": -0.176}, "frame": 63}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 68 | 124 | `{"action": "joints", "delta_deg": {"shoulder_lift": -3.69, "wrist_flex": 4.66, "elbow_flex": 1.67, "shoulder_pan": 0.44, "wrist_roll": -0.176}, "frame": 63}` | 2014, 977, 2957, 2698, 2151, 2497 | 未直接测量；旧约定FK 166.67,18.10,33.34 mm（辅助） | 0064-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 69 | 125 | `{"action": "observe"}` | 2015, 977, 2957, 2698, 2151, 2497 | 未直接测量；旧约定FK 166.70,17.90,33.34 mm（辅助） | [0065-env.png](frames/0065-env.png) | 实测反馈；观察后再决策 |
