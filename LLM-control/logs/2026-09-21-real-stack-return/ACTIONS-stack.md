# 红块跨放绿蓝块：脚本生成动作与反馈表

| 步 | 事件 | 指令 | 实际关节（编码器 ticks，ID 1..6） | 实际 TCP | 新帧 | 说明 |
|---|---:|---|---|---|---|---|
| 1 | 126 | `{"action": "observe"}` | 2015, 977, 2957, 2698, 2151, 2497 | 未直接测量；旧约定FK 166.70,17.90,33.34 mm（辅助） | 0066-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 2 | 127 | `{"action": "joints", "delta_deg": {"elbow_flex": -5, "wrist_flex": -5, "shoulder_pan": -5}, "frame": 66}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 3 | 128 | `{"action": "joints", "delta_deg": {"elbow_flex": -5, "wrist_flex": -5, "shoulder_pan": -5}, "frame": 66}` | 1958, 977, 2927, 2649, 2151, 2497 | 未直接测量；旧约定FK 178.50,32.09,50.29 mm（辅助） | 0067-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 4 | 129 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5, "shoulder_pan": -5}, "frame": 67}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 5 | 130 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5, "shoulder_pan": -5}, "frame": 67}` | 1902, 1033, 2899, 2599, 2151, 2497 | 未直接测量；旧约定FK 182.73,46.32,57.87 mm（辅助） | 0068-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 6 | 131 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5, "shoulder_pan": -3}, "frame": 68}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 7 | 132 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5, "shoulder_pan": -3}, "frame": 68}` | 1871, 1089, 2871, 2551, 2151, 2497 | 未直接测量；旧约定FK 187.83,55.86,64.98 mm（辅助） | 0069-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 8 | 133 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5, "shoulder_pan": -3}, "frame": 69}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 9 | 134 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5, "shoulder_pan": -3}, "frame": 69}` | 1838, 1143, 2843, 2503, 2151, 2497 | 未直接测量；旧约定FK 192.69,66.70,72.29 mm（辅助） | 0070-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 10 | 135 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5, "shoulder_pan": -3}, "frame": 70}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 11 | 136 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5, "shoulder_pan": -3}, "frame": 70}` | 1808, 1199, 2815, 2455, 2151, 2497 | 未直接测量；旧约定FK 197.42,77.54,78.91 mm（辅助） | 0071-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 12 | 137 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5, "shoulder_pan": -2}, "frame": 71}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 13 | 138 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5, "shoulder_pan": -2}, "frame": 71}` | 1788, 1255, 2788, 2408, 2151, 2497 | 未直接测量；旧约定FK 202.87,86.49,84.63 mm（辅助） | 0072-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 14 | 139 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 72}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 15 | 140 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 72}` | 1788, 1309, 2761, 2362, 2151, 2497 | 未直接测量；旧约定FK 211.02,90.74,90.25 mm（辅助） | 0073-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 16 | 141 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 73}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 17 | 142 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 73}` | 1788, 1365, 2735, 2315, 2151, 2497 | 未直接测量；旧约定FK 219.37,95.09,94.72 mm（辅助） | 0074-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 18 | 143 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 74}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 19 | 144 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 74}` | 1787, 1422, 2710, 2269, 2151, 2497 | 未直接测量；旧约定FK 227.49,99.69,97.79 mm（辅助） | 0075-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 20 | 145 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 75}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 21 | 146 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 75}` | 1787, 1479, 2687, 2221, 2151, 2497 | 未直接测量；旧约定FK 235.90,104.09,99.84 mm（辅助） | 0076-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 22 | 147 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 76}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 23 | 148 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 76}` | 1786, 1535, 2661, 2173, 2151, 2497 | 未直接测量；旧约定FK 244.71,109.10,102.78 mm（辅助） | 0077-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 24 | 149 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 77}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 25 | 150 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 77}` | 1784, 1594, 2636, 2125, 2151, 2497 | 未直接测量；旧约定FK 253.19,114.39,103.72 mm（辅助） | 0078-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 26 | 151 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 78}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 27 | 152 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 78}` | 1784, 1651, 2612, 2078, 2151, 2497 | 未直接测量；旧约定FK 261.75,118.92,103.99 mm（辅助） | 0079-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 28 | 153 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 79}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 29 | 154 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 79}` | 1783, 1708, 2589, 2031, 2151, 2497 | 未直接测量；旧约定FK 269.89,123.67,103.16 mm（辅助） | 0080-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 30 | 155 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 80}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 31 | 156 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 80}` | 1782, 1767, 2566, 1986, 2151, 2497 | 未直接测量；旧约定FK 277.61,128.24,100.46 mm（辅助） | 0081-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 32 | 157 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 81}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 33 | 158 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 81}` | 1782, 1827, 2539, 1940, 2151, 2497 | 未直接测量；旧约定FK 286.13,132.78,98.42 mm（辅助） | 0082-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 34 | 159 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 82}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 35 | 160 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 82}` | 1782, 1886, 2513, 1894, 2151, 2497 | 未直接测量；旧约定FK 294.25,137.10,95.57 mm（辅助） | 0083-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 36 | 161 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 83}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 37 | 162 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 83}` | 1782, 1945, 2487, 1848, 2151, 2497 | 未直接测量；旧约定FK 302.06,141.26,91.95 mm（辅助） | 0084-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 38 | 163 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 84}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 39 | 164 | `{"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 84}` | 1782, 2005, 2458, 1801, 2151, 2497 | 未直接测量；旧约定FK 310.12,145.56,88.56 mm（辅助） | 0085-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 40 | 165 | `{"action": "joints", "delta_deg": {"shoulder_lift": 3, "wrist_flex": -3}, "frame": 85}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 41 | 166 | `{"action": "joints", "delta_deg": {"shoulder_lift": 3, "wrist_flex": -3}, "frame": 85}` | 1782, 2041, 2458, 1777, 2151, 2497 | 未直接测量；旧约定FK 310.86,145.96,78.45 mm（辅助） | 0086-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 42 | 167 | `{"action": "joints", "delta_deg": {"shoulder_lift": 3, "wrist_flex": -3}, "frame": 86}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 43 | 168 | `{"action": "joints", "delta_deg": {"shoulder_lift": 3, "wrist_flex": -3}, "frame": 86}` | 1782, 2077, 2457, 1754, 2151, 2497 | 未直接测量；旧约定FK 311.31,146.19,68.41 mm（辅助） | 0087-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 44 | 169 | `{"action": "joints", "delta_deg": {"shoulder_lift": 3, "wrist_flex": -3}, "frame": 87}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 45 | 170 | `{"action": "joints", "delta_deg": {"shoulder_lift": 3, "wrist_flex": -3}, "frame": 87}` | 1782, 2115, 2457, 1731, 2151, 2497 | 未直接测量；旧约定FK 310.97,146.01,57.08 mm（辅助） | 0088-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 46 | 171 | `{"action": "joints", "delta_deg": {"shoulder_lift": 3, "wrist_flex": -3}, "frame": 88}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 47 | 172 | `{"action": "joints", "delta_deg": {"shoulder_lift": 3, "wrist_flex": -3}, "frame": 88}` | 1782, 2153, 2455, 1707, 2151, 2497 | 未直接测量；旧约定FK 310.74,145.89,46.65 mm（辅助） | 0089-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 48 | 173 | `{"action": "joints", "delta_deg": {"shoulder_lift": 2, "wrist_flex": -2}, "frame": 89}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 49 | 174 | `{"action": "joints", "delta_deg": {"shoulder_lift": 2, "wrist_flex": -2}, "frame": 89}` | 1782, 2179, 2453, 1695, 2151, 2497 | 未直接测量；旧约定FK 309.98,145.49,38.78 mm（辅助） | 0090-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 50 | 175 | `{"action": "gripper", "ticks": 2447, "frame": 90}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 51 | 176 | `{"action": "gripper", "ticks": 2447, "frame": 90}` | 1782, 2179, 2453, 1695, 2151, 2450 | 未直接测量；旧约定FK 309.98,145.49,38.78 mm（辅助） | 0091-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 52 | 177 | `{"action": "joints", "delta_deg": {"shoulder_lift": 2, "wrist_flex": -2}, "frame": 91}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 53 | 178 | `{"action": "joints", "delta_deg": {"shoulder_lift": 2, "wrist_flex": -2}, "frame": 91}` | 1782, 2206, 2453, 1683, 2151, 2446 | 未直接测量；旧约定FK 308.38,144.63,29.80 mm（辅助） | 0092-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 54 | 179 | `{"action": "gripper", "ticks": 2390, "frame": 92}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 55 | 180 | `{"action": "gripper", "ticks": 2390, "frame": 92}` | 1782, 2206, 2453, 1683, 2151, 2393 | 未直接测量；旧约定FK 308.38,144.63,29.80 mm（辅助） | 0093-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 56 | 181 | `{"action": "gripper", "ticks": 2350, "frame": 93}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 57 | 182 | `{"action": "gripper", "ticks": 2350, "frame": 93}` | 1782, 2206, 2453, 1683, 2151, 2352 | 未直接测量；旧约定FK 308.38,144.63,29.80 mm（辅助） | 0094-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 58 | 183 | `{"action": "gripper", "ticks": 2315, "frame": 94}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 59 | 184 | `{"action": "gripper", "ticks": 2315, "frame": 94}` | 1782, 2206, 2453, 1683, 2151, 2319 | 未直接测量；旧约定FK 308.38,144.63,29.80 mm（辅助） | [0095-env.png](frames/0095-env.png) | 首次闭爪未成功，红块被推动旋转 |
| 60 | 185 | `{"action": "gripper", "ticks": 2410, "frame": 95}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 61 | 186 | `{"action": "gripper", "ticks": 2410, "frame": 95}` | 1782, 2206, 2453, 1683, 2151, 2406 | 未直接测量；旧约定FK 308.38,144.63,29.80 mm（辅助） | 0096-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 62 | 187 | `{"action": "joints", "delta_deg": {"shoulder_lift": 2.5, "elbow_flex": -5, "wrist_flex": -4}, "frame": 96}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 63 | 188 | `{"action": "joints", "delta_deg": {"shoulder_lift": 2.5, "elbow_flex": -5, "wrist_flex": -4}, "frame": 96}` | 1782, 2238, 2420, 1648, 2151, 2406 | 未直接测量；旧约定FK 317.01,149.23,34.68 mm（辅助） | 0097-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 64 | 189 | `{"action": "gripper", "ticks": 2355, "frame": 97}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 65 | 190 | `{"action": "gripper", "ticks": 2355, "frame": 97}` | 1782, 2238, 2420, 1648, 2151, 2358 | 未直接测量；旧约定FK 317.01,149.23,34.68 mm（辅助） | 0098-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 66 | 191 | `{"action": "gripper", "ticks": 2315, "frame": 98}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 67 | 192 | `{"action": "gripper", "ticks": 2315, "frame": 98}` | 1782, 2238, 2420, 1648, 2151, 2319 | 未直接测量；旧约定FK 317.01,149.23,34.68 mm（辅助） | 0099-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 68 | 193 | `{"action": "gripper", "ticks": 2295, "frame": 99}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 69 | 194 | `{"action": "gripper", "ticks": 2295, "frame": 99}` | 1782, 2238, 2420, 1648, 2151, 2298 | 未直接测量；旧约定FK 317.01,149.23,34.68 mm（辅助） | 0100-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 70 | 195 | `{"action": "gripper", "ticks": 2275, "frame": 100}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 71 | 196 | `{"action": "gripper", "ticks": 2275, "frame": 100}` | 1782, 2238, 2420, 1648, 2151, 2281 | 未直接测量；旧约定FK 317.01,149.23,34.68 mm（辅助） | 0101-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 72 | 197 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5}, "frame": 101}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 73 | 198 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5}, "frame": 101}` | 1782, 2215, 2420, 1648, 2151, 2280 | 未直接测量；旧约定FK 319.41,150.51,44.80 mm（辅助） | [0102-env.png](frames/0102-env.png) | 试提未确认离桌，随后降低并重新夹紧 |
| 74 | 199 | `{"action": "joints", "delta_deg": {"shoulder_lift": 2}, "frame": 102}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 75 | 200 | `{"action": "joints", "delta_deg": {"shoulder_lift": 2}, "frame": 102}` | 1782, 2241, 2418, 1646, 2149, 2280 | 未直接测量；旧约定FK 317.41,149.40,34.49 mm（辅助） | 0103-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 76 | 201 | `{"action": "gripper", "ticks": 2255, "frame": 103}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 77 | 202 | `{"action": "gripper", "ticks": 2255, "frame": 103}` | 1782, 2241, 2418, 1646, 2149, 2273 | 未直接测量；旧约定FK 317.41,149.40,34.49 mm（辅助） | 0104-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 78 | 203 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5}, "frame": 104}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 79 | 204 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5}, "frame": 104}` | 1782, 2219, 2418, 1646, 2151, 2273 | 未直接测量；旧约定FK 319.69,150.66,44.18 mm（辅助） | [0105-env.png](frames/0105-env.png) | 两路图像确认红块提起 |
| 80 | 205 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 2}, "frame": 105}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 81 | 206 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "wrist_flex": 2}, "frame": 105}` | 1782, 2199, 2418, 1668, 2151, 2273 | 未直接测量；旧约定FK 319.35,150.47,48.27 mm（辅助） | 0106-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 82 | 207 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": -5, "wrist_flex": 2}, "frame": 106}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 83 | 208 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": -5, "wrist_flex": 2}, "frame": 106}` | 1782, 2180, 2391, 1690, 2151, 2273 | 未直接测量；旧约定FK 324.81,153.38,61.92 mm（辅助） | 0107-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 84 | 209 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": -5, "wrist_flex": 2}, "frame": 107}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 85 | 210 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": -5, "wrist_flex": 2}, "frame": 107}` | 1782, 2164, 2364, 1712, 2151, 2273 | 未直接测量；旧约定FK 329.45,155.86,74.63 mm（辅助） | 0108-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 86 | 211 | `{"action": "joints", "delta_deg": {"shoulder_pan": 4, "shoulder_lift": -5, "wrist_flex": 1}, "frame": 108}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 87 | 212 | `{"action": "joints", "delta_deg": {"shoulder_pan": 4, "shoulder_lift": -5, "wrist_flex": 1}, "frame": 108}` | 1822, 2152, 2364, 1723, 2151, 2273 | 未直接测量；旧约定FK 338.13,137.58,77.69 mm（辅助） | 0109-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 88 | 213 | `{"action": "joints", "delta_deg": {"shoulder_pan": 5, "shoulder_lift": -5}, "frame": 109}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 89 | 214 | `{"action": "joints", "delta_deg": {"shoulder_pan": 5, "shoulder_lift": -5}, "frame": 109}` | 1872, 2136, 2364, 1723, 2151, 2273 | 未直接测量；旧约定FK 348.60,114.54,85.04 mm（辅助） | 0110-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 90 | 215 | `{"action": "joints", "delta_deg": {"shoulder_pan": 5, "shoulder_lift": -5}, "frame": 110}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 91 | 216 | `{"action": "joints", "delta_deg": {"shoulder_pan": 5, "shoulder_lift": -5}, "frame": 110}` | 1923, 2122, 2364, 1723, 2151, 2273 | 未直接测量；旧约定FK 357.20,90.14,91.48 mm（辅助） | 0111-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 92 | 217 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 3, "wrist_flex": 3}, "frame": 111}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 93 | 218 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 3, "wrist_flex": 3}, "frame": 111}` | 1924, 2104, 2400, 1758, 2151, 2273 | 未直接测量；旧约定FK 347.30,86.85,77.62 mm（辅助） | 0112-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 94 | 219 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": -3}, "frame": 112}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 95 | 220 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": -3}, "frame": 112}` | 1924, 2088, 2401, 1759, 2151, 2273 | 未直接测量；旧约定FK 347.82,87.00,84.13 mm（辅助） | 0113-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 96 | 221 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": -5}, "frame": 113}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 97 | 222 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": -5}, "frame": 113}` | 1924, 2076, 2378, 1759, 2151, 2273 | 未直接测量；旧约定FK 352.91,88.42,98.71 mm（辅助） | 0114-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 98 | 223 | `{"action": "joints", "delta_deg": {"shoulder_pan": 5, "shoulder_lift": -5}, "frame": 114}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 99 | 224 | `{"action": "joints", "delta_deg": {"shoulder_pan": 5, "shoulder_lift": -5}, "frame": 114}` | 1976, 2060, 2377, 1759, 2151, 2273 | 未直接测量；旧约定FK 359.48,63.21,106.39 mm（辅助） | 0115-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 100 | 225 | `{"action": "joints", "delta_deg": {"shoulder_pan": 5, "shoulder_lift": -5, "wrist_flex": 2}, "frame": 115}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 101 | 226 | `{"action": "joints", "delta_deg": {"shoulder_pan": 5, "shoulder_lift": -5, "wrist_flex": 2}, "frame": 115}` | 2027, 2048, 2376, 1782, 2151, 2273 | 未直接测量；旧约定FK 361.81,37.76,106.98 mm（辅助） | 0116-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 102 | 227 | `{"action": "joints", "delta_deg": {"shoulder_pan": 1.5, "shoulder_lift": -5, "elbow_flex": 2, "wrist_flex": 3}, "frame": 116}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 103 | 228 | `{"action": "joints", "delta_deg": {"shoulder_pan": 1.5, "shoulder_lift": -5, "elbow_flex": 2, "wrist_flex": 3}, "frame": 116}` | 2038, 2032, 2402, 1816, 2151, 2273 | 未直接测量；旧约定FK 354.39,31.53,95.91 mm（辅助） | 0117-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 104 | 229 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 1, "wrist_flex": 2}, "frame": 117}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 105 | 230 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 1, "wrist_flex": 2}, "frame": 117}` | 2038, 2013, 2417, 1837, 2151, 2273 | 未直接测量；旧约定FK 349.67,31.07,93.56 mm（辅助） | 0118-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 106 | 231 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 1, "wrist_flex": 2}, "frame": 118}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 107 | 232 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 1, "wrist_flex": 2}, "frame": 118}` | 2039, 1992, 2431, 1859, 2151, 2273 | 未直接测量；旧约定FK 345.06,30.15,92.19 mm（辅助） | 0119-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 108 | 233 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 1, "wrist_flex": 2}, "frame": 119}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 109 | 234 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 1, "wrist_flex": 2}, "frame": 119}` | 2039, 1972, 2445, 1881, 2151, 2273 | 未直接测量；旧约定FK 340.25,29.69,90.34 mm（辅助） | 0120-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 110 | 235 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 1, "wrist_flex": 2}, "frame": 120}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 111 | 236 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 1, "wrist_flex": 2}, "frame": 120}` | 2039, 1949, 2461, 1904, 2151, 2273 | 未直接测量；旧约定FK 334.90,29.18,88.66 mm（辅助） | 0121-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 112 | 237 | `{"action": "joints", "delta_deg": {"shoulder_pan": -1, "shoulder_lift": -5, "elbow_flex": 1, "wrist_flex": 2}, "frame": 121}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 113 | 238 | `{"action": "joints", "delta_deg": {"shoulder_pan": -1, "shoulder_lift": -5, "elbow_flex": 1, "wrist_flex": 2}, "frame": 121}` | 2031, 1928, 2475, 1927, 2151, 2273 | 未直接测量；旧约定FK 329.42,32.26,86.87 mm（辅助） | 0122-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 114 | 239 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 1, "wrist_flex": 2}, "frame": 122}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 115 | 240 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": 1, "wrist_flex": 2}, "frame": 122}` | 2029, 1905, 2489, 1948, 2151, 2273 | 未直接测量；旧约定FK 324.48,32.62,86.22 mm（辅助） | 0123-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 116 | 241 | `{"action": "joints", "delta_deg": {"shoulder_pan": -1.5, "shoulder_lift": -5, "elbow_flex": 1, "wrist_flex": 2}, "frame": 123}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 117 | 242 | `{"action": "joints", "delta_deg": {"shoulder_pan": -1.5, "shoulder_lift": -5, "elbow_flex": 1, "wrist_flex": 2}, "frame": 123}` | 2014, 1882, 2504, 1970, 2151, 2273 | 未直接测量；旧约定FK 318.40,38.47,84.88 mm（辅助） | 0124-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 118 | 243 | `{"action": "joints", "delta_deg": {"shoulder_pan": -1, "shoulder_lift": -5, "elbow_flex": 1, "wrist_flex": 2}, "frame": 124}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 119 | 244 | `{"action": "joints", "delta_deg": {"shoulder_pan": -1, "shoulder_lift": -5, "elbow_flex": 1, "wrist_flex": 2}, "frame": 124}` | 2005, 1859, 2518, 1992, 2151, 2273 | 未直接测量；旧约定FK 312.75,41.58,83.83 mm（辅助） | 0125-env.png（未选入关键帧） | 实测反馈；观察后再决策 |
| 120 | 245 | `{"action": "joints", "delta_deg": {"shoulder_lift": 2}, "frame": 125}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 121 | 246 | `{"action": "joints", "delta_deg": {"shoulder_lift": 2}, "frame": 125}` | 2004, 1884, 2518, 1992, 2151, 2273 | 未直接测量；旧约定FK 311.26,41.78,74.39 mm（辅助） | [0126-env.png](frames/0126-env.png) | 下降后红块转为平正 |
| 122 | 247 | `{"action": "gripper", "ticks": 2305, "frame": 126}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 123 | 248 | `{"action": "gripper", "ticks": 2305, "frame": 126}` | 2004, 1884, 2518, 1992, 2151, 2301 | 未直接测量；旧约定FK 311.26,41.78,74.39 mm（辅助） | 0127-env.png（未选入关键帧） | 逐步松爪 |
| 124 | 249 | `{"action": "gripper", "ticks": 2360, "frame": 127}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 125 | 250 | `{"action": "gripper", "ticks": 2360, "frame": 127}` | 2004, 1884, 2518, 1992, 2151, 2359 | 未直接测量；旧约定FK 311.26,41.78,74.39 mm（辅助） | 0128-env.png（未选入关键帧） | 张开后红块保持 |
| 126 | 251 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": -5}, "frame": 128}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 127 | 252 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": -5}, "frame": 128}` | 2003, 1858, 2494, 1992, 2151, 2359 | 未直接测量；旧约定FK 317.90,43.22,93.62 mm（辅助） | 0129-env.png（未选入关键帧） | 抬离 |
| 128 | 253 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": -5}, "frame": 129}` | 未读取（发令记录） | 无 | — | 目标不等于反馈 |
| 129 | 254 | `{"action": "joints", "delta_deg": {"shoulder_lift": -5, "elbow_flex": -5}, "frame": 129}` | 2003, 1829, 2471, 1992, 2151, 2359 | 未直接测量；旧约定FK 322.93,43.98,114.25 mm（辅助） | 0130-env.png（未选入关键帧） | 继续抬离 |
| 130 | 255 | `{"action": "observe"}` | 2003, 1829, 2471, 1992, 2151, 2359 | 未直接测量；旧约定FK 322.93,43.98,114.25 mm（辅助） | [0131-env.png](frames/0131-env.png) | 释放约42秒后稳定 |
