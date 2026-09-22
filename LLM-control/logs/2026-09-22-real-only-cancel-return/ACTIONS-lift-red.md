# 抬起红色方块

TCP 列为辅助 FK 推导毫米值，不是实测 TCP。完整目标和发令前反馈保留于 JSONL。

| 步/帧 | 事件 | 指令 | 实测 ticks（电机1–6） | 辅助 FK mm | 新帧 | 说明 |
|---|---|---|---|---|---|---|
| 4 | 5 | {"action": "joints", "delta_deg": {"elbow_flex": -8, "wrist_flex": -8, "shoulder_pan": -5}, "frame": 3} | [1958, 979, 2895, 2617, 2151, 2495] | [182.47, 19.6, 59.59] | 0004-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 5 | 7 | {"action": "joints", "delta_deg": {"shoulder_lift": 10, "elbow_flex": -10, "wrist_flex": -5, "shoulder_pan": -5}, "frame": 4} | [1905, 1091, 2810, 2570, 2151, 2495] | [190.46, 34.32, 76.74] | 0005-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 6 | 9 | {"action": "joints", "delta_deg": {"shoulder_lift": 10, "elbow_flex": -10, "shoulder_pan": -3}, "frame": 5} | [1873, 1204, 2726, 2570, 2151, 2495] | [192.9, 43.3, 84.7] | 0006-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 7 | 11 | {"action": "joints", "delta_deg": {"elbow_flex": -10, "wrist_flex": -5}, "frame": 6} | [1872, 1204, 2644, 2525, 2152, 2495] | [208.5, 49.28, 122.48] | 0007-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 8 | 13 | {"action": "joints", "delta_deg": {"shoulder_lift": 10, "elbow_flex": -5, "wrist_flex": -5}, "frame": 7} | [1872, 1317, 2620, 2482, 2152, 2495] | [217.55, 52.6, 113.23] | 0008-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 9 | 15 | {"action": "joints", "delta_deg": {"shoulder_lift": 10, "elbow_flex": -5, "wrist_flex": -8, "shoulder_pan": 5}, "frame": 8} | [1923, 1432, 2597, 2401, 2152, 2495] | [236.39, 42.74, 107.19] | 0009-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 10 | 17 | {"action": "joints", "delta_deg": {"shoulder_lift": 10, "elbow_flex": -5, "wrist_flex": -10, "shoulder_pan": 5}, "frame": 9} | [1976, 1547, 2573, 2297, 2153, 2495] | [258.51, 30.31, 102.53] | 0010-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 11 | 19 | {"action": "joints", "delta_deg": {"shoulder_lift": 10, "elbow_flex": -5, "wrist_flex": -10}, "frame": 10} | [1976, 1662, 2549, 2193, 2153, 2495] | [276.97, 33.89, 94.86] | 0011-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 12 | 21 | {"action": "joints", "delta_deg": {"shoulder_lift": 8, "elbow_flex": -4, "wrist_flex": -8}, "frame": 11} | [1976, 1755, 2533, 2113, 2153, 2495] | [289.53, 36.33, 84.49] | 0012-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 13 | 23 | {"action": "joints", "delta_deg": {"shoulder_lift": 5, "wrist_flex": -5}, "frame": 12} | [1976, 1814, 2533, 2065, 2153, 2495] | [294.0, 37.2, 72.71] | 0013-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 14 | 25 | {"action": "joints", "delta_deg": {"shoulder_lift": 5, "elbow_flex": -5, "wrist_flex": -5}, "frame": 13} | [1976, 1874, 2505, 2018, 2153, 2495] | [304.35, 39.21, 70.11] | 0014-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 15 | 27 | {"action": "joints", "delta_deg": {"shoulder_lift": 4, "wrist_flex": -4}, "frame": 14} | [1976, 1924, 2503, 1982, 2153, 2495] | [306.78, 39.68, 58.97] | 0015-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 16 | 29 | {"action": "joints", "delta_deg": {"shoulder_pan": 2}, "frame": 15} | [1994, 1924, 2503, 1982, 2153, 2495] | [307.77, 32.26, 58.97] | 0016-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 17 | 31 | {"action": "joints", "delta_deg": {"shoulder_lift": 3, "elbow_flex": -3, "wrist_flex": -4}, "frame": 16} | [1994, 1961, 2497, 1946, 2153, 2495] | [311.91, 32.95, 53.81] | 0017-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 18 | 33 | {"action": "gripper", "ticks": 2395, "frame": 17} | [1994, 1961, 2497, 1946, 2153, 2396] | [311.91, 32.95, 53.81] | 0018-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 19 | 35 | {"action": "gripper", "ticks": 2336, "frame": 18} | [1994, 1961, 2497, 1946, 2153, 2337] | [311.91, 32.95, 53.81] | 0019-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 20 | 37 | {"action": "gripper", "ticks": 2297, "frame": 19} | [1994, 1961, 2497, 1946, 2153, 2299] | [311.91, 32.95, 53.81] | 0020-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 21 | 39 | {"action": "gripper", "ticks": 2279, "frame": 20} | [1994, 1961, 2497, 1946, 2153, 2284] | [311.91, 32.95, 53.81] | 0021-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 22 | 41 | {"action": "joints", "delta_deg": {"shoulder_lift": -3, "wrist_flex": 3}, "frame": 21} | [1994, 1955, 2497, 1980, 2153, 2284] | [306.81, 32.1, 50.05] | 0022-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 23 | 43 | {"action": "joints", "delta_deg": {"shoulder_lift": -5}, "frame": 22} | [1994, 1924, 2497, 1980, 2153, 2281] | [309.67, 32.58, 61.5] | 0023-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 24 | 45 | {"action": "gripper", "ticks": 2261, "frame": 23} | [1994, 1924, 2497, 1980, 2153, 2275] | [309.67, 32.58, 61.5] | 0024-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 25 | 47 | {"action": "joints", "delta_deg": {"shoulder_lift": -5}, "frame": 24} | [1994, 1896, 2497, 1980, 2153, 2275] | [311.78, 32.93, 71.95] | 0025-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 26 | 49 | {"action": "joints", "delta_deg": {"shoulder_lift": -6, "wrist_flex": 2}, "frame": 25} | [1994, 1857, 2498, 2002, 2153, 2275] | [310.53, 32.72, 81.99] | 0026-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 27 | 51 | {"action": "joints", "delta_deg": {"shoulder_lift": -7}, "frame": 26} | [1994, 1805, 2498, 2002, 2153, 2275] | [312.49, 33.04, 101.47] | 0027-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 28 | 52 | null | [1994, 1805, 2498, 2002, 2153, 2275] | [312.49, 33.04, 101.47] | 0028-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
