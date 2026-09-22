# 红块放桌、绿块堆叠（用户终止）

TCP 列为辅助 FK 推导毫米值，不是实测 TCP。完整目标和发令前反馈保留于 JSONL。

| 步/帧 | 事件 | 指令 | 实测 ticks（电机1–6） | 辅助 FK mm | 新帧 | 说明 |
|---|---|---|---|---|---|---|
| 29 | 53 | null | [1994, 1805, 2498, 2002, 2153, 2275] | [312.49, 33.04, 101.47] | 0029-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 30 | 55 | {"action": "joints", "delta_deg": {"shoulder_pan": 10}, "frame": 29} | [2103, 1805, 2498, 2002, 2153, 2275] | [314.17, -12.97, 101.47] | 0030-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 31 | 57 | {"action": "joints", "delta_deg": {"shoulder_pan": 8}, "frame": 30} | [2189, 1805, 2498, 2002, 2153, 2275] | [310.07, -49.08, 101.47] | 0031-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 32 | 59 | {"action": "joints", "delta_deg": {"shoulder_lift": 7, "wrist_flex": -3}, "frame": 31} | [2189, 1889, 2498, 1980, 2153, 2275] | [309.58, -49.02, 74.2] | 0032-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 33 | 61 | {"action": "joints", "delta_deg": {"shoulder_lift": 6, "wrist_flex": -3}, "frame": 32} | [2189, 1961, 2498, 1958, 2153, 2275] | [307.22, -48.7, 51.32] | 0033-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 34 | 63 | {"action": "joints", "delta_deg": {"shoulder_lift": 3, "wrist_flex": -1}, "frame": 33} | [2189, 1999, 2497, 1958, 2153, 2275] | [303.34, -48.17, 37.66] | 0034-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 35 | 65 | {"action": "joints", "delta_deg": {"shoulder_lift": 1.5}, "frame": 34} | [2189, 2020, 2497, 1958, 2153, 2275] | [300.7, -47.81, 30.03] | 0035-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 36 | 67 | {"action": "gripper", "ticks": 2335, "frame": 35} | [2189, 2020, 2497, 1958, 2153, 2332] | [300.7, -47.81, 30.03] | 0036-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 37 | 69 | {"action": "gripper", "ticks": 2425, "frame": 36} | [2189, 2020, 2497, 1958, 2153, 2423] | [300.7, -47.81, 30.03] | 0037-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 38 | 71 | {"action": "joints", "delta_deg": {"shoulder_lift": -8}, "frame": 37} | [2189, 1958, 2497, 1958, 2153, 2423] | [307.79, -48.77, 52.79] | 0038-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 39 | 73 | {"action": "joints", "delta_deg": {"shoulder_lift": -8}, "frame": 38} | [2189, 1896, 2498, 1958, 2153, 2423] | [312.47, -49.41, 75.75] | 0039-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 40 | 75 | {"action": "joints", "delta_deg": {"shoulder_pan": -10}, "frame": 39} | [2076, 1896, 2498, 1958, 2149, 2423] | [316.85, -1.47, 75.69] | 0040-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 41 | 77 | {"action": "joints", "delta_deg": {"shoulder_lift": -6, "wrist_roll": 10}, "frame": 40} | [2074, 1857, 2498, 1958, 2264, 2423] | [320.04, -0.11, 92.26] | 0041-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 42 | 79 | {"action": "joints", "delta_deg": {"wrist_roll": 10}, "frame": 41} | [2072, 1857, 2498, 1958, 2377, 2423] | [321.12, 1.52, 93.86] | 0042-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 43 | 81 | {"action": "joints", "delta_deg": {"wrist_roll": 10}, "frame": 42} | [2072, 1857, 2498, 1958, 2493, 2423] | [322.13, 2.64, 95.36] | 0043-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 44 | 83 | {"action": "joints", "delta_deg": {"wrist_roll": 10}, "frame": 43} | [2072, 1857, 2498, 1958, 2605, 2423] | [322.98, 4.01, 96.65] | 0044-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 45 | 85 | {"action": "joints", "delta_deg": {"wrist_roll": 10}, "frame": 44} | [2072, 1857, 2498, 1958, 2718, 2423] | [323.67, 5.63, 97.74] | 0045-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 46 | 87 | {"action": "joints", "delta_deg": {"wrist_roll": 10}, "frame": 45} | [2072, 1857, 2498, 1958, 2829, 2423] | [324.18, 7.43, 98.57] | 0046-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 47 | 89 | {"action": "joints", "delta_deg": {"wrist_roll": 10}, "frame": 46} | [2072, 1857, 2498, 1958, 2941, 2423] | [324.5, 9.38, 99.16] | 0047-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 48 | 91 | {"action": "joints", "delta_deg": {"wrist_roll": 10, "shoulder_pan": -2, "shoulder_lift": 6}, "frame": 47} | [2052, 1928, 2498, 1958, 3054, 2423] | [320.77, 19.94, 71.77] | 0048-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 49 | 93 | {"action": "joints", "delta_deg": {"shoulder_lift": 5, "wrist_flex": -8}, "frame": 48} | [2050, 1985, 2497, 1874, 3054, 2423] | [328.83, 21.44, 66.41] | 0049-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 50 | 95 | {"action": "joints", "delta_deg": {"shoulder_lift": 8, "elbow_flex": -10, "wrist_flex": 2}, "frame": 49} | [2047, 2078, 2406, 1896, 3054, 2423] | [341.19, 23.81, 57.8] | 0050-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 51 | 97 | {"action": "joints", "delta_deg": {"shoulder_lift": -8, "elbow_flex": 8}, "frame": 50} | [2047, 2007, 2498, 1895, 3054, 2423] | [323.5, 22.34, 53.2] | 0051-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 52 | 99 | {"action": "joints", "delta_deg": {"shoulder_lift": -8}, "frame": 51} | [2047, 1944, 2499, 1896, 3054, 2423] | [328.04, 22.72, 77.53] | 0052-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 53 | 101 | {"action": "joints", "delta_deg": {"shoulder_lift": -10, "elbow_flex": 8}, "frame": 52} | [2047, 1858, 2593, 1896, 3054, 2423] | [310.89, 21.29, 74.65] | 0053-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 54 | 103 | {"action": "joints", "delta_deg": {"shoulder_lift": -10, "elbow_flex": 8}, "frame": 53} | [2047, 1766, 2688, 1896, 3054, 2423] | [293.98, 19.89, 71.47] | 0054-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 55 | 105 | {"action": "joints", "delta_deg": {"shoulder_lift": -10, "elbow_flex": 8}, "frame": 54} | [2047, 1670, 2782, 1896, 3054, 2423] | [278.07, 18.57, 67.7] | 0055-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 56 | 107 | {"action": "joints", "delta_deg": {"shoulder_lift": 5}, "frame": 55} | [2047, 1727, 2782, 1896, 3054, 2423] | [273.01, 18.15, 49.58] | 0056-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 57 | 109 | {"action": "joints", "delta_deg": {"shoulder_lift": 3, "wrist_flex": -2, "shoulder_pan": -2}, "frame": 56} | [2025, 1761, 2782, 1882, 3054, 2423] | [270.77, 25.86, 41.59] | 0057-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
| 58 | 110 | null | [2025, 1761, 2782, 1882, 3054, 2423] | [270.77, 25.86, 41.59] | 0058-env/wrist.png | 仅关键帧入档，其余原图保留于本地 runs |
