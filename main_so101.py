"""
将SO101直接导入mujoco，不添加场景
"""


import mujoco
import mujoco.viewer

# 加载模型
model = mujoco.MjModel.from_xml_path('SO101-envs/SO101/so101_new_calib.xml')
data = mujoco.MjData(model)

# 启动查看器
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # 步进仿真
        mujoco.mj_step(model, data)
        # 同步查看器
        viewer.sync()
