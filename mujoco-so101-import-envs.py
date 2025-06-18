"""
直接加载scene_table.xml场景到MuJoCo仿真中
"""

import argparse
import time
import numpy as np
import mujoco
import mujoco.viewer

def do_interactive_sim(robot_id):
  if robot_id == "6dof":
    #   xml_path = "manipulator_grasp/assets/SO101/scene_table.xml"
      xml_path = "manipulator_grasp/assets/SO101/scene_table_cubes.xml"
      try:
          m = mujoco.MjModel.from_xml_path(xml_path)
          print(f"✅ Model loaded successfully from: {xml_path}")
      except Exception as e:
          print(f"❌ Error loading XML: {e}")
          print("请确保以下文件存在：")
          print("  1. manipulator_grasp/assets/SO101/scene_table.xml")
          print("  2. manipulator_grasp/assets/SO101/so101_new_calib.xml")
          print("  3. 相关的STL文件（如果XML中引用了的话）")
          return
  else:
      print(f"Robot ID {robot_id} not supported by this script version.")
      return

  # 创建数据结构
  data = mujoco.MjData(m)
  
  # 重置到初始状态
  mujoco.mj_resetDataKeyframe(m, data, 0)
  
  print("\n=== 场景信息 ===")
  print(f"模型名称: {m.names}")
  print(f"时间步长: {m.opt.timestep}")
  print(f"重力: {m.opt.gravity}")
  print(f"物体数量: {m.nbody}")
  print(f"关节数量: {m.njnt}")
  print(f"几何体数量: {m.ngeom}")
  
  # 列出所有物体
  print("\n=== 场景中的物体 ===")
  for i in range(m.nbody):
      body_name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i)
      if body_name:
          pos = data.xpos[i]
          print(f"  {i:2d}: {body_name:15s} - 位置: [{pos[0]:6.3f}, {pos[1]:6.3f}, {pos[2]:6.3f}]")
  
  print("\n=== 启动仿真 ===")
  print("控制说明：")
  print("  - 鼠标拖拽：旋转视角")
  print("  - 鼠标滚轮：缩放")
  print("  - 右键拖拽：平移视角")
  print("  - 按 Ctrl+C 退出仿真")
  print("  - 在仿真窗口中可以使用MuJoCo的内置控制")
  
  # 启动仿真
  with mujoco.viewer.launch_passive(m, data) as viewer:
      step_count = 0
      start_time = time.time()
      
      while viewer.is_running():
          step_start = time.time()
          
          # 执行仿真步骤
          mujoco.mj_step(m, data)
          viewer.sync()
          
          # 每1000步打印一次状态信息
          if step_count % 1000 == 0:
              current_time = time.time() - start_time
              print(f"仿真步数: {step_count:6d} | 仿真时间: {data.time:6.2f}s | 实际时间: {current_time:6.2f}s")
          
          step_count += 1
          
          # 控制仿真速度（实时仿真）
          time_until_next_step = m.opt.timestep - (time.time() - step_start)
          if time_until_next_step > 0:
              time.sleep(time_until_next_step)

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="加载SO101机械臂场景到MuJoCo仿真")
  parser.add_argument("--robot", choices=["6dof"], default="6dof", help="选择机器人类型")
  args = parser.parse_args()
  do_interactive_sim(args.robot)