"""
简化版SO101机械臂逆运动学测试
专注于mujoco场景加载和IK控制效果验证
修正：添加机械臂基座空间坐标转换逻辑
解决交互输入问题，简化测试序列
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import math
import sys
import os
import threading
import queue
from scipy.spatial.transform import Rotation

# 添加运动学模块路径
sys.path.append(os.path.join(os.path.dirname(__file__), 'lerobot/common/model'))

# 导入运动学库
try:
  from kinematics import RobotKinematics
  print("✅ 成功导入RobotKinematics模块")
except ImportError as e:
  print(f"❌ 导入RobotKinematics失败: {e}")
  sys.exit(1)

class SimpleIKTester:
  """简化的IK测试器 - 支持空间坐标转换"""
  
  def __init__(self, model, data):
      self.model = model
      self.data = data
      
      # 🔧 关键新增：机械臂基座在mujoco空间中的偏移量
      # 从XML中的定义: pos="0.8 0.6 0.71" quat="1 0 0 0.90"
      self.base_offset = np.array([0.8, 0.6, 0.71])
      
      # 基座旋转 - quat="1 0 0 0.90" 表示绕X轴旋转
      base_quat = np.array([1.0, 0.0, 0.0, 0.90])
      base_quat = base_quat / np.linalg.norm(base_quat)  # 归一化
      self.base_rotation = Rotation.from_quat([base_quat[1], base_quat[2], base_quat[3], base_quat[0]])  # [x,y,z,w]格式
      
      print(f"🔧 机械臂基座偏移: {self.base_offset}")
      print(f"🔧 基座旋转角度: {self.base_rotation.as_euler('xyz', degrees=True)}")
      
      # 初始化运动学模块
      try:
          self.kinematics = RobotKinematics("so_new_calibration")
          print("✅ 运动学模块初始化成功")
      except Exception as e:
          print(f"❌ 运动学模块初始化失败: {e}")
          raise
      
      # 获取关节映射
      self.joint_names = ['1', '2', '3', '4', '5', '6']
      self.joint_ids = []
      self.actuator_ids = []
      
      for joint_name in self.joint_names:
          try:
              joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
              actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name)
              self.joint_ids.append(joint_id)
              self.actuator_ids.append(actuator_id)
              print(f"✓ 关节 {joint_name}: joint_id={joint_id}, actuator_id={actuator_id}")
          except:
              self.joint_ids.append(-1)
              self.actuator_ids.append(-1)
              print(f"❌ 关节 {joint_name}: 未找到")
      
      # 控制参数
      self.kp = 50.0
      self.kd = 5.0
      self.current_target_joints = [0, 0, 0, 0, 0, 0]
      
      # 交互控制
      self.command_queue = queue.Queue()
      self.running = True
  
  def world_to_robot_coords(self, world_pos):
      """
      将世界坐标转换为机械臂本体坐标系
      world_pos: mujoco世界坐标系中的位置
      返回: 机械臂本体坐标系中的位置（用于IK计算）
      """
      # 平移变换：减去基座偏移
      translated_pos = np.array(world_pos) - self.base_offset
      
      # 旋转变换：应用基座旋转的逆变换
      robot_pos = self.base_rotation.inv().apply(translated_pos)
      
      return robot_pos
  
  def robot_to_world_coords(self, robot_pos):
      """
      将机械臂本体坐标系转换为世界坐标
      robot_pos: 机械臂本体坐标系中的位置
      返回: mujoco世界坐标系中的位置
      """
      # 旋转变换：应用基座旋转
      rotated_pos = self.base_rotation.apply(robot_pos)
      
      # 平移变换：加上基座偏移
      world_pos = rotated_pos + self.base_offset
      
      return world_pos
  
  def create_pose_matrix(self, x, y, z, roll=0, pitch=0, yaw=0):
      """创建4x4位姿矩阵"""
      pose = np.eye(4, dtype=np.float32)
      r = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True)
      pose[:3, :3] = r.as_matrix()
      pose[:3, 3] = [x, y, z]
      return pose
  
  def get_current_joint_angles_deg(self):
      """获取当前关节角度（度）"""
      angles_rad = [self.data.qpos[joint_id] if joint_id >= 0 else 0 
                   for joint_id in self.joint_ids]
      return [math.degrees(angle) for angle in angles_rad]
  
  def test_ik(self, world_x, world_y, world_z, roll=0, pitch=0, yaw=0, position_only=True):
      """
      测试逆运动学 - 修正：支持世界坐标输入
      world_x, world_y, world_z: 世界坐标系中的目标位置
      """
      print(f"\n🔄 IK测试")
      print(f"   目标位置(世界坐标): ({world_x:.3f}, {world_y:.3f}, {world_z:.3f})")
      
      # 🔧 关键修正：将世界坐标转换为机械臂本体坐标
      robot_target_pos = self.world_to_robot_coords([world_x, world_y, world_z])
      print(f"   目标位置(机械臂坐标): ({robot_target_pos[0]:.3f}, {robot_target_pos[1]:.3f}, {robot_target_pos[2]:.3f})")
      
      if not position_only:
          print(f"   目标姿态: ({roll:.1f}°, {pitch:.1f}°, {yaw:.1f}°)")
      
      # 获取当前关节角度
      current_joints_deg = self.get_current_joint_angles_deg()
      print(f"   当前关节: {[f'{a:.1f}°' for a in current_joints_deg]}")
      
      # 🔧 关键修正：使用机械臂坐标系创建目标位姿
      target_pose = self.create_pose_matrix(
          robot_target_pos[0], robot_target_pos[1], robot_target_pos[2], 
          roll, pitch, yaw
      )
      
      try:
          # 执行IK求解
          target_joints_deg = self.kinematics.ik(
              current_joint_pos=np.array(current_joints_deg, dtype=np.float32),
              desired_ee_pose=target_pose,
              position_only=position_only,
              frame="gripper_tip",
              max_iterations=20,
              learning_rate=0.5
          )
          
          # 限制关节角度范围
          target_joints_deg = np.clip(target_joints_deg, -180.0, 180.0)
          
          print(f"✅ IK求解成功")
          print(f"   目标关节: {[f'{a:.1f}°' for a in target_joints_deg]}")
          
          # 🔧 修正：验证IK解（转换回世界坐标进行比较）
          verification_pose = self.kinematics.forward_kinematics(
              target_joints_deg, "gripper_tip"
          )
          # 将验证结果转换回世界坐标
          verification_world_pos = self.robot_to_world_coords(verification_pose[:3, 3])
          pos_error = np.linalg.norm(verification_world_pos - np.array([world_x, world_y, world_z]))
          print(f"   验证误差(世界坐标): {pos_error*1000:.1f}mm")
          print(f"   验证位置(世界): ({verification_world_pos[0]:.3f}, {verification_world_pos[1]:.3f}, {verification_world_pos[2]:.3f})")
          
          # 设置目标关节角度
          self.current_target_joints = [math.radians(angle) for angle in target_joints_deg]
          
          return True
          
      except Exception as e:
          print(f"❌ IK求解失败: {e}")
          import traceback
          traceback.print_exc()
          return False
  
  def update_control(self):
      """更新控制器"""
      for i, (joint_id, actuator_id) in enumerate(zip(self.joint_ids, self.actuator_ids)):
          if joint_id >= 0 and actuator_id >= 0 and i < len(self.current_target_joints):
              current_pos = self.data.qpos[joint_id]
              current_vel = self.data.qvel[joint_id]
              target_pos = self.current_target_joints[i]
              
              pos_error = target_pos - current_pos
              vel_error = 0 - current_vel
              control_torque = self.kp * pos_error + self.kd * vel_error
              
              self.data.ctrl[actuator_id] = control_torque
  
  def get_current_ee_position_world(self):
      """
      获取当前末端执行器位置（世界坐标）
      修正：返回世界坐标系中的位置
      """
      try:
          # 方法1: 尝试使用gripper site
          try:
              gripper_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'gripper')
              return self.data.site_xpos[gripper_site_id].copy()
          except:
              pass
          
          # 方法2: 尝试使用gripper body
          try:
              gripper_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'gripper')
              return self.data.xpos[gripper_body_id].copy()
          except:
              pass
          
          # 方法3: 使用正运动学计算并转换为世界坐标
          current_joints_deg = self.get_current_joint_angles_deg()
          pose = self.kinematics.forward_kinematics(
              np.array(current_joints_deg, dtype=np.float32), "gripper_tip"
          )
          robot_pos = pose[:3, 3]
          
          # 转换为世界坐标
          world_pos = self.robot_to_world_coords(robot_pos)
          return world_pos
          
      except Exception as e:
          print(f"获取末端执行器位置失败: {e}")
          # 返回默认位置（世界坐标）
          return np.array([1.2, 0.5, 0.8])
  
  def test_coordinate_conversion(self):
      """测试坐标转换功能"""
      print("\n🧪 测试坐标转换功能...")
      
      # 只测试目标点
      test_world_point = [1.2, 0.5, 0.8]
      
      print(f"世界坐标: {test_world_point}")
      
      # 转换为机械臂坐标
      robot_point = self.world_to_robot_coords(test_world_point)
      print(f"机械臂坐标: [{robot_point[0]:.3f}, {robot_point[1]:.3f}, {robot_point[2]:.3f}]")
      
      # 转换回世界坐标
      world_point_back = self.robot_to_world_coords(robot_point)
      print(f"转换回世界坐标: [{world_point_back[0]:.3f}, {world_point_back[1]:.3f}, {world_point_back[2]:.3f}]")
      
      # 计算转换误差
      conversion_error = np.linalg.norm(np.array(test_world_point) - world_point_back)
      print(f"转换误差: {conversion_error*1000:.3f}mm")
      
      if conversion_error < 1e-10:
          print("✅ 坐标转换正确")
      else:
          print("⚠️  坐标转换存在误差")
  
  def run_single_test(self):
      """运行单个测试点"""
      print("\n🧪 开始IK测试...")
      
      # 首先测试坐标转换
      self.test_coordinate_conversion()
      
      # 测试目标点 (1.2, 0.5, 0.8)
      world_x, world_y, world_z = 1.2, 0.5, 0.8
      
      print(f"\n--- 测试目标点: ({world_x}, {world_y}, {world_z}) ---")
      
      # 显示对应的机械臂坐标
      robot_pos = self.world_to_robot_coords([world_x, world_y, world_z])
      print(f"对应机械臂坐标: ({robot_pos[0]:.3f}, {robot_pos[1]:.3f}, {robot_pos[2]:.3f})")
      
      # 执行IK测试
      success = self.test_ik(world_x, world_y, world_z, position_only=True)
      
      if success:
          print("✅ IK求解成功，机械臂将开始移动")
          return True
      else:
          print("❌ IK求解失败")
          return False
  
  def keyboard_input_thread(self):
      """键盘输入线程"""
      while self.running:
          try:
              command = input().strip()
              if command:
                  self.command_queue.put(command)
          except EOFError:
              break
          except KeyboardInterrupt:
              self.running = False
              break
  
  def process_command(self, command):
      """处理命令"""
      parts = command.strip().split()
      
      if not parts:
          return
      
      if parts[0] == 'quit' or parts[0] == 'q':
          self.running = False
          print("🔚 退出程序")
          return
      
      elif len(parts) >= 4 and parts[0] == 'test':
          try:
              world_x = float(parts[1])
              world_y = float(parts[2])
              world_z = float(parts[3])
              
              print(f"\n🎯 用户测试 - 世界坐标: ({world_x:.3f}, {world_y:.3f}, {world_z:.3f})")
              success = self.test_ik(world_x, world_y, world_z, position_only=True)
              
              if success:
                  print("✅ IK求解成功，机械臂开始移动")
              else:
                  print("❌ IK求解失败")
                  
          except ValueError:
              print("❌ 坐标格式错误，请使用: test x y z")
      
      elif parts[0] == 'status':
          current_world_pos = self.get_current_ee_position_world()
          current_joints = self.get_current_joint_angles_deg()
          print(f"\n📊 当前状态:")
          print(f"   末端位置(世界): ({current_world_pos[0]:.3f}, {current_world_pos[1]:.3f}, {current_world_pos[2]:.3f})")
          print(f"   关节角度: {[f'{a:.1f}°' for a in current_joints]}")
          
          # 显示对应的机械臂坐标
          robot_pos = self.world_to_robot_coords(current_world_pos)
          print(f"   末端位置(机械臂): ({robot_pos[0]:.3f}, {robot_pos[1]:.3f}, {robot_pos[2]:.3f})")
      
      elif parts[0] == 'coords':
          self.test_coordinate_conversion()
      
      elif parts[0] == 'target':
          # 快速测试目标点
          success = self.test_ik(1.2, 0.5, 0.8, position_only=True)
          if success:
              print("✅ 目标点IK求解成功")
          else:
              print("❌ 目标点IK求解失败")
      
      elif parts[0] == 'help':
          print("\n🎮 可用命令:")
          print("  test x y z  - 测试指定世界坐标位置")
          print("  target      - 测试目标点(1.2, 0.5, 0.8)")
          print("  status      - 显示当前状态")
          print("  coords      - 测试坐标转换")
          print("  help        - 显示帮助")
          print("  quit        - 退出")
      
      else:
          print("❌ 未知命令，输入 'help' 查看可用命令")

def main():
  """主函数"""
  # 加载mujoco场景
  xml_path = "manipulator_grasp/assets/SO101/scene_table_cubes.xml"
  try:
      model = mujoco.MjModel.from_xml_path(xml_path)
      data = mujoco.MjData(model)
      print(f"✅ 成功加载场景: {xml_path}")
  except Exception as e:
      print(f"❌ 场景加载失败: {e}")
      return
  
  # 重置到初始状态
  mujoco.mj_resetDataKeyframe(model, data, 0)
  
  # 创建IK测试器
  try:
      ik_tester = SimpleIKTester(model, data)
  except Exception as e:
      print(f"❌ IK测试器初始化失败: {e}")
      return
  
  print("\n=== SO101逆运动学测试（支持空间坐标转换）===")
  print("🎯 测试目标：验证IK算法在mujoco场景中的控制效果")
  print("🔧 新增功能：世界坐标与机械臂坐标系的转换")
  
  # 启动仿真
  with mujoco.viewer.launch_passive(model, data) as viewer:
      # 等待viewer启动
      time.sleep(1.0)
      
      # 运行单个测试
      ik_tester.run_single_test()
      
      # 启动键盘输入线程
      input_thread = threading.Thread(target=ik_tester.keyboard_input_thread, daemon=True)
      input_thread.start()
      
      print("\n🎮 交互模式已启动！")
      print("可用命令:")
      print("  test x y z  - 测试指定世界坐标位置")
      print("  target      - 测试目标点(1.2, 0.5, 0.8)")
      print("  status      - 显示当前状态")
      print("  coords      - 测试坐标转换")
      print("  help        - 显示帮助")
      print("  quit        - 退出")
      print("\n⚠️  注意：输入的坐标是世界坐标系（mujoco空间）")
      print("示例: test 1.0 0.6 0.9")
      print("\n请输入命令:")
      
      # 主循环
      while viewer.is_running() and ik_tester.running:
          try:
              # 更新控制
              ik_tester.update_control()
              mujoco.mj_step(model, data)
              viewer.sync()
              
              # 处理命令队列
              try:
                  command = ik_tester.command_queue.get_nowait()
                  ik_tester.process_command(command)
              except queue.Empty:
                  pass
              
              time.sleep(0.01)
              
          except KeyboardInterrupt:
              print("\n🔚 用户中断，退出仿真")
              break
  
  ik_tester.running = False
  print("✅ 测试完成")

if __name__ == "__main__":
  main()