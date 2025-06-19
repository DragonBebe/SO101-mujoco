"""
交互式键盘控制版本 - SO101机械臂逆运动学控制
支持键盘输入目标位置和位姿 - 基于lerobot真实实现
修正：考虑机械臂在mujoco空间中的基座偏移量
"""

import argparse
import time
import numpy as np
import mujoco
import mujoco.viewer
import math
import threading
import sys
import os
from scipy.spatial.transform import Rotation

# 添加运动学模块路径
sys.path.append(os.path.join(os.path.dirname(__file__), 'lerobot/common/model'))

# 导入真实的运动学库
try:
  from kinematics import RobotKinematics
  print("✅ 成功导入RobotKinematics模块")
except ImportError as e:
  print(f"❌ 导入RobotKinematics失败: {e}")
  print("请确保kinematics.py文件在正确的路径下")
  sys.exit(1)

class InteractiveSO101IKController:
  """
  交互式SO101机械臂IK控制器
  支持键盘输入目标位置和位姿
  修正：考虑机械臂基座在mujoco空间中的偏移量
  """
  
  def __init__(self, model, data):
      self.model = model
      self.data = data
      
      # 🔧 关键修正：机械臂基座在mujoco空间中的偏移量
      # 从XML中的定义: pos="0.8 0.6 0.71" quat="1 0 0 0.90"
      self.base_offset = np.array([0.8, 0.6, 0.71])
      
      # 基座旋转 - quat="1 0 0 0.90" 表示绕X轴旋转
      # 注意：四元数格式可能是 [w, x, y, z] 或 [x, y, z, w]
      # 根据mujoco文档，通常是 [w, x, y, z]
      base_quat = np.array([1.0, 0.0, 0.0, 0.90])
      base_quat = base_quat / np.linalg.norm(base_quat)  # 归一化
      self.base_rotation = Rotation.from_quat([base_quat[1], base_quat[2], base_quat[3], base_quat[0]])  # [x,y,z,w]格式
      
      print(f"🔧 机械臂基座偏移: {self.base_offset}")
      print(f"🔧 基座旋转角度: {self.base_rotation.as_euler('xyz', degrees=True)}")
      
      # 初始化运动学模块 - 使用SO101的新标定参数
      try:
          self.kinematics = RobotKinematics("so_new_calibration")
          print("✅ 运动学模块初始化成功")
      except Exception as e:
          print(f"❌ 运动学模块初始化失败: {e}")
          raise
      
      # 修正：使用XML中的实际关节名称
      self.joint_names = ['1', '2', '3', '4', '5', '6']
      self.joint_ids = []
      self.actuator_ids = []
      
      print("\n=== 关节映射检查 ===")
      for i, joint_name in enumerate(self.joint_names):
          try:
              joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
              actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name)
              self.joint_ids.append(joint_id)
              self.actuator_ids.append(actuator_id)
              print(f"✓ 关节 {joint_name}: joint_id={joint_id}, actuator_id={actuator_id}")
          except Exception as e:
              self.joint_ids.append(-1)
              self.actuator_ids.append(-1)
              print(f"❌ 关节 {joint_name}: 未找到 - {e}")
      
      # 检查末端执行器
      try:
          self.ee_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'gripper')
          print(f"✓ 末端执行器 'gripper': body_id={self.ee_body_id}")
      except:
          self.ee_body_id = -1
          print("❌ 末端执行器 'gripper': 未找到")
      
      # 检查gripper site
      try:
          self.gripper_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'gripper')
          print(f"✓ 夹爪site 'gripper': site_id={self.gripper_site_id}")
      except:
          self.gripper_site_id = -1
          print("❌ 夹爪site 'gripper': 未找到")
      
      # 控制参数
      self.kp = 50.0
      self.kd = 5.0
      
      # 当前状态 - 基于lerobot实现
      self.target_position = [1.2, 0.5, 0.8]  # 修正：使用你建议的初始位置
      self.target_orientation = [0, 0, 0]  # 欧拉角 (roll, pitch, yaw)
      self.current_target_joints = [0, 0, 0, 0, 0, 0]
      self.position_only = True  # 默认只控制位置
      
      # 🔧 新增：跟踪当前状态（基于lerobot实现）
      self.current_ee_pos = None
      self.current_joint_pos = None
      
      # 交互控制
      self.running = True
      
      # 统计有效关节
      valid_joints = sum(1 for jid in self.joint_ids if jid >= 0)
      print(f"\n✅ 控制器初始化完成 - 找到 {valid_joints}/6 个有效关节")
  
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
  
  def euler_to_rotation_matrix(self, roll, pitch, yaw):
      """欧拉角转旋转矩阵"""
      r = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True)
      return r.as_matrix()
  
  def create_pose_matrix(self, x, y, z, roll=0, pitch=0, yaw=0):
      """创建4x4位姿矩阵"""
      pose = np.eye(4, dtype=np.float32)
      pose[:3, :3] = self.euler_to_rotation_matrix(roll, pitch, yaw)
      pose[:3, 3] = [x, y, z]
      return pose
  
  def move_to_pose(self, x, y, z, roll=0, pitch=0, yaw=0, position_only=None):
      """
      移动到指定位姿 - 基于lerobot SO100FollowerEndEffector实现
      修正：输入的是世界坐标，需要转换为机械臂本体坐标进行IK计算
      """
      if position_only is None:
          position_only = self.position_only
      
      print(f"\n🎯 目标世界坐标: ({x:.3f}, {y:.3f}, {z:.3f})")
      
      # 🔧 关键修正：将世界坐标转换为机械臂本体坐标
      robot_target_pos = self.world_to_robot_coords([x, y, z])
      print(f"🔧 转换后机械臂坐标: ({robot_target_pos[0]:.3f}, {robot_target_pos[1]:.3f}, {robot_target_pos[2]:.3f})")
      
      # 创建目标位姿矩阵（在机械臂坐标系中）
      target_pose = self.create_pose_matrix(
          robot_target_pos[0], robot_target_pos[1], robot_target_pos[2], 
          roll, pitch, yaw
      )
      
      # 🔧 基于lerobot实现：获取当前关节位置
      if self.current_joint_pos is None:
          # 读取当前关节角度（弧度转度）
          current_joints_rad = self.get_current_joint_angles()
          current_joints_deg = [math.degrees(angle) for angle in current_joints_rad]
          # 确保有6个关节
          while len(current_joints_deg) < 6:
              current_joints_deg.append(0.0)
          self.current_joint_pos = np.array(current_joints_deg, dtype=np.float32)
      
      # 🔧 基于lerobot实现：计算当前末端执行器位置
      if self.current_ee_pos is None:
          try:
              self.current_ee_pos = self.kinematics.forward_kinematics(
                  self.current_joint_pos, frame="gripper_tip"
              )
          except Exception as e:
              print(f"❌ 正运动学计算失败: {e}")
              # 使用默认位姿（机械臂坐标系）
              self.current_ee_pos = np.eye(4, dtype=np.float32)
              self.current_ee_pos[:3, 3] = [0.2, 0.1, 0.2]
      
      # 🔧 基于lerobot实现：设置期望的末端执行器位置
      desired_ee_pos = np.eye(4, dtype=np.float32)
      if not position_only:
          # 如果控制位姿，使用目标旋转
          desired_ee_pos[:3, :3] = target_pose[:3, :3]
      else:
          # 如果只控制位置，保持当前姿态
          desired_ee_pos[:3, :3] = self.current_ee_pos[:3, :3]
      
      # 设置目标位置（机械臂坐标系）
      desired_ee_pos[:3, 3] = robot_target_pos
      
      # 使用IK求解目标关节角度
      try:
          print(f"\n🔄 开始IK求解...")
          print(f"   目标位置(世界): ({x:.3f}, {y:.3f}, {z:.3f})")
          print(f"   目标位置(机械臂): ({robot_target_pos[0]:.3f}, {robot_target_pos[1]:.3f}, {robot_target_pos[2]:.3f})")
          if not position_only:
              print(f"   目标姿态: ({roll:.1f}°, {pitch:.1f}°, {yaw:.1f}°)")
          print(f"   当前关节角度: {[f'{angle:.1f}°' for angle in self.current_joint_pos]}")
          
          # 🔧 关键修正：直接传入6个关节（基于lerobot实现）
          target_joints_deg = self.kinematics.ik(
              current_joint_pos=self.current_joint_pos,  # 传入6个关节
              desired_ee_pose=desired_ee_pos,
              position_only=position_only,
              frame="gripper_tip",
              max_iterations=15,
              learning_rate=0.5
          )
          
          # 🔧 基于lerobot实现：限制关节角度范围
          target_joints_deg = np.clip(target_joints_deg, -180.0, 180.0)
          
          # 转换回弧度（前5个关节用于控制）
          target_joints_rad = [math.radians(angle) for angle in target_joints_deg[:5]]
          # 保持当前夹爪位置
          current_gripper = self.current_joint_pos[5] if len(self.current_joint_pos) > 5 else 0.0
          target_joints_rad.append(math.radians(current_gripper))
          
          self.current_target_joints = target_joints_rad
          
          # 🔧 基于lerobot实现：更新状态
          self.current_ee_pos = desired_ee_pos.copy()
          self.current_joint_pos = target_joints_deg.copy()
          
          # 保存世界坐标作为目标
          self.target_position = [x, y, z]
          self.target_orientation = [roll, pitch, yaw]
          
          pose_type = "位置" if position_only else "位姿"
          print(f"✅ IK求解成功 - 移动到 {pose_type}")
          print(f"   世界坐标: ({x:.3f}, {y:.3f}, {z:.3f})")
          print(f"   机械臂坐标: ({robot_target_pos[0]:.3f}, {robot_target_pos[1]:.3f}, {robot_target_pos[2]:.3f})")
          if not position_only:
              print(f"   姿态: ({roll:.1f}°, {pitch:.1f}°, {yaw:.1f}°)")
          
          # 显示目标关节角度
          print("目标关节角度:")
          for i, angle in enumerate(self.current_target_joints):
              print(f"  关节{i+1}: {math.degrees(angle):6.2f}°")
          
          # 验证IK解
          try:
              verification_pose = self.kinematics.forward_kinematics(
                  target_joints_deg, "gripper_tip"
              )
              # 将验证结果转换回世界坐标进行比较
              verification_world_pos = self.robot_to_world_coords(verification_pose[:3, 3])
              pos_error = np.linalg.norm(verification_world_pos - np.array([x, y, z]))
              print(f"   验证误差(世界坐标): {pos_error*1000:.1f}mm")
              print(f"   验证位置(世界): ({verification_world_pos[0]:.3f}, {verification_world_pos[1]:.3f}, {verification_world_pos[2]:.3f})")
          except Exception as e:
              print(f"   验证失败: {e}")
          
          return True
          
      except Exception as e:
          print(f"❌ IK求解失败: {e}")
          import traceback
          traceback.print_exc()
          return False
  
  def update_control(self):
      """更新控制"""
      for i, (joint_id, actuator_id) in enumerate(zip(self.joint_ids, self.actuator_ids)):
          if joint_id >= 0 and actuator_id >= 0 and i < len(self.current_target_joints):
              current_pos = self.data.qpos[joint_id]
              current_vel = self.data.qvel[joint_id]
              target_pos = self.current_target_joints[i]
              
              pos_error = target_pos - current_pos
              vel_error = 0 - current_vel
              control_torque = self.kp * pos_error + self.kd * vel_error
              
              self.data.ctrl[actuator_id] = control_torque
  
  def get_current_position(self):
      """
      获取当前末端执行器位置
      修正：返回世界坐标系中的位置
      """
      # 方法1: 使用gripper site
      if self.gripper_site_id >= 0:
          return self.data.site_xpos[self.gripper_site_id].copy()
      
      # 方法2: 使用gripper body
      if self.ee_body_id >= 0:
          return self.data.xpos[self.ee_body_id].copy()
      
      # 方法3: 使用正运动学
      current_joints = self.get_current_joint_angles()
      current_joints_deg = [math.degrees(angle) for angle in current_joints]
      # 确保有6个关节
      while len(current_joints_deg) < 6:
          current_joints_deg.append(0.0)
      
      try:
          # 正运动学得到机械臂坐标系中的位置
          pose = self.kinematics.forward_kinematics(
              np.array(current_joints_deg, dtype=np.float32), "gripper_tip"
          )
          robot_pos = pose[:3, 3]
          
          # 转换为世界坐标
          world_pos = self.robot_to_world_coords(robot_pos)
          return world_pos
      except Exception as e:
          print(f"正运动学失败: {e}")
          return np.array([1.2, 0.5, 0.8])  # 默认位置（世界坐标）
  
  def get_current_orientation(self):
      """获取当前末端执行器姿态"""
      current_joints = self.get_current_joint_angles()
      current_joints_deg = [math.degrees(angle) for angle in current_joints]
      # 确保有6个关节
      while len(current_joints_deg) < 6:
          current_joints_deg.append(0.0)
      
      try:
          pose = self.kinematics.forward_kinematics(
              np.array(current_joints_deg, dtype=np.float32), "gripper_tip"
          )
          
          # 提取旋转矩阵并转换为欧拉角
          rotation_matrix = pose[:3, :3]
          r = Rotation.from_matrix(rotation_matrix)
          euler_angles = r.as_euler('xyz', degrees=True)
          
          return euler_angles
      except Exception as e:
          print(f"姿态计算失败: {e}")
          return np.array([0.0, 0.0, 0.0])  # 默认姿态
  
  def get_current_joint_angles(self):
      """获取当前关节角度（弧度）"""
      return [self.data.qpos[joint_id] if joint_id >= 0 else 0 
              for joint_id in self.joint_ids]
  
  def reset_state(self):
      """重置状态 - 基于lerobot实现"""
      self.current_ee_pos = None
      self.current_joint_pos = None
  
  def keyboard_input_thread(self):
      """键盘输入线程"""
      print("\n🎮 交互式IK控制已启动!")
      print("命令格式:")
      print("  move x y z                    - 移动到位置 (仅位置)")
      print("  pose x y z roll pitch yaw     - 移动到位姿 (位置+姿态)")
      print("  mode position/pose            - 切换控制模式")
      print("  status                        - 显示当前状态")
      print("  joints                        - 显示关节角度")
      print("  presets                       - 显示预设位置")
      print("  test                          - 测试运动学")
      print("  reset                         - 重置状态")
      print("  quit                          - 退出程序")
      print("\n⚠️  注意：所有坐标都是世界坐标系（mujoco空间）")
      print("示例:")
      print("  move 1.2 0.5 0.8             - 移动到初始位置")
      print("  move 1.0 0.6 0.9             - 移动到桌面上方")
      print("  pose 1.1 0.7 0.85 0 90 0     - 移动到指定位姿")
      
      # 预设位置和位姿 - 修正：使用世界坐标系，考虑机械臂偏移
      presets = {
          '1': {'pos': (1.2, 0.5, 0.8), 'ori': (0, 0, 0), 'desc': "初始位置"},
          '2': {'pos': (1.0, 0.6, 0.9), 'ori': (0, 30, 0), 'desc': "桌面上方"},
          '3': {'pos': (0.9, 0.7, 0.85), 'ori': (0, 0, 45), 'desc': "右侧位置"},
          '4': {'pos': (1.1, 0.5, 0.75), 'ori': (0, -15, 0), 'desc': "桌面位置"},
          '5': {'pos': (1.0, 0.4, 0.9), 'ori': (0, 0, -30), 'desc': "左前方"},
      }
      
      while self.running:
          try:
              command = input("\n> ").strip().lower()
              
              if command == 'quit' or command == 'q':
                  self.running = False
                  break
              
              elif command == 'reset':
                  print("\n🔄 重置状态...")
                  self.reset_state()
                  print("✅ 状态已重置")
              
              elif command == 'test':
                  print("\n🧪 测试运动学模块...")
                  try:
                      # 测试正运动学 - 使用6个关节
                      test_joints = np.array([0, 0, 0, 0, 0, 0], dtype=np.float32)
                      pose = self.kinematics.forward_kinematics(test_joints, "gripper_tip")
                      robot_pos = pose[:3, 3]
                      world_pos = self.robot_to_world_coords(robot_pos)
                      print(f"   零位正运动学结果(机械臂坐标): {robot_pos}")
                      print(f"   零位正运动学结果(世界坐标): {world_pos}")
                      
                      # 测试坐标转换
                      test_world = [1.2, 0.5, 0.8]
                      test_robot = self.world_to_robot_coords(test_world)
                      test_world_back = self.robot_to_world_coords(test_robot)
                      print(f"   坐标转换测试:")
                      print(f"     世界坐标: {test_world}")
                      print(f"     机械臂坐标: {test_robot}")
                      print(f"     转换回世界坐标: {test_world_back}")
                      
                      # 测试IK
                      target_pose = self.create_pose_matrix(test_robot[0], test_robot[1], test_robot[2])
                      ik_result = self.kinematics.ik(
                          test_joints,
                          target_pose,
                          position_only=True,
                          max_iterations=10
                      )
                      print(f"   IK测试结果: {[f'{a:.1f}°' for a in ik_result]}")
                      print("✅ 运动学模块测试完成")
                  except Exception as e:
                      print(f"❌ 运动学测试失败: {e}")
                      import traceback
                      traceback.print_exc()
              
              elif command == 'status':
                  current_pos = self.get_current_position()
                  current_ori = self.get_current_orientation()
                  target_pos = self.target_position
                  target_ori = self.target_orientation
                  
                  pos_error = np.linalg.norm(np.array(current_pos) - np.array(target_pos))
                  ori_error = np.linalg.norm(np.array(current_ori) - np.array(target_ori))
                  
                  print(f"\n📊 当前状态:")
                  print(f"   控制模式: {'仅位置' if self.position_only else '位置+姿态'}")
                  print(f"   当前位置(世界): [{current_pos[0]:6.3f}, {current_pos[1]:6.3f}, {current_pos[2]:6.3f}]")
                  print(f"   目标位置(世界): [{target_pos[0]:6.3f}, {target_pos[1]:6.3f}, {target_pos[2]:6.3f}]")
                  print(f"   位置误差: {pos_error*1000:.1f}mm")
                  
                  # 显示机械臂坐标系中的位置
                  robot_current = self.world_to_robot_coords(current_pos)
                  robot_target = self.world_to_robot_coords(target_pos)
                  print(f"   当前位置(机械臂): [{robot_current[0]:6.3f}, {robot_current[1]:6.3f}, {robot_current[2]:6.3f}]")
                  print(f"   目标位置(机械臂): [{robot_target[0]:6.3f}, {robot_target[1]:6.3f}, {robot_target[2]:6.3f}]")
                  
                  if not self.position_only:
                      print(f"   当前姿态: [{current_ori[0]:6.1f}°, {current_ori[1]:6.1f}°, {current_ori[2]:6.1f}°]")
                      print(f"   目标姿态: [{target_ori[0]:6.1f}°, {target_ori[1]:6.1f}°, {target_ori[2]:6.1f}°]")
                      print(f"   姿态误差: {ori_error:.1f}°")
              
              elif command == 'joints':
                  current_joints = self.get_current_joint_angles()
                  target_joints = self.current_target_joints
                  print(f"\n🔧 关节角度:")
                  for i in range(min(6, len(current_joints), len(target_joints))):
                      current_deg = math.degrees(current_joints[i])
                      target_deg = math.degrees(target_joints[i])
                      error_deg = abs(current_deg - target_deg)
                      print(f"   关节{i+1}: 当前={current_deg:6.2f}°, 目标={target_deg:6.2f}°, 误差={error_deg:5.2f}°")
              
              elif command == 'presets':
                  print("\n📍 预设位置 (世界坐标系):")
                  for key, preset in presets.items():
                      pos = preset['pos']
                      ori = preset['ori']
                      desc = preset['desc']
                      print(f"   {key}: {desc}")
                      print(f"      位置: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")
                      print(f"      姿态: ({ori[0]:.0f}°, {ori[1]:.0f}°, {ori[2]:.0f}°)")
                  print("\n使用方法: 输入数字选择预设位置")
              
              elif command in presets:
                  preset = presets[command]
                  x, y, z = preset['pos']
                  roll, pitch, yaw = preset['ori']
                  desc = preset['desc']
                  print(f"\n🎯 移动到预设位置: {desc}")
                  self.move_to_pose(x, y, z, roll, pitch, yaw, position_only=False)
              
                
              elif command.startswith('mode'):
                parts = command.split()
                if len(parts) == 2:
                    if parts[1] == 'position':
                            self.position_only = True
                            print("✅ 切换到位置控制模式")
                    elif parts[1] == 'pose':
                            self.position_only = False
                            print("✅ 切换到位姿控制模式")
                    else:
                            print("❌ 请使用: mode position 或 mode pose")
                else:
                        print("❌ 请使用: mode position 或 mode pose")
                
              elif command.startswith('move'):
                    parts = command.split()
                    if len(parts) == 4:
                        try:
                            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                            self.move_to_pose(x, y, z, position_only=True)
                        except ValueError:
                            print("❌ 坐标格式错误，请使用: move x y z")
                    else:
                        print("❌ 命令格式错误，请使用: move x y z")
                
              elif command.startswith('pose'):
                    parts = command.split()
                    if len(parts) == 7:
                        try:
                            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                            roll, pitch, yaw = float(parts[4]), float(parts[5]), float(parts[6])
                            self.move_to_pose(x, y, z, roll, pitch, yaw, position_only=False)
                        except ValueError:
                            print("❌ 参数格式错误，请使用: pose x y z roll pitch yaw")
                    else:
                        print("❌ 命令格式错误，请使用: pose x y z roll pitch yaw")
                
              else:
                    print("❌ 未知命令。输入 'presets' 查看预设位置，'test' 测试运动学，或 'quit' 退出")
                    
          except KeyboardInterrupt:
                self.running = False
                break
          except EOFError:
                self.running = False
                break

# 其余代码保持不变...
def print_model_info(model, data):
    """打印模型信息用于调试"""
    print("\n=== 模型信息 ===")
    print(f"关节数量: {model.njnt}")
    print(f"执行器数量: {model.nu}")
    print(f"物体数量: {model.nbody}")
    
    print("\n关节列表:")
    for i in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        joint_type = model.jnt_type[i]
        print(f"  {i}: {joint_name} (type: {joint_type})")
    
    print("\n执行器列表:")
    for i in range(model.nu):
        actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        print(f"  {i}: {actuator_name}")

def do_interactive_sim(robot_id):
    if robot_id == "6dof":
        xml_path = "manipulator_grasp/assets/SO101/scene_table_cubes.xml"
        try:
            m = mujoco.MjModel.from_xml_path(xml_path)
            print(f"✅ Model loaded successfully from: {xml_path}")
        except Exception as e:
            print(f"❌ Error loading XML: {e}")
            return
    else:
        print(f"Robot ID {robot_id} not supported.")
        return

    data = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, data, 0)
    
    # 打印模型信息用于调试
    print_model_info(m, data)
    
    try:
        controller = InteractiveSO101IKController(m, data)
    except Exception as e:
        print(f"❌ 控制器初始化失败: {e}")
        return
    
    # 设置初始位置
    print("\n🎯 设置初始位置...")
    controller.move_to_pose(1.2, 0.5, 0.8, 0, 0, 0)
    
    # 启动键盘输入线程
    input_thread = threading.Thread(target=controller.keyboard_input_thread, daemon=True)
    input_thread.start()
    
    print("\n=== 启动交互式IK仿真 ===")
    print("🎮 控制说明：")
    print("  - 鼠标左键拖拽：旋转视角")
    print("  - 鼠标滚轮：缩放")
    print("  - 鼠标右键拖拽：平移视角")
    print("  - 在终端输入命令控制机械臂")
    print("  - 支持位置控制和位姿控制")
    print("  - 输入 'test' 测试运动学模块")
    print("  - 按 Ctrl+C 退出仿真")
    
    # 状态显示计数器
    status_counter = 0
    
    # 启动仿真
    with mujoco.viewer.launch_passive(m, data) as viewer:
        while viewer.is_running() and controller.running:
            step_start = time.time()
            
            # 更新控制
            controller.update_control()
            
            # 执行仿真步骤
            mujoco.mj_step(m, data)
            viewer.sync()
            
            # 每1000步显示一次状态
            status_counter += 1
            if status_counter % 2000 == 0:  # 每4秒显示一次
                current_pos = controller.get_current_position()
                target_pos = controller.target_position
                error = np.linalg.norm(np.array(current_pos) - np.array(target_pos))
                print(f"\n📊 状态更新:")
                print(f"   当前位置: [{current_pos[0]:6.3f}, {current_pos[1]:6.3f}, {current_pos[2]:6.3f}]")
                print(f"   目标位置: [{target_pos[0]:6.3f}, {target_pos[1]:6.3f}, {target_pos[2]:6.3f}]")
                print(f"   位置误差: {error*1000:.1f}mm")
            
            # 控制仿真速度
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
    
    controller.running = False
    print("\n🔚 仿真结束")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="交互式SO101机械臂IK控制")
    parser.add_argument("--robot", choices=["6dof"], default="6dof", help="选择机器人类型")
    args = parser.parse_args()
    do_interactive_sim(args.robot)

