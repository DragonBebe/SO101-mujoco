"""
简化版SO101机械臂逆运动学测试
机械臂以XML文件中定义的自然状态启动
专注于基础IK控制和测试
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

# 添加运动学模块路径
sys.path.append(os.path.join(os.path.dirname(__file__), 'lerobot/common/model'))

# 导入运动学库
try:
  from kinematics import RobotKinematics
  print("✅ 成功导入RobotKinematics模块")
except ImportError as e:
  print(f"❌ 导入RobotKinematics失败: {e}")
  sys.exit(1)

class SimplifiedIKTester:
  """简化版IK测试器 - 机械臂保持自然状态"""
  
  def __init__(self, model, data):
      self.model = model
      self.data = data
      
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
      self.kp = 50.0  # 位置增益
      self.kd = 1.0   # 速度增益
      
      # 🔧 关键修改：使用当前自然状态作为初始目标，而不是强制设为零位
      self.current_target_joints = self.get_current_joint_angles_rad()
      print(f"🔧 使用自然初始状态: {[f'{math.degrees(a):.1f}°' for a in self.current_target_joints]}")
      
      # 交互控制
      self.command_queue = queue.Queue()
      self.running = True
      
      print("🔧 简化版控制器初始化完成（保持自然状态）")
  
  def create_pose_matrix(self, x, y, z, roll=0, pitch=0, yaw=0):
      """创建4x4位姿矩阵"""
      pose = np.eye(4, dtype=np.float32)
      
      # 简化的旋转矩阵计算
      if roll != 0 or pitch != 0 or yaw != 0:
          from scipy.spatial.transform import Rotation
          r = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True)
          pose[:3, :3] = r.as_matrix()
      
      pose[:3, 3] = [x, y, z]
      return pose
  
  def get_current_joint_angles_rad(self):
      """获取当前关节角度（弧度）"""
      angles_rad = [self.data.qpos[joint_id] if joint_id >= 0 else 0 
                   for joint_id in self.joint_ids]
      return angles_rad
  
  def get_current_joint_angles_deg(self):
      """获取当前关节角度（度）"""
      angles_rad = self.get_current_joint_angles_rad()
      return [math.degrees(angle) for angle in angles_rad]
  
  def test_ik(self, x, y, z, roll=0, pitch=0, yaw=0, position_only=True):
      """
      简化版IK测试 - 直接使用输入坐标
      x, y, z: 目标位置坐标
      """
      print(f"\n🔄 IK测试")
      print(f"   目标位置: ({x:.3f}, {y:.3f}, {z:.3f})")
      
      if not position_only:
          print(f"   目标姿态: ({roll:.1f}°, {pitch:.1f}°, {yaw:.1f}°)")
      
      # 获取当前关节角度
      current_joints_deg = self.get_current_joint_angles_deg()
      print(f"   当前关节: {[f'{a:.1f}°' for a in current_joints_deg]}")
      
      # 创建目标位姿矩阵
      target_pose = self.create_pose_matrix(x, y, z, roll, pitch, yaw)
      
      try:
          # 执行IK求解
          target_joints_deg = self.kinematics.ik(
              current_joint_pos=np.array(current_joints_deg, dtype=np.float32),
              desired_ee_pose=target_pose,
              position_only=position_only,
              frame="gripper_tip",
              max_iterations=50,
              learning_rate=0.3
          )
          
          # 限制关节角度范围
          target_joints_deg = np.clip(target_joints_deg, -180.0, 180.0)
          
          print(f"✅ IK求解成功")
          print(f"   目标关节: {[f'{a:.1f}°' for a in target_joints_deg]}")
          
          # 验证IK解
          verification_pose = self.kinematics.forward_kinematics(
              target_joints_deg, "gripper_tip"
          )
          verification_pos = verification_pose[:3, 3]
          pos_error = np.linalg.norm(verification_pos - np.array([x, y, z]))
          print(f"   验证误差: {pos_error*1000:.1f}mm")
          print(f"   验证位置: ({verification_pos[0]:.3f}, {verification_pos[1]:.3f}, {verification_pos[2]:.3f})")
          
          # 设置目标关节角度
          self.current_target_joints = [math.radians(angle) for angle in target_joints_deg]
          
          return True
          
      except Exception as e:
          print(f"❌ IK求解失败: {e}")
          return False
  
  def update_control(self):
      """更新PD控制器"""
      for i, (joint_id, actuator_id) in enumerate(zip(self.joint_ids, self.actuator_ids)):
          if joint_id >= 0 and actuator_id >= 0 and i < len(self.current_target_joints):
              current_pos = self.data.qpos[joint_id]
              current_vel = self.data.qvel[joint_id]
              target_pos = self.current_target_joints[i]
              
              # PD控制
              pos_error = target_pos - current_pos
              vel_error = 0 - current_vel
              control_torque = self.kp * pos_error + self.kd * vel_error
              
              # 限制扭矩
              control_torque = np.clip(control_torque, -20.0, 20.0)
              self.data.ctrl[actuator_id] = control_torque
  
  def get_current_ee_position(self):
      """获取当前末端执行器位置"""
      try:
          # 尝试使用gripper site
          try:
              gripper_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'gripper')
              return self.data.site_xpos[gripper_site_id].copy()
          except:
              pass
          
          # 尝试使用gripper body
          try:
              gripper_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'gripper')
              return self.data.xpos[gripper_body_id].copy()
          except:
              pass
          
          # 使用正运动学计算
          current_joints_deg = self.get_current_joint_angles_deg()
          pose = self.kinematics.forward_kinematics(
              np.array(current_joints_deg, dtype=np.float32), "gripper_tip"
          )
          return pose[:3, 3]
          
      except Exception as e:
          print(f"获取末端执行器位置失败: {e}")
          return np.array([0.0, 0.0, 0.0])
  
  def run_preset_tests(self):
      """运行预设测试点"""
      print("\n🧪 开始预设IK测试...")
      
      # 定义测试点
      test_points = [
          (0.3, 0.0, 0.3, "前方点"),
          (0.0, 0.3, 0.3, "左侧点"),
          (0.2, 0.2, 0.4, "对角点"),
          (0.1, 0.0, 0.5, "高点")
      ]
      
      for x, y, z, description in test_points:
          print(f"\n--- 测试{description}: ({x}, {y}, {z}) ---")
          success = self.test_ik(x, y, z, position_only=True)
          
          if success:
              print(f"✅ {description}IK求解成功")
              time.sleep(1.0)  # 短暂等待
          else:
              print(f"❌ {description}IK求解失败")
  
  def keyboard_input_thread(self):
      """键盘输入线程"""
      while self.running:
          try:
              command = input().strip()
              if command:
                  self.command_queue.put(command)
          except (EOFError, KeyboardInterrupt):
              self.running = False
              break
  
  def process_command(self, command):
      """处理用户命令"""
      parts = command.strip().split()
      
      if not parts:
          return
      
      cmd = parts[0].lower()
      
      if cmd in ['quit', 'q', 'exit']:
          self.running = False
          print("🔚 退出程序")
      
      elif cmd == 'move' and len(parts) >= 4:
          try:
              x = float(parts[1])
              y = float(parts[2])
              z = float(parts[3])
              
              print(f"\n🎯 移动到: ({x:.3f}, {y:.3f}, {z:.3f})")
              success = self.test_ik(x, y, z, position_only=True)
              
              if success:
                  print("✅ IK求解成功，机械臂开始移动")
              else:
                  print("❌ IK求解失败")
                  
          except ValueError:
              print("❌ 坐标格式错误，请使用: move x y z")
      
      elif cmd == 'joints' and len(parts) >= 7:
          try:
              angles = [float(parts[i]) for i in range(1, 7)]
              self.current_target_joints = [math.radians(angle) for angle in angles]
              print(f"🎯 设置关节角度: {[f'{a:.1f}°' for a in angles]}")
          except ValueError:
              print("❌ 角度格式错误，请使用: joints a1 a2 a3 a4 a5 a6")
      
      elif cmd == 'status':
          current_pos = self.get_current_ee_position()
          current_joints = self.get_current_joint_angles_deg()
          target_joints = [math.degrees(a) for a in self.current_target_joints]
          
          print(f"\n📊 当前状态:")
          print(f"   末端位置: ({current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f})")
          print(f"   当前关节: {[f'{a:.1f}°' for a in current_joints]}")
          print(f"   目标关节: {[f'{a:.1f}°' for a in target_joints]}")
      
      elif cmd == 'hold':
          # 🔧 新增功能：保持当前位置
          self.current_target_joints = self.get_current_joint_angles_rad()
          current_joints = self.get_current_joint_angles_deg()
          print(f"🔒 保持当前位置: {[f'{a:.1f}°' for a in current_joints]}")
      
      elif cmd == 'home':
          self.current_target_joints = [0, 0, 0, 0, 0, 0]
          print("🏠 回到零位")
      
      elif cmd == 'preset':
          self.run_preset_tests()
      
      elif cmd == 'help':
          print("\n🎮 可用命令:")
          print("  move x y z           - 移动到指定位置")
          print("  joints a1 a2 a3 a4 a5 a6 - 设置关节角度(度)")
          print("  status               - 显示当前状态")
          print("  hold                 - 保持当前位置")
          print("  home                 - 回到零位")
          print("  preset               - 运行预设测试")
          print("  help                 - 显示帮助")
          print("  quit                 - 退出")
          print("\n示例:")
          print("  move 0.3 0.0 0.3")
          print("  joints 0 -90 90 45 0 -30")
      
      else:
          print("❌ 未知命令，输入 'help' 查看可用命令")

def main():
  """主函数"""
  # 加载mujoco场景
  xml_path = "manipulator_grasp/assets/SO101/scene_so101.xml"
  try:
      model = mujoco.MjModel.from_xml_path(xml_path)
      data = mujoco.MjData(model)
      print(f"✅ 成功加载场景: {xml_path}")
  except Exception as e:
      print(f"❌ 场景加载失败: {e}")
      return
  
  # 🔧 关键修改：不重置到keyframe，保持XML文件中定义的初始状态
  # 注释掉原来的重置代码：
  # mujoco.mj_resetDataKeyframe(model, data, 0)
  
  # 只进行一次前向仿真步骤来初始化物理状态
  mujoco.mj_forward(model, data)
  print("🔧 保持XML文件中定义的自然初始状态")
  
  # 创建简化版IK测试器
  try:
      ik_tester = SimplifiedIKTester(model, data)
  except Exception as e:
      print(f"❌ IK测试器初始化失败: {e}")
      return
  
  print("\n=== 简化版SO101逆运动学测试（自然状态）===")
  print("🎯 机械臂保持XML文件中定义的自然状态")
  print("🔧 专注于基础IK控制和测试")
  
  # 显示初始状态信息
  initial_pos = ik_tester.get_current_ee_position()
  initial_joints = ik_tester.get_current_joint_angles_deg()
  print(f"📊 初始状态:")
  print(f"   末端位置: ({initial_pos[0]:.3f}, {initial_pos[1]:.3f}, {initial_pos[2]:.3f})")
  print(f"   关节角度: {[f'{a:.1f}°' for a in initial_joints]}")
  
  # 启动仿真
  with mujoco.viewer.launch_passive(model, data) as viewer:
      # 等待viewer启动
      time.sleep(1.0)
      
      # 启动键盘输入线程
      input_thread = threading.Thread(target=ik_tester.keyboard_input_thread, daemon=True)
      input_thread.start()
      
      print("\n🎮 交互模式已启动！")
      print("输入 'help' 查看可用命令")
      print("输入 'status' 查看当前状态")
      print("输入 'hold' 保持当前位置")
      print("输入 'move 0.3 0.0 0.3' 测试移动")
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