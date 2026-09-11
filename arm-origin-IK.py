import argparse
import time
import threading
import queue
import math
import sys
import os
import numpy as np
import mujoco
import mujoco.viewer

# 添加运动学模块路径
sys.path.append(os.path.join(os.path.dirname(__file__), 'lerobot/common/model'))

try:
  from kinematics import RobotKinematics
  print("✅ 成功导入RobotKinematics模块")
except ImportError as e:
  print(f"❌ 导入RobotKinematics失败: {e}")

class SimpleArmController:
  """精简版机械臂控制器 - 专注于关节控制"""
  
  def __init__(self, model, data):
      self.model = model
      self.data = data
      
      # 初始化运动学模块
      try:
          self.kinematics = RobotKinematics("so_new_calibration")
          print("✅ 运动学模块初始化成功")
      except:
          print("❌ 运动学模块初始化失败，使用基础控制模式")
          self.kinematics = None
      
      # 机械臂关节配置 - 假设前6个关节是机械臂
      self.arm_joints = list(range(min(6, model.nq)))
      self.arm_actuators = list(range(min(6, model.nu)))
      
      print(f"🦾 机械臂关节数: {len(self.arm_joints)}")
      print(f"🎛️ 执行器数: {len(self.arm_actuators)}")
      
      # PD控制参数
      self.kp = 50.0  # 位置增益
      self.kd = 5.0   # 速度增益
      
      # 目标关节角度
      self.target_joints = [0.0] * len(self.arm_joints)
      
      # 交互控制
      self.command_queue = queue.Queue()
      self.running = True
      self.enabled = True
      
      print("🔧 控制器初始化完成")
  
  def get_current_joints_deg(self):
      """获取当前关节角度（度）"""
      return [math.degrees(self.data.qpos[i]) for i in self.arm_joints]
  
  def set_joint_angles(self, angles_deg):
      """设置目标关节角度（度）"""
      if len(angles_deg) != len(self.arm_joints):
          print(f"❌ 角度数量不匹配: 需要{len(self.arm_joints)}个，提供{len(angles_deg)}个")
          return False
      
      # 限制关节角度范围
      self.target_joints = [
          np.clip(math.radians(angle), -math.pi, math.pi) 
          for angle in angles_deg
      ]
      
      print(f"🎯 设置目标角度: {[f'{a:.1f}°' for a in angles_deg]}")
      return True
  
  def move_to_position(self, x, y, z):
      """使用IK移动到指定位置"""
      if not self.kinematics:
          print("❌ 运动学模块未可用，无法执行IK")
          return False
      
      print(f"🎯 IK移动到: ({x:.3f}, {y:.3f}, {z:.3f})")
      
      try:
          # 获取当前关节角度
          current_joints = self.get_current_joints_deg()
          
          # 创建目标位姿矩阵
          target_pose = np.eye(4, dtype=np.float32)
          target_pose[:3, 3] = [x, y, z]
          
          # 执行逆运动学
          target_joints = self.kinematics.ik(
              current_joint_pos=np.array(current_joints, dtype=np.float32),
              desired_ee_pose=target_pose,
              position_only=True,
              frame="gripper_tip",
              max_iterations=50,
              learning_rate=0.3
          )
          
          # 设置目标角度
          return self.set_joint_angles(target_joints[:len(self.arm_joints)])
          
      except Exception as e:
          print(f"❌ IK计算失败: {e}")
          return False
  
  def update_control(self):
      """更新PD控制"""
      if not self.enabled:
          return
      
      for i, (joint_id, actuator_id) in enumerate(zip(self.arm_joints, self.arm_actuators)):
          if i < len(self.target_joints):
              # 当前状态
              current_pos = self.data.qpos[joint_id]
              current_vel = self.data.qvel[joint_id]
              target_pos = self.target_joints[i]
              
              # PD控制
              error = target_pos - current_pos
              torque = self.kp * error - self.kd * current_vel
              
              # 限制扭矩
              torque = np.clip(torque, -20.0, 20.0)
              self.data.ctrl[actuator_id] = torque
  
  def keyboard_input_thread(self):
      """键盘输入处理线程"""
      while self.running:
          try:
              cmd = input().strip()
              if cmd:
                  self.command_queue.put(cmd)
          except (EOFError, KeyboardInterrupt):
              self.running = False
              break
  
  def process_command(self, cmd):
      """处理用户命令"""
      parts = cmd.split()
      if not parts:
          return
      
      command = parts[0].lower()
      
      if command in ['quit', 'q', 'exit']:
          self.running = False
          print("🔚 退出程序")
      
      elif command == 'move' and len(parts) >= 4:
          # 移动到指定位置: move x y z
          try:
              x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
              self.move_to_position(x, y, z)
          except ValueError:
              print("❌ 格式错误: move x y z")
      
      elif command == 'joints' and len(parts) >= 2:
          # 设置关节角度: joints a1 a2 a3 a4 a5 a6
          try:
              angles = [float(parts[i]) for i in range(1, min(len(parts), len(self.arm_joints) + 1))]
              self.set_joint_angles(angles)
          except ValueError:
              print("❌ 格式错误: joints a1 a2 a3 a4 a5 a6")
      
      elif command == 'status':
          # 显示当前状态
          current = self.get_current_joints_deg()
          target = [math.degrees(a) for a in self.target_joints]
          print(f"📊 当前关节: {[f'{a:.1f}°' for a in current]}")
          print(f"🎯 目标关节: {[f'{a:.1f}°' for a in target]}")
      
      elif command == 'home':
          # 回到零位
          self.set_joint_angles([0.0] * len(self.arm_joints))
          print("🏠 回到零位")
      
      elif command == 'preset':
          # 预设位置
          if len(parts) >= 2:
              preset_name = parts[1].lower()
              if preset_name == 'ready':
                  self.set_joint_angles([0, -90, 90, 45, 0, -30])
              elif preset_name == 'up':
                  self.set_joint_angles([0, -45, 45, 0, 0, 0])
              else:
                  print("❌ 未知预设，可用: ready, up")
          else:
              print("❌ 格式: preset <name>")
      
      elif command == 'cube':
          # 推立方体任务序列
          print("🎲 执行推立方体任务...")
          cube_positions = [
              [0.06, 0.135, 0.08],   # 立方体上方
              [0.06, 0.135, 0.03],   # 接触立方体
              [-0.06, 0.135, 0.03],  # 推到另一边
              [-0.06, 0.135, 0.08]   # 抬起
          ]
          
          for i, pos in enumerate(cube_positions):
              print(f"  步骤{i+1}: 移动到 {pos}")
              if self.move_to_position(pos[0], pos[1], pos[2]):
                  time.sleep(2.0)  # 等待运动完成
              else:
                  print("❌ 任务中断")
                  break
      
      elif command == 'help':
          print("\n🎮 可用命令:")
          print("  move x y z        - 移动到指定位置")
          print("  joints a1 a2 ... - 设置关节角度(度)")
          print("  status            - 显示当前状态")
          print("  home              - 回到零位")
          print("  preset <name>     - 预设位置(ready/up)")
          print("  cube              - 推立方体任务")
          print("  quit              - 退出")
          print("\n示例:")
          print("  move 0.06 0.135 0.05")
          print("  joints 0 -90 90 45 0 -30")
      
      else:
          print("❌ 未知命令，输入 'help' 查看帮助")

def main():
  """主函数"""
  parser = argparse.ArgumentParser(description="精简版SO101机械臂控制器")
  parser.add_argument("--xml", default="manipulator_grasp/assets/SO101/push_cube_loop.xml", 
                     help="XML模型文件路径")
  args = parser.parse_args()
  
  # 加载模型
  try:
      print(f"🔄 加载模型: {args.xml}")
      model = mujoco.MjModel.from_xml_path(args.xml)
      data = mujoco.MjData(model)
      
      # 重置到初始状态
      mujoco.mj_resetData(model, data)
      if model.nkey > 0:
          mujoco.mj_resetDataKeyframe(model, data, 0)
      
      print(f"✅ 模型加载成功: nq={model.nq}, nu={model.nu}")
      
  except Exception as e:
      print(f"❌ 模型加载失败: {e}")
      return
  
  # 创建控制器
  try:
      controller = SimpleArmController(model, data)
  except Exception as e:
      print(f"❌ 控制器创建失败: {e}")
      return
  
  # 启动仿真
  with mujoco.viewer.launch_passive(model, data) as viewer:
      # 启动键盘输入线程
      input_thread = threading.Thread(target=controller.keyboard_input_thread, daemon=True)
      input_thread.start()
      
      print("\n=== 精简版SO101机械臂控制器 ===")
      print("📝 输入 'help' 查看命令")
      print("🎮 示例: move 0.06 0.135 0.05")
      print("=" * 40)
      
      # 主仿真循环
      while viewer.is_running() and controller.running:
          step_start = time.time()
          
          # 处理用户命令
          try:
              cmd = controller.command_queue.get_nowait()
              controller.process_command(cmd)
          except queue.Empty:
              pass
          
          # 更新控制
          controller.update_control()
          
          # 仿真步进
          mujoco.mj_step(model, data)
          viewer.sync()
          
          # 控制仿真频率
          time_until_next = model.opt.timestep - (time.time() - step_start)
          if time_until_next > 0:
              time.sleep(time_until_next)
  
  controller.running = False
  print("✅ 仿真结束")

if __name__ == "__main__":
  main()