"""
SO101机械臂IK运算专用测试版本
专注功能：
1. 纯IK算法测试（机械臂在原点，坐标系重合）
2. 直接空间坐标到关节角度转换
3. 精度验证和误差分析
4. 简化的控制接口
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import math
import sys
import os

# 添加运动学模块路径
sys.path.append(os.path.join(os.path.dirname(__file__), 'lerobot/common/model'))

try:
  from kinematics import Robot, RobotKinematics, RobotUtils
  print("✅ 成功导入运动学模块")
except ImportError as e:
  print(f"❌ 导入运动学模块失败: {e}")
  sys.exit(1)

class SimpleIKController:
  """简化IK控制器 - 专注于IK算法测试"""
  
  def __init__(self, model, data):
      self.model = model
      self.data = data
      
      # 初始化kinematics库
      try:
          self.robot = Robot(robot_type="so100")
          self.kin = RobotKinematics()
          print("✅ kinematics库初始化成功")
      except Exception as e:
          print(f"❌ kinematics库初始化失败: {e}")
          sys.exit(1)
      
      # 获取关节映射
      self.joint_names = ['1', '2', '3', '4', '5', '6']
      self.joint_ids = []
      self.actuator_ids = []
      
      for joint_name in self.joint_names:
          joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
          actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name)
          self.joint_ids.append(joint_id)
          self.actuator_ids.append(actuator_id)
      
      # 控制参数
      self.kp = 50.0
      self.kd = 5.0
      
      # 初始化目标关节角度
      self.target_joints = [self.data.qpos[joint_id] for joint_id in self.joint_ids]
      
      print("🎯 简化IK控制器初始化完成")
      print("📍 机械臂基座在原点，坐标系重合")
      self._show_current_position()
  
  def get_current_end_effector_position(self):
      """获取当前末端执行器位置"""
      try:
          # 获取当前关节角度
          current_joints_mech = np.array([self.data.qpos[joint_id] for joint_id in self.joint_ids])
          
          # 转换为DH角度
          current_joints_dh = self.robot.from_mech_to_dh(current_joints_mech)
          
          # 正向运动学计算
          T_current = self.kin.forward_kinematics(self.robot, current_joints_dh)
          
          # 提取位置信息
          position = T_current[:3, 3]
          
          return {
              'position': position,
              'transformation_matrix': T_current,
              'joint_angles_dh': current_joints_dh,
              'joint_angles_mech': current_joints_mech
          }
      except Exception as e:
          print(f"❌ 获取末端位置失败: {e}")
          return None
  
  def _show_current_position(self):
      """显示当前末端位置"""
      pos_info = self.get_current_end_effector_position()
      if pos_info:
          pos = pos_info['position']
          joints_deg = np.rad2deg(pos_info['joint_angles_mech'])
          print(f"📍 正向运动学计算-当前末端位置: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
          print(f"🔧 当前关节角度: {[f'{a:.1f}°' for a in joints_deg]}")
  
  def test_ik_single_point(self, target_pos, use_orientation=True):
      """
      测试单点IK运算
      参数：
      - target_pos: 目标位置 [x, y, z]
      - use_orientation: 是否保持当前姿态
      """
      target_pos = np.array(target_pos)
      
      print(f"\n🎯 IK测试 - 目标位置: ({target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f})")
      
      # 获取当前状态
      current_info = self.get_current_end_effector_position()
      if current_info is None:
          return False, None
      
      current_pos = current_info['position']
      T_current = current_info['transformation_matrix']
      q_current_dh = current_info['joint_angles_dh']
      
      # 计算移动距离
      move_distance = np.linalg.norm(target_pos - current_pos)
      print(f"📏 移动距离: {move_distance:.3f}m ({move_distance*1000:.1f}mm)")
      
      try:
          # 构建目标变换矩阵
          T_goal = T_current.copy()
          T_goal[:3, 3] = target_pos
          
          # IK求解
          print("⚙️  开始IK求解...")
          start_time = time.time()
          
          q_solution_dh = self.kin.inverse_kinematics(
              self.robot, 
              q_current_dh, 
              T_goal, 
              use_orientation=use_orientation,
              k=0.8,
              n_iter=100
          )
          
          solve_time = time.time() - start_time
          print(f"⏱️  IK求解耗时: {solve_time:.3f}秒")
          
          # 验证IK解的准确性
          T_verify = self.kin.forward_kinematics(self.robot, q_solution_dh)
          actual_pos = T_verify[:3, 3]
          
          # 计算误差
          position_error = np.linalg.norm(actual_pos - target_pos)
          
          print(f"📊 IK验证结果:")
          print(f"   目标位置: ({target_pos[0]:.6f}, {target_pos[1]:.6f}, {target_pos[2]:.6f})")
          print(f"   实际位置: ({actual_pos[0]:.6f}, {actual_pos[1]:.6f}, {actual_pos[2]:.6f})")
          print(f"   位置误差: {position_error:.6f}m = {position_error*1000:.3f}mm")
          
          # 误差评估
          if position_error < 0.001:  # 1mm
              print("✅ 精度优秀 (误差 < 1mm)")
          elif position_error < 0.005:  # 5mm
              print("✅ 精度良好 (误差 < 5mm)")
          elif position_error < 0.01:  # 1cm
              print("⚠️  精度一般 (误差 < 1cm)")
          else:
              print("❌ 精度较差 (误差 > 1cm)")
          
          # 转换为机械角度
          q_solution_mech = self.robot.from_dh_to_mech(q_solution_dh)
          solution_joints_deg = np.rad2deg(q_solution_mech)
          
          print(f"🔧 解算关节角度: {[f'{a:.1f}°' for a in solution_joints_deg]}")
          
          # 检查关节限制
          try:
              q_check = np.append(q_solution_mech, 0.0)  # 添加gripper角度
              self.robot.check_joint_limits(q_check)
              print("✅ 关节限制检查通过")
          except Exception as limit_error:
              print(f"⚠️  关节限制警告: {limit_error}")
          
          # 更新目标关节角度
          self.target_joints = q_solution_mech.tolist()
          
          return True, {
              'target_position': target_pos,
              'actual_position': actual_pos,
              'position_error': position_error,
              'joint_angles_deg': solution_joints_deg,
              'solve_time': solve_time
          }
          
      except Exception as e:
          print(f"❌ IK求解失败: {e}")
          return False, None
  
  def test_ik_multiple_points(self, test_points):
      """测试多个点的IK运算"""
      print(f"\n🧪 批量IK测试 - {len(test_points)}个测试点")
      print("=" * 60)
      
      results = []
      total_time = 0
      success_count = 0
      
      for i, point in enumerate(test_points, 1):
          print(f"\n📍 测试点 {i}/{len(test_points)}")
          success, result = self.test_ik_single_point(point)
          
          if success:
              success_count += 1
              total_time += result['solve_time']
              results.append(result)
              
              # 执行移动验证
              print("🔄 执行移动验证...")
              self._execute_movement(duration=2.0)
          else:
              results.append(None)
          
          print("-" * 40)
      
      # 统计结果
      print(f"\n📊 批量测试统计:")
      print(f"   成功率: {success_count}/{len(test_points)} ({success_count/len(test_points)*100:.1f}%)")
      
      if success_count > 0:
          avg_time = total_time / success_count
          errors = [r['position_error'] for r in results if r is not None]
          avg_error = np.mean(errors)
          max_error = np.max(errors)
          
          print(f"   平均求解时间: {avg_time:.3f}秒")
          print(f"   平均位置误差: {avg_error:.6f}m ({avg_error*1000:.3f}mm)")
          print(f"   最大位置误差: {max_error:.6f}m ({max_error*1000:.3f}mm)")
      
      return results
  
  def _execute_movement(self, duration=3.0):
      """执行机械臂移动"""
      start_time = time.time()
      while time.time() - start_time < duration:
          self._update_control()
          mujoco.mj_step(self.model, self.data)
          time.sleep(0.01)
  
  def _update_control(self):
      """更新PD控制器"""
      for i, (joint_id, actuator_id) in enumerate(zip(self.joint_ids, self.actuator_ids)):
          if i < len(self.target_joints):
              current_pos = self.data.qpos[joint_id]
              current_vel = self.data.qvel[joint_id]
              target_pos = self.target_joints[i]
              
              pos_error = target_pos - current_pos
              vel_error = 0 - current_vel
              control_torque = self.kp * pos_error + self.kd * vel_error
              
              self.data.ctrl[actuator_id] = control_torque

def main():
  """主函数"""
  xml_path = "manipulator_grasp/assets/SO101/scene_so101.xml"
  model = mujoco.MjModel.from_xml_path(xml_path)
  data = mujoco.MjData(model)
  
  mujoco.mj_resetDataKeyframe(model, data, 0)
  
  controller = SimpleIKController(model, data)
  
  print("\n=== 机械臂IK运算专用测试系统 ===")
  print("🎯 专注功能：纯IK算法测试和验证")
  print("📍 机械臂基座在原点，坐标系重合")
  
  # 预定义测试点
  test_points_basic = [
      [0.3, 0.0, 0.2],    # 正前方
      [0.0, 0.3, 0.2],    # 正右方
      [-0.2, 0.2, 0.3],   # 左前方
      [0.4, -0.1, 0.1],   # 右后下方
      [0.0, 0.0, 0.5],    # 正上方
  ]
  
  test_points_precision = [
      [0.25, 0.15, 0.25],
      [0.35, -0.05, 0.15],
      [-0.15, 0.25, 0.35],
      [0.2, 0.2, 0.4],
      [0.45, 0.0, 0.2],
  ]
  
  with mujoco.viewer.launch_passive(model, data) as viewer:
      time.sleep(1.0)
      
      print("\n🎮 IK测试命令:")
      print("\n📍 单点测试:")
      print("  ik x y z - 测试指定位置的IK运算")
      print("  例如: ik 0.3 0.1 0.2")
      
      print("\n🧪 批量测试:")
      print("  test_basic - 测试5个基础位置点")
      print("  test_precision - 测试5个精度验证点")
      print("  test_custom - 自定义测试点序列")
      
      print("\n📊 信息查询:")
      print("  pos - 查看当前位置")
      print("  home - 回到初始位置")
      print("  quit - 退出")
      
      while viewer.is_running():
          try:
              user_input = input("\n请输入命令: ").strip()
              
              if user_input.lower() == 'quit':
                  break
              
              if user_input.lower() == 'pos':
                  controller._show_current_position()
                  continue
              
              if user_input.lower() == 'home':
                  print("🏠 回到初始位置...")
                  success, result = controller.test_ik_single_point([0.0, 0.0, 0.4])
                  if success:
                      controller._execute_movement(3.0)
                      print("✅ 已回到初始位置")
                  continue
              
              # 单点IK测试
              if user_input.lower().startswith('ik'):
                  parts = user_input.split()
                  if len(parts) >= 4:
                      try:
                          x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                          success, result = controller.test_ik_single_point([x, y, z])
                          
                          if success:
                              print("\n⏳ 执行移动...")
                              controller._execute_movement(3.0)
                              print("✅ 移动完成")
                              controller._show_current_position()
                      except ValueError:
                          print("❌ 坐标格式错误，请输入数字")
                  else:
                      print("❌ 命令格式: ik x y z")
                  continue
              
              # 批量测试
              if user_input.lower() == 'test_basic':
                  controller.test_ik_multiple_points(test_points_basic)
                  continue
              
              if user_input.lower() == 'test_precision':
                  controller.test_ik_multiple_points(test_points_precision)
                  continue
              
              if user_input.lower() == 'test_custom':
                  print("请输入测试点序列 (格式: x1,y1,z1 x2,y2,z2 ...):")
                  custom_input = input().strip()
                  try:
                      custom_points = []
                      for point_str in custom_input.split():
                          coords = point_str.split(',')
                          if len(coords) == 3:
                              x, y, z = float(coords[0]), float(coords[1]), float(coords[2])
                              custom_points.append([x, y, z])
                      
                      if custom_points:
                          controller.test_ik_multiple_points(custom_points)
                      else:
                          print("❌ 未解析到有效测试点")
                  except ValueError:
                      print("❌ 格式错误，例如: 0.3,0.1,0.2 0.4,0.0,0.3")
                  continue
              
              print("❌ 未知命令，请参考帮助信息")
              
          except KeyboardInterrupt:
              break
          except EOFError:
              break
  
  print("✅ IK测试程序结束")

if __name__ == "__main__":
  main()