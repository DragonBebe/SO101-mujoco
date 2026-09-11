"""
SO101机械臂调试控制脚本
- joints: 直接控制关节角度，测试正运动学
- move: 通过IK控制末端位置，测试逆运动学
- 验证正运动学和坐标系转换
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
from scipy.spatial.transform import Rotation as R

# 添加运动学模块路径
sys.path.append(os.path.join(os.path.dirname(__file__), 'lerobot/common/model'))

# 导入运动学库
try:
    from kinematics import RobotKinematics
    HAS_REAL_KINEMATICS = True
    print("✅ 成功导入RobotKinematics模块")
except ImportError as e:
    print(f"❌ 导入RobotKinematics失败: {e}")
    sys.exit(1)

class DebugController:
    """调试控制器 - 支持关节控制和IK控制"""
    
    def __init__(self, model, data):
        self.model = model
        self.data = data
        
        # 🔧 机械臂基座在mujoco空间中的偏移量和旋转
        self.base_offset = np.array([0.8, 0.6, 0.71])
        
        # 基座旋转 - 根据观察到的84度旋转
        base_quat = np.array([1.0, 0.0, 0.0, 0.90])
        base_quat = base_quat / np.linalg.norm(base_quat)
        self.base_rotation = R.from_quat([base_quat[1], base_quat[2], base_quat[3], base_quat[0]])
        
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
        self.kp = 100.0
        self.kd = 10.0
        
        # 目标关节角度（弧度）
        self.target_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # 交互控制
        self.command_queue = queue.Queue()
        self.running = True
        self.moving = False
    
    def world_to_robot_coords(self, world_pos):
        """世界坐标转换为机械臂本体坐标系 - 修正x/y轴反向问题"""
        # 首先减去基座偏移
        translated_pos = np.array(world_pos) - self.base_offset
        
        # 🔧 修正坐标轴映射：交换x和y轴
        # MuJoCo: (x, y, z) -> 运动学: (y, x, z)
        corrected_pos = np.array([translated_pos[1], translated_pos[0], translated_pos[2]])
        
        # 应用基座旋转的逆变换
        robot_pos = self.base_rotation.inv().apply(corrected_pos)
        
        print(f"🔄 坐标转换 (世界→机械臂):")
        print(f"   世界坐标: ({world_pos[0]:.3f}, {world_pos[1]:.3f}, {world_pos[2]:.3f})")
        print(f"   平移后: ({translated_pos[0]:.3f}, {translated_pos[1]:.3f}, {translated_pos[2]:.3f})")
        print(f"   轴交换后: ({corrected_pos[0]:.3f}, {corrected_pos[1]:.3f}, {corrected_pos[2]:.3f})")
        print(f"   机械臂坐标: ({robot_pos[0]:.3f}, {robot_pos[1]:.3f}, {robot_pos[2]:.3f})")
        
        return robot_pos
    
    def robot_to_world_coords(self, robot_pos):
        """机械臂本体坐标系转换为世界坐标 - 修正x/y轴反向问题"""
        # 应用基座旋转
        rotated_pos = self.base_rotation.apply(robot_pos)
        
        # 🔧 修正坐标轴映射：交换x和y轴
        # 运动学: (x, y, z) -> MuJoCo: (y, x, z)
        corrected_pos = np.array([rotated_pos[1], rotated_pos[0], rotated_pos[2]])
        
        # 加上基座偏移
        world_pos = corrected_pos + self.base_offset
        
        print(f"🔄 坐标转换 (机械臂→世界):")
        print(f"   机械臂坐标: ({robot_pos[0]:.3f}, {robot_pos[1]:.3f}, {robot_pos[2]:.3f})")
        print(f"   旋转后: ({rotated_pos[0]:.3f}, {rotated_pos[1]:.3f}, {rotated_pos[2]:.3f})")
        print(f"   轴交换后: ({corrected_pos[0]:.3f}, {corrected_pos[1]:.3f}, {corrected_pos[2]:.3f})")
        print(f"   世界坐标: ({world_pos[0]:.3f}, {world_pos[1]:.3f}, {world_pos[2]:.3f})")
        
        return world_pos
    
    def get_current_joint_angles_deg(self):
        """获取当前关节角度（度）"""
        angles_rad = [self.data.qpos[joint_id] if joint_id >= 0 else 0 
                     for joint_id in self.joint_ids]
        return [math.degrees(angle) for angle in angles_rad]
    
    def get_mujoco_ee_position(self):
        """从MuJoCo中直接获取末端执行器位置"""
        try:
            # 尝试找到末端执行器的site或body
            ee_names = ['gripper_tip', 'end_effector', 'ee_link', 'tool0']
            
            for name in ee_names:
                try:
                    site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, name)
                    if site_id >= 0:
                        ee_pos = self.data.site_xpos[site_id].copy()
                        print(f"📍 从MuJoCo site '{name}' 获取位置: {ee_pos}")
                        return ee_pos
                except:
                    pass
                
                try:
                    body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
                    if body_id >= 0:
                        ee_pos = self.data.xpos[body_id].copy()
                        print(f"📍 从MuJoCo body '{name}' 获取位置: {ee_pos}")
                        return ee_pos
                except:
                    pass
            
            print("⚠️  未找到末端执行器，使用最后一个body位置")
            return self.data.xpos[-1].copy()
            
        except Exception as e:
            print(f"❌ 获取MuJoCo末端位置失败: {e}")
            return np.array([0, 0, 0])
    
    def get_kinematics_ee_position(self):
        """使用运动学模块计算末端执行器位置"""
        try:
            current_joints_deg = self.get_current_joint_angles_deg()
            
            # 使用运动学模块计算正运动学
            pose_matrix = self.kinematics.forward_kinematics(
                np.array(current_joints_deg, dtype=np.float32), 
                "gripper_tip"
            )
            
            robot_pos = pose_matrix[:3, 3]
            world_pos = self.robot_to_world_coords(robot_pos)
            
            return world_pos
            
        except Exception as e:
            print(f"❌ 运动学计算失败: {e}")
            return np.array([0, 0, 0])
    
    def create_pose_matrix(self, x, y, z, roll=0, pitch=0, yaw=0):
        """创建4x4位姿矩阵"""
        pose = np.eye(4, dtype=np.float32)
        r = R.from_euler('xyz', [roll, pitch, yaw], degrees=True)
        pose[:3, :3] = r.as_matrix()
        pose[:3, 3] = [x, y, z]
        return pose
    
    def set_joint_angles(self, joint_angles_deg):
        """设置目标关节角度（度）- 测试正运动学"""
        print(f"\n🎯 【正运动学测试】设置关节角度: {[f'{a:.1f}°' for a in joint_angles_deg]}")
        
        # 限制关节角度范围
        joint_limits = [
            [-180, 180],   # 关节1
            [-11.5, 180],  # 关节2
            [-85.9, 85.9], # 关节3
            [-180, 180],   # 关节4
            [-180, 180],   # 关节5
            [-180, 180],   # 关节6
        ]
        
        limited_angles = []
        for i, angle in enumerate(joint_angles_deg):
            if i < len(joint_limits):
                limited_angle = np.clip(angle, joint_limits[i][0], joint_limits[i][1])
                limited_angles.append(limited_angle)
                if abs(limited_angle - angle) > 0.1:
                    print(f"⚠️  关节{i+1}角度被限制: {angle:.1f}° → {limited_angle:.1f}°")
            else:
                limited_angles.append(angle)
        
        # 转换为弧度并设置目标
        self.target_joints = [math.radians(angle) for angle in limited_angles]
        
        # 等待一小段时间让关节移动
        time.sleep(0.5)
        
        # 显示位置信息
        self.show_position_comparison()
    
    def move_to_position(self, world_x, world_y, world_z, roll=0, pitch=0, yaw=0):
        """移动到指定位置 - 测试逆运动学"""
        print(f"\n🎯 【逆运动学测试】移动到位置")
        print(f"   目标位置(世界坐标): ({world_x:.3f}, {world_y:.3f}, {world_z:.3f})")
        
        # 转换为机械臂坐标系
        robot_target_pos = self.world_to_robot_coords([world_x, world_y, world_z])
        
        # 检查目标是否在合理范围内
        target_distance = np.linalg.norm(robot_target_pos)
        if target_distance > 0.5:  # 50cm工作半径限制
            print(f"⚠️  目标距离较远: {target_distance*100:.1f}cm，可能超出工作空间")
        
        # 获取当前关节角度
        current_joints_deg = self.get_current_joint_angles_deg()
        
        # 创建目标位姿矩阵
        target_pose_matrix = self.create_pose_matrix(
            robot_target_pos[0], robot_target_pos[1], robot_target_pos[2],
            roll, pitch, yaw
        )
        
        try:
            # 执行IK求解
            target_joints_deg = self.kinematics.ik(
                current_joint_pos=np.array(current_joints_deg, dtype=np.float32),
                desired_ee_pose=target_pose_matrix,
                position_only=True,
                frame="gripper_tip",
                max_iterations=100,
                learning_rate=0.1
            )
            
            print(f"✅ IK求解成功")
            print(f"   目标关节角度: {[f'{a:.1f}°' for a in target_joints_deg]}")
            
            # 验证IK解
            verification_pose = self.kinematics.forward_kinematics(
                np.array(target_joints_deg), "gripper_tip"
            )
            verification_world_pos = self.robot_to_world_coords(verification_pose[:3, 3])
            pos_error = np.linalg.norm(verification_world_pos - np.array([world_x, world_y, world_z]))
            print(f"   验证误差: {pos_error*1000:.1f}mm")
            
            # 应用关节限制
            joint_limits = [
                [-180, 180],   # 关节1
                [-11.5, 180],  # 关节2
                [-85.9, 85.9], # 关节3
                [-180, 180],   # 关节4
                [-180, 180],   # 关节5
                [-180, 180],   # 关节6
            ]
            
            limited_joints = []
            for i, angle in enumerate(target_joints_deg):
                if i < len(joint_limits):
                    limited_angle = np.clip(angle, joint_limits[i][0], joint_limits[i][1])
                    limited_joints.append(limited_angle)
                    if abs(limited_angle - angle) > 0.1:
                        print(f"⚠️  关节{i+1}角度被限制: {angle:.1f}° → {limited_angle:.1f}°")
                else:
                    limited_joints.append(angle)
            
            # 设置目标关节角度
            self.target_joints = [math.radians(angle) for angle in limited_joints]
            self.moving = True
            
            # 等待移动完成后显示结果
            time.sleep(1.0)
            self.show_position_comparison()
            
            return True
            
        except Exception as e:
            print(f"❌ IK求解失败: {e}")
            return False
    
    def show_position_comparison(self):
        """显示位置对比信息"""
        print(f"\n📊 位置对比:")
        
        # 当前关节角度
        current_joints = self.get_current_joint_angles_deg()
        print(f"   当前关节角度: {[f'{a:.1f}°' for a in current_joints]}")
        
        # MuJoCo中的末端位置
        mujoco_pos = self.get_mujoco_ee_position()
        print(f"   MuJoCo末端位置: ({mujoco_pos[0]:.3f}, {mujoco_pos[1]:.3f}, {mujoco_pos[2]:.3f})")
        
        # 运动学计算的末端位置
        kinematics_pos = self.get_kinematics_ee_position()
        print(f"   运动学末端位置: ({kinematics_pos[0]:.3f}, {kinematics_pos[1]:.3f}, {kinematics_pos[2]:.3f})")
        
        # 计算差异
        if np.linalg.norm(kinematics_pos) > 0:
            position_error = np.linalg.norm(mujoco_pos - kinematics_pos)
            print(f"   位置差异: {position_error*1000:.1f}mm")
            
            if position_error > 0.05:  # 5cm
                print(f"⚠️  位置差异较大，可能存在坐标系或运动学模型问题")
            else:
                print(f"✅ 位置差异在可接受范围内")
        
        print("-" * 60)
    
    def update_control(self):
        """更新PD控制器"""
        for i, (joint_id, actuator_id) in enumerate(zip(self.joint_ids, self.actuator_ids)):
            if joint_id >= 0 and actuator_id >= 0 and i < len(self.target_joints):
                current_pos = self.data.qpos[joint_id]
                current_vel = self.data.qvel[joint_id]
                target_pos = self.target_joints[i]
                
                pos_error = target_pos - current_pos
                vel_error = 0 - current_vel
                control_torque = self.kp * pos_error + self.kd * vel_error
                
                # 限制控制力矩
                control_torque = np.clip(control_torque, -200.0, 200.0)
                self.data.ctrl[actuator_id] = control_torque
    
    def check_movement_complete(self):
        """检查移动是否完成"""
        if not self.moving:
            return True
            
        # 检查关节是否接近目标
        tolerance = math.radians(2.0)  # 2度容差
        for i, joint_id in enumerate(self.joint_ids):
            if joint_id >= 0 and i < len(self.target_joints):
                current_pos = self.data.qpos[joint_id]
                target_pos = self.target_joints[i]
                if abs(current_pos - target_pos) > tolerance:
                    return False
        
        self.moving = False
        return True
    
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
        """处理命令"""
        parts = command.strip().split()
        
        if not parts:
            return
        
        cmd = parts[0].lower()
        
        if cmd in ['quit', 'q', 'exit']:
            self.running = False
            print("🔚 退出程序")
            return
        
        elif cmd == 'joints' and len(parts) >= 2:
            try:
                # 解析关节角度
                angles = []
                for i in range(1, min(len(parts), 7)):  # 最多6个关节
                    angles.append(float(parts[i]))
                
                # 如果提供的角度少于6个，用0补齐
                while len(angles) < 6:
                    angles.append(0.0)
                
                self.set_joint_angles(angles)
                
            except ValueError:
                print("❌ 关节角度格式错误，请使用: joints j1 j2 j3 j4 j5 j6")
        
        elif cmd == 'move' and len(parts) >= 4:
            try:
                x = float(parts[1])
                y = float(parts[2])
                z = float(parts[3])
                
                print(f"\n🎯 用户命令: 移动到 ({x:.3f}, {y:.3f}, {z:.3f})")
                
                success = self.move_to_position(x, y, z)
                
                if success:
                    print("✅ IK求解并开始移动...")
                else:
                    print("❌ 移动失败")
                    
            except ValueError:
                print("❌ 坐标格式错误，请使用: move x y z")
        
        elif cmd == 'status':
            self.show_position_comparison()
        
        elif cmd == 'help':
            print("\n🎮 可用命令:")
            print("  joints j1 j2 j3 j4 j5 j6  - 设置关节角度（度）[测试正运动学]")
            print("  move x y z                - 移动到世界坐标位置 [测试逆运动学]")
            print("  status                    - 显示当前状态和位置对比")
            print("  help                      - 显示帮助")
            print("  quit                      - 退出")
            print("\n示例:")
            print("  joints 0 30 -20 15 0 0   # 设置关节角度，测试正运动学")
            print("  move 1.0 0.6 0.8         # 移动到指定位置，测试逆运动学")
            print("  status                   # 查看位置对比")
            print("\n🔧 坐标系修正:")
            print("  已修正MuJoCo和运动学模块之间的x/y轴反向问题")
        
        else:
            print("❌ 未知命令，输入 'help' 查看可用命令")
    
    def run_debug_control(self):
        """运行调试控制"""
        print("🚀 启动调试控制模式...")
        print("\n🔧 调试模式 - 测试正运动学和逆运动学")
        print("• joints: 直接控制关节角度，测试正运动学")
        print("• move: 通过IK控制末端位置，测试逆运动学")
        print("• 已修正MuJoCo和运动学模块之间的x/y轴反向问题")
        print("输入 'help' 查看所有可用命令")
        print("\n快速开始:")
        print("  joints 0 30 -20 15 0 0   # 测试正运动学")
        print("  move 1.0 0.6 0.8         # 测试逆运动学")
        print("  status                   # 查看位置对比")
        print("\n请输入命令:")
        
        # 启动查看器
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            # 启动键盘输入线程
            input_thread = threading.Thread(target=self.keyboard_input_thread, daemon=True)
            input_thread.start()
            
            # 显示初始状态
            time.sleep(1)
            self.show_position_comparison()
            
            # 主循环
            while viewer.is_running() and self.running:
                try:
                    # 步进仿真
                    mujoco.mj_step(self.model, self.data)
                    
                    # 更新控制
                    self.update_control()
                    
                    # 处理命令队列
                    try:
                        command = self.command_queue.get_nowait()
                        self.process_command(command)
                    except queue.Empty:
                        pass
                    
                    # 检查移动完成状态
                    if self.moving and self.check_movement_complete():
                        print("✅ 移动完成")
                    
                    # 同步查看器
                    viewer.sync()
                    
                    # 控制频率
                    time.sleep(0.01)
                    
                except KeyboardInterrupt:
                    print("\n🔚 用户中断，退出仿真")
                    break
        
        self.running = False
        print("✅ 调试完成")

def main():
    """主程序"""
    print("SO101机械臂调试控制脚本")
    print("测试正运动学(joints)和逆运动学(move)")
    print("已修正MuJoCo和运动学模块之间的x/y轴反向问题")
    
    # 加载MuJoCo场景
    xml_path = "manipulator_grasp/assets/SO101/scene_so101.xml"
    
    # 查找场景文件
    possible_paths = [
        xml_path
    ]
    
    model = None
    for path in possible_paths:
        try:
            if os.path.exists(path):
                model = mujoco.MjModel.from_xml_path(path)
                data = mujoco.MjData(model)
                print(f"✅ 成功加载场景: {path}")
                break
        except Exception as e:
            print(f"尝试加载 {path} 失败: {e}")
            continue
    
    if model is None:
        print("❌ 找不到可用的MuJoCo场景文件")
        return
    
    # 重置到初始状态
    mujoco.mj_resetDataKeyframe(model, data, 0)
    
    # 创建调试控制器
    try:
        controller = DebugController(model, data)
        controller.run_debug_control()
    except Exception as e:
        print(f"❌ 控制器启动失败: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()