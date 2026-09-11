"""
简化版SO101机械臂IK控制 + Gripper控制
功能：
1. 给定空间坐标控制机械臂末端执行器移动
2. 双重验证末端执行器位置：MuJoCo读取 + 正运动学计算
3. Gripper控制（默认关闭）
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import math
import sys
import os
from scipy.spatial.transform import Rotation

# 添加运动学模块路径
sys.path.append(os.path.join(os.path.dirname(__file__), 'lerobot/common/model'))

try:
    from kinematics import RobotKinematics
    print("✅ 成功导入RobotKinematics模块")
except ImportError as e:
    print(f"❌ 导入RobotKinematics失败: {e}")
    sys.exit(1)

class SimpleArmController:
    """简化的机械臂控制器（含Gripper控制）"""
    
    def __init__(self, model, data):
        self.model = model
        self.data = data
        
        # 机械臂基座变换参数
        self.base_offset = np.array([0.8, 0.6, 0.71])
        base_quat = np.array([1.0, 0.0, 0.0, 0.90])
        base_quat = base_quat / np.linalg.norm(base_quat)
        self.base_rotation = Rotation.from_quat([base_quat[1], base_quat[2], base_quat[3], base_quat[0]])
        
        # 初始化运动学模块
        self.kinematics = RobotKinematics("so_new_calibration")
        
        # 获取关节映射
        self.joint_names = ['1', '2', '3', '4', '5', '6']
        self.joint_ids = []
        self.actuator_ids = []
        
        for joint_name in self.joint_names:
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name)
            self.joint_ids.append(joint_id)
            self.actuator_ids.append(actuator_id)
        
        # 🔧 新增：Gripper控制映射
        self.gripper_joint_ids = []
        self.gripper_actuator_ids = []
        
        # 查找gripper相关的关节和执行器
        gripper_joint_names = ['left_finger_joint', 'right_finger_joint']  # 常见的gripper关节名
        
        for gripper_joint_name in gripper_joint_names:
            try:
                joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, gripper_joint_name)
                actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, gripper_joint_name)
                self.gripper_joint_ids.append(joint_id)
                self.gripper_actuator_ids.append(actuator_id)
                print(f"✓ 找到gripper关节: {gripper_joint_name}")
            except:
                # 如果找不到具体名称，尝试通用名称
                try:
                    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'gripper')
                    actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper')
                    self.gripper_joint_ids.append(joint_id)
                    self.gripper_actuator_ids.append(actuator_id)
                    print(f"✓ 找到gripper关节: gripper")
                    break
                except:
                    continue
        
        if not self.gripper_joint_ids:
            print("⚠️  未找到gripper关节，gripper控制将不可用")
        
        # 控制参数
        self.kp = 50.0
        self.kd = 5.0
        self.current_target_joints = [0, 0, 0, 0, 0, 0]
        
        # 🔧 新增：Gripper控制参数
        self.gripper_kp = 100.0  # gripper需要更强的控制力
        self.gripper_kd = 10.0
        self.gripper_closed_position = 0.0    # 关闭位置（弧度）
        self.gripper_open_position = 0.8      # 打开位置（弧度）
        self.current_gripper_target = self.gripper_closed_position  # 默认关闭
        
        print(f"🤏 Gripper默认状态: 关闭 (目标位置: {self.gripper_closed_position:.2f})")
    
    def world_to_robot_coords(self, world_pos):
        """世界坐标转机械臂坐标"""
        translated_pos = np.array(world_pos) - self.base_offset
        robot_pos = self.base_rotation.inv().apply(translated_pos)
        return robot_pos
    
    def robot_to_world_coords(self, robot_pos):
        """机械臂坐标转世界坐标"""
        rotated_pos = self.base_rotation.apply(robot_pos)
        world_pos = rotated_pos + self.base_offset
        return world_pos
    
    def set_gripper_state(self, state):
        """
        设置gripper状态
        state: 'open' 或 'close'
        """
        if not self.gripper_joint_ids:
            print("⚠️  Gripper不可用")
            return False
        
        if state.lower() == 'open':
            self.current_gripper_target = self.gripper_open_position
            print(f"🤏 设置Gripper为: 打开 (目标位置: {self.gripper_open_position:.2f})")
        elif state.lower() == 'close':
            self.current_gripper_target = self.gripper_closed_position
            print(f"🤏 设置Gripper为: 关闭 (目标位置: {self.gripper_closed_position:.2f})")
        else:
            print("❌ Gripper状态错误，请使用 'open' 或 'close'")
            return False
        
        return True
    
    def get_gripper_state(self):
        """获取当前gripper状态"""
        if not self.gripper_joint_ids:
            return "不可用"
        
        # 获取第一个gripper关节的当前位置
        current_pos = self.data.qpos[self.gripper_joint_ids[0]]
        
        # 判断是接近打开还是关闭状态
        open_diff = abs(current_pos - self.gripper_open_position)
        close_diff = abs(current_pos - self.gripper_closed_position)
        
        if close_diff < open_diff:
            return f"关闭 (当前位置: {current_pos:.3f})"
        else:
            return f"打开 (当前位置: {current_pos:.3f})"
    
    def move_to_position(self, world_x, world_y, world_z):
        """
        控制机械臂移动到指定世界坐标位置
        返回: (成功标志, IK解的关节角度)
        """
        print(f"\n🎯 目标位置: ({world_x:.3f}, {world_y:.3f}, {world_z:.3f})")
        
        # 转换为机械臂坐标系
        robot_target_pos = self.world_to_robot_coords([world_x, world_y, world_z])
        print(f"   机械臂坐标: ({robot_target_pos[0]:.3f}, {robot_target_pos[1]:.3f}, {robot_target_pos[2]:.3f})")
        
        # 获取当前关节角度（度）
        current_joints_deg = [math.degrees(self.data.qpos[joint_id]) for joint_id in self.joint_ids]
        
        # 创建目标位姿矩阵
        target_pose = np.eye(4, dtype=np.float32)
        target_pose[:3, 3] = robot_target_pos
        
        try:
            # IK求解
            target_joints_deg = self.kinematics.ik(
                current_joint_pos=np.array(current_joints_deg, dtype=np.float32),
                desired_ee_pose=target_pose,
                position_only=True,
                frame="gripper_tip",################################################################################################################################################################
                max_iterations=200,
                learning_rate=0.05
            )
            
            # 设置目标关节角度（转换为弧度）
            self.current_target_joints = [math.radians(angle) for angle in target_joints_deg]
            
            print(f"✅ IK求解成功")
            print(f"   目标关节角度: {[f'{a:.1f}°' for a in target_joints_deg]}")
            
            return True, target_joints_deg
            
        except Exception as e:
            print(f"❌ IK求解失败: {e}")
            return False, None
    
    def get_ee_position_mujoco(self):
        """方法1: 从MuJoCo中读取moving_jaw位置"""
        try:
            # 尝试获取moving_jaw的位置
            jaw_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'moving_jaw')
            return self.data.xpos[jaw_body_id].copy()
        except:
            try:
                # 备选：尝试gripper
                gripper_body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'gripper')
                return self.data.xpos[gripper_body_id].copy()
            except:
                print("⚠️  无法从MuJoCo读取末端位置")
                return None
    
    def get_ee_position_kinematics(self):
        """方法2: 通过正运动学计算末端执行器位置"""
        try:
            # 获取当前关节角度
            current_joints_deg = [math.degrees(self.data.qpos[joint_id]) for joint_id in self.joint_ids]
            
            # 正运动学计算
            pose = self.kinematics.forward_kinematics(
                np.array(current_joints_deg, dtype=np.float32), "gripper_tip"
            )
            
            # 转换为世界坐标
            robot_pos = pose[:3, 3]
            world_pos = self.robot_to_world_coords(robot_pos)
            
            return world_pos
            
        except Exception as e:
            print(f"⚠️  正运动学计算失败: {e}")
            return None
    
    def verify_position(self, target_world_pos):
        """验证末端执行器位置"""
        print(f"\n🔍 位置验证:")
        print(f"   目标位置: ({target_world_pos[0]:.3f}, {target_world_pos[1]:.3f}, {target_world_pos[2]:.3f})")
        
        # 方法1: MuJoCo读取
        mujoco_pos = self.get_ee_position_mujoco()
        if mujoco_pos is not None:
            error1 = np.linalg.norm(mujoco_pos - target_world_pos)
            print(f"   MuJoCo读取: ({mujoco_pos[0]:.3f}, {mujoco_pos[1]:.3f}, {mujoco_pos[2]:.3f})")
            print(f"   MuJoCo误差: {error1*1000:.1f}mm")
        
        # 方法2: 正运动学计算
        kinematics_pos = self.get_ee_position_kinematics()
        if kinematics_pos is not None:
            error2 = np.linalg.norm(kinematics_pos - target_world_pos)
            print(f"   正运动学: ({kinematics_pos[0]:.3f}, {kinematics_pos[1]:.3f}, {kinematics_pos[2]:.3f})")
            print(f"   正运动学误差: {error2*1000:.1f}mm")
        
        # 两种方法的一致性检查
        if mujoco_pos is not None and kinematics_pos is not None:
            consistency_error = np.linalg.norm(mujoco_pos - kinematics_pos)
            print(f"   两种方法差异: {consistency_error*1000:.1f}mm")
            
            if consistency_error < 0.01:  # 1cm阈值
                print("✅ 两种验证方法结果一致")
            else:
                print("⚠️  两种验证方法存在差异")
        
        # 🔧 新增：显示gripper状态
        gripper_state = self.get_gripper_state()
        print(f"   Gripper状态: {gripper_state}")
    
    def update_control(self):
        """更新PD控制器（机械臂 + Gripper）"""
        # 更新机械臂关节控制
        for i, (joint_id, actuator_id) in enumerate(zip(self.joint_ids, self.actuator_ids)):
            if i < len(self.current_target_joints):
                current_pos = self.data.qpos[joint_id]
                current_vel = self.data.qvel[joint_id]
                target_pos = self.current_target_joints[i]
                
                pos_error = target_pos - current_pos
                vel_error = 0 - current_vel
                control_torque = self.kp * pos_error + self.kd * vel_error
                
                self.data.ctrl[actuator_id] = control_torque
        
        # 🔧 新增：更新gripper控制
        for joint_id, actuator_id in zip(self.gripper_joint_ids, self.gripper_actuator_ids):
            current_pos = self.data.qpos[joint_id]
            current_vel = self.data.qvel[joint_id]
            target_pos = self.current_gripper_target
            
            pos_error = target_pos - current_pos
            vel_error = 0 - current_vel
            control_torque = self.gripper_kp * pos_error + self.gripper_kd * vel_error
            
            self.data.ctrl[actuator_id] = control_torque

def main():
    """主函数"""
    # 加载场景
    xml_path = "manipulator_grasp/assets/SO101/scene_so101.xml"
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    
    # 重置到初始状态
    mujoco.mj_resetDataKeyframe(model, data, 0)
    
    # 创建控制器
    controller = SimpleArmController(model, data)
    
    print("=== 简化版机械臂IK控制 + Gripper控制 ===")
    print("功能: 指定坐标 -> IK求解 -> 控制移动 -> 双重验证")
    print("新增: Gripper控制（默认关闭状态）")
    
    # 启动仿真
    with mujoco.viewer.launch_passive(model, data) as viewer:
        time.sleep(1.0)
        
        print("\n🎮 交互控制:")
        print("坐标输入格式: x y z (世界坐标)")
        print("Gripper控制: 'open' 或 'close'")
        print("示例: 0 0.5 0.1")
        print("示例: open")
        print("输入 'quit' 退出")
        
        while viewer.is_running():
            try:
                # 获取用户输入
                user_input = input("\n请输入目标坐标或gripper命令: ").strip()
                
                if user_input.lower() == 'quit':
                    break
                
                # 🔧 新增：处理gripper命令
                if user_input.lower() in ['open', 'close']:
                    success = controller.set_gripper_state(user_input)
                    if success:
                        print("⏳ Gripper状态更新中...")
                        # 短暂等待gripper动作完成
                        start_time = time.time()
                        while time.time() - start_time < 1.0:  # 等待1秒
                            controller.update_control()
                            mujoco.mj_step(model, data)
                            viewer.sync()
                            time.sleep(0.01)
                        print(f"✅ Gripper状态更新完成: {controller.get_gripper_state()}")
                    continue
                
                # 解析坐标
                coords = user_input.split()
                if len(coords) != 3:
                    print("❌ 请输入3个坐标值: x y z，或gripper命令: open/close")
                    continue
                
                try:
                    x, y, z = float(coords[0]), float(coords[1]), float(coords[2])
                except ValueError:
                    print("❌ 坐标格式错误")
                    continue
                
                # 执行移动
                success, target_joints = controller.move_to_position(x, y, z)
                
                if success:
                    print("\n⏳ 机械臂移动中...")
                    
                    # 等待移动完成
                    start_time = time.time()
                    while time.time() - start_time < 3.0:  # 等待3秒
                        controller.update_control()
                        mujoco.mj_step(model, data)
                        viewer.sync()
                        time.sleep(0.01)
                    
                    # 验证位置
                    controller.verify_position([x, y, z])
                else:
                    print("❌ 移动失败")
                
            except KeyboardInterrupt:
                break
            except EOFError:
                break
    
    print("✅ 程序结束")

if __name__ == "__main__":
    main()
