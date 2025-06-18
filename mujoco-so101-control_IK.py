"""
交互式键盘控制版本 - SO101机械臂逆运动学控制
支持键盘输入目标位置 - 修正版本
"""

import argparse
import time
import numpy as np
import mujoco
import mujoco.viewer
import math
import threading
from IK_SO101 import SO101RobotController

class InteractiveSO101Controller:
    """
    交互式SO101机械臂控制器
    支持键盘输入目标位置
    """
    
    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.ik_controller = SO101RobotController()
        
        # 修正：使用XML中的实际关节名称
        self.joint_names = ['1', '2', '3', '4', '5', '6']
        self.joint_ids = []
        self.actuator_ids = []
        
        print("\\n=== 关节映射检查 ===")
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
        
        # 当前状态
        self.target_position = [0.2, 0.1, 0.2]
        self.current_target_joints = [0, 0, 0, 0, 0, 0]
        self.elbow_up = True
        
        # 交互控制
        self.running = True
        self.new_command = False
        
        # 统计有效关节
        valid_joints = sum(1 for jid in self.joint_ids if jid >= 0)
        print(f"\\n✅ 控制器初始化完成 - 找到 {valid_joints}/6 个有效关节")
    
    def move_to_position(self, x, y, z):
        """移动到指定位置"""
        result = self.ik_controller.move_to_position(x, y, z, preferred_elbow_up=self.elbow_up)
        
        if result['success']:
            self.current_target_joints = result['joint_angles']
            self.target_position = [x, y, z]
            print(f"✅ IK求解成功 - 移动到 ({x:.3f}, {y:.3f}, {z:.3f}) - {result['config']}")
            
            # 显示目标关节角度
            print("目标关节角度:")
            for i, angle in enumerate(self.current_target_joints):
                print(f"  关节{i+1}: {math.degrees(angle):6.2f}°")
            
            return True
        else:
            print(f"❌ 无法到达位置 ({x:.3f}, {y:.3f}, {z:.3f}): {result['error']}")
            return False
    
    def update_control(self):
        """更新控制"""
        for i, (joint_id, actuator_id) in enumerate(zip(self.joint_ids, self.actuator_ids)):
            if joint_id >= 0 and actuator_id >= 0:
                current_pos = self.data.qpos[joint_id]
                current_vel = self.data.qvel[joint_id]
                target_pos = self.current_target_joints[i]
                
                pos_error = target_pos - current_pos
                vel_error = 0 - current_vel
                control_torque = self.kp * pos_error + self.kd * vel_error
                
                self.data.ctrl[actuator_id] = control_torque
    
    def get_current_position(self):
        """获取当前末端执行器位置"""
        # 方法1: 使用gripper site
        if self.gripper_site_id >= 0:
            return self.data.site_xpos[self.gripper_site_id].copy()
        
        # 方法2: 使用gripper body
        if self.ee_body_id >= 0:
            return self.data.xpos[self.ee_body_id].copy()
        
        # 方法3: 使用正运动学
        current_joints = [self.data.qpos[joint_id] if joint_id >= 0 else 0 
                         for joint_id in self.joint_ids]
        return self.ik_controller.forward_kinematics(current_joints)
    
    def get_current_joint_angles(self):
        """获取当前关节角度"""
        return [self.data.qpos[joint_id] if joint_id >= 0 else 0 
                for joint_id in self.joint_ids]
    
    def keyboard_input_thread(self):
        """键盘输入线程"""
        print("\\n🎮 交互式控制已启动!")
        print("命令格式:")
        print("  move x y z     - 移动到位置 (x,y,z)")
        print("  elbow up/down  - 切换肘部配置")
        print("  status         - 显示当前状态")
        print("  joints         - 显示关节角度")
        print("  presets        - 显示预设位置")
        print("  quit           - 退出程序")
        print("\\n示例: move 0.2 0.1 0.25")
        
        # 预设位置 - 调整为相对于机械臂基座的坐标
        presets = {
            '1': (0.20, 0.10, 0.20, "前方位置"),
            '2': (0.15, 0.15, 0.25, "对角线高位"),
            '3': (0.00, 0.25, 0.15, "侧方位置"),
            '4': (0.25, 0.00, 0.15, "正前方"),
            '5': (0.15, -0.15, 0.20, "右前方"),
        }
        
        while self.running:
            try:
                command = input("\\n> ").strip().lower()
                
                if command == 'quit' or command == 'q':
                    self.running = False
                    break
                
                elif command == 'status':
                    current_pos = self.get_current_position()
                    target_pos = self.target_position
                    error = np.linalg.norm(np.array(current_pos) - np.array(target_pos))
                    print(f"\\n📊 当前状态:")
                    print(f"   当前位置: [{current_pos[0]:6.3f}, {current_pos[1]:6.3f}, {current_pos[2]:6.3f}]")
                    print(f"   目标位置: [{target_pos[0]:6.3f}, {target_pos[1]:6.3f}, {target_pos[2]:6.3f}]")
                    print(f"   位置误差: {error*1000:.1f}mm")
                    print(f"   肘部配置: {'向上' if self.elbow_up else '向下'}")
                
                elif command == 'joints':
                    current_joints = self.get_current_joint_angles()
                    target_joints = self.current_target_joints
                    print(f"\\n🔧 关节角度:")
                    for i in range(6):
                        current_deg = math.degrees(current_joints[i])
                        target_deg = math.degrees(target_joints[i])
                        error_deg = abs(current_deg - target_deg)
                        print(f"   关节{i+1}: 当前={current_deg:6.2f}°, 目标={target_deg:6.2f}°, 误差={error_deg:5.2f}°")
                
                elif command == 'presets':
                    print("\\n📍 预设位置:")
                    for key, (x, y, z, desc) in presets.items():
                        print(f"   {key}: {desc} ({x:.2f}, {y:.2f}, {z:.2f})")
                    print("\\n使用方法: 输入数字选择预设位置")
                
                elif command in presets:
                    x, y, z, desc = presets[command]
                    print(f"\\n🎯 移动到预设位置: {desc}")
                    self.move_to_position(x, y, z)
                
                elif command.startswith('move'):
                    parts = command.split()
                    if len(parts) == 4:
                        try:
                            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                            self.move_to_position(x, y, z)
                        except ValueError:
                            print("❌ 坐标格式错误，请使用: move x y z")
                    else:
                        print("❌ 命令格式错误，请使用: move x y z")
                
                elif command.startswith('elbow'):
                    if 'up' in command:
                        self.elbow_up = True
                        print("✅ 肘部配置设为向上")
                    elif 'down' in command:
                        self.elbow_up = False
                        print("✅ 肘部配置设为向下")
                    else:
                        print("❌ 请使用: elbow up 或 elbow down")
                
                else:
                    print("❌ 未知命令。输入 'presets' 查看预设位置，或 'quit' 退出")
                    
            except KeyboardInterrupt:
                self.running = False
                break
            except EOFError:
                self.running = False
                break

def print_model_info(model, data):
    """打印模型信息用于调试"""
    print("\\n=== 模型信息 ===")
    print(f"关节数量: {model.njnt}")
    print(f"执行器数量: {model.nu}")
    print(f"物体数量: {model.nbody}")
    
    print("\\n关节列表:")
    for i in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        joint_type = model.jnt_type[i]
        print(f"  {i}: {joint_name} (type: {joint_type})")
    
    print("\\n执行器列表:")
    for i in range(model.nu):
        actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        print(f"  {i}: {actuator_name}")
    
    print("\\n物体列表:")
    for i in range(model.nbody):
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        print(f"  {i}: {body_name}")

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
        controller = InteractiveSO101Controller(m, data)
    except Exception as e:
        print(f"❌ 控制器初始化失败: {e}")
        return
    
    # 设置初始位置
    print("\\n🎯 设置初始位置...")
    controller.move_to_position(0.2, 0.1, 0.2)
    
    # 启动键盘输入线程
    input_thread = threading.Thread(target=controller.keyboard_input_thread, daemon=True)
    input_thread.start()
    
    print("\\n=== 启动交互式仿真 ===")
    print("🎮 控制说明：")
    print("  - 鼠标左键拖拽：旋转视角")
    print("  - 鼠标滚轮：缩放")
    print("  - 鼠标右键拖拽：平移视角")
    print("  - 在终端输入命令控制机械臂")
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
            
            # 每100步显示一次状态
            status_counter += 1
            if status_counter % 500 == 0:  # 每1秒显示一次 (假设500Hz)
                current_pos = controller.get_current_position()
                target_pos = controller.target_position
                error = np.linalg.norm(np.array(current_pos) - np.array(target_pos))
                print(f"\\n📊 状态更新:")
                print(f"   当前位置: [{current_pos[0]:6.3f}, {current_pos[1]:6.3f}, {current_pos[2]:6.3f}]")
                print(f"   目标位置: [{target_pos[0]:6.3f}, {target_pos[1]:6.3f}, {target_pos[2]:6.3f}]")
                print(f"   位置误差: {error*1000:.1f}mm")
            
            # 控制仿真速度
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
    
    controller.running = False
    print("\\n🔚 仿真结束")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="交互式SO101机械臂逆运动学控制")
    parser.add_argument("--robot", choices=["6dof"], default="6dof", help="选择机器人类型")
    args = parser.parse_args()
    do_interactive_sim(args.robot)