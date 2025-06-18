# 创建完整的SO101逆运动学解决方案
import numpy as np
import math

class SO101RobotController:
    """
    SO101机械臂完整控制器
    包含逆运动学、工作空间分析、轨迹规划等功能
    """
    
    def __init__(self):
        # 机械臂参数（从XML文件提取）
        self.l1 = 0.1159  # 上臂长度 (m)
        self.l2 = 0.1350  # 下臂长度 (m) 
        self.l3 = 0.0650  # 腕部到末端的长度 (m)
        self.base_height = 0.0949  # 基座高度 (m)
        
        # 关节限制 (弧度)
        self.joint_limits = {
            'joint1': (-1.92, 1.92),    # ±110°
            'joint2': (-1.75, 1.75),    # ±100.3°
            'joint3': (-1.75, 1.57),    # -100.3° to 89.95°
            'joint4': (-1.66, 1.66),    # ±95.1°
            'joint5': (-2.79, 2.79),    # ±159.8°
            'joint6': (-0.17, 1.75)     # -9.7° to 100.3°
        }
        
        # 工作空间参数
        self.max_reach = self.l1 + self.l2 + self.l3  # 最大臂展
        self.min_reach = abs(self.l1 - self.l2)       # 最小臂展
        
    def check_workspace(self, x, y, z):
        """检查目标点是否在工作空间内"""
        # 计算到基座的距离
        r_xy = math.sqrt(x**2 + y**2)
        z_adjusted = z - self.base_height
        total_distance = math.sqrt(r_xy**2 + z_adjusted**2)
        
        # 考虑末端执行器长度
        wrist_distance = total_distance - self.l3
        
        return (wrist_distance >= self.min_reach and 
                wrist_distance <= self.l1 + self.l2 and
                z >= self.base_height - 0.05)  # 基座以下5cm的容限
    
    def inverse_kinematics(self, target_x, target_y, target_z, 
                          elbow_up=True, end_effector_angle=0):
        """
        逆运动学求解
        
        参数:
        target_x, target_y, target_z: 目标位置坐标 (m)
        elbow_up: True为肘部向上配置，False为肘部向下
        end_effector_angle: 末端执行器角度 (弧度)
        
        返回:
        joint_angles: [θ1, θ2, θ3, θ4, θ5, θ6] 关节角度列表
        success: 求解是否成功
        """
        
        # 检查工作空间
        if not self.check_workspace(target_x, target_y, target_z):
            return [0, 0, 0, 0, 0, 0], False
        
        try:
            # 1. 基座旋转角度
            theta1 = math.atan2(target_y, target_x)
            
            # 2. 计算腕部位置
            r_xy = math.sqrt(target_x**2 + target_y**2)
            z_adjusted = target_z - self.base_height
            
            # 考虑末端执行器角度的腕部位置
            wrist_r = r_xy - self.l3 * math.cos(end_effector_angle)
            wrist_z = z_adjusted - self.l3 * math.sin(end_effector_angle)
            
            # 3. 2D逆运动学求解肩部和肘部
            theta2, theta3 = self._solve_2d_kinematics(wrist_r, wrist_z, elbow_up)
            
            # 4. 腕部俯仰角度（保持末端执行器角度）
            theta5 = end_effector_angle - (theta2 + theta3)
            
            # 5. 其他关节
            theta4 = 0.0  # 腕部旋转
            theta6 = 0.0  # 夹爪
            
            joint_angles = [theta1, theta2, theta3, theta4, theta5, theta6]
            
            # 检查关节限制
            if self._check_joint_limits(joint_angles):
                return joint_angles, True
            else:
                return joint_angles, False
                
        except Exception as e:
            print(f"逆运动学计算错误: {e}")
            return [0, 0, 0, 0, 0, 0], False
    
    def _solve_2d_kinematics(self, x, y, elbow_up=True):
        """2D平面逆运动学求解"""
        r = math.sqrt(x**2 + y**2)
        
        # 使用余弦定理计算肘部角度
        cos_theta3 = (r**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        cos_theta3 = max(-1, min(1, cos_theta3))  # 限制范围
        
        if elbow_up:
            theta3 = math.acos(cos_theta3)
        else:
            theta3 = -math.acos(cos_theta3)
        
        # 计算肩部角度
        beta = math.atan2(y, x)
        gamma = math.atan2(self.l2 * math.sin(theta3), 
                          self.l1 + self.l2 * math.cos(theta3))
        theta2 = beta - gamma
        
        return theta2, theta3
    
    def _check_joint_limits(self, joint_angles):
        """检查关节角度限制"""
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        for i, (angle, joint_name) in enumerate(zip(joint_angles, joint_names)):
            min_limit, max_limit = self.joint_limits[joint_name]
            if angle < min_limit or angle > max_limit:
                return False
        return True
    
    def forward_kinematics(self, joint_angles):
        """正运动学计算"""
        theta1, theta2, theta3, theta4, theta5, theta6 = joint_angles
        
        # 计算末端执行器位置
        # 肩部位置
        shoulder_pos = [0, 0, self.base_height]
        
        # 肘部位置
        elbow_x = self.l1 * math.cos(theta2) * math.cos(theta1)
        elbow_y = self.l1 * math.cos(theta2) * math.sin(theta1)
        elbow_z = self.base_height + self.l1 * math.sin(theta2)
        
        # 腕部位置
        wrist_x = elbow_x + self.l2 * math.cos(theta2 + theta3) * math.cos(theta1)
        wrist_y = elbow_y + self.l2 * math.cos(theta2 + theta3) * math.sin(theta1)
        wrist_z = elbow_z + self.l2 * math.sin(theta2 + theta3)
        
        # 末端执行器位置
        end_x = wrist_x + self.l3 * math.cos(theta2 + theta3 + theta5) * math.cos(theta1)
        end_y = wrist_y + self.l3 * math.cos(theta2 + theta3 + theta5) * math.sin(theta1)
        end_z = wrist_z + self.l3 * math.sin(theta2 + theta3 + theta5)
        
        return end_x, end_y, end_z
    
    def find_all_solutions(self, target_x, target_y, target_z):
        """找到目标位置的所有可能解"""
        solutions = []
        
        for elbow_up in [True, False]:
            joint_angles, success = self.inverse_kinematics(
                target_x, target_y, target_z, elbow_up=elbow_up
            )
            
            if success:
                # 验证精度
                calc_x, calc_y, calc_z = self.forward_kinematics(joint_angles)
                error = math.sqrt((target_x-calc_x)**2 + (target_y-calc_y)**2 + (target_z-calc_z)**2)
                
                if error < 0.001:  # 1mm精度
                    solutions.append({
                        'joint_angles': joint_angles,
                        'config': '肘部向上' if elbow_up else '肘部向下',
                        'error': error
                    })
        
        return solutions
    
    def move_to_position(self, target_x, target_y, target_z, preferred_elbow_up=True):
        """
        移动到指定位置的主要接口函数
        
        返回:
        result: 包含关节角度和状态信息的字典
        """
        # 首先尝试首选配置
        joint_angles, success = self.inverse_kinematics(
            target_x, target_y, target_z, elbow_up=preferred_elbow_up
        )
        
        if success:
            return {
                'success': True,
                'joint_angles': joint_angles,
                'joint_angles_deg': [math.degrees(angle) for angle in joint_angles],
                'config': '肘部向上' if preferred_elbow_up else '肘部向下',
                'target_position': [target_x, target_y, target_z]
            }
        
        # 如果首选配置失败，尝试其他配置
        joint_angles, success = self.inverse_kinematics(
            target_x, target_y, target_z, elbow_up=not preferred_elbow_up
        )
        
        if success:
            return {
                'success': True,
                'joint_angles': joint_angles,
                'joint_angles_deg': [math.degrees(angle) for angle in joint_angles],
                'config': '肘部向下' if preferred_elbow_up else '肘部向上',
                'target_position': [target_x, target_y, target_z]
            }
        
        return {
            'success': False,
            'error': '目标位置无法到达或超出关节限制',
            'target_position': [target_x, target_y, target_z]
        }

# 使用示例和测试
def demo_usage():
    """演示SO101机械臂逆运动学的使用"""
    
    # 创建控制器实例
    robot = SO101RobotController()
    
    print("=== SO101机械臂逆运动学控制器演示 ===")
    print(f"机械臂参数:")
    print(f"  上臂长度: {robot.l1*1000:.1f}mm")
    print(f"  下臂长度: {robot.l2*1000:.1f}mm") 
    print(f"  腕部长度: {robot.l3*1000:.1f}mm")
    print(f"  基座高度: {robot.base_height*1000:.1f}mm")
    print(f"  最大臂展: {robot.max_reach*1000:.1f}mm")
    
    # 测试几个典型位置
    test_positions = [
        (0.20, 0.10, 0.20, "前方位置"),
        (0.00, 0.25, 0.15, "侧方位置"),
        (0.15, 0.15, 0.25, "对角线位置"),
        (0.10, -0.20, 0.12, "后侧位置")
    ]
    
    for x, y, z, description in test_positions:
        print(f"\n--- {description}: ({x:.2f}, {y:.2f}, {z:.2f}) ---")
        
        # 检查工作空间
        if robot.check_workspace(x, y, z):
            print("✓ 位置在工作空间内")
            
            # 移动到该位置
            result = robot.move_to_position(x, y, z)
            
            if result['success']:
                print(f"✓ 逆运动学求解成功 - {result['config']}")
                print("关节角度:")
                joint_names = ['基座', '肩部', '肘部', '腕旋转', '腕俯仰', '夹爪']
                for name, angle_deg in zip(joint_names, result['joint_angles_deg']):
                    print(f"  {name}: {angle_deg:8.2f}°")
                
                # 验证精度
                calc_pos = robot.forward_kinematics(result['joint_angles'])
                error = math.sqrt(sum((a-b)**2 for a, b in zip([x,y,z], calc_pos)))
                print(f"位置误差: {error*1000:.3f}mm")
                
            else:
                print(f"✗ {result['error']}")
        else:
            print("✗ 位置超出工作空间")
    
    # 演示多解情况
    print(f"\n--- 多解分析: 位置 (0.18, 0.12, 0.20) ---")
    solutions = robot.find_all_solutions(0.18, 0.12, 0.20)
    print(f"找到 {len(solutions)} 个解:")
    
    for i, sol in enumerate(solutions):
        print(f"\n解 {i+1}: {sol['config']} (误差: {sol['error']*1000:.3f}mm)")
        joint_names = ['基座', '肩部', '肘部', '腕旋转', '腕俯仰', '夹爪']
        angles_deg = [math.degrees(angle) for angle in sol['joint_angles']]
        for name, angle_deg in zip(joint_names, angles_deg):
            print(f"  {name}: {angle_deg:8.2f}°")

# 运行演示
demo_usage()