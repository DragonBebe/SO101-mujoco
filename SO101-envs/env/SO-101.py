import numpy as np
import spatialmath as sm
from manipulator_grasp.arm.robot import Robot

class SO101(Robot):
    """
    SO-101 机械臂类
    """
    
    def __init__(self):
        # SO-101 的 DH 参数（需要根据实际规格调整）
        # 这里是示例参数，需要替换为 SO-101 的实际 DH 参数
        links = [
            # DH 参数: [theta, d, a, alpha] (单位: 弧度和米)
            [0, 0.1625, 0, np.pi/2],      # Joint 1
            [0, 0, -0.425, 0],            # Joint 2  
            [0, 0, -0.3922, 0],           # Joint 3
            [0, 0.1333, 0, np.pi/2],      # Joint 4
            [0, 0.0997, 0, -np.pi/2],     # Joint 5
            [0, 0.0996, 0, 0],            # Joint 6
        ]
        
        # 关节限制（弧度）
        joint_limits = [
            [-2*np.pi, 2*np.pi],          # Joint 1
            [-2*np.pi, 2*np.pi],          # Joint 2
            [-2*np.pi, 2*np.pi],          # Joint 3
            [-2*np.pi, 2*np.pi],          # Joint 4
            [-2*np.pi, 2*np.pi],          # Joint 5
            [-2*np.pi, 2*np.pi],          # Joint 6
        ]
        
        super().__init__(
            name="SO-101",
            links=links,
            joint_limits=joint_limits,
            base=sm.SE3(),
            tool=sm.SE3()
        )
    
    @property
    def joint_names(self):
        """SO-101 的关节名称"""
        return [
            "joint_1",
            "joint_2", 
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6"
        ]
    
    def get_default_config(self):
        """获取默认配置"""
        return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])