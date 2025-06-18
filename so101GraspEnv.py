"""
通过mocap逐步移动机械臂的base
"""


import argparse
import time
import numpy as np
import mujoco
import mujoco.viewer

def do_interactive_sim(robot_id):
    if robot_id == "6dof":
        xml_path = "manipulator_grasp/assets/SO101/scene_table.xml"
        try:
            m = mujoco.MjModel.from_xml_path(xml_path)
        except Exception as e:
            print(f"Error loading XML: {e}")
            return
    else:
        print(f"Robot ID {robot_id} not supported by this script version.")
        return

    print(f"Model loaded successfully from: {xml_path}")
    
    # 创建数据结构
    data = mujoco.MjData(m)
    
    # 找到相关的 body IDs
    base_body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base")
    mocap_body_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "mocap")
    
    print(f"Base body ID: {base_body_id}")
    print(f"Mocap body ID: {mocap_body_id}")
    
    # 定义轨迹点
    start_pos = np.array([0.0, 0.0, 0.0])      # 起始位置（原点）
    safe_pos = np.array([0.8, 0.6, 1.5])       # 安全高度位置
    final_pos = np.array([0.8, 0.6, 0.70])     # 最终目标位置
    
    print(f"\\nTrajectory plan:")
    print(f"  Start position: {start_pos}")
    print(f"  Safe position:  {safe_pos}")
    print(f"  Final position: {final_pos}")
    
    # 检查并修复约束
    print("\\n=== ANALYZING AND FIXING CONSTRAINTS ===")
    print(f"Total constraints: {m.neq}")
    
    for i in range(m.neq):
        if m.eq_type[i] == mujoco.mjtEq.mjEQ_WELD:
            body1_id = m.eq_obj1id[i]
            body2_id = m.eq_obj2id[i]
            
            if body1_id == mocap_body_id and body2_id == base_body_id:
                print(f"Found mocap-base constraint at index {i}")
                
                # 获取当前约束数据
                eq_data = m.eq_data[i]
                print(f"Current constraint data: {eq_data}")
                
                # 修复约束数据
                print("Fixing constraint data...")
                # anchor1 (mocap) = [0, 0, 0] - 保持不变
                # anchor2 (base) = [0, 0, 0] - 修复这个
                eq_data[3:6] = [0, 0, 0]  # 设置 anchor2 为 [0, 0, 0]
                
                print(f"Fixed constraint data: {eq_data}")
                
                # 验证修复
                anchor1 = eq_data[0:3]
                anchor2 = eq_data[3:6]
                print(f"Anchor1 (mocap): {anchor1}")
                print(f"Anchor2 (base): {anchor2}")
                
                if np.allclose(anchor2, [0, 0, 0]):
                    print("✅ Constraint successfully fixed!")
                else:
                    print("❌ Constraint fix failed!")
                    return
    
    # 重置到初始状态
    mujoco.mj_resetDataKeyframe(m, data, 0)
    
    # 获取mocap数据索引
    mocap_data_idx = m.body_mocapid[mocap_body_id]
    if mocap_data_idx < 0:
        print("❌ Mocap body not found!")
        return
    
    print("\\n=== STARTING SIMULATION WITH TRAJECTORY ===")
    print("Press Ctrl+C to exit simulation")
    
    # 轨迹参数
    phase1_duration = 4.0  # 第一阶段：原点到安全位置（3秒）
    phase2_duration = 1.0  # 第二阶段：在安全位置停留（1秒）
    phase3_duration = 3.0  # 第三阶段：安全位置到最终位置（2秒）
    
    total_duration = phase1_duration + phase2_duration + phase3_duration
    
    print(f"Trajectory timing:")
    print(f"  Phase 1 (lift): {phase1_duration}s")
    print(f"  Phase 2 (hold): {phase2_duration}s") 
    print(f"  Phase 3 (lower): {phase3_duration}s")
    print(f"  Total time: {total_duration}s")
    
    # 启动仿真
    with mujoco.viewer.launch_passive(m, data) as viewer:
        step_count = 0
        start_time = time.time()
        
        while viewer.is_running():
            step_start = time.time()
            current_time = time.time() - start_time
            
            # 计算当前目标位置
            if current_time <= phase1_duration:
                # 阶段1：从原点到安全位置
                progress = current_time / phase1_duration
                # 使用平滑插值（S曲线）
                smooth_progress = 3 * progress**2 - 2 * progress**3
                current_target = start_pos + smooth_progress * (safe_pos - start_pos)
                phase = "LIFTING"
                
            elif current_time <= phase1_duration + phase2_duration:
                # 阶段2：在安全位置停留
                current_target = safe_pos
                phase = "HOLDING"
                
            elif current_time <= total_duration:
                # 阶段3：从安全位置到最终位置
                progress = (current_time - phase1_duration - phase2_duration) / phase3_duration
                # 使用平滑插值（S曲线）
                smooth_progress = 3 * progress**2 - 2 * progress**3
                current_target = safe_pos + smooth_progress * (final_pos - safe_pos)
                phase = "LOWERING"
                
            else:
                # 完成后保持在最终位置
                current_target = final_pos
                phase = "COMPLETED"
            
            # 设置mocap位置
            data.mocap_pos[mocap_data_idx] = current_target
            
            # 执行仿真步骤
            mujoco.mj_step(m, data)
            viewer.sync()
            
            # 每500步打印一次状态信息
            if step_count % 500 == 0:
                current_base_pos = data.xpos[base_body_id]
                current_mocap_pos = data.xpos[mocap_body_id]
                distance_to_target = np.linalg.norm(current_base_pos - current_target)
                
                print(f"Step {step_count:5d} | Time: {current_time:5.2f}s | Phase: {phase:9s} | "
                      f"Target: [{current_target[0]:5.2f}, {current_target[1]:5.2f}, {current_target[2]:5.2f}] | "
                      f"Base: [{current_base_pos[0]:5.2f}, {current_base_pos[1]:5.2f}, {current_base_pos[2]:5.2f}] | "
                      f"Error: {distance_to_target:.4f}")
            
            step_count += 1
            
            # 控制仿真速度
            time_until_next_step = m.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Choose between 5dof and 6dof lowcost robot simulation.")
    parser.add_argument("--robot", choices=["6dof"], default="6dof", help="Choose the lowcost robot type")
    args = parser.parse_args()
    do_interactive_sim(args.robot)