#!/usr/bin/env python3
import math
import time
import threading
import numpy as np
import rclpy
import pandas as pd
import os
import matplotlib.pyplot as plt

from control_system import ControlSystem


class ControlSystemMonitor(ControlSystem):
    def zero_pose(self):
        self.get_logger().info("skip zero_pose in monitor mode")

    def ros_robot_init_proc(self):
        self.get_logger().info("skip ros_robot_init_proc in monitor mode")

    def vehicle_velocity_msg_callback(self, msg):
        pass

    def vehicle_feedback_msg_callback(self, msg):
        pass

    def publish_joint_angles_command(self, target_position):
        self.get_logger().debug("skip publish_joint_angles_command in monitor mode")

    def publish_joint_velocity_value_command(self, velocity_value):
        self.get_logger().debug("skip publish_joint_velocity_value_command in monitor mode")

    def publish_joint_pid_ctrl_command(self, kp, ki, kd):
        self.get_logger().debug("skip publish_joint_pid_ctrl_command in monitor mode")

    def publish_joint_pid_param_get_trigger_command(self):
        self.get_logger().debug("skip publish_joint_pid_param_get_trigger_command in monitor mode")


def format_vec(arr, digits=3):
    arr = np.asarray(arr).reshape(-1)
    return "[" + ", ".join(f"{x:.{digits}f}" for x in arr) + "]"


def quat_wxyz_to_rpy(quat_wxyz):
    quat_wxyz = np.asarray(quat_wxyz, dtype=np.float64).reshape(-1)
    w, x, y, z = quat_wxyz

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw], dtype=np.float32)

Bd = 0.1
joint_velocity_filt = np.zeros(12, dtype=np.float32)
imu_angular_velocity_filt = np.zeros(3, dtype=np.float32)

def build_plotjuggler_payload(state, control_system, action=None):
    global joint_velocity_filt, imu_angular_velocity_filt
    imu_quat = np.asarray(state["imu_quat"], dtype=np.float32).reshape(-1)
    imu_angular_velocity_raw = control_system.ang_vel_scale * np.asarray(state["imu_angular_velocity"], dtype=np.float32).reshape(-1)
    imu_angular_velocity_filt = (1 - Bd) * imu_angular_velocity_filt + Bd * imu_angular_velocity_raw
    joint_position = np.asarray(np.degrees(state["joint_position"]), dtype=np.float32).reshape(-1)
    joint_velocity_raw = np.asarray(np.degrees(state["joint_velocity"]), dtype=np.float32).reshape(-1)
    joint_velocity_filt = (1 - Bd) * joint_velocity_filt + Bd * joint_velocity_raw
    gravity_in_body = np.asarray(state["gravity_in_body"], dtype=np.float32).reshape(-1)
    rpy = quat_wxyz_to_rpy(imu_quat)

    payload = {
        "imu_quat": imu_quat.tolist(),
        "rpy": rpy.tolist(),
        "imu_angular_velocity_raw": imu_angular_velocity_raw.tolist(),
        "joint_velocity_raw": joint_velocity_raw.tolist(),
        "joint_velocity_filt": joint_velocity_filt.tolist(),
        "joint_position": joint_position.tolist(),
        "gyro_x_raw": float(imu_angular_velocity_raw[0]),
        "gyro_y_raw": float(imu_angular_velocity_raw[1]),
        "gyro_z_raw": float(imu_angular_velocity_raw[2]),
        "gyro_x_filt": float(imu_angular_velocity_filt[0]),
        "gyro_y_filt": float(imu_angular_velocity_filt[1]),
        "gyro_z_filt": float(imu_angular_velocity_filt[2]),
        "quat": imu_quat.tolist(),
        "gravity_in_body": gravity_in_body.tolist(),
    }
    if action is not None:
        payload["action"] = np.asarray(np.degrees(action), dtype=np.float32).reshape(-1).tolist()
    return payload


def send_plotjuggler(node, trans_data):
    if node.enable_udp and node.udp_sender:
        node.udp_sender.send_imu_data(trans_data)


# Data storage lists
action_array = []
joint_position_array = []
joint_velocity_array = []
joint_velocity_filtered_array = []
timestamp_array = []
start_time = None
deviation = np.radians(np.array([0,0,0,0,0,0,0,-10,0,0,0,0], dtype=np.float32))
def main():
    global action_array, joint_position_array, joint_velocity_array, joint_velocity_filtered_array, timestamp_array, start_time
    current_path = os.path.dirname(os.path.abspath(__file__))
    
    rclpy.init()
    wait_count = 0
    start_time = time.time()
    
    # Monitor mode only (print data)
    # node = ControlSystemMonitor("config_mini.yaml")
    control_system = ControlSystem('config_mini.yaml')

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(control_system)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    last_print_time = 0.0
    print_period = 0.02
    zero_position = np.array([0,0,0,0,0,0,0,0,0,0,0,0])
    zero_position_arm = np.array([0,0,0,0,0,0,0,0]) 
    try:
        while rclpy.ok():
            now = time.time()
            # print(now)
            # wait_count += 1
            # if wait_count < 400:
            #     action = control_system.default_angles    
            # elif wait_count >= 400 and wait_count < 800:
            #     action = control_system.default_angles + deviation
            # elif wait_count == 800:
            #     wait_count = 0
            action = zero_position
            control_system.publish_joint_angles_command(zero_position, zero_position_arm)

            if now - last_print_time >= print_period:
                state = control_system.robot_control_loop_get_state()
                plotjuggler_data = build_plotjuggler_payload(state, control_system, action=action)
                send_plotjuggler(control_system, plotjuggler_data)

                joint_position = np.asarray(state["joint_position"]).reshape(-1)
                joint_velocity = np.asarray(state["joint_velocity"]).reshape(-1)
                joint_velocity_filtered = np.asarray(state["joint_velocity_filtered"]).reshape(-1)
                imu_angular_velocity = np.asarray(state["imu_angular_velocity"]).reshape(-1)
                gravity_in_body = np.asarray(state["gravity_in_body"]).reshape(-1)
                imu_quat = np.asarray(state["imu_quat"]).reshape(-1)

                # Store data to lists
                timestamp = now - start_time
                dt = now - last_print_time
                timestamp_array.append(timestamp)
                joint_position_array.append(np.degrees(joint_position))
                joint_velocity_array.append(np.degrees(joint_velocity))
                joint_velocity_filtered_array.append(np.degrees(joint_velocity_filtered))
                action_array.append(np.degrees(action))
                # print("=" * 80)
                # print("joint_position(rad):", format_vec(joint_position))
                # print("joint_position(deg):", format_vec(np.degrees(joint_position)))
                # print("joint_velocity(rad/s):", format_vec(joint_velocity))
                # print("imu_quat[wxyz]:", format_vec(imu_quat))
                print("imu_angular_velocity:", format_vec(imu_angular_velocity))
                # print("gravity_in_body:", format_vec(gravity_in_body))
                # print("dt: {:.3f} s".format(dt))
                last_print_time = now


            time.sleep(0.0001)
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        # Save data to xlsx file
        # if len(joint_position_array) > 0:
        #     # Convert to numpy arrays
        #     joint_position_array_np = np.array(joint_position_array).reshape(-1, 12)
        #     joint_velocity_array_np = np.array(joint_velocity_array).reshape(-1, 12)
        #     joint_velocity_filtered_array_np = np.array(joint_velocity_filtered_array).reshape(-1, 12)
        #     action_array_np = np.array(action_array).reshape(-1, 12)
        #     timestamp_array_np = np.array(timestamp_array).reshape(-1, 1)
            
        #     # Create DataFrame
        #     df_action = pd.DataFrame(
        #         action_array_np,
        #         columns=[f"joint_{i}_action_deg" for i in range(12)]
        #     )
        #     df_action.insert(0, "timestamp", timestamp_array_np)

        #     df_position = pd.DataFrame(
        #         joint_position_array_np,
        #         columns=[f"joint_{i}_position_deg" for i in range(12)]
        #     )
        #     df_position.insert(0, "timestamp", timestamp_array_np)
            
        #     df_velocity = pd.DataFrame(
        #         joint_velocity_array_np,
        #         columns=[f"joint_{i}_velocity_deg_s" for i in range(12)]
        #     )
        #     df_velocity.insert(0, "timestamp", timestamp_array_np)

        #     df_velocity_filtered = pd.DataFrame(
        #         joint_velocity_filtered_array_np,
        #         columns=[f"joint_{i}_velocity_filtered_deg_s" for i in range(12)]
        #     )
        #     df_velocity_filtered.insert(0, "timestamp", timestamp_array_np)

        #     # Merge raw and filtered velocity into the same DataFrame
        #     df_velocity_combined = df_velocity.copy()
        #     for i in range(12):
        #         df_velocity_combined[f"joint_{i}_velocity_filtered_deg_s"] = df_velocity_filtered[f"joint_{i}_velocity_filtered_deg_s"]

        #     # Save to xlsx file
        #     output_dir = os.path.join(current_path, "test_data/trajectory")
        #     os.makedirs(output_dir, exist_ok=True)

        #     # Use timestamp to generate unique filename
        #     timestamp_str = time.strftime("%Y%m%d_%H%M%S", time.localtime(start_time))
        #     position_file = os.path.join(output_dir, f"joint_position_{timestamp_str}.xlsx")
        #     velocity_file = os.path.join(output_dir, f"joint_velocity_{timestamp_str}.xlsx")
        #     action_file = os.path.join(output_dir, f"joint_action_{timestamp_str}.xlsx")
        #     df_position.to_excel(position_file, index=False)
        #     df_velocity_combined.to_excel(velocity_file, index=False)
        #     df_action.to_excel(action_file, index=False)

        #     print(f"Joint command data saved to: {action_file}")
        #     print(f"\nJoint position data saved to: {position_file}")
        #     print(f"Joint velocity (raw + filtered) data saved to: {velocity_file}")
            

        #     # Plot joint position and command chart (combined)
        #     print("\nGenerating joint position vs command comparison chart...")
        #     fig_pos, axes_pos = plt.subplots(4, 3, figsize=(15, 12))
        #     fig_pos.suptitle("Joint Position vs Action (degrees)", fontsize=16)
        #     axes_pos = axes_pos.flatten()
            
        #     for i in range(12):
        #         col_name_pos = f"joint_{i}_position_deg"
        #         col_name_action = f"joint_{i}_action_deg"
        #         axes_pos[i].plot(df_position["timestamp"], df_position[col_name_pos], 'b-', linewidth=1.5, label="Position", alpha=0.8)
        #         axes_pos[i].plot(df_action["timestamp"], df_action[col_name_action], 'r--', linewidth=1.5, label="Action", alpha=0.8)
        #         axes_pos[i].set_title(f"Joint {i}")
        #         axes_pos[i].set_xlabel("Time (s)")
        #         axes_pos[i].set_ylabel("Angle (deg)")
        #         axes_pos[i].grid(True, alpha=0.3)
        #         axes_pos[i].legend(loc='best', fontsize=8)
            
        #     plt.tight_layout()
        #     position_plot_file = os.path.join(output_dir, f"joint_position_action_{timestamp_str}.png")
        #     plt.savefig(position_plot_file, dpi=100)
        #     print(f"Joint position vs command chart saved to: {position_plot_file}")
        #     plt.close(fig_pos)
            
        #     # Plot joint velocity chart (raw vs filtered comparison)
        #     print("Generating joint velocity chart (raw vs filtered comparison)...")
        #     fig_vel, axes_vel = plt.subplots(4, 3, figsize=(15, 12))
        #     fig_vel.suptitle("Joint Velocity (degrees/s)", fontsize=16)
        #     axes_vel = axes_vel.flatten()

        #     for i in range(12):
        #         col_raw = f"joint_{i}_velocity_deg_s"
        #         col_filt = f"joint_{i}_velocity_filtered_deg_s"
        #         axes_vel[i].plot(df_velocity["timestamp"], df_velocity[col_raw], 'r-', linewidth=1, alpha=0.6, label="raw")
        #         axes_vel[i].plot(df_velocity_filtered["timestamp"], df_velocity_filtered[col_filt], 'b-', linewidth=1.5, label="filtered")
        #         axes_vel[i].set_title(f"Joint {i}")
        #         axes_vel[i].set_xlabel("Time (s)")
        #         axes_vel[i].set_ylabel("Velocity (deg/s)")
        #         axes_vel[i].grid(True, alpha=0.3)
        #         axes_vel[i].legend(loc='best', fontsize=8)

        #     plt.tight_layout()
        #     velocity_plot_file = os.path.join(output_dir, f"joint_velocity_{timestamp_str}.png")
        #     plt.savefig(velocity_plot_file, dpi=100)
        #     print(f"Joint velocity chart saved to: {velocity_plot_file}")
        #     plt.close(fig_vel)
        
        control_system.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()