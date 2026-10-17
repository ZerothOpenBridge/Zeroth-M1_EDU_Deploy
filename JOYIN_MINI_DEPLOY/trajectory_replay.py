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
def load_trajectory(xlsx_path):
    """Load trajectory from xlsx saved by deploy_swing_leg.py.
    Returns traj_time (N,) in seconds and traj_action (N, 12) in radians."""
    df = pd.read_excel(xlsx_path)
    traj_time = df['time'].values.astype(np.float64)
    action_cols = [f'target_{i}' for i in range(12)]
    traj_action = df[action_cols].values.astype(np.float64)
    print(f"Loaded trajectory: {len(traj_time)} frames, "
          f"duration={traj_time[-1]:.2f}s, dt≈{np.mean(np.diff(traj_time))*1000:.1f}ms")
    return traj_time, traj_action


def interp_action(traj_time, traj_action, elapsed):
    """Interpolate action at elapsed seconds. Clamps to last frame after end."""
    elapsed = np.clip(elapsed, traj_time[0], traj_time[-1])
    return np.array([np.interp(elapsed, traj_time, traj_action[:, i]) for i in range(12)])


def main():
    global action_array, joint_position_array, joint_velocity_array, joint_velocity_filtered_array, timestamp_array, start_time
    import argparse
    parser = argparse.ArgumentParser()
    args_traj = 'test_data/swing_leg/swing_leg_data_20260609_234515.xlsx'
    traj_time, traj_action = load_trajectory(args_traj)
    traj_duration = traj_time[-1]
    parser.add_argument('--sim-xlsx', type=str, default=None,
                        help='(Optional) Sim xlsx for sim vs real comparison plots')
    args = parser.parse_args()
    current_path = os.path.dirname(os.path.abspath(__file__))

    rclpy.init()
    start_time = time.time()

    control_system = ControlSystem('config_mini.yaml')

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=8)  # Increase thread count for kinematic/IMU callback concurrency
    executor.add_node(control_system)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    last_print_time = 0.0
    last_cmd_time = 0.0
    print_period = 0.02   # 50Hz sampling
    cmd_period   = 0.02   # 50Hz command publishing (matches hardware control freq to avoid GIL contention crushing kinematic callback)
    traj_start_time = None  # set on first command send
    traj_done = False
    joint_position_prev = np.zeros(12, dtype=np.float32)
    action = traj_action[0]  # Initialize to avoid undefined action in first print frame
    try:
        while rclpy.ok():
            now = time.time()

            if traj_start_time is None:
                traj_start_time = now

            elapsed = now - traj_start_time

            # --- 50Hz command publishing (rate-limited to reduce GIL contention) ---
            if now - last_cmd_time >= cmd_period:
                if elapsed >= traj_duration:
                    if not traj_done:
                        print(f"Trajectory finished ({traj_duration:.2f}s). Holding last frame.")
                        traj_done = True
                    action = traj_action[-1]
                else:
                    # Time-based interpolation — robust to any real loop timing
                    action = interp_action(traj_time, traj_action, elapsed)
                control_system.publish_joint_angles_command(action)
                last_cmd_time = now

            if now - last_print_time >= print_period:
                state = control_system.robot_control_loop_get_state()
                plotjuggler_data = build_plotjuggler_payload(state, control_system, action=action)
                send_plotjuggler(control_system, plotjuggler_data)
                # cancel bad joint_position data
                joint_position = np.asarray(state["joint_position"]).reshape(-1).copy()
                for i in range(12):
                    # Reject bad data: absolute value exceeds limit OR frame-to-frame jump exceeds ~28deg/20ms
                    if (abs(joint_position[i]) > 100 * math.pi / 180 or
                            abs(joint_position[i] - joint_position_prev[i]) > 0.5):
                        joint_position[i] = joint_position_prev[i]

                joint_velocity = np.asarray(state["joint_velocity"]).reshape(-1)
                joint_velocity_filtered = np.asarray(state["joint_velocity_filtered"]).reshape(-1)
                imu_angular_velocity = np.asarray(state["imu_angular_velocity"]).reshape(-1)
                gravity_in_body = np.asarray(state["gravity_in_body"]).reshape(-1)
                imu_quat = np.asarray(state["imu_quat"]).reshape(-1)

                # Store data to lists, time-aligned to trajectory start (corresponds to sim t=0)
                timestamp = elapsed
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
                # print("imu_angular_velocity:", format_vec(imu_angular_velocity))
                # print("gravity_in_body:", format_vec(gravity_in_body))
                # print("dt: {:.3f} s".format(dt))
                last_print_time = now
                joint_position_prev = joint_position.copy()  # Ensure independent copy


            time.sleep(0.001)   # 1ms sleep: much smaller than 20ms period, greatly reduces GIL contention
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        # Save data to xlsx files
        if len(joint_position_array) > 0:
            # Convert to numpy arrays
            joint_position_array_np = np.array(joint_position_array).reshape(-1, 12)
            joint_velocity_array_np = np.array(joint_velocity_array).reshape(-1, 12)
            joint_velocity_filtered_array_np = np.array(joint_velocity_filtered_array).reshape(-1, 12)
            action_array_np = np.array(action_array).reshape(-1, 12)
            timestamp_array_np = np.array(timestamp_array).reshape(-1, 1)

            # Create DataFrames
            df_action = pd.DataFrame(
                action_array_np,
                columns=[f"joint_{i}_action_deg" for i in range(12)]
            )
            df_action.insert(0, "timestamp", timestamp_array_np)

            df_position = pd.DataFrame(
                joint_position_array_np,
                columns=[f"joint_{i}_position_deg" for i in range(12)]
            )
            df_position.insert(0, "timestamp", timestamp_array_np)
            
            df_velocity = pd.DataFrame(
                joint_velocity_array_np,
                columns=[f"joint_{i}_velocity_deg_s" for i in range(12)]
            )
            df_velocity.insert(0, "timestamp", timestamp_array_np)

            df_velocity_filtered = pd.DataFrame(
                joint_velocity_filtered_array_np,
                columns=[f"joint_{i}_velocity_filtered_deg_s" for i in range(12)]
            )
            df_velocity_filtered.insert(0, "timestamp", timestamp_array_np)

            # Merge raw and filtered velocity into one DataFrame
            df_velocity_combined = df_velocity.copy()
            for i in range(12):
                df_velocity_combined[f"joint_{i}_velocity_filtered_deg_s"] = df_velocity_filtered[f"joint_{i}_velocity_filtered_deg_s"]

            # Save to xlsx files
            output_dir = os.path.join(current_path, "test_data/trajectory")
            os.makedirs(output_dir, exist_ok=True)

            # Use timestamp to generate unique filenames
            timestamp_str = time.strftime("%Y%m%d_%H%M%S", time.localtime(start_time))
            position_file = os.path.join(output_dir, f"joint_position_{timestamp_str}.xlsx")
            velocity_file = os.path.join(output_dir, f"joint_velocity_{timestamp_str}.xlsx")
            action_file = os.path.join(output_dir, f"joint_action_{timestamp_str}.xlsx")
            df_position.to_excel(position_file, index=False)
            df_velocity_combined.to_excel(velocity_file, index=False)
            df_action.to_excel(action_file, index=False)

            print(f"Joint action data saved to: {action_file}")
            print(f"\nJoint position data saved to: {position_file}")
            print(f"Joint velocity (raw + filtered) data saved to: {velocity_file}")


            # Plot joint position vs action (merged)
            # print("\nGenerating joint position vs action comparison plots...")
            # fig_pos, axes_pos = plt.subplots(4, 3, figsize=(15, 12))
            # fig_pos.suptitle("Joint Position vs Action (degrees)", fontsize=16)
            # axes_pos = axes_pos.flatten()
            
            # for i in range(12):
            #     col_name_pos = f"joint_{i}_position_deg"
            #     col_name_action = f"joint_{i}_action_deg"
            #     axes_pos[i].plot(df_position["timestamp"], df_position[col_name_pos], 'b-', linewidth=1.5, label="Position", alpha=0.8)
            #     axes_pos[i].plot(df_action["timestamp"], df_action[col_name_action], 'r--', linewidth=1.5, label="Action", alpha=0.8)
                
            #     axes_pos[i].set_title(f"Joint {i}")
            #     axes_pos[i].set_xlabel("Time (s)")
            #     axes_pos[i].set_ylabel("Angle (deg)")
            #     axes_pos[i].grid(True, alpha=0.3)
            #     axes_pos[i].legend(loc='best', fontsize=8)
            
            # plt.tight_layout()
            # position_plot_file = os.path.join(output_dir, f"joint_position_action_{timestamp_str}.png")
            # plt.savefig(position_plot_file, dpi=100)
            # print(f"Joint position vs action comparison plots saved to: {position_plot_file}")
            # plt.close(fig_pos)

            # Plot joint velocity (raw vs filtered comparison)
            # print("Generating joint velocity plots (raw vs filtered comparison)...")
            # fig_vel, axes_vel = plt.subplots(4, 3, figsize=(15, 12))
            # fig_vel.suptitle("Joint Velocity (degrees/s)", fontsize=16)
            # axes_vel = axes_vel.flatten()

            # for i in range(12):
            #     col_raw = f"joint_{i}_velocity_deg_s"
            #     col_filt = f"joint_{i}_velocity_filtered_deg_s"
            #     axes_vel[i].plot(df_velocity["timestamp"], df_velocity[col_raw], 'r-', linewidth=1, alpha=0.6, label="raw")
            #     axes_vel[i].plot(df_velocity_filtered["timestamp"], df_velocity_filtered[col_filt], 'b-', linewidth=1.5, label="filtered")
            #     axes_vel[i].set_title(f"Joint {i}")
            #     axes_vel[i].set_xlabel("Time (s)")
            #     axes_vel[i].set_ylabel("Velocity (deg/s)")
            #     axes_vel[i].grid(True, alpha=0.3)
            #     axes_vel[i].legend(loc='best', fontsize=8)

            # plt.tight_layout()
            # velocity_plot_file = os.path.join(output_dir, f"joint_velocity_{timestamp_str}.png")
            # plt.savefig(velocity_plot_file, dpi=100)
            # print(f"Joint velocity plots saved to: {velocity_plot_file}")
            # plt.close(fig_vel)

            # --- Sim vs real comparison plots ---
            if args.sim_xlsx:
                sim_df = pd.read_excel(args.sim_xlsx)
                t_sim = sim_df['time'].values
                sim_pos = np.degrees(sim_df[[f'qpos_{i}' for i in range(12)]].values)
                sim_vel = np.degrees(sim_df[[f'qvel_{i}' for i in range(12)]].values)
                action_sim = np.degrees(sim_df[[f'target_{i}' for i in range(12)]].values)

                t_real = np.array(timestamp_array)
                real_pos = joint_position_array_np
                real_vel = joint_velocity_array_np
                action_real = action_array_np

                fig_cmp_pos, axes_cmp = plt.subplots(4, 3, figsize=(15, 12))
                fig_cmp_pos.suptitle("Position: Sim (green) vs Real (blue) (deg)", fontsize=14)
                axes_cmp = axes_cmp.flatten()
                for i in range(12):
                    axes_cmp[i].plot(t_sim, sim_pos[:, i], 'g-', linewidth=1.5, label='sim', alpha=0.8)
                    axes_cmp[i].plot(t_real, real_pos[:, i], 'b-', linewidth=1.5, label='real', alpha=0.8)
                    axes_cmp[i].plot(t_sim, action_sim[:, i], 'g--', linewidth=1, label='sim action', alpha=0.4)
                    axes_cmp[i].plot(t_real, action_real[:, i], 'b--', linewidth=1, label='real action', alpha=0.4)
                    axes_cmp[i].set_title(f"Joint {i}")
                    axes_cmp[i].set_xlabel("Time (s)")
                    axes_cmp[i].set_ylabel("Angle (deg)")
                    axes_cmp[i].grid(True, alpha=0.3)
                    axes_cmp[i].legend(loc='best', fontsize=8)
                plt.tight_layout()
                fig_cmp_pos.savefig(os.path.join(output_dir, f"compare_pos_{timestamp_str}.png"), dpi=150, bbox_inches='tight')

                fig_cmp_vel, axes_cmpv = plt.subplots(4, 3, figsize=(15, 12))
                fig_cmp_vel.suptitle("Velocity: Sim (green) vs Real (orange) (deg/s)", fontsize=14)
                axes_cmpv = axes_cmpv.flatten()
                for i in range(12):
                    axes_cmpv[i].plot(t_sim, sim_vel[:, i], 'g-', linewidth=1.5, label='sim', alpha=0.8)
                    axes_cmpv[i].plot(t_real, real_vel[:, i], color='orange', linewidth=1.5, label='real', alpha=0.8)
                    axes_cmpv[i].set_title(f"Joint {i}")
                    axes_cmpv[i].set_xlabel("Time (s)")
                    axes_cmpv[i].set_ylabel("Velocity (deg/s)")
                    axes_cmpv[i].grid(True, alpha=0.3)
                    axes_cmpv[i].legend(loc='best', fontsize=8)
                plt.tight_layout()
                fig_cmp_vel.savefig(os.path.join(output_dir, f"compare_vel_{timestamp_str}.png"), dpi=150, bbox_inches='tight')

                # --- Time-aligned comparison: interpolate real data to sim time axis ---
                # Common time axis: overlapping range of both
                t_end = min(t_sim[-1], t_real[-1])
                t_common = t_sim[t_sim <= t_end]

                real_pos_aligned = np.column_stack(
                    [np.interp(t_common, t_real, real_pos[:, i]) for i in range(12)]
                )
                real_vel_aligned = np.column_stack(
                    [np.interp(t_common, t_real, real_vel[:, i]) for i in range(12)]
                )
                sim_pos_aligned = sim_pos[:len(t_common)]
                sim_vel_aligned = sim_vel[:len(t_common)]

                # Save table: time-aligned real and sim position data
                traj_compare = pd.DataFrame({"timestamp": t_common})
                traj_compare = pd.concat([traj_compare, pd.DataFrame(
                    real_pos_aligned, columns=[f"real_{i}_deg" for i in range(12)]
                )], axis=1)
                traj_compare = pd.concat([traj_compare, pd.DataFrame(
                    sim_pos_aligned, columns=[f"sim_{i}_deg" for i in range(12)]
                )], axis=1)
                traj_replay_file = os.path.join(output_dir, f"compare_traj_replay_{timestamp_str}.xlsx")
                traj_compare.to_excel(traj_replay_file, index=False)

                print(f"Joint response comparison data saved to: {traj_replay_file}")
                
                
                # fig_aln_pos, axes_aln_pos = plt.subplots(4, 3, figsize=(15, 12))
                # fig_aln_pos.suptitle("Position Aligned: Sim (green) vs Real (blue) (deg)", fontsize=14)
                # axes_aln_pos = axes_aln_pos.flatten()
                # for i in range(12):
                #     axes_aln_pos[i].plot(t_common, sim_pos_aligned[:, i], 'g-', linewidth=1.5, label='sim', alpha=0.8)
                #     axes_aln_pos[i].plot(t_common, real_pos_aligned[:, i], 'b-', linewidth=1.5, label='real', alpha=0.8)
                #     axes_aln_pos[i].set_title(f"Joint {i}")
                #     axes_aln_pos[i].set_xlabel("Time (s)")
                #     axes_aln_pos[i].set_ylabel("Angle (deg)")
                #     axes_aln_pos[i].grid(True, alpha=0.3)
                #     axes_aln_pos[i].legend(loc='best', fontsize=8)
                # plt.tight_layout()
                # fig_aln_pos.savefig(os.path.join(output_dir, f"compare_pos_aligned_{timestamp_str}.png"), dpi=150, bbox_inches='tight')

                # fig_aln_vel, axes_aln_vel = plt.subplots(4, 3, figsize=(15, 12))
                # fig_aln_vel.suptitle("Velocity Aligned: Sim (green) vs Real (orange) (deg/s)", fontsize=14)
                # axes_aln_vel = axes_aln_vel.flatten()
                # for i in range(12):
                #     axes_aln_vel[i].plot(t_common, sim_vel_aligned[:, i], 'g-', linewidth=1.5, label='sim', alpha=0.8)
                #     axes_aln_vel[i].plot(t_common, real_vel_aligned[:, i], color='orange', linewidth=1.5, label='real', alpha=0.8)
                #     axes_aln_vel[i].set_title(f"Joint {i}")
                #     axes_aln_vel[i].set_xlabel("Time (s)")
                #     axes_aln_vel[i].set_ylabel("Velocity (deg/s)")
                #     axes_aln_vel[i].grid(True, alpha=0.3)
                #     axes_aln_vel[i].legend(loc='best', fontsize=8)
                # plt.tight_layout()
                # fig_aln_vel.savefig(os.path.join(output_dir, f"compare_vel_aligned_{timestamp_str}.png"), dpi=150, bbox_inches='tight')

                # --- Phase-space error plot: Δpos vs Δvel ---
                dpos = real_pos_aligned - sim_pos_aligned  # (N, 12) deg
                dvel = real_vel_aligned - sim_vel_aligned  # (N, 12) deg/s

                fig_phase, axes_phase = plt.subplots(4, 3, figsize=(15, 12))
                fig_phase.suptitle("Phase-space error: Δq (real−sim) [deg] vs Δq̇ (real−sim) [deg/s]", fontsize=13)
                axes_phase = axes_phase.flatten()
                for i in range(12):
                    sc = axes_phase[i].plot(dpos[:, i], dvel[:, i],'b-', linewidth=1.5, label='real', alpha=0.8)
                    axes_phase[i].axhline(0, color='k', linewidth=0.8)
                    axes_phase[i].axvline(0, color='k', linewidth=0.8)
                    axes_phase[i].set_title(f"Joint {i}")
                    axes_phase[i].set_xlabel("Δq (deg)")
                    axes_phase[i].set_ylabel("Δq̇ (deg/s)")
                    axes_phase[i].grid(True, alpha=0.3)
                plt.tight_layout()
                fig_phase.savefig(os.path.join(output_dir, f"phase_error_{timestamp_str}.png"), dpi=150, bbox_inches='tight')

                print(f"Comparison plots saved to: {output_dir}")
                plt.show()

                
        
        control_system.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()