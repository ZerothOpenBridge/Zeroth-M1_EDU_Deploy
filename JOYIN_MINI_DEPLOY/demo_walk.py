"""
RL deploy code for the Zeroth M1 robots

start walking by running: python demo_walk.py

"""


from udp_sender import UDPPlotJugglerSender
import os
import numpy
import math
from numba import njit
from ischedule import run_loop, schedule,run_pending
from control_system import ControlSystem
import numpy as np
import time
import rclpy
import onnxruntime as ort
import argparse
import keyboard
import threading
import time
import sys
import termios
import tty



policy_file_path = None
policy_model = None
policy_action = None
obs_buf_stack = None
MAX_SPEED_X = 0.2
MAX_SPEED_Y = 0.01
MAX_SPEED_YAW_RATE=1.1

exit_flag = False


class cmd:
    vx = 0.0
    vy = 0.0
    dyaw = 0

def getch():
    """Read a single character from stdin (non-blocking)"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def keyboard_listener():
    """keyboard listening thread"""
    print("keyboard listening started:")
    print("8:forward 2:backward 4:left 6:right")
    print("7/1:turn left  9/3:turn right  0:stop  ESC:exit keyboard listening")
    
    while True:
        try:
            key = getch()
            
            if key == '8':
                cmd.vx += 0.01
            elif key == '2':
                cmd.vx -= 0.01
            elif key == '4':
                cmd.vy += 0.01
                # cmd.vy =0.1
            elif key == '6':
                cmd.vy -= 0.01
                # cmd.vy = -0.1
            elif key == '7' or key == '1':
                cmd.dyaw += 0.05
                # cmd.dyaw = 0.3
            elif key == '9' or key == '3':
                cmd.dyaw -= 0.05
                # cmd.dyaw = -0.3
            elif key == '0':
                cmd.vx = 0.0
                cmd.vy = 0.0
                cmd.dyaw = 0.0
            elif key == '\x1b':  # ESC key
                break
                
            print("command: ", cmd.vx, cmd.vy, cmd.dyaw)
            
        except Exception as e:
            print(f"input error: {e}")
            break

# use keyboard listening (assign motion speeds using keyboard)
# keyboard_thread = threading.Thread(target=keyboard_listener, daemon=True)
# keyboard_thread.start()
def clip(n, min_val, max_val):
    return max(min_val, min(n, max_val))

def joystick_mapper(max_speed_x, max_speed_y, max_speed_yaw_rate, control_system):
    """mapping joystick values to motion speeds"""
    print("[joystick_mapper] thread started")
    global exit_flag
    _log_counter = 0
    while True:
        try:
            fb  = getattr(control_system, 'joy_left_stick_fb',  0.0)
            lr  = getattr(control_system, 'joy_left_stick_lr',  0.0)
            yaw = getattr(control_system, 'joy_right_stick_lr', 0.0)
            cmd.vx   = fb  * max_speed_x
            cmd.vy   = lr  * max_speed_y
            cmd.dyaw = yaw * max_speed_yaw_rate

            cmd.vx = clip(cmd.vx,-0.5*max_speed_x, max_speed_x)
            exit_flag = getattr(control_system, 'joy_X', False)
            if _log_counter % 50 == 0:
                print(f"[joystick] vx={cmd.vx:.3f}  vy={cmd.vy:.3f}  dyaw={cmd.dyaw:.3f}")
            _log_counter += 1
        except Exception as e:
            print(f"[joystick_mapper] error: {e}")
        time.sleep(0.02)            

def get_obs(commands, sin_phase, cos_phase, gravity_projection, ang_vel, q, dq, action, control_system):
    if control_system.obs_config == "config1":
        obs_buf = numpy.concatenate([
            commands * control_system.cmd_scale,
            ang_vel * control_system.ang_vel_scale,
            gravity_projection * control_system.gravity_scale,
            q * control_system.dof_pos_scale,
            # dq * control_system.dof_vel_scale,
            action,
            sin_phase,
            cos_phase,
        ], axis=-1)
    elif control_system.obs_config == "config2":
        obs_buf = numpy.concatenate([
            commands * control_system.cmd_scale,
            ang_vel[:, 2:3] * control_system.ang_vel_scale,  # yaw only
            gravity_projection * control_system.gravity_scale,
            q * control_system.dof_pos_scale,
            dq * control_system.dof_vel_scale,
            action,
            sin_phase,
            cos_phase,
        ], axis=-1)
    else:
        raise ValueError(f"get_obs: unknown obs_config='{control_system.obs_config}', expected 'config1' or 'config2'")
    return obs_buf



class PolicyDeployer:
    def __init__(self, model_path, control_system):
        self.session = ort.InferenceSession(model_path)
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name

    def preprocess(self, new_obs):
        """Update observation history and build network input"""
        new_obs = np.asarray(new_obs, dtype=np.float32).reshape(1, -1)
        self.history.append(new_obs)
        return np.concatenate(self.history).reshape(1, control_system.num_obs*control_system.num_stack).astype(np.float32)

    def postprocess(self, output):
        """Convert network output to joint control commands"""
        return output.reshape(-1).tolist()

    def inference(self, obs):
        
        outputs = self.session.run([self.output_name], {self.input_name: obs})
        current_action = outputs[0].reshape(1, -1)

        return self.postprocess(current_action)
        # return self.postprocess(self.smoothed_action)


# init variables
roll_rate, pitch_rate, yaw_rate, roll_rate_prev, pitch_rate_prev, yaw_rate_prev = [0, 0, 0, 0, 0, 0]
IMU_RATE_BUFFER_SIZE = 5
imu_rate_buffer = np.zeros((IMU_RATE_BUFFER_SIZE, 2), dtype=np.float32)  # col0=roll_rate, col1=pitch_rate
imu_rate_buffer_index = 0
imu_rate_buffer_count = 0
imu_rate_raw_array = []      # [(roll_raw, pitch_raw), ...]
imu_rate_filtered_array = [] # [(roll_filt, pitch_filt), ...]
imu_rate_timestamp_array = []
imu_rate_start_time = None

def main():
    global parsed_args,udp_sender
    current_path = os.path.dirname(os.path.abspath(__file__))
    # print(current_path)

    parser = argparse.ArgumentParser(description='ROS 2 Robot Control System Node')
    parser.add_argument(
        '--config',
        type=str,
        default='config_mini.yaml',
        help='control system config file path, relative to this script or absolute path',
    )
    parser.add_argument(
        '--model',
        type=str,
        default = 'model/model.onnx',
        # default = 'model/your_model_name.onnx' # use your custom model here
        help='onnx model path, relative to this script or absolute path',
    )
    parser.add_argument(
        '--read_xlsx',
        #type=bool,
        action='store_true',
        default=False,
    )
    parser.add_argument(
        '--write_xlsx',
        #type=bool,
        action='store_true',
        default=False, 
    )
    parsed_args = parser.parse_args()
    global policy_file_path, policy_model, control_system

    rclpy.init()

    config_dir = parsed_args.config
    if not os.path.isabs(config_dir):
        config_dir = os.path.join(current_path, config_dir)
    control_system = ControlSystem(config_dir)
    
    # Set control loop frequency
    target_control_frequency = 50  # control frequency, 50Hz
    target_control_period_in_s = 1.0 / target_control_frequency  # control period

    # Start joystick thread(comment out keyboard and uncomment this when using joystick)
    joystick_thread = threading.Thread(target=joystick_mapper, args=(MAX_SPEED_X, MAX_SPEED_Y, MAX_SPEED_YAW_RATE, control_system), daemon=True)
    joystick_thread.start()

    model_path = parsed_args.model
    if not os.path.isabs(model_path):
        model_path = os.path.join(current_path, model_path)
    #st()
    policy_model = PolicyDeployer(model_path, control_system)

    global policy_control_num_of_joints, policy_control_index_of_joints
    global joint_measured_position_for_policy, joint_measured_velocity_for_policy
    global joint_measured_position_for_policy_prev
    num_actions = control_system.num_actions
    policy_control_num_of_joints = num_actions
    joint_measured_position_for_policy_prev = numpy.zeros(num_actions, dtype=numpy.float32)
    if num_actions == 12:
        # policy order: L_leg×6, R_leg×6  →  leg state index 0..11
        policy_control_index_of_joints = numpy.array([0,1,2,3,4,5, 6,7,8,9,10,11])
    else:
        # 18dof: policy order [L_arm×3, L_leg×6, R_arm×3, R_leg×6]
        # combined state: arm[0..7] then leg[0..11], offset by 8 for leg
        # arm indices 0,1,2 (L) and 4,5,6 (R); leg indices 8..19 → stored as 0..11 with +8 offset
        # We build a unified 20-element view: arm[0..7] + leg[0..11]
        policy_control_index_of_joints = numpy.array([
            0, 1, 2,           # L_arm: shoulder_pitch, shoulder_roll, elbow_roll
            8, 9,10,11,12,13,  # L_leg: hip_yaw..ankle_roll
            4, 5, 6,           # R_arm: shoulder_pitch, shoulder_roll, elbow_roll
           14,15,16,17,18,19,  # R_leg: hip_yaw..ankle_roll
        ])
    print(f"Using config: {config_dir}")
    print(f"Using model: {model_path}")

    # Register the algorithm callback at the target frequency
    schedule(algorithm, interval=target_control_period_in_s)

    # Use MultiThreadedExecutor so ROS 2 callbacks run at high frequency in background threads
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=8)
    executor.add_node(control_system)

    # Start executor in a background thread to handle callbacks asynchronously
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    import gc
    gc.disable()   # ← Disable GC during control loop to eliminate random pauses

    # FIFO scheduling
    try:
        param = os.sched_param(50)   # Priority 1-99, 50 is reasonable, not highest
        os.sched_setscheduler(0, os.SCHED_FIFO, param)
        print("[INFO] SCHED_FIFO RT scheduling enabled, priority=50")
    except PermissionError:
        print("[WARN] Root permission required to set RT scheduling")

    # bind core for policy inference
    os.sched_setaffinity(0, {7})  # Bind main loop to CPU7 (last core, usually least busy)
    print("[INFO] CPU affinity set to core 7")
    print("[INFO] GC disabled for control loop")
    print("[INFO] ROS 2 executor started in background, listening for callbacks...")

    try:
        # ROS 2 node and ischedule loop both run normally here;
        # callbacks are dispatched asynchronously by the executor thread 
        global start_time
        start_time = time.perf_counter()  # recorded once at deployment start
        enter_count = 0
        enter_time = 0
        while rclpy.ok():
           
            run_pending()

            # Yield briefly so the executor thread can process callbacks (non-blocking)
            time.sleep(0.001)
            if exit_flag == True:
                break 
 
    except KeyboardInterrupt:
        #st()
        pass
    finally:
        # Shut down executor and clean up resources
        executor.shutdown()
        control_system.destroy_node()
        rclpy.shutdown()


@njit(cache=True, fastmath=True)
def quat_wxyz_to_rpy(quat_wxyz):
    """
    Convert quaternion [w, x, y, z] to roll-pitch-yaw (rad).
    Input:  float64[4]
    Output: float32[3]
    """
    w = quat_wxyz[0]; x = quat_wxyz[1]
    y = quat_wxyz[2]; z = quat_wxyz[3]

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if sinp >  1.0: sinp =  1.0
    if sinp < -1.0: sinp = -1.0
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    out = np.empty(3, dtype=np.float32)
    out[0] = roll; out[1] = pitch; out[2] = yaw
    return out

# warmup: trigger JIT at module load time to avoid stutter on first call
_wq_rpy = np.array([1., 0., 0., 0.], dtype=np.float32)  # Align with imu_quat(float32) at call site
quat_wxyz_to_rpy(_wq_rpy)

Bd = 0.1
joint_velocity_filt = np.zeros(12, dtype=np.float32)
def build_plotjuggler_payload(state, action=None):
    global joint_velocity_filt, rpy
    imu_quat = np.asarray(state["imu_quat"], dtype=np.float32).reshape(-1)
    imu_angular_velocity = control_system.ang_vel_scale * np.asarray(state["imu_angular_velocity"], dtype=np.float32).reshape(-1)
    joint_position = np.asarray(np.degrees(state["joint_position"]), dtype=np.float32).reshape(-1)
    joint_velocity_raw = np.asarray(np.degrees(state["joint_velocity"]), dtype=np.float32).reshape(-1)
    joint_velocity_filt = (1 - Bd) * joint_velocity_filt + Bd * joint_velocity_raw
    gravity_in_body = np.asarray(state["gravity_in_body"], dtype=np.float32).reshape(-1)
    rpy = quat_wxyz_to_rpy(imu_quat)

    payload = {
        "imu_quat": imu_quat.tolist(),
        "rpy": rpy.tolist(),
        "imu_angular_velocity_raw": imu_angular_velocity.tolist(),
        "joint_velocity_raw": joint_velocity_raw.tolist(),
        "joint_velocity_filt": joint_velocity_filt.tolist(),
        "joint_position": joint_position.tolist(),
        "gyro_x_raw": float(imu_angular_velocity[0]),
        "gyro_y_raw": float(imu_angular_velocity[1]),
        "gyro_z_raw": float(imu_angular_velocity[2]),
        "gyro_x_filt": roll_rate,
        "gyro_y_filt": pitch_rate,
        "gyro_z_filt": yaw_rate,
        "quat": imu_quat.tolist(),
        "gravity_in_body": gravity_in_body.tolist(),
    }
    if action is not None:
        payload["action"] = np.asarray(np.degrees(action), dtype=np.float32).reshape(-1).tolist()
    return payload


def send_plotjuggler(node, trans_data):
    if node.enable_udp and node.udp_sender:
        node.udp_sender.send_imu_data(trans_data)
        
t_curr = 0
joint_measured_position_for_policy_prev = None  # initialized in main() after num_actions is known

# Tracker for diagnosing gravity data update frequency
gravity_update_tracker = {
    'last_value': None,
    'update_count': 0,
    'loop_count': 0,
    'last_print_time': time.time()
}


robot_num_of_joints = 18
# policy_control_num_of_joints, policy_control_index_of_joints, default_angles_for_policy
# are set in main() after control_system is initialized, based on num_actions from config.
policy_control_num_of_joints = None
policy_control_index_of_joints = None
default_angles_for_policy = None
joint_measured_position_for_policy = None
joint_measured_velocity_for_policy = None
joint_target_position = numpy.zeros(12)
joint_target_position_arm = numpy.zeros(8)

rpy = np.array([0, 0, 0], dtype=np.float32) 
def algorithm():
    commands = numpy.array([cmd.vx, cmd.vy, cmd.dyaw]).reshape(-1,3)
    global policy_model, policy_action, obs_buf_stack, iteration, start_time, exit_flag
    global t_curr, joint_measured_position_for_policy_prev
    global roll_rate, pitch_rate, yaw_rate, roll_rate_prev, pitch_rate_prev, yaw_rate_prev
    global imu_rate_buffer, imu_rate_buffer_index, imu_rate_buffer_count  # used by commented-out median filter
    global imu_rate_raw_array, imu_rate_filtered_array, imu_rate_timestamp_array, imu_rate_start_time
    global joint_target_position, joint_measured_position_for_policy, joint_measured_velocity_for_policy
    

    # update state
    """
    state:
    - imu:
      - quat (x, y, z, w)
      - euler angle (rpy) [deg]
      - angular velocity [deg/s]
      - linear acceleration [m/s^2]
    - joint (in urdf):
      - position [deg]
      - velocity [deg/s]
      - torque [Nm]
    """

    state_dict = control_system.robot_control_loop_get_state()
    #st()
    # --------------------------------------------------

        
    # Timing
    # start_time = time.perf_counter()
    t_prev = t_curr
    t_curr = time.perf_counter()
    dt = t_curr - t_prev
    t_elapsed = t_curr - start_time
    # if dt < 10.0:  # ignore the first abnormally large dt (t_curr starts at 0)
    #     print(f"dt: {dt:.4f} seconds")
    # print("t_elapsed: ", t_elapsed)
    # print("obs_config:", control_system.obs_config)
    imu_measured_quat = state_dict.get("imu_quat", [1, 0, 0, 0])
    imu_measured_angular_velocity = state_dict.get("imu_angular_velocity", [0, 0, 0]).reshape(-1,3)
    joint_measured_position = state_dict.get("joint_position", [0] * robot_num_of_joints)
    joint_measured_velocity = state_dict.get("joint_velocity_filtered", [0] * robot_num_of_joints)
    gravity_in_body=state_dict.get("gravity_in_body",[0,0,-1]).reshape(-1,3)
    # print(rpy)
    # print("gravity_in_body: ",gravity_in_body)
    # print("omega: ", imu_measured_angular_velocity*control_system.ang_vel_scale)
    # print(imu_measured_quat)
   
        
    # constants
    default_joint_position = control_system.default_angles.copy()
    default_joint_position_arm = control_system.default_angles_arm.copy()
    gravity_vector = numpy.array([0.0, 0.0, -1.0])
    base_measured_quat = numpy.array([0.0, 0.0, 0.0, 1.0])
    #base_measured_angular_velocity = numpy.array([0.0, 0.0, 0.0 ])
    

    # IMU assignment
    # for i in range(4):
    #     base_measured_quat[i] = imu_measured_quat[i]

    # for i in range(3):
    #     base_measured_angular_velocity[i] = imu_measured_angular_velocity[i]

    num_actions = control_system.num_actions

    # build a unified position/velocity array: arm[0..7] + leg[0..11]
    if num_actions == 18:
        joint_pos_arm = state_dict.get("joint_position_arm", numpy.zeros(8))
        joint_vel_arm = state_dict.get("joint_velocity_arm", numpy.zeros(8))
        joint_combined_position = numpy.concatenate([joint_pos_arm, joint_measured_position])
        joint_combined_velocity = numpy.concatenate([joint_vel_arm, joint_measured_velocity])
    else:
        joint_combined_position = joint_measured_position
        joint_combined_velocity = joint_measured_velocity

    joint_measured_position_for_policy = numpy.zeros(num_actions)
    joint_measured_velocity_for_policy = numpy.zeros(num_actions)
    for i in range(num_actions):
        idx = policy_control_index_of_joints[i]
        joint_measured_position_for_policy[i] = joint_combined_position[idx]
        joint_measured_velocity_for_policy[i] = joint_combined_velocity[idx]
    joint_measured_velocity_for_policy = joint_measured_velocity_for_policy.reshape(1, -1)
    

    # cancel unaccurate joint position measurement
    # for i in range(len(joint_measured_position_for_policy)):
    #     if abs(joint_measured_position_for_policy[i]) > 100/180*math.pi:
    #         joint_measured_position_for_policy[i] = joint_measured_position_for_policy_prev[i]

    _LIMIT = 100 / 180 * math.pi
    mask = np.abs(joint_measured_position_for_policy) > _LIMIT
    joint_measured_position_for_policy[mask] = joint_measured_position_for_policy_prev[mask]

    if policy_action is None:
        policy_action = numpy.zeros(num_actions)


    # 18-dof default_angles for policy order: [L_arm×3, L_leg×6, R_arm×3, R_leg×6]
    if num_actions == 18:
        default_angles_for_policy = numpy.concatenate([
            control_system.default_angles_arm[[0, 1, 2]],  # L arm
            control_system.default_angles[0:6],             # L leg
            control_system.default_angles_arm[[4, 5, 6]],  # R arm
            control_system.default_angles[6:12],            # R leg
        ])
    else:
        default_angles_for_policy = control_system.default_angles

    measured_position_offset_for_policy = (
        joint_measured_position_for_policy - default_angles_for_policy
    ).reshape(1, -1)

    action = policy_action.reshape(1, -1)

    # update phase
    phase = t_elapsed % control_system.stepping_period / control_system.stepping_period
    sin_phase = numpy.sin(2 * numpy.pi * phase).reshape(1, 1)
    cos_phase = numpy.cos(2 * numpy.pi * phase).reshape(1, 1)
    

    # Process IMU angular rates
    # gravity_in_body = np.array([[0, 0, -1.0]])
    # imu_measured_quat = np.array([[1, 0, 0, 0]])
    roll_rate = imu_measured_angular_velocity[0,0]
    pitch_rate = imu_measured_angular_velocity[0,1]
    yaw_rate = imu_measured_angular_velocity[0,2]
    # Zero out roll/pitch for open-loop trajectory
    roll_rate = 0
    pitch_rate = 0
    # yaw_rate = 0
    roll_rate_raw = roll_rate
    pitch_rate_raw = pitch_rate

    # concatenate observations
    obs_buf = get_obs(commands, sin_phase, cos_phase, gravity_in_body, imu_measured_angular_velocity,
                      measured_position_offset_for_policy, joint_measured_velocity_for_policy,
                      action, control_system)

    obs_len = obs_buf.shape[-1]
    stack_size = control_system.num_stack

    # observation stacking
    if obs_buf_stack is None:
        obs_buf_stack = numpy.zeros((1, control_system.num_obs * stack_size), dtype=numpy.float32)
    obs_buf_stack = numpy.concatenate([
        obs_buf_stack[:, obs_len:],
        obs_buf,
    ], axis=1).astype(numpy.float32)

    torch_policy_action = numpy.array(policy_model.inference(numpy.array(obs_buf_stack)))
    policy_action = torch_policy_action

    # action → joint target position (policy output + default offset)
    raw_target = torch_policy_action.reshape(-1) * control_system.action_scale + default_angles_for_policy

    if num_actions == 12:
        # 12dof: policy output is leg-only, arm stays at default
        joint_target_position = raw_target.reshape(-1)
        joint_target_position_arm = control_system.default_angles_arm.copy()
    else:
        # 18dof policy order: [L_arm×3, L_leg×6, R_arm×3, R_leg×6]
        # split back into leg (12-dim) and arm (8-dim, gripper slots stay at default)
        joint_target_position = numpy.concatenate([raw_target[3:9], raw_target[12:18]])  # L_leg + R_leg
        joint_target_position_arm = control_system.default_angles_arm.copy()
        joint_target_position_arm[0:3] = raw_target[0:3]   # L arm
        joint_target_position_arm[4:7] = raw_target[9:12]  # R arm

    # send data to PlotJuggler
    plotjuggler_data = build_plotjuggler_payload(state_dict, action=joint_target_position)
    send_plotjuggler(control_system, plotjuggler_data)
    
    # reset to default dof pos when falling
    if abs(rpy[0]) > 0.3 or abs(rpy[1]) > 0.3:
        joint_target_position = default_joint_position
        joint_target_position_arm = default_joint_position_arm
        exit_flag = True

    control_dict = {
        "pd_control_kp": 0,
        "pd_control_kd": 0,
        "target_position": joint_target_position,
        "target_position_arm": joint_target_position_arm,
    }

    
    # output control
    control_system.robot_control_loop_set_control(control_dict=control_dict)

    # update prev parameters
    joint_measured_position_for_policy_prev = joint_measured_position_for_policy
    roll_rate_prev = roll_rate
    pitch_rate_prev = pitch_rate
    yaw_rate_prev = yaw_rate

if __name__ == "__main__":
    main()