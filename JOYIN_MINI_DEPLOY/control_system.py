import time
from pdb import set_trace as st
from typing import Dict, Any
import yaml
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Imu, Joy
from std_msgs.msg import Int32MultiArray, UInt16MultiArray, Int16MultiArray, Float32MultiArray, UInt8, UInt8MultiArray
from hardware.msg import Bat
from control.msg import PidControl
from hardware.msg import KinematicState
import logging
import argparse
import time
import math
import sys
import numpy as np
from numba import njit
from collections import deque
import os
from udp_sender import UDPPlotJugglerSender

# accelerate 



# def quat_rotate_vector(q, v):
#     """
#     Rotate vector v by quaternion q without scipy dependency
#     q: [x, y, z, w]
#     v: [x, y, z]
#     Returns the rotated vector
#     """
#     x, y, z, w = q
#     # Quaternion conjugate
#     q_conj = np.array([-x, -y, -z, w])
#     # Convert vector to quaternion
#     v_quat = np.array([v[0], v[1], v[2], 0.0])
#     # Quaternion multiplication
#     def quat_mult(a, b):
#         ax, ay, az, aw = a
#         bx, by, bz, bw = b
#         return np.array([
#             aw*bx + ax*bw + ay*bz - az*by,
#             aw*by - ax*bz + ay*bw + az*bx,
#             aw*bz + ax*by - ay*bx + az*bw,
#             aw*bw - ax*bx - ay*by - az*bz
#         ])
#     # Rotation
#     qv = quat_mult(q, v_quat)
#     qvq = quat_mult(qv, q_conj)
#     return qvq[:3]

@njit(cache=True, fastmath=True)
def quat_rotate_vector(q, v):
    """
    Rotate vector v by quaternion q.
    q: float64[4] — [x, y, z, w]
    v: float64[3]
    Returns: float64[3]
    cache=True 
    fastmath=True 
    """
    px, py, pz, pw = q[0], q[1], q[2], q[3]
    vx, vy, vz = v[0], v[1], v[2]

    # Step 1: q ⊗ [vx, vy, vz, 0]  (all scalars, no heap allocation)
    qv0 = pw*vx + py*vz - pz*vy
    qv1 = pw*vy - px*vz + pz*vx
    qv2 = pw*vz + px*vy - py*vx
    qv3 = -px*vx - py*vy - pz*vz

    # Step 2: qv ⊗ conj(q), conj([px,py,pz,pw]) = [-px,-py,-pz,pw]
    r0 = -qv3*px + qv0*pw - qv1*pz + qv2*py
    r1 = -qv3*py + qv0*pz + qv1*pw - qv2*px
    r2 = -qv3*pz - qv0*py + qv1*px + qv2*pw

    out = np.empty(3, dtype=np.float64)
    out[0] = r0;  out[1] = r1;  out[2] = r2
    return out

# Trigger JIT compilation at module load to avoid latency on first callback
# First run (no cache): ~1-3s; subsequent loads from __pycache__: ~50ms
_warmup_q = np.array([0., 0., 0., 1.], dtype=np.float64)
_warmup_v = np.array([0., 0., -1.], dtype=np.float64)
quat_rotate_vector(_warmup_q, _warmup_v)

class ControlSystem(Node):
    """
    A robot control system class that manages control loops and state updates.
    
    Attributes:
        state (dict): Current state of the robot (position, velocity, etc.)
        control_params (dict): Current control parameters (e.g., PID gains, target positions)
        running (bool): Flag indicating if the control loop is active
    """
    
    def __init__(self,cfg):
        Node.__init__(self, 'RL_control_node')
        """Initialize the control system with default state and control parameters."""
        self.declare_parameter("config_file", "test")
        # st()
        config_file = self.get_parameter("config_file").value
        print(config_file)
        # st()
        self.shoulder_yaw_offset = math.radians(0.0) # shoulder yaw offset
        self.elbow_yaw_offset = math.radians(0)
        self.hip_yaw_offset = [math.radians(0), math.radians(0)]
        self.left_hip_yaw_compensation = math.radians(0.0)
        self.right_hip_yaw_compensation = math.radians(0.0)
        self.hip_roll_turning_offset = math.radians(0.0)
        self.hip_roll_init_offset = math.radians(0.0)
        self.hip_roll_offset = [math.radians(0.0) + self.hip_roll_init_offset, math.radians(0) + self.hip_roll_init_offset]       # hip roll offset
        self.left_hip_roll_compensation = math.radians(0.0)
        self.right_hip_roll_compensation = math.radians(0.0) # -3
        self.hip_pitch_offset_degree = 0.0
        self.hip_pitch_offset = math.radians(self.hip_pitch_offset_degree)  # hip pitch offset
        self.hip_pitch_offset_back_walk = math.radians(0.0)
        self.hip_pitch_offset_gap_compensation = math.radians(0.0)
        self.hip_pitch_offset_degree_max = 20
        self.hip_pitch_offset_degree_min = -20
        # self.hip_pitch_offset = math.radians(0)  # hip pitch offset
        self.left_hip_pitch_compensation = math.radians(0.0)
        self.right_hip_pitch_compensation = math.radians(0.0)
        self.left_knee_pitch_compensation = math.radians(0.0)
        self.right_knee_pitch_compensation = math.radians(0.0)
        self.ankle_pitch_offset = [math.radians(0.0), math.radians(0.0)] # ankle pitch compensation
        self.left_ankle_pitch_compensation = math.radians(0.0)
        self.right_ankle_pitch_compensation = math.radians(0.0) # 1
        self.ankle_pitch_offset_swing = math.radians(0.0) # ankle roll offset before landing
        self.ankle_roll_offset = math.radians(0.0)  # ankle roll offset
        self.ankle_roll_offset_before_landing = math.radians(0.0) # ankle roll offset before landing
        self.left_ankle_roll_compensation = math.radians(0.0) - self.left_hip_roll_compensation - self.hip_roll_offset[0]
        self.right_ankle_roll_compensation = math.radians(0.0) - self.right_hip_roll_compensation - self.hip_roll_offset[1]

        self.imu_euler_angle = [9999.0, 9999.0, 9999.0]
        self.imu_euler_angle_last = [0.0, 0.0, 0.0]
        self.imu_init_zero_shift = [0.0, 0.0, 0.0]
        self.imu_error = [0.0, 0.0, 0.0]
        self.imu_error_static_check_threshold = [1.0, 1.0, 1.0]
        self.imu_error_static_check_threshold = [10, 10, 10]
        self.imu_static_pose_cnt = 0
        self.imu_static_pose_confirm_threshold = 100
        self.is_imu_init_ok = False
        self.is_imu_data_update = False
        self.gravity_vector = np.array([0.0, 0.0, -1.0])  # gravity vector, assuming IMU Z-axis points down
        self._q_rot_buf = np.zeros(4, dtype=np.float64)  # pre-allocated buffer for quat_rotate_vector
        
        
        with open(f"{cfg}", "r") as f:
            config = yaml.load(f, Loader=yaml.FullLoader)
            # self.policy_path = config["policy_path"]            
            self.kp = int(config["kp"])
            self.kd = int(config["kd"])

            self.default_angles = np.array(config["default_angles"], dtype=np.float32)
            self.default_angles_arm = np.array(config["default_angles_arm"], dtype=np.float32)
            self.offset_angles=np.array(config["offset_angles"],dtype=np.float32)
            self.offset_angles_arm=np.array(config["offset_angles_arm"],dtype=np.float32)
            self.stepping_period = config["stepping_period"]
            self.ang_vel_scale = config["ang_vel_scale"]
            self.dof_pos_scale = config["dof_pos_scale"]
            self.dof_vel_scale = config["dof_vel_scale"]
            self.action_scale = config["action_scale"]
            self.cmd_scale = np.array(config["cmd_scale"], dtype=np.float32)

            self.num_actions = config["num_actions"]
            self.num_obs = config["num_obs"]
            self.num_stack=config["num_stack"]
            self.gravity_scale = config["gravity_scale"]
            self.obs_config = config.get("obs_config")  # "config1" or "config2"

        self.read_factory_calibration()

        self.state = {
            'imu_quat': np.array([1.0, 0.0, 0.0, 0], dtype=np.float32),  # x, y, z coordinates
            'imu_angular_velocity':np.array([0.0, 0.0, 0.0], dtype=np.float32),
            'joint_position': np.zeros((12), dtype=np.float32),
            'joint_velocity': np.zeros((12), dtype=np.float32),
            'joint_velocity_filtered': np.zeros((12), dtype=np.float32),
            'joint_position_arm': np.zeros((8), dtype=np.float32),
            'joint_velocity_arm': np.zeros((8), dtype=np.float32),
            'gravity_in_body': self.gravity_vector
        }

        # joint velocity filter buffer (moving median filter)
        self.joint_velocity_buffer_size = 5
        self.joint_velocity_buffer = np.zeros((self.joint_velocity_buffer_size, 12), dtype=np.float32)
        self.joint_velocity_buffer_index = 0
        self.joint_velocity_buffer_count = 0  # num of valid data

        # bluetooth joystick init
        self.joy_left_stick_lr  = 0.0
        self.joy_left_stick_fb  = 0.0
        self.joy_left_trigger   = 0.0
        self.joy_right_stick_lr = 0.0
        self.joy_right_stick_fb = 0.0
        self.joy_right_trigger  = 0.0
        self.joy_dpad_lr        = 0.0
        self.joy_dpad_ud        = 0.0
        self.joy_A = self.joy_B = self.joy_X = self.joy_Y = False
        self.joy_LB = self.joy_RB = self.joy_back = self.joy_start = self.joy_play = False
        self._joy_last_time = 0.0  # for joystick callback speed clipping

        # callback groups：high-freq sensors（IMU/Kinematic）sent individually，Joy sent
        self._sensor_cbg = ReentrantCallbackGroup()
        self._joy_cbg    = MutuallyExclusiveCallbackGroup()

        # motor messages (22 in total: 8 in arms, 12 in legs, 2 in head)
        self.motor_load = np.zeros((22), dtype=np.int16)  # motor load 
        self.motor_current = np.zeros((22), dtype=np.int16)  # motor current 

        udp_host="192.168.51.48"
        udp_port=9870
        self.enable_udp = True
        self.udp_sender = UDPPlotJugglerSender(host=udp_host, port=udp_port, enable_udp=self.enable_udp)
        # self.dir_trans=[1,1,-1,1,1,-1,1,1,1,-1,-1,-1]
        # self.final_trans=[1,1,-1,1,1,1,1,1,1,-1,-1,1]
        self.dir_trans_arm = [-1,1,-1,1,1,1,-1,1]
        self.dir_trans = [1,1,-1,1,1,-1,  1,1,1,-1,-1,-1] #new machine
       
        # start UDP sender
        if self.enable_udp and self.udp_sender:
            self.udp_sender.start()
            '''
        self.control_params = {
            'kp': 1.0,  # Proportional gain
            'kd': 0.01, # Derivative gain
            'target_position': [0.0, 0.0, 0.0],
        }
        '''
        #  create subscribers 
        # Best effort, keep the lastest frame, avoid jamming when the executors are busy 
        
        qos_profile_reliable = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE 
        )

        qos_profile_best = QoSProfile(
            depth=1, 
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # --------------Subscribe kinematic feedback topic----------------
                
        self.kinematic_sub = self.create_subscription(
            KinematicState,
            'HardwareKinematic_topic',
            self.kinematic_state_feedback_msg_callback,
            qos_profile_best,
            callback_group=self._sensor_cbg
        )

        # --------------Subscribe bluetooth joystick topic----------------
                
        self.joy_bt_sub = self.create_subscription(
            Joy,
            'joy_filter_bt',
            self.joy_bt_callback,
            qos_profile_reliable,
            callback_group=self._joy_cbg
        )

        # --------------Subscribe IMU data topic----------------

        self.imu_sub = self.create_subscription(
            Imu,
            '/HardwareImuBipedMotion_topic',
            self.imu_feedback_callback,
            qos_profile_best,
            callback_group=self._sensor_cbg
        )

        # --------------Subscribe motor load topic----------------
        
        # self.motor_load_sub = self.create_subscription(
        #     Int16MultiArray,
        #     '/HardwareMotorLoad_topic',
        #     self.motor_load_callback,
        #     qos_profile
        # )

        # --------------Subscribe motor current topic----------------
        
        # self.motor_current_sub = self.create_subscription(
        #     Int16MultiArray,
        #     '/HardwareMotorCurrent_topic',
        #     self.motor_current_callback,
        #     qos_profile
        # )


        # -------------- Create publishers ----------------
                
        self.pid_set_phase_counter = 0
        self.pid_set_phase_max = 4
        
        self.joint_angles_pub = self.create_publisher(
            Float32MultiArray,
            'ControlMotorAngles_topic',
            qos_profile_reliable
        )
        
        self.joint_velocitys_pub = self.create_publisher(
            Int16MultiArray,
            'ControlMotorSpeed_topic',
            qos_profile_reliable
        )
        '''
        self.joint_action_times_pub = self.create_publisher(
            Int16MultiArray,
            'ControlMotorTime_topic',
            qos_profile
        )
        '''
        self.joint_torque_state_pub = self.create_publisher(
            UInt8,
            'ControlEnableMotor_topic',
            qos_profile_reliable
        )
        
        self.joint_pid_control_pub = self.create_publisher(
            PidControl,
            'ControlPidControl_topic',
            qos_profile_reliable
        )

        self.joint_pid_param_get_trigger_pub = self.create_publisher(
            UInt8,
            'ControlGetPid_topic',
            qos_profile_reliable
        )

        # robot control relative parameters
        self.robot_velocity_trans_factor = 0.087890625  # transfrom from encoder_ticks/sec to deg/sec 

        print("zero pose starts...")
        self.zero_pose()
        self.ros_robot_init_proc()#TODO:
        #self.unload_effort()

        self.get_logger().info("Motion Control Node ready! Waiting for commands...")


    def read_factory_calibration(self, json_path="/private/hw/control/miniBotMotionCoreParam.json"):
        import json as _json
        deg_to_rad = math.pi / 180.0
        try:
            with open(json_path, "r") as f:
                core_param = _json.load(f)
        except Exception as e:
            self.get_logger().warn(f"Cannot read calibration file {json_path}: {e}, using offset values from YAML")
            return

        jc = core_param.get("joint_compensation", {})
        sc = core_param.get("shoulder_compensation", {})

        # ===== Arms =====
        syw = sc.get("walking_shoulder_yaw_offset", [])
        spo = sc.get("shoulder_pitch_offset", [])
        l_sh_pitch = syw[0]+3 if len(syw) > 0 else 0.0
        r_sh_pitch = syw[1]+3 if len(syw) > 1 else 0.0
        l_sh_roll  = spo[0] if len(spo) > 0 else 0.0
        r_sh_roll  = spo[1] if len(spo) > 1 else 0.0
        l_elbow    = jc.get("shoulder_yaw_offset", 0.0)
        r_elbow    = jc.get("elbow_yaw_offset", 0.0)

        # [0]l_sh_pitch [1]l_sh_roll [2]l_elbow [3]l_gripper(0)
        # [4]r_sh_pitch [5]r_sh_roll [6]r_elbow [7]r_gripper(0)
        self.offset_angles_arm = np.array([
            l_sh_pitch * deg_to_rad * (-1.0),
            l_sh_roll  * deg_to_rad *   1.0,
            l_elbow    * deg_to_rad *   1.0,
            0.0,
            r_sh_pitch * deg_to_rad * (-1.0),
            r_sh_roll  * deg_to_rad * (-1.0),
            r_elbow    * deg_to_rad * (-1.0),
            0.0,
        ], dtype=np.float32)

        # ===== Legs =====
        hip_yaw_arr   = jc.get("hip_yaw_offset", [])
        hip_roll_arr  = jc.get("hip_roll_offset", [])
        ank_pitch_arr = jc.get("ankle_pitch_offset", [])
        hip_roll_init = jc.get("hip_roll_init_offset", 0.0)
        hip_pitch_deg = jc.get("hip_pitch_offset_degree", 0.0)
        ank_roll_base = jc.get("ankle_roll_offset", 0.0)

        l_hip_yaw_comp   = jc.get("left_hip_yaw_compensation",    0.0)
        r_hip_yaw_comp   = jc.get("right_hip_yaw_compensation",   0.0)
        l_hip_roll_comp  = jc.get("left_hip_roll_compensation",   0.0)
        r_hip_roll_comp  = jc.get("right_hip_roll_compensation",  0.0)
        l_hip_pitch_comp = jc.get("left_hip_pitch_compensation",  0.0)
        r_hip_pitch_comp = jc.get("right_hip_pitch_compensation", 0.0)
        l_knee_comp      = jc.get("left_knee_pitch_compensation",  0.0)
        r_knee_comp      = jc.get("right_knee_pitch_compensation", 0.0)
        l_ank_pitch_comp = jc.get("left_ankle_pitch_compensation",  0.0)
        r_ank_pitch_comp = jc.get("right_ankle_pitch_compensation", 0.0)
        l_ank_roll_comp  = jc.get("left_ankle_roll_compensation",   0.0)
        r_ank_roll_comp  = jc.get("right_ankle_roll_compensation",  0.0)

        l_hip_yaw_base   = hip_yaw_arr[0]   if len(hip_yaw_arr)   > 0 else 0.0
        r_hip_yaw_base   = hip_yaw_arr[1]   if len(hip_yaw_arr)   > 1 else 0.0
        l_hip_roll_base  = hip_roll_arr[0]  if len(hip_roll_arr)  > 0 else 0.0
        r_hip_roll_base  = hip_roll_arr[1]  if len(hip_roll_arr)  > 1 else 0.0
        l_ank_pitch_base = ank_pitch_arr[0] if len(ank_pitch_arr) > 0 else 0.0
        r_ank_pitch_base = ank_pitch_arr[1] if len(ank_pitch_arr) > 1 else 0.0

        # [0]l_hip_yaw [1]l_hip_roll [2]l_hip_pitch [3]l_knee [4]l_ank_pitch [5]l_ank_roll
        # [6]r_hip_yaw [7]r_hip_roll [8]r_hip_pitch [9]r_knee [10]r_ank_pitch [11]r_ank_roll
        self.offset_angles = np.array([
            (l_hip_yaw_base  + l_hip_yaw_comp)                     * deg_to_rad *  1.0,
            (l_hip_roll_base + hip_roll_init + l_hip_roll_comp)     * deg_to_rad *  1.0,
            (hip_pitch_deg   + l_hip_pitch_comp)                    * deg_to_rad * (-1.0),
            l_knee_comp                                              * deg_to_rad * (-1.0),
            (l_ank_pitch_base + l_ank_pitch_comp)                   * deg_to_rad * (-1.0),
            (ank_roll_base   + l_ank_roll_comp  - l_hip_roll_comp)  * deg_to_rad *  1.0,
            (r_hip_yaw_base  + r_hip_yaw_comp)                      * deg_to_rad * (-1.0),
            (r_hip_roll_base + hip_roll_init    + r_hip_roll_comp)  * deg_to_rad * (-1.0),
            (hip_pitch_deg   + r_hip_pitch_comp)                    * deg_to_rad * (-1.0),
            r_knee_comp                                              * deg_to_rad * (-1.0),
            (r_ank_pitch_base + r_ank_pitch_comp)                   * deg_to_rad * (-1.0),
            (ank_roll_base   + r_ank_roll_comp  - r_hip_roll_comp)  * deg_to_rad * (-1.0),
        ], dtype=np.float32)

        self.get_logger().info(f"Loaded offset_angles from calibration file: {np.round(self.offset_angles, 4).tolist()}")
        self.get_logger().info(f"Loaded offset_angles_arm from calibration file: {np.round(self.offset_angles_arm, 4).tolist()}")

    def zero_pose(self):
        for i in range(4):
            default_angles=np.array(self.default_angles)
            default_angles_arm = np.array(self.default_angles_arm)
            self.publish_joint_angles_command(default_angles, default_angles_arm)
            # time.sleep(0.3)
        # st()

    def ros_imu_init_proc(self):
        if self.is_imu_data_update:
            # self.get_logger().info(f"imu init start")
            for i in range(3):
                self.imu_error[i] = self.imu_euler_angle[i] - self.imu_euler_angle_last[i]
                self.imu_euler_angle_last[i] = self.imu_euler_angle[i]

            self.get_logger().info(f"imu init error: {self.imu_error[0]}, {self.imu_error[1]}, {self.imu_error[2]}")

            if (self.imu_error[0] < self.imu_error_static_check_threshold[0]) and (self.imu_error[1] < self.imu_error_static_check_threshold[1]) and (self.imu_error[2] < self.imu_error_static_check_threshold[2]):
                self.imu_static_pose_cnt += 1
            else:
                self.imu_static_pose_cnt = 0
            print(self.imu_static_pose_cnt)
            if self.imu_static_pose_cnt > self.imu_static_pose_confirm_threshold:
                self.imu_init_zero_shift[0] = self.imu_euler_angle[0]
                self.imu_init_zero_shift[1] = self.imu_euler_angle[1]
                self.imu_init_zero_shift[2] = self.imu_euler_angle[2]

                self.imu_static_pose_cnt = 0

                self.is_imu_init_ok = True

                self.get_logger().info(f"imu init finish, {self.imu_init_zero_shift[0]}, {self.imu_init_zero_shift[1]}, {self.imu_init_zero_shift[2]}")
                time.sleep(3)
                #st()

            self.is_imu_data_update = False
    def ros_robot_servo_pid_init_proc(self):
        if self.pid_set_phase_counter <= self.pid_set_phase_max:
            if self.pid_set_phase_counter < self.pid_set_phase_max/2:
                self.publish_joint_pid_ctrl_command(self.kp, 0, self.kd) 
                
            self.publish_joint_pid_param_get_trigger_command()
            self.pid_set_phase_counter += 1
    def publish_joint_torque_ctrl_command(self, torque_on_off_state):
        """Publish joint torque control command"""
        try:
            msg = UInt8()
            msg.data = torque_on_off_state
            self.joint_torque_state_pub.publish(msg)
        except KeyError as e:
            self.get_logger().error(f"Missing joint torque state: {str(e)}")
    def unload_effort(self):# 0 off,1 on
        self.publish_joint_torque_ctrl_command(0)

    def ros_robot_init_proc(self):
        self.get_logger().info("Initializing robot...")
        self.publish_joint_velocity_value_command(4090) # default servo speed, caution: modifying this would cause decay of policy
        self.ros_robot_init_pose_joint_angle_set_proc()
        self.ros_robot_servo_pid_init_proc()
        time.sleep(5.0)  # wait for 5 s

    def ros_robot_init_pose_joint_angle_set_proc(self):
        target_dict={"target_position": self.default_angles}
        # default_angles=np.array(self.default_angles)*self.dir_trans
        default_angles=np.array(self.default_angles)
        default_angles_arm = np.array(self.default_angles_arm)
        
        self.publish_joint_angles_command(default_angles,default_angles_arm)
        
    
    def kinematic_state_feedback_msg_callback(self, msg):

        for i in range(12):
            self.state['joint_position'][i]= math.radians(msg.joint_angles.data[i+8])*self.dir_trans[i]-self.offset_angles[i]
            self.state['joint_velocity'][i] = math.radians(msg.joint_velocities.data[i+8] * self.robot_velocity_trans_factor)*self.dir_trans[i]
        for i in range(8):
            self.state['joint_position_arm'][i] = math.radians(msg.joint_angles.data[i]) * self.dir_trans_arm[i] - self.offset_angles_arm[i]
            self.state['joint_velocity_arm'][i] = math.radians(msg.joint_velocities.data[i] * self.robot_velocity_trans_factor) * self.dir_trans_arm[i]
        # data     = np.array(msg.joint_angles.data[8:20],     dtype=np.float32)
        # data_vel = np.array(msg.joint_velocities.data[8:20], dtype=np.float32)  # fix: .data
        # self.state['joint_position'][:] = np.radians(data) * self.dir_trans - self.offset_angles
        # self.state['joint_velocity'][:] = np.radians(data_vel * self.robot_velocity_trans_factor) * self.dir_trans

        # data_arm     = np.array(msg.joint_angles.data[0:8],     dtype=np.float32)
        # data_vel_arm = np.array(msg.joint_velocities.data[0:8], dtype=np.float32)  # fix: .data, dype→dtype
        # self.state['joint_position_arm'][:] = np.radians(data_arm) * self.dir_trans_arm - self.offset_angles_arm
        # self.state['joint_velocity_arm'][:] = np.radians(data_vel_arm * self.robot_velocity_trans_factor) * self.dir_trans_arm  # fix: dir_trans→dir_trans_arm
        
        # Applying move median filter to the joint velocity callback values
        self.joint_velocity_buffer[self.joint_velocity_buffer_index] = self.state['joint_velocity'].copy()
        self.joint_velocity_buffer_index = (self.joint_velocity_buffer_index + 1) % self.joint_velocity_buffer_size
        self.joint_velocity_buffer_count = min(self.joint_velocity_buffer_count + 1, self.joint_velocity_buffer_size)
        
        # ====== Kinematic callback freq analysis ======
        
        if not hasattr(self, '_kinematic_callback_counter'):
            self._kinematic_callback_counter = 0
            self._kinematic_callback_last_print_time = time.time()
        self._kinematic_callback_counter += 1
        if time.time() - self._kinematic_callback_last_print_time > 1.0:
            print(f"[Kinematic callback freq] {self._kinematic_callback_counter}Hz (joint_pos[0]={self.state['joint_position'][0]:.4f})")
            self._kinematic_callback_counter = 0
            self._kinematic_callback_last_print_time = time.time()
        # ==========================================

    def joy_bt_callback(self, msg):
        # set to 20 Hz
        now = time.time()
        if now - self._joy_last_time < 0.05:
            return
        self._joy_last_time = now

        self.joy_left_stick_lr  = msg.axes[0]
        self.joy_left_stick_fb  = msg.axes[1]
        self.joy_left_trigger   = msg.axes[2]
        self.joy_right_stick_lr = msg.axes[3]
        self.joy_right_stick_fb = msg.axes[4]
        self.joy_right_trigger  = msg.axes[5]
        self.joy_dpad_lr        = msg.axes[6]
        self.joy_dpad_ud        = msg.axes[7]

        self.joy_B     = bool(msg.buttons[0])
        self.joy_A     = bool(msg.buttons[1])
        self.joy_Y     = bool(msg.buttons[2])
        self.joy_X     = bool(msg.buttons[3])
        self.joy_LB    = bool(msg.buttons[4])
        self.joy_RB    = bool(msg.buttons[5])
        self.joy_back  = bool(msg.buttons[6])
        self.joy_start = bool(msg.buttons[7])
        self.joy_play  = bool(msg.buttons[8])
    
    def imu_feedback_callback(self, msg):
        
        # quat data
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w
        self.state['imu_quat'] = np.array([w, x, y, z])
        
        # compute gravity projection
        self._q_rot_buf[0] = -x
        self._q_rot_buf[1] = -y
        self._q_rot_buf[2] = -z
        self._q_rot_buf[3] =  w
        gravity_in_body = quat_rotate_vector(self._q_rot_buf, self.gravity_vector)
        self.state['gravity_in_body'] = gravity_in_body.reshape(-1, 3)
        
        # obtaining base augular velocity
        self.state['imu_angular_velocity'][0] = -msg.angular_velocity.x
        self.state['imu_angular_velocity'][1] = -msg.angular_velocity.y
        self.state['imu_angular_velocity'][2] = -msg.angular_velocity.z
        # ====== IMU callback frequency analysis ======
        if not hasattr(self, '_imu_callback_counter'):
            self._imu_callback_counter = 0
            self._imu_callback_last_print_time = time.time()
        self._imu_callback_counter += 1
        if time.time() - self._imu_callback_last_print_time > 1.0:
            print(f"[IMU callback freq] {self._imu_callback_counter}Hz (gravity={gravity_in_body})")
            self._imu_callback_counter = 0
            self._imu_callback_last_print_time = time.time()
        # ==========================================

    def motor_load_callback(self, msg):
        """Motor load data callback

        Data mapping:
        data[0-7]:   left/right arm joints (8)
        data[8-19]:  left/right leg joints (12)
        data[20-21]: head joints (2)
        """
        try:
            # Validate data dimensions
            if len(msg.data) != 22:
                self.get_logger().error(f"motor_load dimension error: expected 22, got {len(msg.data)}")
                return
            # Extract only the 12 leg joint values (same layout as kinematic_state_feedback_msg_callback)
            for i in range(12):
                self.motor_load[i] = msg.data[i+8]
        except Exception as e:
            self.get_logger().error(f"Error in motor_load_callback: {str(e)}")

    def motor_current_callback(self, msg):
        """Motor current data callback

        Data mapping:
        data[0-7]:   left/right arm joints (8)
        data[8-19]:  left/right leg joints (12)
        data[20-21]: head joints (2)
        """
        try:
            # Validate data dimensions
            if len(msg.data) != 22:
                self.get_logger().error(f"motor_current dimension error: expected 22, got {len(msg.data)}")
                return
            # Extract only the 12 leg joint values (same layout as kinematic_state_feedback_msg_callback)
            for i in range(12):
                self.motor_current[i] = msg.data[i+8]
        except Exception as e:
            self.get_logger().error(f"Error in motor_current_callback: {str(e)}")

    def robot_control_loop_get_state(self) -> Dict[Any, Any]:
        """
        Get the current state of the robot.
        """
        valid_buffer = self.joint_velocity_buffer[:self.joint_velocity_buffer_count]
        if valid_buffer.shape[0] > 0:
            self.state['joint_velocity_filtered'] = np.median(valid_buffer, axis=0)
        return self.state
    
    def robot_control_loop_set_control(self, control_dict: Dict[Any, Any]) -> bool:
        """
        Update control parameters for the robot.
        
        Args:
            control_dict: Dictionary containing new control parameters
            
        Returns:
            bool: True if parameters were successfully updated
        """
        
        target_position = control_dict.get('target_position')
        target_position_arm = control_dict.get('target_position_arm')
        self.publish_joint_angles_command(target_position, target_position_arm)

        
                
        return True
    def publish_joint_angles_command(self,target_position, target_position_arm):

        try:
            msg = Float32MultiArray()
            msg.data = [0.0] * 20
            # 0-3: left shoulder pitch/roll, elbow roll, gripper(unused, set to 0)
            # 4-7: right shoulder pitch/roll, elbow roll, gripper(unused, set to 0)
            # 8-13: left hip yaw/roll/pitch, knee pitch, ankle pitch, ankle roll
            # 14-19: right hip yaw/roll/pitch, knee pitch, ankle pitch, ankle roll 
            for i in range(8):
                msg.data[i] = math.degrees(target_position_arm[i] + self.offset_angles_arm[i]) * self.dir_trans_arm[i]
            
            for i in range(12):
                msg.data[i+8] = math.degrees(target_position[i]+self.offset_angles[i]) * self.dir_trans[i]
            
            self.joint_angles_pub.publish(msg)

        except KeyError as e:
            self.get_logger().error(f"Missing joint angle: {str(e)}")

    def publish_joint_velocity_value_command(self, velocity_value):
        
        try:
            msg = Int16MultiArray()
            msg.data = [0] * 20
        
            for i in range (20):
                msg.data[i] = velocity_value
            self.joint_velocitys_pub.publish(msg)

        except KeyError as e:
            self.get_logger().error(f"Missing joint velocity: {str(e)}")

    def publish_joint_pid_ctrl_command(self, kp, ki, kd):
        try:
            msg = PidControl()

            for i in range(20):
                msg.index = i
                msg.kp = kp
                msg.ki = ki
                msg.kd = kd
                self.joint_pid_control_pub.publish(msg)
                time.sleep(0.002)

        except Exception as e:
            self.get_logger().error(f"Failed to publish PID control command: {str(e)}")

    def publish_joint_pid_param_get_trigger_command(self):
        """publish triggers for getting PID params """
        try:
            msg = UInt8()
            msg.data = 1  # triggering getting PID
            self.joint_pid_param_get_trigger_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish PID parameter get trigger command: {str(e)}")

'''
try spinning control system (caution: robot will return to default pos at max speed)

'''

# if __name__ == "__main__":
#     try:
#         rclpy.init()
#         control_system = ControlSystem('config_mini.yaml')
#         rclpy.spin(control_system)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         if rclpy.ok():
#             control_system.destroy_node()
#             rclpy.shutdown()
