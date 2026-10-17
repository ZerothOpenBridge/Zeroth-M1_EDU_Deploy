# JRobot Sensor ROS 2 Interface Reference

<div align="right">
  <a href="readme.md"><strong>English</strong></a> |
  <a href="readme_cn.md">中文</a>
</div>

Sensor-centric ROS 2 API for secondary development on the Joyin robot platform.

## Version ChangeList


| Version    | Date       | Changes                                                                                                                                                                                                                                                       |
| ---------- | ---------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **v0.0.3** | 2026-10-12 | Removed invalid section, update control related apis and rename doc name as ros2_api.md.                                                                                                                                                                        |
| **v0.0.2** | 2026-09-30 | Add media ROS 2 APIs: iTOF, RGB camera (`/camera/image_jpeg`, `/camera/image_h264`, `/camera/jpeg_high_resolution`), mic/speaker (`/audio/capture_pcm`, `/audio/speaker_pcm`), LCD (`/emoji/index`, `/lcd/video/nv12`).                                       |
| **v0.0.1** | 2026-07-17 | Initial sensor-centric ROS 2 interface reference for overseas secondary development. Covers IMU, LiDAR, iTOF, joints, battery, vehicle wheels, head pose, downview, and keys. Documents lifecycle parameters, QoS notes, TF caveats, and integration recipes. |


---



## Device Access

Login the robot over USB ADB, then work from the on-device developer workspace. See [jrobot_adb_setup.md](jrobot_adb_setup.md) to set up the ADB environment ([中文版](jrobot_adb_setup_cn.md)).

### 1. Connect via ADB shell

On the host PC (USB cable connected to the Type-C interface of robot):

```bash
# Verify the device is visible
adb devices

# Open an interactive shell on the device
adb shell
```



### 2. Enter the developer workspace

Inside the ADB shell:

```bash
cd /userdata/developer/
```

Use this directory for deploying binaries, configs, and secondary-development demos on the device.

---



### 3. Setup WiFi network

```bash
wifi_connect <SSID> <PASSWORD>
ifconfig wlan0
```



### 4. Build RL environment and demo reference

See: [https://github.com/Zeroth-OpenBridge](https://github.com/Zeroth-OpenBridge)

Setup your own training environment for M1 edu here: [JOYIN_MINI_GYM](JOYIN_MINI_GYM/README.md)

Once your policy verified in the simulation environment, deploy here: [JOYIN_MINI_DEPLOY](JOYIN_MINI_DEPLOY/README.md) 

**Important:** When establishing your custom codes (i.e., giving joint angle commands or vehicle speed commands), please ensure that the default embedded `control_pm` node (in `run.sh`) is shut down:

```bash
cd /opt/joyin
# Open run.sh and comment out the following line:
# ${BIN_DIR}/plugman ${CFG_DIR}/control_pm.json > /dev/null 2>&1 &
```




## Quick Start (5 min)

```bash
# 1. Confirm the /hardware node is running
ros2 node list | grep hardware
ros2 node info /hardware

# 2. List live sensor / media topics
ros2 topic list | grep -E 'Hardware|camera|audio|emoji|cmd_vel'

# 3. Smoke-test perception + media
ros2 topic hz /HardwareImuBipedMotion_topic
ros2 topic hz /HardwareLidar_topic
ros2 topic echo /HardwareItof/depth_image --once
ros2 topic hz /camera/image_jpeg
ros2 topic hz /audio/capture_pcm

# 4. Check sensor health (ROS params)
ros2 param get /hardware hardware.Lidar.status
ros2 param get /hardware hardware.ToF.status
ros2 param get /hardware hardware.MCU.status
```

**Minimal perception / media subscriber set**

```
/HardwareLidar_topic            sensor_msgs/LaserScan
/HardwareItof/depth_image       sensor_msgs/Image
/HardwareItof/point_cloud       sensor_msgs/PointCloud2
/HardwareImuBipedMotion_topic   sensor_msgs/Imu
/camera/image_jpeg              sensor_msgs/CompressedImage
/camera/image_h264              sensor_msgs/Image
/audio/capture_pcm              sensor_msgs/Image   # PCM payload
```

---



## Before You Integrate — Read This


| Topic                | Detail                                                                                                                                                                                                                                                                         |
| -------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **iTOF start/stop**  | `/itof_enable` and `/itof_disable` are placeholders: calls always succeed and **do not** stop the stream. After boot, subscribe directly to `/HardwareItof/depth_image` and `/HardwareItof/point_cloud` with **best_effort** QoS (must match the publisher).                   |
| **LiDAR start/stop** | Use `/lidar_enable` / `/lidar_disable` (`std_srvs/Trigger`) to control the scan thread and power. `hardware.Lidar.status=3` (WORKING) does **not** guarantee live scans — if `/HardwareLidar_topic` is silent, call `/lidar_enable` then `ros2 topic hz /HardwareLidar_topic`. |
| **TF frames**        | Hardware publishes `frame_id` only for IMU/LiDAR: `imu_link` and `laser_frame` have **no TF**. Provide `static_transform_publisher` or URDF (`base_link` → `imu_link`, `base_link` → `laser_frame`). iTOF broadcasts `itof_base_frame` → `itof_optical_frame`.                 |
| **Health params**    | Use live status: `hardware.MCU.status`, `hardware.Lidar.status`, `hardware.ToF.status` (`3` = WORKING). Do **not** rely on `hardware.IMU.status` or `hardware.Motor.status` (placeholders).                                                                                    |
| **IMU topic**        | Prefer `/HardwareImuBipedMotion_topic` (~200 Hz) for secondary development. `/HardwareImuStd_topic` is the same payload downsampled (~100 Hz), kept for built-in apps / compatibility.                                                                                         |
| **QoS**              | `/HardwareKinematic_topic`: `best_effort`, depth 1. `/HardwareLidar_topic`: **reliable**, depth 1 (match the publisher). `/HardwareBat_topic`: `best_effort`. iTOF depth/cloud: `best_effort`.                                                                                 |
| **LCD video**        | Enter video mode with `/lcd_disable`, publish NV12 to `/lcd/video/nv12`, then `/lcd_enable` to resume emoji.                                                                                                                                                                   |


---



## 1. Sensor Overview


| Category             | Primary Output                                                             | How It Starts                |
| -------------------- | -------------------------------------------------------------------------- | ---------------------------- |
| IMU                  | `/HardwareImuBipedMotion_topic`                                            | MCU plugin auto-starts       |
| Joint proprioception | `/HardwareKinematic_topic`                                                 | MCU plugin auto-starts       |
| 2D LiDAR             | `/HardwareLidar_topic`                                                     | LiDAR plugin auto-starts     |
| Depth camera (iTOF)  | `/HardwareItof/point_cloud`, `/HardwareItof/depth_image`                   | iTOF plugin auto-starts      |
| RGB camera           | `/camera/image_jpeg`, `/camera/image_h264`, `/camera/jpeg_high_resolution` | Auto-starts with media stack |
| Microphone           | `/audio/capture_pcm`                                                       | Auto-starts with media stack |
| Speaker              | `/audio/speaker_pcm` (subscribe)                                           | Auto-starts with media stack |
| LCD                  | `/emoji/index` (subscribe), `/lcd/video/nv12` (subscribe)                  | Auto-starts with media stack |
| Head                 | `/HardwareHeadPosition_topic`                                              | MCU plugin auto-starts       |
| Downview             | `/HardwareDownviewFeedback_topic`                                          | MCU UART (event-driven)      |
| Battery              | `/HardwareBat_topic`                                                       | MCU plugin auto-starts       |
| Vehicle wheels       | `/HardwareVehicleFeedback_topic`                                           | MCU UART feedback            |


---



## 2. Sensor Lifecycle



### 2.1 ROS Parameters


| Parameter                   | Type     | Description                                              |
| --------------------------- | -------- | -------------------------------------------------------- |
| `hardware.<Sensor>.enabled` | `bool`   | Declared for all modules; **does not gate data streams** |
| `hardware.<Sensor>.status`  | `int`    | Runtime status code                                      |
| `hardware.<Sensor>.version` | `string` | Firmware / driver version                                |


**Sensor names:** `IMU`, `Lidar`, `ToF`, `Motor`, `MCU`


| Parameter                                       | Who updates `status` | `enabled` stops data? |
| ----------------------------------------------- | -------------------- | --------------------- |
| `hardware.MCU.enabled` / `status` / `version`   | MCU plugin           | No                    |
| `hardware.Lidar.enabled` / `status` / `version` | LiDAR plugin         | No                    |
| `hardware.ToF.enabled` / `status` / `version`   | iTOF plugin          | No                    |
| `hardware.IMU.enabled` / `status` / `version`   | Manager only         | No                    |
| `hardware.Motor.enabled` / `status` / `version` | Manager only         | No                    |


**Status codes:** `0` DISABLED · `1` ENABLED · `2` INITIALIZING · `3` WORKING · `4` ERROR · `5` READ_ERROR

### 2.2 Stream Control Services


| Service          | Type               | Functional? | Behavior                                       |
| ---------------- | ------------------ | ----------- | ---------------------------------------------- |
| `/lidar_enable`  | `std_srvs/Trigger` | Yes         | Restart scan thread (also auto-starts on init) |
| `/lidar_disable` | `std_srvs/Trigger` | Yes         | Stop scan + GPIO power off                     |
| `/itof_enable`   | `std_srvs/Trigger` | No          | Placeholder — always returns success           |
| `/itof_disable`  | `std_srvs/Trigger` | No          | Placeholder — does not stop stream             |


```bash
ros2 service call /lidar_enable std_srvs/srv/Trigger
ros2 param get /hardware hardware.Lidar.status
```

---



## 3. Inertial Sensing (IMU)

**Recommended for secondary development:** `/HardwareImuBipedMotion_topic` (~200 Hz). Both IMU topics share the same MCU sample and `sensor_msgs/Imu` fields; Std is a 2:1 downsample.


| Topic                           | Type              | QoS                      | Description                                                                |
| ------------------------------- | ----------------- | ------------------------ | -------------------------------------------------------------------------- |
| `/HardwareImuBipedMotion_topic` | `sensor_msgs/Imu` | best_effort, KeepLast(1) | Full-rate 6-DoF IMU (~200 Hz) — **use this**                               |
| `/HardwareImuStd_topic`         | `sensor_msgs/Imu` | best_effort, KeepLast(1) | Same payload, ~100 Hz — legacy / built-in apps (perception, factory, etc.) |
| `/HardwareImuCal_topic`         | `std_msgs/UInt8`  | KeepLast(1)              | Calibration status                                                         |


`/HardwareImuBipedMotion_topic` (and `/HardwareImuStd_topic`) fields:


| Field                 | Unit       | Notes                                        |
| --------------------- | ---------- | -------------------------------------------- |
| `header.frame_id`     | —          | `imu_link` (frame_id only — no TF broadcast) |
| `angular_velocity`    | rad/s      | Gyro                                         |
| `linear_acceleration` | m/s²       | Accel                                        |
| `orientation`         | quaternion | Fused attitude (w, x, y, z)                  |


Notes:

- IMU data embedded in `/HardwareKinematic_topic` (§9) is **not** updated; subscribe to `/HardwareImuBipedMotion_topic` instead.
- `orientation_covariance[0..2]` currently carries roll/pitch/yaw (×0.001), **not** a standard covariance matrix.

**Calibration:** publish to `/ControlImuCal_topic` (`std_msgs/UInt8`, `data` = mode byte)

```bash
ros2 topic hz /HardwareImuBipedMotion_topic
ros2 topic echo /HardwareImuBipedMotion_topic --once
ros2 topic pub /ControlImuCal_topic std_msgs/msg/UInt8 "{data: 1}" --once
```

---



## 4. 2D LiDAR


| Topic                  | Type                    | QoS | Description |
| ---------------------- | ----------------------- | --- | ----------- |
| `/HardwareLidar_topic` | `sensor_msgs/LaserScan` | 1   | 360° scan   |



| Property   | Value                           |
| ---------- | ------------------------------- |
| `frame_id` | `laser_frame` (no TF broadcast) |
| Range      | 0.001 – 15.0 m                  |
| FOV        | ±180°, counter-clockwise        |


```bash
ros2 topic echo /HardwareLidar_topic
ros2 service call /lidar_enable std_srvs/srv/Trigger {}   # enable LiDAR
ros2 service call /lidar_disable std_srvs/srv/Trigger {}  # disable LiDAR
```

---



## 5. Depth Camera (iTOF)

Primary outputs for secondary development (hardware ToF plugin; auto-starts with the plugin):


| Topic                       | Type                      | QoS (typical) | Description                          |
| --------------------------- | ------------------------- | ------------- | ------------------------------------ |
| `/HardwareItof/point_cloud` | `sensor_msgs/PointCloud2` | KeepLast(1)   | 3D point cloud                       |
| `/HardwareItof/depth_image` | `sensor_msgs/Image`       | KeepLast(1)   | Depth map, `16UC1`, values in **mm** |


Also published (optional):


| Topic                       | Type                     | Description       |
| --------------------------- | ------------------------ | ----------------- |
| `/HardwareItof/gray_image`  | `sensor_msgs/Image`      | `mono8` IR / gray |
| `/HardwareItof/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics        |


`/HardwareItof/depth_image`


| Field             | Notes                              |
| ----------------- | ---------------------------------- |
| `encoding`        | `16UC1`                            |
| `data`            | Per-pixel depth in millimetres     |
| `header.frame_id` | Optical / ToF frame (see TF below) |


`/HardwareItof/point_cloud`


| Field             | Notes                           |
| ----------------- | ------------------------------- |
| `header.frame_id` | Same optical frame as depth     |
| Point fields      | XYZ (and intensity if provided) |


**TF broadcast by hardware:** `itof_base_frame` → `itof_optical_frame`.

**No TF from hardware:** `imu_link`, `laser_frame` — supply via URDF or static TF.

**Lifecycle:** `/itof_enable` / `/itof_disable` are placeholders — stream auto-starts with plugin init; subscribe directly.

```bash
ros2 topic echo /HardwareItof/depth_image --once
ros2 topic hz /HardwareItof/point_cloud
ros2 service call /itof_enable std_srvs/srv/Trigger {}   # enable iTOF
ros2 service call /itof_disable std_srvs/srv/Trigger {}  # disable iTOF
ros2 run tf2_tools view_frames
```

---



## 6. RGB Camera

Streams publish only when subscribers exist (except H264 main path keeps encode path warm). Default resolutions: JPEG 640×480, H264 1280×720, high-res JPEG 2560×1440 (platform defaults).


| Topic                          | Type                          | Dir | Rate (typ.) | Description                                    |
| ------------------------------ | ----------------------------- | --- | ----------- | ---------------------------------------------- |
| `/camera/image_jpeg`           | `sensor_msgs/CompressedImage` | Pub | ~10 Hz      | Low-res JPEG (VIN), default 640×480            |
| `/camera/image_h264`           | `sensor_msgs/Image`           | Pub | ~15 Hz      | Main H.264 Annex-B bitstream, default 1280×720 |
| `/camera/jpeg_high_resolution` | `sensor_msgs/CompressedImage` | Pub | ~5 Hz       | High-res JPEG (VIN), default 2560×1440         |


QoS: `KeepLast(1)` (system-default reliability / durability unless overridden).

`/camera/image_jpeg` / `/camera/jpeg_high_resolution`


| Field             | Value / Notes                     |
| ----------------- | --------------------------------- |
| `format`          | `jpeg`                            |
| `header.frame_id` | `camera_frame`                    |
| `header.stamp`    | Device capture timestamp (ns→ROS) |
| `data`            | Raw JPEG bytes                    |


`/camera/image_h264`


| Field             | Value / Notes                     |
| ----------------- | --------------------------------- |
| `encoding`        | `h264`                            |
| `height`          | `1`                               |
| `width` / `step`  | Byte length of this NAL/AU packet |
| `header.frame_id` | `camera_frame`                    |
| `data`            | H.264 access unit bytes (Annex-B) |


Notes:

- New subscriber on `/camera/image_h264` triggers an IDR request for fast decoder start.
- Main H264 source may be VIN or USB (`video.h264.source`); other JPEG channels stay on VIN.
- Related (not required for minimal integration): `/camera/image_h264_low`, `/camera/image_h264_cloud`, `/camera/image_nv12`, `/camera/camera_info`, `/camera/camera_intrinsics`, TF `base_link` → `camera_link`.

```bash
ros2 topic hz /camera/image_jpeg
ros2 topic hz /camera/image_h264
ros2 topic hz /camera/jpeg_high_resolution
ros2 topic echo /camera/image_h264 --field encoding --once
```

---



## 7. Audio

PCM is carried in `sensor_msgs/Image` (not `audio_common_msgs`): `height=1`, `width=byte_length`, little-endian samples in `data`. Format string:

```
encoding = "pcm_<sample_rate>_<bits>_<channels>"
# e.g. mic (default capture): pcm_16000_16_1
# e.g. speaker (default play): pcm_48000_16_2
```



### 7.1 Microphone (publish)


| Topic                | Type                | Dir | QoS depth | Description                         |
| -------------------- | ------------------- | --- | --------- | ----------------------------------- |
| `/audio/capture_pcm` | `sensor_msgs/Image` | Pub | 100       | Processed mic PCM (ANC / AEC / raw) |


Default mic publish: **16 kHz, 16-bit, mono**, ~64 ms chunks (1024 frames). Only publishes when subscribers exist.

`header.frame_id` is a running packet counter (decimal string). `header.stamp` is wall-clock aligned with video PTS when available.

```bash
ros2 topic hz /audio/capture_pcm
ros2 topic echo /audio/capture_pcm --field encoding --once
```



### 7.2 Speaker (subscribe)


| Topic                | Type                | Dir | QoS                     | Description                |
| -------------------- | ------------------- | --- | ----------------------- | -------------------------- |
| `/audio/speaker_pcm` | `sensor_msgs/Image` | Sub | KeepLast(10) + reliable | PCM to play on loudspeaker |


Default play format: **48 kHz, 16-bit, stereo** → expect `encoding=pcm_48000_16_2`. Recommend ~60 ms chunks to match the speaker queue.

Optional related topics: `/audio/speaker_pcm_status` (`std_msgs/Int32` playback done), `/audio/speaker_pcm_interrupt`, `/audio/speaker_control`.

```bash
ros2 topic pub /audio/speaker_pcm sensor_msgs/msg/Image \
  "{header: {frame_id: '0'}, height: 1, width: 5760, encoding: 'pcm_48000_16_2', is_bigendian: false, step: 5760, data: []}"
```

---



## 8. LCD Emoji

Drives the face LCD from emoji assets (`mini_<index>` under the configured image dir, default `/app/joyin/config/emoji`), or from NV12 video frames in video mode.


| Topic             | Type                | Dir | QoS          | Description                                      |
| ----------------- | ------------------- | --- | ------------ | ------------------------------------------------ |
| `/emoji/index`    | `std_msgs/Int32`    | Sub | KeepLast(10) | Emoji / animation index to display on LCD        |
| `/lcd/video/nv12` | `sensor_msgs/Image` | Sub | KeepLast(10) | NV12 video frame → RGB565 blit (video mode only) |




### 8.1 `/emoji/index`


| Field  | Notes                                                                           |
| ------ | ------------------------------------------------------------------------------- |
| `data` | Integer index **0–71**, mapped to asset `mini_<index>` (GIF/PNG/JPEG or Lottie) |


Dropped when LCD not started or while in video mode (`/lcd_disable`). Event-driven (no fixed rate), default index value is 59.

```bash
ros2 topic pub /emoji/index std_msgs/msg/Int32 "{data: 0}" --once
ros2 topic pub /emoji/index std_msgs/msg/Int32 "{data: 59}" --once
```



### 8.2 `/lcd/video/nv12`

Enter video mode first (`/lcd_disable` stops emoji and enables video). Frames are converted to RGB565 and written to the panel. Exit with `/lcd_enable` to resume emoji.


| Field      | Requirement                                         |
| ---------- | --------------------------------------------------- |
| `encoding` | `nv12` or `NV12`                                    |
| `width`    | Even, > 0                                           |
| `height`   | Even, > 0                                           |
| `step`     | ≥ `width` (Y / UV plane stride)                     |
| `data`     | Contiguous `Y + UV`, size ≥ `step * height * 3 / 2` |


Invalid encoding, odd size, or undersized `data` → frame dropped (WARN log).

```bash
ros2 service call /lcd_disable std_srvs/srv/Trigger {}
# publish NV12 frames to /lcd/video/nv12 (encoding=nv12, even WxH)
ros2 service call /lcd_enable std_srvs/srv/Trigger {}   # resume emoji
```

Related (optional): `/lcd_enable`, `/lcd_disable`.

---



## 9. Joint & Motor Proprioception

20 servos (`ARTHROSIS_NUM_TOTAL = 20`). Primary joint state: `/HardwareKinematic_topic`.

**Joint definition:**


| Index   | Joint Name                                                                          |
| ------- | ----------------------------------------------------------------------------------- |
| 0 - 3   | Left shoulder pitch, left shoulder roll, left elbow roll, left gripper (unused)     |
| 4 - 7   | Right shoulder pitch, right shoulder roll, right elbow roll, right gripper (unused) |
| 8 - 13  | Left hip yaw, roll, pitch, left knee pitch, left ankle pitch, ankle roll            |
| 14 - 19 | Right hip yaw, roll, pitch, right knee pitch, right ankle pitch, ankle roll         |




### Feedback


| Topic                         | Type                       | Description                                                   |
| ----------------------------- | -------------------------- | ------------------------------------------------------------- |
| `/HardwareKinematic_topic`    | `hardware/KinematicState`  | IMU(as placeholder only) + 20 joint angles (deg) + velocities |
| `/HardwareMotorLoad_topic`    | `std_msgs/Int16MultiArray` | Load (22)                                                     |
| `/HardwareMotorCurrent_topic` | `std_msgs/Int16MultiArray` | Current (22)                                                  |
| `/HardwareMotorTemp_topic`    | `std_msgs/UInt8MultiArray` | Temperature (22)                                              |
| `/HardwareMotorErr_topic`     | `std_msgs/UInt8MultiArray` | Fault codes (22)                                              |




### Control (publish to these)


| Topic                         | Type                         | Payload                 |
| ----------------------------- | ---------------------------- | ----------------------- |
| `/ControlMotorAngles_topic`   | `std_msgs/Float32MultiArray` | 20 angles, degrees      |
| `/ControlMotorSpeed_topic`    | `std_msgs/Int16MultiArray`   | 20 maximum speed limits (servo ticks/s) |
| `/ControlEnableMotor_topic`   | `std_msgs/UInt8MultiArray`   | 1–2 bytes               |
| `/ControlPidControl_topic`    | `control/PidControl`         | `{index, kp, ki, kd}`   |


**Example usage:**

**`/ControlMotorAngles_topic`** — Publish 20 joint angles in degrees:

```python
# Example from control_system.py: publish_joint_angles_command()
msg = Float32MultiArray()
msg.data = [0.0] * 20

# Indices 0-3: left shoulder pitch/roll, elbow roll, gripper (unused, set to 0)
# Indices 4-7: right shoulder pitch/roll, elbow roll, gripper (unused, set to 0)
# Indices 8-13: left hip yaw/roll/pitch, knee pitch, ankle pitch, ankle roll
# Indices 14-19: right hip yaw/roll/pitch, knee pitch, ankle pitch, ankle roll

# Convert from standard-form joint angles (radians) to raw motor angles (degrees)
for i in range(8):
    msg.data[i] = math.degrees(target_position_arm[i] + offset_angles_arm[i]) * dir_trans_arm[i]

for i in range(12):
    msg.data[i+8] = math.degrees(target_position[i] + offset_angles[i]) * dir_trans[i]

joint_angles_pub.publish(msg)
```

See [control_system.py](JOYIN_MINI_DEPLOY/control_system.py) for a complete RL deployment example with offset/direction transformations.

**`/ControlMotorSpeed_topic`** — Set maximum speed limit for all 20 joints:

```python
# Example from control_system.py: publish_joint_velocity_value_command()
msg = Int16MultiArray()
msg.data = [0] * 20

# Set the same velocity limit for all 20 joints
# Value represents maximum angular speed in servo encoder ticks/s
# Typical range: 0-4096 (e.g., 4090 is near maximum speed)
for i in range(20):
    msg.data[i] = velocity_value  # e.g., 4090 for high-speed motion

joint_velocitys_pub.publish(msg)
```

This sets the speed ceiling for motion commands. Lower values produce slower, more controlled movements; higher values allow faster transitions. The value is in servo encoder ticks per second, where 4090 represents near-maximum speed capability(default).

See [control_system.py](JOYIN_MINI_DEPLOY/control_system.py) `publish_joint_velocity_value_command()` for the implementation.
 


**`/ControlEnableMotor_topic` payload:**

`data[0]` — Motor torque control:
- `0x00`: Disable torque output
- `0x01`: Enable torque output
- `0x80`: Set current position as center value (do not send outside calibration process)

`data[1]` — Individual motor enable/disable (bit field, 0 = disabled, 1 = enabled):
- `bit0`: Left arm
- `bit1`: Right arm
- `bit2`: Head
- `bit3`: Legs
- `bit4`: Head yaw
- `bit5`: Head pitch
- `bit6`: Fall protection auto-release (0 = enable auto-release on fall detection, 1 = disable)

**`/ControlPidControl_topic`** — Configure PID parameters for individual joints:

```python
# Example from control_system.py: publish_joint_pid_ctrl_command()
msg = PidControl()

# Set PID gains for all 20 joints individually
# Each joint requires a separate message with its index
for i in range(20):
    msg.index = i    # Joint index (0-19)
    msg.kp = kp      # Proportional gain
    msg.ki = ki      # Integral gain
    msg.kd = kd      # Derivative gain
    joint_pid_control_pub.publish(msg)
    time.sleep(0.002)  # Small delay between messages
```

**Important:** default value kp(20), kd(2) are used for motor system identification, changes of PID gains would lead to issues in RL model sim2real transfer.

See [control_system.py](JOYIN_MINI_DEPLOY/control_system.py) `publish_joint_pid_ctrl_command()` for the implementation.




**Mechanical limits (raw angles):**


| Joint                     | Range (degrees) |
| ------------------------- | --------------- |
| Left/right shoulder pitch | -165 ~ 165      |
| Left shoulder roll        | -38 ~ 100       |
| Right shoulder roll       | -100 ~ 38       |
| Left elbow roll           | -40 ~ 100       |
| Right elbow roll          | -100 ~ 40       |
| Left/right hip yaw        | -18 ~ 18        |
| Left/right hip roll       | -20 ~ 20        |
| Left hip pitch            | -160 ~ 50       |
| Right hip pitch           | -50 ~ 160       |
| Left knee pitch           | -155 ~ 60       |
| Right knee pitch          | -60 ~ 155       |
| Left ankle pitch          | -70 ~ 145       |
| Right ankle pitch         | -145 ~ 70       |
| Head yaw                  | -90 ~ 90        |
| Head pitch                | -40 ~ 25        |


caution: exceed these ranges can cause servo failures or damages. The zero point for the joint raw angles of M1 correspond to a crouch pose (for calibration usage). In RL deployment, use the standard-form joint angle, see `kinematic_state_feedback_msg_callback` in [control_system.py](JOYIN_MINI_DEPLOY/control_system.py).   

### Factory Action


| Action                        | Type                   | Goal                             |
| ----------------------------- | ---------------------- | -------------------------------- |
| `/HardwareMcuFctCheck_action` | `hardware/McuFctCheck` | `fct_check_mode: 0` (servo test) |


---



## 10. Battery


| Topic                | Type           | QoS |
| -------------------- | -------------- | --- |
| `/HardwareBat_topic` | `hardware/Bat` | 1   |



| Field                                                      | Unit      |
| ---------------------------------------------------------- | --------- |
| `power`                                                    | % SOC     |
| `bat_vol`                                                  | mV        |
| `bat_cur`                                                  | mA        |
| `bat_temp`                                                 | —         |
| `plate_vol`                                                | mV        |
| `on_charger`, `is_charging`, `charge_full`, `charge_error` | 0/1 flags |


Charge control: `/BusinessEnableCharge_topic` (`std_msgs/UInt8`)

---



## 11. Vehicle Platform


| Topic                            | Type                       | QoS                      |
| -------------------------------- | -------------------------- | ------------------------ |
| `/HardwareVehicleFeedback_topic` | `hardware/VehicleFeedback` | best_effort, KeepLast(1) |


`data[10]`: `[reserved, left_mile, right_mile, left_speed, right_speed, left_fault, right_fault, comm_status, left_current, right_current]`

`data[7]` is `comm_status` (raw MCU field).


| Topic                        | Dir | Type                                                       |
| ---------------------------- | --- | ---------------------------------------------------------- |
| `/ControlVehicleSpeed_topic` | Sub | `std_msgs/Int16MultiArray` `[left, right]` (MCU wheel cmd) |


---



## 12. Head Control


| Topic                         | Type                       | Dir | QoS                      | Description                          |
| ----------------------------- | -------------------------- | --- | ------------------------ | ------------------------------------ |
| `/HardwareHeadPosition_topic` | `hardware/HeadPosition`    | Pub | best_effort, KeepLast(1) | Head yaw/pitch feedback              |
| `/ControlHeadControl_topic`   | `std_msgs/Int16MultiArray` | Sub | best_effort, KeepLast(1) | Head yaw/pitch command (MCU control) |


### 12.1 Feedback — `/HardwareHeadPosition_topic`

`hardware/HeadPosition.data` (length 4, `int16`):


| Index | Field     | Meaning          | Unit                              |
| ----- | --------- | ---------------- | --------------------------------- |
| 0     | `h_angle` | Yaw (horizontal) | 0.01° (`degrees = data[0] / 100`) |
| 1     | `h_speed` | Yaw speed        | same scale as angle (MCU raw)     |
| 2     | `v_angle` | Pitch (vertical) | 0.01° (`degrees = data[2] / 100`) |
| 3     | `v_speed` | Pitch speed      | same scale as angle (MCU raw)     |


Timestamped MCU path is decimated (~publish every 10th sample). Match publisher QoS (`best_effort`).

Mechanical limits (degrees): yaw **-90 ~ 90**, pitch **-40 ~ 25**.


### 12.2 Control — `/ControlHeadControl_topic`

MCU requires **exactly 6** `int16` values (otherwise the command is dropped):


| Index | Field       | Notes                                                          |
| ----- | ----------- | -------------------------------------------------------------- |
| 0     | yaw         | Target yaw, **0.01°** (`int16`; e.g. 9000 = 90°)               |
| 1     | pitch       | Target pitch, **0.01°**                                        |
| 2     | yaw_speed   | Motion speed (MCU raw `int16`; typical non-zero default ≈ 100) |
| 3     | pitch_speed | Motion speed (MCU raw `int16`)                                 |
| 4     | reserved    | Set to `0`                                                     |
| 5     | reserved    | Set to `0`                                                     |


```bash
# Look roughly forward (yaw=0, pitch=0); non-zero speeds required by many MCU paths
# /ControlHeadControl_topic 需要叠加零点才能移动到正确位置，通过 /ControlHeadMotion_topic 发送0角度指令，echo/ControlHeadControl_topic 查阅在0角度时头部舵机的零点
# /ControlHeadMotion_topic 的单位为0.1°

ros2 topic pub /ControlHeadControl_topic std_msgs/msg/Int16MultiArray  "{data: [0, 0, 100, 100, 0, 0]}" --once

ros2 topic echo /HardwareHeadPosition_topic --once
```

> Some higher-level apps still publish `/ControlHeadMotion_topic` (often 4 elements). That topic is **not** subscribed by `hardware_mcu_plugin`. Use `/ControlHeadControl_topic` for direct MCU head control.


---



## 13. Downview


| Topic                             | Type                        | Dir | QoS         | Description                                      |
| --------------------------------- | --------------------------- | --- | ----------- | ------------------------------------------------ |
| `/HardwareDownviewFeedback_topic` | `std_msgs/UInt32MultiArray` | Pub | KeepLast(1) | Left/right downward range (cliff / ground sense) |


`data` length **2**:


| Index | Field   | Unit | Notes                   |
| ----- | ------- | ---- | ----------------------- |
| 0     | `left`  | mm   | Left downview distance  |
| 1     | `right` | mm   | Right downview distance |


Published only when MCU sends a downview UART frame (**event-driven / on-demand**). The topic may stay silent if no frame arrives — do not treat silence as a QoS mismatch alone.

```bash
ros2 topic echo /HardwareDownviewFeedback_topic
```


---



## 14. Custom Message Definitions

```
hardware/msg/Bat
  uint8 power; int16 bat_vol, bat_cur, bat_temp, plate_vol
  uint8 on_charger, is_charging, charge_full, charge_error

hardware/msg/KinematicState
  sensor_msgs/Imu imu_data
  float32[] joint_angles    # 20, degrees (MCU: motor_angle/100)
  int16[]   joint_velocities

hardware/msg/VehicleFeedback
  int32[] data

hardware/msg/HeadPosition
  std_msgs/Header header
  int16[] data              # [h_angle, h_speed, v_angle, v_speed], 0.01°

control/msg/PidControl
  uint8 index, kp, ki, kd
```

---



## 15. Integration Recipes



### Perception / SLAM node

Subscribe with matching QoS (`best_effort`, depth 1 where noted):

```
/HardwareLidar_topic
/HardwareItof/depth_image
/HardwareItof/point_cloud
/HardwareImuBipedMotion_topic
```

Provide static TF: `base_link` → `laser_frame`, `base_link` → `imu_link`, `base_link` → `itof_base_frame`.

### Camera + mic (VUI / telepresence)

```
/camera/image_jpeg              # preview
/camera/image_h264              # streaming encode
/camera/jpeg_high_resolution    # high-res stills / OCR
/audio/capture_pcm              # mic PCM (sensor_msgs/Image)
```



### Speaker + LCD

```bash
# Play: publish pcm_48000_16_2 chunks to /audio/speaker_pcm
# Face emoji:
ros2 topic pub /emoji/index std_msgs/msg/Int32 "{data: 25}" --once
# Face video (NV12):
ros2 service call /lcd_disable std_srvs/srv/Trigger {}
# → publish sensor_msgs/Image (encoding=nv12) to /lcd/video/nv12
ros2 service call /lcd_enable std_srvs/srv/Trigger {}
```



### Teleop (on cart)

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```



### Joint control

```bash
# 20 float values in degrees
ros2 topic pub /ControlMotorAngles_topic std_msgs/msg/Float32MultiArray "{data: [...]}" -r 50
ros2 topic echo /HardwareKinematic_topic
```



### Head control

```bash
ros2 topic pub /ControlHeadControl_topic std_msgs/msg/Int16MultiArray \
  "{data: [0, 0, 100, 100, 0, 0]}" --once
ros2 topic echo /HardwareHeadPosition_topic
```

### Downview

```bash
# May block until MCU emits a frame
ros2 topic echo /HardwareDownviewFeedback_topic --once
```

### Health monitoring

```bash
ros2 param get /hardware hardware.Lidar.status
ros2 param get /hardware hardware.ToF.status
ros2 param get /hardware hardware.MCU.status
```

---



## 16. Units & QoS Cheat Sheet


| Quantity         | Unit                     |
| ---------------- | ------------------------ |
| Joint angles     | degrees                  |
| IMU gyro / accel | rad/s, m/s²              |
| iTOF depth       | mm (`16UC1`)             |
| Mic PCM          | 16 kHz / 16-bit / mono   |
| Speaker PCM      | 48 kHz / 16-bit / stereo |
| Wheel control    | mm/s                     |
| Head angles      | 0.01° on wire (`/100` → degrees) |
| Downview range   | mm                       |



| Topic                            | QoS                    |
| -------------------------------- | ---------------------- |
| `/HardwareKinematic_topic`       | best_effort, depth 1   |
| `/HardwareVehicleFeedback_topic` | best_effort, depth 1   |
| `/HardwareHeadPosition_topic`    | best_effort, depth 1   |
| `/ControlHeadControl_topic`      | best_effort, depth 1   |
| `/HardwareDownviewFeedback_topic`| KeepLast(1)            |
| `/HardwareLidar_topic`           | reliable, depth 1      |
| `/HardwareImuBipedMotion_topic`  | best_effort, depth 1   |
| `/HardwareImuStd_topic`          | best_effort, depth 1   |
| `/camera/image_jpeg`             | KeepLast(1)            |
| `/camera/image_h264`             | KeepLast(1)            |
| `/camera/jpeg_high_resolution`   | KeepLast(1)            |
| `/audio/capture_pcm`             | depth 100              |
| `/audio/speaker_pcm`             | reliable, KeepLast(10) |
| `/emoji/index`                   | KeepLast(10)           |
| `/lcd/video/nv12`                | KeepLast(10)           |


---



## 17. Full Topic Index


| Sensor   | Publish                                                                                                                                       | Subscribe / Service                                                                                                                                                     |
| -------- | --------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| IMU      | `/HardwareImuBipedMotion_topic` (~200 Hz, **recommended**), `/HardwareImuStd_topic` (~100 Hz, same payload / legacy), `/HardwareImuCal_topic` | `/ControlImuCal_topic`                                                                                                                                                  |
| Joints   | `/HardwareKinematic_topic`, `/HardwareMotorLoad_topic`, `/HardwareMotorCurrent_topic`, `/HardwareMotorTemp_topic`, `/HardwareMotorErr_topic`  | `/ControlMotorAngles_topic`, `/ControlMotorSpeed_topic`, `/ControlMotorRangeSet_topic`, `/ControlEnableMotor_topic`, `/ControlPidControl_topic`, `/ControlGetPid_topic` |
| LiDAR    | `/HardwareLidar_topic`                                                                                                                        | `/lidar_enable`, `/lidar_disable`                                                                                                                                       |
| iTOF     | `/HardwareItof/point_cloud`, `/HardwareItof/depth_image`, `/HardwareItof/gray_image`, `/HardwareItof/camera_info`                             | `/itof_enable`, `/itof_disable` (placeholders)                                                                                                                          |
| Camera   | `/camera/image_jpeg`, `/camera/image_h264`, `/camera/jpeg_high_resolution`                                                                    | —                                                                                                                                                                       |
| Mic      | `/audio/capture_pcm`                                                                                                                          | —                                                                                                                                                                       |
| Speaker  | `/audio/speaker_pcm_status` (optional)                                                                                                        | `/audio/speaker_pcm`                                                                                                                                                    |
| LCD      | —                                                                                                                                             | `/emoji/index`, `/lcd/video/nv12`, `/lcd_enable`, `/lcd_disable`                                                                                                        |
| Battery  | `/HardwareBat_topic`                                                                                                                          | `/BusinessEnableCharge_topic`                                                                                                                                           |
| Wheels   | `/HardwareVehicleFeedback_topic`                                                                                                              | `/ControlVehicleSpeed_topic`                                                                                                                                            |
| Head     | `/HardwareHeadPosition_topic` (`[h_angle, h_speed, v_angle, v_speed]`, 0.01°)                                                                  | `/ControlHeadControl_topic` (`[yaw, pitch, yaw_speed, pitch_speed, 0, 0]`, 0.01°)                                                                                        |
| Downview | `/HardwareDownviewFeedback_topic` (`[left, right]` mm; event-driven)                                                                          | —                                                                                                                                                                       |
| Keys     | `/HardwareKey_topic` (`std_msgs/UInt16`), `/SystemConfigButton`                                                                               | —                                                                                                                                                                       |
| Health   | —                                                                                                                                             | `hardware.MCU.status`, `hardware.Lidar.status`, `hardware.ToF.status`                                                                                                   |


---

