# JRobot 传感器 ROS 2 接口参考文档

<div align="right">
  <a href="readme.md">English</a> |
  <a href="readme_cn.md"><strong>中文</strong></a>
</div>

在 Joyin 机器人平台上进行二次开发的传感器为中心的 ROS 2 API。

## 版本更新日志


| 版本       | 日期       | 更新内容                                                                                                                                                             |
| ---------- | ---------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **v0.0.3** | 2026-10-12 | 移除无效章节，更新控制相关 API，并将文档重命名为 ros2_api.md。                                                                                                       |
| **v0.0.2** | 2026-09-30 | 新增媒体 ROS 2 API：iTOF、RGB 相机（`/camera/image_jpeg`、`/camera/image_h264`、`/camera/jpeg_high_resolution`）、麦克风/扬声器（`/audio/capture_pcm`、`/audio/speaker_pcm`）、LCD（`/emoji/index`、`/lcd/video/nv12`）。 |
| **v0.0.1** | 2026-07-17 | 初始传感器为中心的 ROS 2 接口参考文档，用于海外二次开发。涵盖 IMU、LiDAR、iTOF、关节、电池、车轮、头部姿态、下视和按键。记录生命周期参数、QoS 注意事项、TF 警告和集成示例。 |


---



## 设备访问

通过 USB ADB 登录机器人，然后在设备上的开发者工作空间中工作。请参阅 [jrobot_adb_setup.md](jrobot_adb_setup.md) 设置 ADB 环境（[中文版](jrobot_adb_setup_cn.md)）。

### 1. 通过 ADB shell 连接

在主机 PC 上（USB 线缆连接到机器人的 Type-C 接口）：

```bash
# 验证设备是否可见
adb devices

# 在设备上打开交互式 shell
adb shell
```



### 2. 进入开发者工作空间

在 ADB shell 内：

```bash
cd /userdata/developer/
```

使用此目录在设备上部署二进制文件、配置和二次开发示例。

---



### 3. 设置 WiFi 网络

```bash
wifi_connect <SSID> <PASSWORD>
ifconfig wlan0
```



### 4. 构建 RL 环境和示例参考

参见：[https://github.com/Zeroth-OpenBridge](https://github.com/Zeroth-OpenBridge)

在此设置 M1 edu 的训练环境：[JOYIN_MINI_GYM](https://github.com/ZerothOpenBridge/Zeroth-M1_EDU_Train)

在模拟环境中验证策略后，在此部署：[JOYIN_MINI_DEPLOY](https://github.com/ZerothOpenBridge/Zeroth-M1_EDU_Deploy)

**重要提示：** 当建立自定义代码（即发送关节角度命令或车轮速度命令）时，请确保关闭默认嵌入的 `control_pm` 节点（在 `run.sh` 中）：

```bash
cd /opt/joyin
# 打开 run.sh 并注释掉以下行：
# ${BIN_DIR}/plugman ${CFG_DIR}/control_pm.json > /dev/null 2>&1 &
```




## 快速入门（5 分钟）

```bash
# 1. 确认 /hardware 节点正在运行
ros2 node list | grep hardware
ros2 node info /hardware

# 2. 列出实时传感器/媒体话题
ros2 topic list | grep -E 'Hardware|camera|audio|emoji|cmd_vel'

# 3. 测试感知和媒体
ros2 topic hz /HardwareImuBipedMotion_topic
ros2 topic hz /HardwareLidar_topic
ros2 topic echo /HardwareItof/depth_image --once
ros2 topic hz /camera/image_jpeg
ros2 topic hz /audio/capture_pcm

# 4. 检查传感器健康状态（ROS 参数）
ros2 param get /hardware hardware.Lidar.status
ros2 param get /hardware hardware.ToF.status
ros2 param get /hardware hardware.MCU.status
```

**最小感知/媒体订阅集**

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



## 集成前必读


| 主题               | 详细说明                                                                                                                                                                                                                               |
| ------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **iTOF 启动/停止** | `/itof_enable` 和 `/itof_disable` 是占位符：调用总是成功但**不会**停止流。启动后，直接订阅 `/HardwareItof/depth_image` 和 `/HardwareItof/point_cloud`，使用 **best_effort** QoS（必须匹配发布者）。                                   |
| **LiDAR 启动/停止** | 使用 `/lidar_enable` / `/lidar_disable`（`std_srvs/Trigger`）控制扫描线程和电源。`hardware.Lidar.status=3`（WORKING）**不保证**实时扫描 — 如果 `/HardwareLidar_topic` 无数据，调用 `/lidar_enable` 然后 `ros2 topic hz /HardwareLidar_topic`。 |
| **TF 坐标系**      | 硬件仅为 IMU/LiDAR 发布 `frame_id`：`imu_link` 和 `laser_frame` **没有 TF**。提供 `static_transform_publisher` 或 URDF（`base_link` → `imu_link`、`base_link` → `laser_frame`）。iTOF 广播 `itof_base_frame` → `itof_optical_frame`。     |
| **健康参数**       | 使用实时状态：`hardware.MCU.status`、`hardware.Lidar.status`、`hardware.ToF.status`（`3` = WORKING）。**不要**依赖 `hardware.IMU.status` 或 `hardware.Motor.status`（占位符）。                                                           |
| **IMU 话题**       | 优先使用 `/HardwareImuBipedMotion_topic`（~200 Hz）进行二次开发。`/HardwareImuStd_topic` 是相同载荷的降采样（~100 Hz），保留给内置应用/兼容性。                                                                                          |
| **QoS**            | `/HardwareKinematic_topic`：`best_effort`，深度 1。`/HardwareLidar_topic`：**reliable**，深度 1（匹配发布者）。`/HardwareBat_topic`：`best_effort`。iTOF 深度/云：`best_effort`。                                                      |
| **LCD 视频**       | 使用 `/lcd_disable` 进入视频模式，发布 NV12 到 `/lcd/video/nv12`，然后 `/lcd_enable` 恢复表情。                                                                                                                                        |


---



## 1. 传感器概览


| 类别            | 主要输出                                                                     | 如何启动                |
| --------------- | ---------------------------------------------------------------------------- | ----------------------- |
| IMU             | `/HardwareImuBipedMotion_topic`                                              | MCU 插件自动启动        |
| 关节本体感觉    | `/HardwareKinematic_topic`                                                   | MCU 插件自动启动        |
| 2D LiDAR        | `/HardwareLidar_topic`                                                       | LiDAR 插件自动启动      |
| 深度相机（iTOF） | `/HardwareItof/point_cloud`、`/HardwareItof/depth_image`                     | iTOF 插件自动启动       |
| RGB 相机        | `/camera/image_jpeg`、`/camera/image_h264`、`/camera/jpeg_high_resolution`   | 媒体栈自动启动          |
| 麦克风          | `/audio/capture_pcm`                                                         | 媒体栈自动启动          |
| 扬声器          | `/audio/speaker_pcm`（订阅）                                                  | 媒体栈自动启动          |
| LCD             | `/emoji/index`（订阅）、`/lcd/video/nv12`（订阅）                             | 媒体栈自动启动          |
| 头部            | `/HardwareHeadPosition_topic`                                                | MCU 插件自动启动        |
| 下视            | `/HardwareDownviewFeedback_topic`                                            | MCU UART（事件驱动）    |
| 电池            | `/HardwareBat_topic`                                                         | MCU 插件自动启动        |
| 车轮            | `/HardwareVehicleFeedback_topic`                                             | MCU UART 反馈           |


---



## 2. 传感器生命周期



### 2.1 ROS 参数


| 参数                        | 类型     | 描述                                  |
| --------------------------- | -------- | ------------------------------------- |
| `hardware.<Sensor>.enabled` | `bool`   | 为所有模块声明；**不控制数据流**      |
| `hardware.<Sensor>.status`  | `int`    | 运行时状态码                          |
| `hardware.<Sensor>.version` | `string` | 固件/驱动版本                         |


**传感器名称：** `IMU`、`Lidar`、`ToF`、`Motor`、`MCU`


| 参数                                        | 谁更新 `status` | `enabled` 停止数据？ |
| ------------------------------------------- | --------------- | -------------------- |
| `hardware.MCU.enabled` / `status` / `version`   | MCU 插件        | 否                   |
| `hardware.Lidar.enabled` / `status` / `version` | LiDAR 插件      | 否                   |
| `hardware.ToF.enabled` / `status` / `version`   | iTOF 插件       | 否                   |
| `hardware.IMU.enabled` / `status` / `version`   | 仅管理器        | 否                   |
| `hardware.Motor.enabled` / `status` / `version` | 仅管理器        | 否                   |


**状态码：** `0` DISABLED · `1` ENABLED · `2` INITIALIZING · `3` WORKING · `4` ERROR · `5` READ_ERROR

### 2.2 流控制服务


| 服务             | 类型               | 功能性？ | 行为                                |
| ---------------- | ------------------ | -------- | ----------------------------------- |
| `/lidar_enable`  | `std_srvs/Trigger` | 是       | 重启扫描线程（初始化时也自动启动）  |
| `/lidar_disable` | `std_srvs/Trigger` | 是       | 停止扫描 + GPIO 断电                |
| `/itof_enable`   | `std_srvs/Trigger` | 否       | 占位符 — 总是返回成功               |
| `/itof_disable`  | `std_srvs/Trigger` | 否       | 占位符 — 不停止流                   |


```bash
ros2 service call /lidar_enable std_srvs/srv/Trigger
ros2 param get /hardware hardware.Lidar.status
```

---



## 3. 惯性传感（IMU）

**推荐用于二次开发：** `/HardwareImuBipedMotion_topic`（~200 Hz）。两个 IMU 话题共享相同的 MCU 采样和 `sensor_msgs/Imu` 字段；Std 是 2:1 降采样。


| 话题                            | 类型              | QoS                      | 描述                                             |
| ------------------------------- | ----------------- | ------------------------ | ------------------------------------------------ |
| `/HardwareImuBipedMotion_topic` | `sensor_msgs/Imu` | best_effort, KeepLast(1) | 全速率 6-DoF IMU（~200 Hz）— **使用此话题**      |
| `/HardwareImuStd_topic`         | `sensor_msgs/Imu` | best_effort, KeepLast(1) | 相同载荷，~100 Hz — 旧版/内置应用（感知、工厂等） |
| `/HardwareImuCal_topic`         | `std_msgs/UInt8`  | KeepLast(1)              | 校准状态                                         |


`/HardwareImuBipedMotion_topic`（和 `/HardwareImuStd_topic`）字段：


| 字段                  | 单位       | 备注                                  |
| --------------------- | ---------- | ------------------------------------- |
| `header.frame_id`     | —          | `imu_link`（仅 frame_id — 无 TF 广播） |
| `angular_velocity`    | rad/s      | 陀螺仪                                |
| `linear_acceleration` | m/s²       | 加速度计                              |
| `orientation`         | quaternion | 融合姿态（w, x, y, z）                |


备注：

- `/HardwareKinematic_topic`（§9）中嵌入的 IMU 数据**未**更新；请订阅 `/HardwareImuBipedMotion_topic`。
- `orientation_covariance[0..2]` 当前携带 roll/pitch/yaw（×0.001），**不是**标准协方差矩阵。

**校准：** 发布到 `/ControlImuCal_topic`（`std_msgs/UInt8`，`data` = 模式字节）

```bash
ros2 topic hz /HardwareImuBipedMotion_topic
ros2 topic echo /HardwareImuBipedMotion_topic --once
ros2 topic pub /ControlImuCal_topic std_msgs/msg/UInt8 "{data: 1}" --once
```

---



## 4. 2D LiDAR


| 话题                   | 类型                    | QoS | 描述     |
| ---------------------- | ----------------------- | --- | -------- |
| `/HardwareLidar_topic` | `sensor_msgs/LaserScan` | 1   | 360° 扫描 |



| 属性       | 值                              |
| ---------- | ------------------------------- |
| `frame_id` | `laser_frame`（无 TF 广播）     |
| 量程       | 0.001 – 15.0 m                  |
| FOV        | ±180°，逆时针                   |


```bash
ros2 topic echo /HardwareLidar_topic
ros2 service call /lidar_enable std_srvs/srv/Trigger {}   # 启用 LiDAR
ros2 service call /lidar_disable std_srvs/srv/Trigger {}  # 禁用 LiDAR
```

---



## 5. 深度相机（iTOF）

用于二次开发的主要输出（硬件 ToF 插件；随插件自动启动）：


| 话题                        | 类型                      | QoS（典型）  | 描述                               |
| --------------------------- | ------------------------- | ------------ | ---------------------------------- |
| `/HardwareItof/point_cloud` | `sensor_msgs/PointCloud2` | KeepLast(1)  | 3D 点云                            |
| `/HardwareItof/depth_image` | `sensor_msgs/Image`       | KeepLast(1)  | 深度图，`16UC1`，值以 **毫米** 为单位 |


也发布（可选）：


| 话题                        | 类型                     | 描述            |
| --------------------------- | ------------------------ | --------------- |
| `/HardwareItof/gray_image`  | `sensor_msgs/Image`      | `mono8` IR / 灰度 |
| `/HardwareItof/camera_info` | `sensor_msgs/CameraInfo` | 内参            |


`/HardwareItof/depth_image`


| 字段              | 备注                         |
| ----------------- | ---------------------------- |
| `encoding`        | `16UC1`                      |
| `data`            | 每像素深度，以毫米为单位     |
| `header.frame_id` | 光学/ToF 坐标系（见下文 TF） |


`/HardwareItof/point_cloud`


| 字段              | 备注                         |
| ----------------- | ---------------------------- |
| `header.frame_id` | 与深度相同的光学坐标系       |
| 点字段            | XYZ（如果提供则包含强度）    |


**硬件广播的 TF：** `itof_base_frame` → `itof_optical_frame`。

**硬件不提供的 TF：** `imu_link`、`laser_frame` — 通过 URDF 或静态 TF 提供。

**生命周期：** `/itof_enable` / `/itof_disable` 是占位符 — 流随插件初始化自动启动；直接订阅。

```bash
ros2 topic echo /HardwareItof/depth_image --once
ros2 topic hz /HardwareItof/point_cloud
ros2 service call /itof_enable std_srvs/srv/Trigger {}   # 启用 iTOF
ros2 service call /itof_disable std_srvs/srv/Trigger {}  # 禁用 iTOF
ros2 run tf2_tools view_frames
```

---



## 6. RGB 相机

仅当存在订阅者时流才发布（除了 H264 主路径保持编码路径预热）。默认分辨率：JPEG 640×480，H264 1280×720，高分辨率 JPEG 2560×1440（平台默认值）。


| 话题                           | 类型                          | 方向 | 速率（典型） | 描述                                |
| ------------------------------ | ----------------------------- | ---- | ------------ | ----------------------------------- |
| `/camera/image_jpeg`           | `sensor_msgs/CompressedImage` | 发布 | ~10 Hz       | 低分辨率 JPEG（VIN），默认 640×480   |
| `/camera/image_h264`           | `sensor_msgs/Image`           | 发布 | ~15 Hz       | 主 H.264 Annex-B 流，默认 1280×720   |
| `/camera/jpeg_high_resolution` | `sensor_msgs/CompressedImage` | 发布 | ~5 Hz        | 高分辨率 JPEG（VIN），默认 2560×1440 |


QoS：`KeepLast(1)`（系统默认可靠性/持久性，除非覆盖）。

`/camera/image_jpeg` / `/camera/jpeg_high_resolution`


| 字段              | 值/备注                        |
| ----------------- | ------------------------------ |
| `format`          | `jpeg`                         |
| `header.frame_id` | `camera_frame`                 |
| `header.stamp`    | 设备捕获时间戳（ns→ROS）       |
| `data`            | 原始 JPEG 字节                 |


`/camera/image_h264`


| 字段              | 值/备注                         |
| ----------------- | ------------------------------- |
| `encoding`        | `h264`                          |
| `height`          | `1`                             |
| `width` / `step`  | 此 NAL/AU 包的字节长度          |
| `header.frame_id` | `camera_frame`                  |
| `data`            | H.264 访问单元字节（Annex-B）   |


备注：

- `/camera/image_h264` 的新订阅者触发 IDR 请求，以便解码器快速启动。
- 主 H264 源可能是 VIN 或 USB（`video.h264.source`）；其他 JPEG 通道保持在 VIN 上。
- 相关（对于最小集成不需要）：`/camera/image_h264_low`、`/camera/image_h264_cloud`、`/camera/image_nv12`、`/camera/camera_info`、`/camera/camera_intrinsics`、TF `base_link` → `camera_link`。

```bash
ros2 topic hz /camera/image_jpeg
ros2 topic hz /camera/image_h264
ros2 topic hz /camera/jpeg_high_resolution
ros2 topic echo /camera/image_h264 --field encoding --once
```

---



## 7. 音频

PCM 承载在 `sensor_msgs/Image` 中（不是 `audio_common_msgs`）：`height=1`，`width=byte_length`，小端样本在 `data` 中。格式字符串：

```
encoding = "pcm_<sample_rate>_<bits>_<channels>"
# 例如麦克风（默认捕获）：pcm_16000_16_1
# 例如扬声器（默认播放）：pcm_48000_16_2
```



### 7.1 麦克风（发布）


| 话题                 | 类型                | 方向 | QoS 深度 | 描述                            |
| -------------------- | ------------------- | ---- | -------- | ------------------------------- |
| `/audio/capture_pcm` | `sensor_msgs/Image` | 发布 | 100      | 处理的麦克风 PCM（ANC / AEC / 原始） |


默认麦克风发布：**16 kHz，16-bit，单声道**，~64 ms 块（1024 帧）。仅当存在订阅者时发布。

`header.frame_id` 是运行的包计数器（十进制字符串）。`header.stamp` 在可用时与视频 PTS 对齐。

```bash
ros2 topic hz /audio/capture_pcm
ros2 topic echo /audio/capture_pcm --field encoding --once
```



### 7.2 扬声器（订阅）


| 话题                 | 类型                | 方向 | QoS                     | 描述                   |
| -------------------- | ------------------- | ---- | ----------------------- | ---------------------- |
| `/audio/speaker_pcm` | `sensor_msgs/Image` | 订阅 | KeepLast(10) + reliable | 在扬声器上播放的 PCM    |


默认播放格式：**48 kHz，16-bit，立体声** → 期望 `encoding=pcm_48000_16_2`。建议 ~60 ms 块以匹配扬声器队列。

可选相关话题：`/audio/speaker_pcm_status`（`std_msgs/Int32` 播放完成），`/audio/speaker_pcm_interrupt`，`/audio/speaker_control`。

```bash
ros2 topic pub /audio/speaker_pcm sensor_msgs/msg/Image \
  "{header: {frame_id: '0'}, height: 1, width: 5760, encoding: 'pcm_48000_16_2', is_bigendian: false, step: 5760, data: []}"
```

---



## 8. LCD 表情

从表情资源驱动面部 LCD（配置的图像目录下的 `mini_<index>`，默认 `/app/joyin/config/emoji`），或在视频模式下从 NV12 视频帧驱动。


| 话题              | 类型                | 方向 | QoS          | 描述                                   |
| ----------------- | ------------------- | ---- | ------------ | -------------------------------------- |
| `/emoji/index`    | `std_msgs/Int32`    | 订阅 | KeepLast(10) | 在 LCD 上显示的表情/动画索引           |
| `/lcd/video/nv12` | `sensor_msgs/Image` | 订阅 | KeepLast(10) | NV12 视频帧 → RGB565 blit（仅视频模式） |




### 8.1 `/emoji/index`


| 字段   | 备注                                                                  |
| ------ | --------------------------------------------------------------------- |
| `data` | 整数索引 **0–71**，映射到资源 `mini_<index>`（GIF/PNG/JPEG 或 Lottie） |


当 LCD 未启动或处于视频模式（`/lcd_disable`）时丢弃。事件驱动（无固定速率），默认索引值为 59。

```bash
ros2 topic pub /emoji/index std_msgs/msg/Int32 "{data: 0}" --once
ros2 topic pub /emoji/index std_msgs/msg/Int32 "{data: 59}" --once
```



### 8.2 `/lcd/video/nv12`

首先进入视频模式（`/lcd_disable` 停止表情并启用视频）。帧被转换为 RGB565 并写入面板。使用 `/lcd_enable` 退出以恢复表情。


| 字段       | 要求                                       |
| ---------- | ------------------------------------------ |
| `encoding` | `nv12` 或 `NV12`                           |
| `width`    | 偶数，> 0                                  |
| `height`   | 偶数，> 0                                  |
| `step`     | ≥ `width`（Y / UV 平面步幅）               |
| `data`     | 连续 `Y + UV`，大小 ≥ `step * height * 3 / 2` |


无效的编码、奇数大小或过小的 `data` → 帧丢弃（WARN 日志）。

```bash
ros2 service call /lcd_disable std_srvs/srv/Trigger 
# 发布 NV12 帧到 /lcd/video/nv12（encoding=nv12，偶数 WxH）
ros2 service call /lcd_enable std_srvs/srv/Trigger {}   # 恢复表情
```

相关（可选）：`/lcd_enable`，`/lcd_disable`。

---



## 9. 关节和电机本体感觉

20 个舵机（`ARTHROSIS_NUM_TOTAL = 20`）。主关节状态：`/HardwareKinematic_topic`。

**关节定义：**


| 索引    | 关节名称                                              |
| ------- | ----------------------------------------------------- |
| 0 - 3   | 左肩俯仰、左肩侧摆、左肘侧摆、左夹爪（未使用，设为0）        |
| 4 - 7   | 右肩俯仰、右肩侧摆、右肘侧摆、右夹爪（未使用，设为0）        |
| 8 - 13  | 左髋偏航、侧摆、俯仰、左膝俯仰、左踝俯仰、踝侧摆      |
| 14 - 19 | 右髋偏航、侧摆、俯仰、右膝俯仰、右踝俯仰、踝侧摆      |




### 反馈


| 话题                          | 类型                       | 描述                                                |
| ----------------------------- | -------------------------- | --------------------------------------------------- |
| `/HardwareKinematic_topic`    | `hardware/KinematicState`  | IMU（仅作为占位符）+ 20 个关节角度（度）+ 速度      |
| `/HardwareMotorLoad_topic`    | `std_msgs/Int16MultiArray` | 负载（22）                                          |
| `/HardwareMotorCurrent_topic` | `std_msgs/Int16MultiArray` | 电流（22）                                          |
| `/HardwareMotorTemp_topic`    | `std_msgs/UInt8MultiArray` | 温度（22）                                          |
| `/HardwareMotorErr_topic`     | `std_msgs/UInt8MultiArray` | 故障代码（22）                                      |




### 控制（发布到这些话题）


| 话题                          | 类型                         | 载荷                         |
| ----------------------------- | ---------------------------- | ---------------------------- |
| `/ControlMotorAngles_topic`   | `std_msgs/Float32MultiArray` | 20 个角度，度                |
| `/ControlMotorSpeed_topic`    | `std_msgs/Int16MultiArray`   | 20 个最大速度限制（舵机 ticks/s） |
| `/ControlMotorRangeSet_topic` | `std_msgs/Int16MultiArray`   | 20 个限制                    |
| `/ControlEnableMotor_topic`   | `std_msgs/UInt8MultiArray`   | 1–2 字节                     |
| `/ControlPidControl_topic`    | `control/PidControl`         | `{index, kp, ki, kd}`        |
| `/ControlGetPid_topic`        | `std_msgs/UInt8`             | 请求 PID 回读                |


**示例用法：**

**`/ControlMotorAngles_topic`** — 发布 20 个关节角度（度）：

```python
# 示例来自 control_system.py: publish_joint_angles_command()
msg = Float32MultiArray()
msg.data = [0.0] * 20

# 索引 0-3：左肩俯仰/侧摆、肘侧摆、夹爪（未使用，设为 0）
# 索引 4-7：右肩俯仰/侧摆、肘侧摆、夹爪（未使用，设为 0）
# 索引 8-13：左髋偏航/侧摆/俯仰、膝俯仰、踝俯仰、踝侧摆
# 索引 14-19：右髋偏航/侧摆/俯仰、膝俯仰、踝俯仰、踝侧摆

# 从标准形式关节角度（弧度）转换为原始电机角度（度）
for i in range(8):
    msg.data[i] = math.degrees(target_position_arm[i] + offset_angles_arm[i]) * dir_trans_arm[i]

for i in range(12):
    msg.data[i+8] = math.degrees(target_position[i] + offset_angles[i]) * dir_trans[i]

joint_angles_pub.publish(msg)
```

参见 [control_system.py](https://github.com/ZerothOpenBridge/Zeroth-M1_EDU_Deploy/blob/main/JOYIN_MINI_DEPLOY/control_system.py)，获取包含偏移/方向转换的完整 RL 部署示例。

**`/ControlMotorSpeed_topic`** — 设置所有 20 个关节的最大速度限制：

```python
# 示例来自 control_system.py: publish_joint_velocity_value_command()
msg = Int16MultiArray()
msg.data = [0] * 20

# 为所有 20 个关节设置相同的速度限制
# 值表示最大角速度，以舵机编码器 ticks/s 为单位
# 典型范围：0-4096（例如，4090 接近最大速度）
for i in range(20):
    msg.data[i] = velocity_value  # 例如，4090 用于高速运动

joint_velocitys_pub.publish(msg)
```

这设置了运动命令的速度上限。较低的值产生较慢、更受控的运动；较高的值允许更快的过渡。该值以舵机编码器 ticks/s 为单位，其中 4090 表示接近最大速度能力（默认）。

参见 [control_system.py](https://github.com/ZerothOpenBridge/Zeroth-M1_EDU_Deploy/blob/main/JOYIN_MINI_DEPLOY/control_system.py) `publish_joint_velocity_value_command()` 了解实现。
 


**`/ControlEnableMotor_topic` 载荷：**

`data[0]` — 整体扭矩控制：
- `0x00`：禁用所有关节的扭矩输出
- `0x01`：启用扭矩输出
- `0x80`：将当前位置设为中心值（仅在校准流程中使用，其他情况下不要发送）

`data[1]` — 单部位使能掩码（位字段，`0 = 禁用`，`1 = 启用`）：
- `bit0`：左臂
- `bit1`：右臂
- `bit2`：头部
- `bit3`：腿部
- `bit4`：头偏航
- `bit5`：头俯仰
- `bit6`：跌倒保护自动释放（`0 = 跌倒检测时启用自动释放`，`1 = 禁用`）
- `bit7`：头偏航自动卸力（`0 = 启用`，`1 = 关闭`）

示例：只禁用左臂。在 SSH 中执行：

```bash
ros2 topic pub /ControlEnableMotor_topic std_msgs/UInt8MultiArray "data: [1, 62]" --once
```

此时使用的掩码为 `0b00111110 = 62`：

```text
0b00111110 = 62
  ││││││└─ bit0 = 0（左臂禁用）
  │││││└── bit1 = 1（右臂启用）
  ││││└─── bit2 = 1（头部启用）
  │││└──── bit3 = 1（腿部启用）
  ││└───── bit4 = 1（头偏航启用）
  │└────── bit5 = 1（头俯仰启用）
  └─────── bit6 = 0（保留 / 未使用）
```

说明：
- `data[0]` 中写 `0` 表示禁用所有关节。
- `data[0]` 中写 `1` 表示启用扭矩输出；若要控制某些部位，需通过第二个字节 `data[1]` 的位掩码来选择。

**`/ControlPidControl_topic`** — 为单个关节配置 PID 参数：

```python
# 示例来自 control_system.py: publish_joint_pid_ctrl_command()
msg = PidControl()

# 为所有 20 个关节单独设置 PID 增益
# 每个关节需要一个带有其索引的单独消息
for i in range(20):
    msg.index = i    # 关节索引（0-19）
    msg.kp = kp      # 比例增益
    msg.ki = ki      # 积分增益
    msg.kd = kd      # 微分增益
    joint_pid_control_pub.publish(msg)
    time.sleep(0.002)  # 消息之间的小延迟
```

**重要：** PID 调整影响舵机响应特性。最初使用保守值，逐步调整。每个关节按顺序接收自己的 PID 消息，有小延迟以避免压垮 MCU 消息队列。

参见 [control_system.py](https://github.com/ZerothOpenBridge/Zeroth-M1_EDU_Deploy/blob/main/JOYIN_MINI_DEPLOY/control_system.py) `publish_joint_pid_ctrl_command()` 了解实现。


**机械限位（原始角度）：**


| 关节               | 范围（度）  |
| ------------------ | ----------- |
| 左/右肩俯仰        | -165 ~ 165  |
| 左肩侧摆           | -38 ~ 100   |
| 右肩侧摆           | -100 ~ 38   |
| 左肘侧摆           | -40 ~ 100   |
| 右肘侧摆           | -100 ~ 40   |
| 左/右髋偏航        | -18 ~ 18    |
| 左/右髋侧摆        | -20 ~ 20    |
| 左髋俯仰           | -160 ~ 50   |
| 右髋俯仰           | -50 ~ 160   |
| 左膝俯仰           | -155 ~ 60   |
| 右膝俯仰           | -60 ~ 155   |
| 左踝俯仰           | -70 ~ 145   |
| 右踝俯仰           | -145 ~ 70   |
| 头偏航             | -90 ~ 90    |
| 头俯仰             | -40 ~ 25    |


注意：超出这些范围可能导致舵机故障或损坏。M1 的关节原始角度的零点对应于蹲姿（用于校准）。在 RL 部署中，使用标准形式的关节角度，参见 [control_system.py](https://github.com/ZerothOpenBridge/Zeroth-M1_EDU_Deploy/blob/main/JOYIN_MINI_DEPLOY/control_system.py) 中的 `kinematic_state_feedback_msg_callback`。   

### 工厂动作


| 动作                          | 类型                   | 目标                        |
| ----------------------------- | ---------------------- | --------------------------- |
| `/HardwareMcuFctCheck_action` | `hardware/McuFctCheck` | `fct_check_mode: 0`（舵机测试） |


---



## 10. 电池


| 话题                 | 类型           | QoS |
| -------------------- | -------------- | --- |
| `/HardwareBat_topic` | `hardware/Bat` | 1   |



| 字段                                                       | 单位      |
| ---------------------------------------------------------- | --------- |
| `power`                                                    | % SOC     |
| `bat_vol`                                                  | mV        |
| `bat_cur`                                                  | mA        |
| `bat_temp`                                                 | —         |
| `plate_vol`                                                | mV        |
| `on_charger`, `is_charging`, `charge_full`, `charge_error` | 0/1 标志位 |


充电控制：`/BusinessEnableCharge_topic`（`std_msgs/UInt8`）

---



## 11. 车轮平台


| 话题                             | 类型                       | QoS                      |
| -------------------------------- | -------------------------- | ------------------------ |
| `/HardwareVehicleFeedback_topic` | `hardware/VehicleFeedback` | best_effort, KeepLast(1) |


`data[10]`：`[reserved, left_mile, right_mile, left_speed, right_speed, left_fault, right_fault, comm_status, left_current, right_current]`

`data[7]` 是 `comm_status`（原始 MCU 字段）。


| 话题                         | 方向 | 类型                                                         |
| ---------------------------- | ---- | ------------------------------------------------------------ |
| `/ControlVehicleSpeed_topic` | 订阅 | `std_msgs/Int16MultiArray` `[left, right]`（MCU 车轮命令） |


---



## 12. 头部控制


| 话题                          | 类型                       | 方向 | QoS                      | 描述                      |
| ----------------------------- | -------------------------- | ---- | ------------------------ | ------------------------- |
| `/HardwareHeadPosition_topic` | `hardware/HeadPosition`    | 发布 | best_effort, KeepLast(1) | 头部偏航/俯仰反馈         |
| `/ControlHeadControl_topic`   | `std_msgs/Int16MultiArray` | 订阅 | best_effort, KeepLast(1) | 头部偏航/俯仰命令（MCU控制） |


### 12.1 反馈 — `/HardwareHeadPosition_topic`

`hardware/HeadPosition.data`（长度 4，`int16`）：


| 索引 | 字段      | 含义         | 单位                              |
| ---- | --------- | ------------ | --------------------------------- |
| 0    | `h_angle` | 偏航（水平） | 0.01°（`degrees = data[0] / 100`） |
| 1    | `h_speed` | 偏航速度     | 与角度相同的比例（MCU 原始值）    |
| 2    | `v_angle` | 俯仰（垂直） | 0.01°（`degrees = data[2] / 100`） |
| 3    | `v_speed` | 俯仰速度     | 与角度相同的比例（MCU 原始值）    |


时间戳 MCU 路径被抽取（~每 10 个样本发布一次）。匹配发布者 QoS（`best_effort`）。

机械限位（度）：偏航 **-90 ~ 90**，俯仰 **-40 ~ 25**。


### 12.2 控制 — `/ControlHeadControl_topic`

MCU 要求**恰好 6** 个 `int16` 值（否则命令被丢弃）：


| 索引 | 字段        | 备注                                                          |
| ---- | ----------- | ------------------------------------------------------------- |
| 0    | yaw         | 目标偏航，**0.01°**（`int16`；例如 9000 = 90°）               |
| 1    | pitch       | 目标俯仰，**0.01°**                                           |
| 2    | yaw_speed   | 运动速度（MCU 原始 `int16`；典型非零默认值 ≈ 100）            |
| 3    | pitch_speed | 运动速度（MCU 原始 `int16`）                                  |
| 4    | reserved    | 设为 `0`                                                      |
| 5    | reserved    | 设为 `0`                                                      |


```bash
# 大致向前看（yaw=0，pitch=0）；许多 MCU 路径需要非零速度
# /ControlHeadControl_topic 需要叠加零点才能移动到正确位置，通过 /ControlHeadMotion_topic 发送0角度指令，echo/ControlHeadControl_topic 查阅在0角度时头部舵机的零点
# /ControlHeadMotion_topic 的单位为0.1°

ros2 topic pub /ControlHeadControl_topic std_msgs/msg/Int16MultiArray  "{data: [0, 0, 100, 100, 0, 0]}" --once

ros2 topic echo /HardwareHeadPosition_topic --once
```

> 一些高层应用仍然发布 `/ControlHeadMotion_topic`（通常 4 个元素）。该话题**不被** `hardware_mcu_plugin` 订阅。使用 `/ControlHeadControl_topic` 进行直接 MCU 头部控制。


---



## 13. 下视


| 话题                              | 类型                        | 方向 | QoS         | 描述                                 |
| --------------------------------- | --------------------------- | ---- | ----------- | ------------------------------------ |
| `/HardwareDownviewFeedback_topic` | `std_msgs/UInt32MultiArray` | 发布 | KeepLast(1) | 左/右向下范围（悬崖/地面感知）       |


`data` 长度 **2**：


| 索引 | 字段    | 单位 | 备注               |
| ---- | ------- | ---- | ------------------ |
| 0    | `left`  | mm   | 左下视距离         |
| 1    | `right` | mm   | 右下视距离         |


仅当 MCU 发送下视 UART 帧时发布（**事件驱动/按需**）。如果没有帧到达，话题可能保持沉默 — 不要仅将沉默视为 QoS 不匹配。

```bash
ros2 topic echo /HardwareDownviewFeedback_topic
```


---



## 14. 自定义消息定义

```
hardware/msg/Bat
  uint8 power; int16 bat_vol, bat_cur, bat_temp, plate_vol
  uint8 on_charger, is_charging, charge_full, charge_error

hardware/msg/KinematicState
  sensor_msgs/Imu imu_data
  float32[] joint_angles    # 20，度（MCU：motor_angle/100）
  int16[]   joint_velocities

hardware/msg/VehicleFeedback
  int32[] data

hardware/msg/HeadPosition
  std_msgs/Header header
  int16[] data              # [h_angle, h_speed, v_angle, v_speed]，0.01°

control/msg/PidControl
  uint8 index, kp, ki, kd
```

---



## 15. 集成示例



### 感知 / SLAM 节点

使用匹配的 QoS（`best_effort`，深度 1，在注明处）订阅：

```
/HardwareLidar_topic
/HardwareItof/depth_image
/HardwareItof/point_cloud
/HardwareImuBipedMotion_topic
```

提供静态 TF：`base_link` → `laser_frame`，`base_link` → `imu_link`，`base_link` → `itof_base_frame`。

### 相机 + 麦克风（VUI / 远程呈现）

```
/camera/image_jpeg              # 预览
/camera/image_h264              # 流编码
/camera/jpeg_high_resolution    # 高分辨率静态图像 / OCR
/audio/capture_pcm              # 麦克风 PCM（sensor_msgs/Image）
```



### 扬声器 + LCD

```bash
# 播放：发布 pcm_48000_16_2 块到 /audio/speaker_pcm
# 面部表情：
ros2 topic pub /emoji/index std_msgs/msg/Int32 "{data: 25}" --once
# 面部视频（NV12）：
ros2 service call /lcd_disable std_srvs/srv/Trigger {}
# → 发布 sensor_msgs/Image（encoding=nv12）到 /lcd/video/nv12
ros2 service call /lcd_enable std_srvs/srv/Trigger {}
```



### 遥控（在小车上）

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```



### 关节控制

```bash
# 20 个浮点值，单位为度
ros2 topic pub /ControlMotorAngles_topic std_msgs/msg/Float32MultiArray "{data: [...]}" -r 50
ros2 topic echo /HardwareKinematic_topic
```



### 头部控制

```bash
ros2 topic pub /ControlHeadControl_topic std_msgs/msg/Int16MultiArray \
  "{data: [0, 0, 100, 100, 0, 0]}" --once
ros2 topic echo /HardwareHeadPosition_topic
```

### 下视

```bash
# 可能会阻塞，直到 MCU 发出帧
ros2 topic echo /HardwareDownviewFeedback_topic --once
```

### 健康监控

```bash
ros2 param get /hardware hardware.Lidar.status
ros2 param get /hardware hardware.ToF.status
ros2 param get /hardware hardware.MCU.status
```

---



## 16. 单位和 QoS 速查表


| 数量             | 单位                     |
| ---------------- | ------------------------ |
| 关节角度         | 度                       |
| IMU 陀螺仪/加速度计 | rad/s，m/s²              |
| iTOF 深度        | mm（`16UC1`）            |
| 麦克风 PCM       | 16 kHz / 16-bit / 单声道 |
| 扬声器 PCM       | 48 kHz / 16-bit / 立体声 |
| 车轮控制         | mm/s                     |
| 头部角度         | 0.01°（线上）（`/100` → 度） |
| 下视范围         | mm                       |



| 话题                               | QoS                    |
| ---------------------------------- | ---------------------- |
| `/HardwareKinematic_topic`         | best_effort，深度 1    |
| `/HardwareVehicleFeedback_topic`   | best_effort，深度 1    |
| `/HardwareHeadPosition_topic`      | best_effort，深度 1    |
| `/ControlHeadControl_topic`        | best_effort，深度 1    |
| `/HardwareDownviewFeedback_topic`  | KeepLast(1)            |
| `/HardwareLidar_topic`             | reliable，深度 1       |
| `/HardwareImuBipedMotion_topic`    | best_effort，深度 1    |
| `/HardwareImuStd_topic`            | best_effort，深度 1    |
| `/camera/image_jpeg`               | KeepLast(1)            |
| `/camera/image_h264`               | KeepLast(1)            |
| `/camera/jpeg_high_resolution`     | KeepLast(1)            |
| `/audio/capture_pcm`               | 深度 100               |
| `/audio/speaker_pcm`               | reliable，KeepLast(10) |
| `/emoji/index`                     | KeepLast(10)           |
| `/lcd/video/nv12`                  | KeepLast(10)           |


---



## 17. 完整话题索引


| 传感器   | 发布                                                                                                                                       | 订阅 / 服务                                                                                                                                                     |
| -------- | ------------------------------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| IMU      | `/HardwareImuBipedMotion_topic`（~200 Hz，**推荐**），`/HardwareImuStd_topic`（~100 Hz，相同载荷/旧版），`/HardwareImuCal_topic` | `/ControlImuCal_topic`                                                                                                                                          |
| 关节     | `/ControlMotorAngles_topic`，`/ControlMotorSpeed_topic`，`/ControlMotorRangeSet_topic`，`/ControlEnableMotor_topic`，`/ControlPidControl_topic`，`/ControlGetPid_topic` | `/HardwareKinematic_topic`，`/HardwareMotorLoad_topic`，`/HardwareMotorCurrent_topic`，`/HardwareMotorTemp_topic`，`/HardwareMotorErr_topic` |
| LiDAR    | `/HardwareLidar_topic`                                                                                                                     | `/lidar_enable`，`/lidar_disable`                                                                                                                               |
| iTOF     | `/HardwareItof/point_cloud`，`/HardwareItof/depth_image`，`/HardwareItof/gray_image`，`/HardwareItof/camera_info`                         | `/itof_enable`，`/itof_disable`（占位符）                                                                                                                       |
| 相机     | `/camera/image_jpeg`，`/camera/image_h264`，`/camera/jpeg_high_resolution`                                                                 | —                                                                                                                                                               |
| 麦克风   | `/audio/capture_pcm`                                                                                                                       | —                                                                                                                                                               |
| 扬声器   | `/audio/speaker_pcm_status`（可选）                                                                                                        | `/audio/speaker_pcm`                                                                                                                                            |
| LCD      | —                                                                                                                                          | `/emoji/index`，`/lcd/video/nv12`，`/lcd_enable`，`/lcd_disable`                                                                                                |
| 电池     | `/HardwareBat_topic`                                                                                                                       | `/BusinessEnableCharge_topic`                                                                                                                                   |
| 车轮     | `/HardwareVehicleFeedback_topic`                                                                                                           | `/ControlVehicleSpeed_topic`                                                                                                                                    |
| 头部     | `/HardwareHeadPosition_topic`（`[h_angle, h_speed, v_angle, v_speed]`，0.01°）                                                             | `/ControlHeadControl_topic`（`[yaw, pitch, yaw_speed, pitch_speed, 0, 0]`，0.01°）                                                                               |
| 下视     | `/HardwareDownviewFeedback_topic`（`[left, right]` mm；事件驱动）                                                                          | —                                                                                                                                                               |
| 按键     | `/HardwareKey_topic`（`std_msgs/UInt16`），`/SystemConfigButton`                                                                           | —                                                                                                                                                               |
| 健康状态 | —                                                                                                                                          | `hardware.MCU.status`，`hardware.Lidar.status`，`hardware.ToF.status`                                                                                           |


---

