# 元点智能M1人形机器人 - 强化学习部署工程

[🌎 English](README.md) | 🇨🇳 中文

---

## 项目简介

本仓库包含**元点智能（Zeroth）M1 人形机器人的强化学习策略部署框架**。提供从仿真训练到真机部署的完整流程，支持 ONNX 格式的 RL 策略直接部署到机器人硬件上。

<div align="center">

| Isaac Gym 仿真 | 真机部署 |
|:---:|:---:|
| ![仿真](JOYIN_MINI_DEPLOY/demos/M1_walking_in_simulation.gif) | ![真机部署](JOYIN_MINI_DEPLOY/demos/M1_deploy.gif) |
| *Isaac Gym 中的策略训练* | *在 M1 硬件上部署* |

</div>

---

## 📁 文件说明

| 文件/目录 | 说明 |
|---|---|
| **demo_walk.py** | 行走演示程序 - RL 策略部署的主入口，实现机器人运动控制 |
| **control_system.py** | ROS2 控制系统类 - 订阅机器人传感器数据（IMU、关节状态）并发布关节控制指令 |
| **spin_control_system.py** | 观测数据回读示例程序 |
| **config_mini.yaml** | 机器人配置文件 - 包含默认关节角度、零位偏置、观测维度、缩放因子等参数 |
| **udp_sender.py** | 实时数据流类 - 通过 UDP 协议向 PlotJuggler 发送遥测数据 |
| **model/** | 存放训练好的 RL 策略模型（ONNX 格式） |
| **pylibs/** | 部署所需的 Python 依赖库 |
| **trajectory_replay.py** | 轨迹回放工具 - 用于对比仿真与真机的策略响应 |

---

## 🚀 部署指南

### 1. 克隆仓库

```bash
git clone <仓库地址>
cd JOYIN_MINI_DEPLOY
```

### 2. 连接机器人

将开发电脑与机器人 WiFi 连接到**同一局域网**，然后通过 SSH 登录机器人：

```bash
ssh <机器人用户名>@<机器人IP地址>
```


### 3. 启动部署
将训练好的 ONNX 模型放入 `model/` 目录，然后终端输入：
```bash
python demo_walk.py --model=model/your_model_name.onnx
```

---


### 个性化策略部署


**单帧观测组成：**
目前部署工程的策略输入为以下数据的拼接：

- 指令速度（vx, vy, yaw_rate）× `cmd_scale`
- IMU 角速度（3 轴）× `ang_vel_scale`
- 体坐标系重力向量 × `gravity_scale`
- 关节位置偏移 × `dof_pos_scale`
- 上一帧动作
- 步态相位编码（sin, cos）

---


### 观测堆叠

M1机器人使用**观测历史堆叠**（默认：5 帧）作为策略网络输入。堆叠后的观测会拼接成一个向量，送入 ONNX 模型进行动作推理。

**在 `config_mini.yaml`写入和训练相同的配置：**
包括：
```yaml
num_obs: <单帧观测维度>
num_stack: 5  # 堆叠帧数
stepping_period: <包含每条腿踏步的摆动相和触地相>
default_angles: <腿部各关节的默认角度>
default_angles_arm: <手臂各关节的默认角度>
num_actions: <策略使用到的关节数>
```
注意：config文件中的kp, kd 是机器人固有设定，更改会导致策略的部署效果恶化。
---

### 观测变量获取

- 速度指令: 在部署程序(demo_walk.py)中由 `cmd.vx`, `cmd.vy`, `cmd.dyaw` 给定
- IMU 角速度：control_system.py的state字典中 `imu_angular_velocity`定义，索引0,1,2分别对应roll, pitch, yaw 方向。
- 体坐标系重力投影：由state字典中`gravity_in_body`给定。
- 关节位置：腿部由state字典中`joint_position` 给定、手臂由`joint_position_arm`给定。
- 关节速度：腿部由state字典中`joint_velocity` 给定、手臂由`joint_velocity_arm`给定。
- 上一帧动作：部署程序(demo_walk.py)中`action`给定。
- 步态相位编码：部署程序(demo_walk.py)中的algorithm 类中`sin_phase`,`cos_phase`给定。


## 📝 备注

- 机器人策略网络默认以 **50Hz 控制频率**运行
- 支持通过 PlotJuggler 进行实时遥测可视化（参见 `udp_sender.py`）
- 如需轨迹回放和仿真对比验证，可使用 `trajectory_replay.py`

---

## 🎉 致谢

本仓库开发离不开以下开源项目的支持与贡献，特此感谢：

- **legged_gym**: 提供训练框架与环境搭建的核心代码库
- **rsl_rl**: 提供 PPO 等强化学习算法的高效实现
- **Isaac Gym**: GPU 加速的高性能并行物理仿真平台

---

## 📧 技术支持

如有技术问题或咨询，请联系**元点智能（Zeroth）**。

---

**许可证：** GNU General Public License v2.0 (GPL-2.0)

本项目采用 GPL-2.0 协议发布。相关版权声明请参考仓库根目录的 `LICENSE` 文件。

版权声明示例：

> Copyright (C) 2026 苏州乐享智能科技有限公司
> Copyright (C) 2026 Suzhou JoyIn Technology Co., Ltd.
>
> This program is free software; you can redistribute it and/or modify
> it under the terms of the GNU General Public License as published by
> the Free Software Foundation; either version 2 of the License, or
> (at your option) any later version.

注意：如果你计划将该工程分发给第三方，GPL-2.0 要求源码和许可证一并提供，并保留相关版权声明。
