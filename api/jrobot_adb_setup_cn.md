# JRobot ADB 环境配置指南

面向二次开发人员的主机端 ADB 驱动与工具安装说明。配置完成后，可通过 USB 登录机器人并进入开发目录。

| 项目 | 说明 |
|------|------|
| 文档版本 | **v0.1** |
| 适用平台 | Windows / Linux |
| 资源目录 | `tool/res/` |
| 设备工作目录 | `/userdata/developer/` |

相关文档：传感器 ROS 2 接口见 [`readme.md`](./readme.md)。

---

## 版本变更

| 版本 | 日期 | 变更说明 |
|------|------|----------|
| **v0.1** | 2026-07-17 | 初版：整理 Windows / Linux 主机端 ADB 驱动与工具安装步骤，供极客二次开发使用。 |

---

## 快速验证

安装完成后，USB Type-C 连接机器人，在主机执行：

```bash
adb devices
adb shell
cd /userdata/developer/
```

若 `adb devices` 能列出设备序列号且状态为 `device`，说明环境配置成功。

---

## 1. Windows：ADB 与驱动安装

### 1.1 安装 USB 驱动

资源包：`res/sunrise5_winusb.zip`

1. 解压 `sunrise5_winusb.zip`
2. **以管理员身份**运行解压目录中的 `install_driver.bat`
3. 按提示完成驱动安装

### 1.2 安装 ADB 工具包

资源包：`res/platform-tools-latest-windows.zip`

1. 解压 `platform-tools-latest-windows.zip` 到本地目录（例如 `C:\platform-tools`）
2. 打开 **系统属性 → 高级 → 环境变量**
3. 将该目录路径加入用户或系统的 `Path`
4. 打开命令提示符（CMD）或 PowerShell，验证：

```bat
adb --version
```

能正常输出版本号即表示工具包配置成功。

---

## 2. Linux：ADB 与 USB 规则配置

### 2.1 安装 ADB

```bash
sudo apt update
sudo apt install adb
```

### 2.2 配置 USB udev 规则

规则文件：`res/51-drobotics-usb.rules`

```bash
# 拷贝 udev 规则（在 tool 目录下执行）
sudo cp res/51-drobotics-usb.rules /etc/udev/rules.d/

# 使能新规则
sudo udevadm control --reload-rules && sudo udevadm trigger

# 将当前用户加入 plugdev 组
sudo usermod -aG plugdev $(whoami)
```

重新登录 shell 后，用 `groups` 确认当前用户已在 `plugdev` 组中。

### 2.3 重启 ADB 服务

```bash
adb kill-server
adb start-server
adb devices
```

---

## 3. 登录设备

1. 使用 USB 线连接主机与机器人 **Type-C** 接口
2. 在主机执行：

```bash
adb devices          # 确认设备可见
adb shell            # 进入设备 shell
cd /userdata/developer/   # 二次开发工作目录
```

可在该目录部署二进制、配置文件及二次开发 demo。

---

## 注意事项

| 问题 | 处理建议 |
|------|----------|
| `adb devices` 无设备 | 检查 USB 线 / Type-C 口；Windows 确认驱动已安装；Linux 确认 udev 规则与 `plugdev` 组 |
| Linux 权限不足 | 执行 `sudo usermod -aG plugdev $(whoami)` 后**重新登录**再试 |
| `unauthorized` | 在设备端确认 ADB 授权；必要时重启 `adbd` 或重新插拔 USB |
| 多设备冲突 | 使用 `adb -s <serial> shell` 指定目标设备 |

---

## 资源清单

| 文件 | 用途 |
|------|------|
| `res/sunrise5_winusb.zip` | Windows USB 驱动安装包 |
| `res/platform-tools-latest-windows.zip` | Windows ADB / platform-tools |
| `res/51-drobotics-usb.rules` | Linux udev USB 权限规则 |
