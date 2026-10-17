# JRobot ADB Setup Guide

Host-side ADB driver and toolchain setup for secondary development on the Joyin robot. After setup, connect over USB, open a shell, and work from the on-device developer workspace.


| Item               | Value                 |
| ------------------ | --------------------- |
| Document version   | **v0.0.1**            |
| Supported hosts    | Windows / Linux / IOS |
| Resource directory | `res/`                |

---

## Version Changelog


| Version    | Date       | Changes                                                                                  |
| ---------- | ---------- | ---------------------------------------------------------------------------------------- |
| **v0.0.1** | 2026-07-17 | Initial ADB setup guide for Windows and Linux hosts, for overseas secondary development. |


---

## Quick Check

After installation, connect the robot via USB Type-C and run on the host:

```bash
adb devices
adb shell
cd /userdata/developer/
```

If `adb devices` lists a serial number with state `device`, the setup is ready.

---

## 1. Windows: ADB and Driver Install

### 1.1 Install the USB driver

Package: `res/sunrise5_winusb.zip`

1. Extract `sunrise5_winusb.zip`
2. Run `install_driver.bat` **as Administrator**
3. Complete the installer prompts

### 1.2 Install the ADB toolchain

Package: `res/platform-tools-latest-windows.zip`

1. Extract `platform-tools-latest-windows.zip` (e.g. `C:\platform-tools`)
2. Open **System Properties → Advanced → Environment Variables**
3. Add that directory to the user or system `Path`
4. Open CMD or PowerShell and verify:

```bat
adb --version
```

A valid version string means the toolchain is configured correctly.

---

## 2. Linux/IOS: ADB and USB Rules

### 2.1 Install ADB

```bash
sudo apt update
sudo apt install adb
```

### 2.2 Configure USB udev rules

Rules file: `res/51-drobotics-usb.rules`

```bash
# Copy udev rules (run from the tool directory)
sudo cp res/51-drobotics-usb.rules /etc/udev/rules.d/

# Reload rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# Add your user to the plugdev group
sudo usermod -aG plugdev $(whoami)
```

Log out and back in (or open a new shell), then run `groups` and confirm `plugdev` is listed.

### 2.3 Restart the ADB server

```bash
adb kill-server
adb start-server
adb devices
```

---

## 3. Log into the Device

1. Connect the host PC to the robot **Type-C** port with a USB cable
2. On the host:

```bash
adb devices          # confirm the device is visible
adb shell            # open a shell on the device
cd /userdata/developer/   # secondary-development workspace
```

Use this directory to deploy binaries, configs, and demos on the device.

---

## Troubleshooting


| Symptom                    | What to do                                                                                                                     |
| -------------------------- | ------------------------------------------------------------------------------------------------------------------------------ |
| No device in `adb devices` | Check the USB cable / Type-C port; on Windows confirm the driver is installed; on Linux confirm udev rules and `plugdev` group |
| Linux permission denied    | Run `sudo usermod -aG plugdev $(whoami)`, then **re-login** and retry                                                          |
| State `unauthorized`       | Accept ADB authorization on the device; restart `adbd` or re-plug USB if needed                                                |
| Multiple devices           | Use `adb -s <serial> shell` to select the target                                                                               |


---

## Resource List


| File                                    | Purpose                         |
| --------------------------------------- | ------------------------------- |
| `res/sunrise5_winusb.zip`               | Windows USB driver installer    |
| `res/platform-tools-latest-windows.zip` | Windows ADB / platform-tools    |
| `res/51-drobotics-usb.rules`            | Linux udev USB permission rules |


