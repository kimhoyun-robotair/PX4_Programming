# 🚀 PX4_Programming

This repository contains example packages for controlling **PX4** using **MAVLink** and **ROS 2 offboard control**.

📚 It is recommended to read this repository alongside the documentation linked below:  
🔗 [Notion Documentation](https://juicy-beef-295.notion.site/PX4-1d04af2187bc80ef976cf3d4c5527ecf)

**Please note that all packages are developed targeting PX4 firmware version 1.15.**
---

## 📦 Included Packages

- **`precision_landing`**  
  A precision landing package using **YOLO** and **Aruco markers** with PX4 and ROS 2.

- **`px4_control_wgs84`**  
  Converts GPS coordinates given in **WGS84** to **local NED coordinates**, enabling long-range autonomous flights.  
  Supports **multicopter**, **fixed-wing**, and **VTOL**.

- **`px4_msgs`**  
  Interface message package required for controlling PX4 using ROS 2.

- **`px4_offboard_control`**  
  Contains various PX4 control examples implemented using **ROS 2**.

- **`px4-offboard`**  
  An official PX4 example that performs circular motion after takeoff using ROS 2-based PX4 control.

---

## 🛠️ Installation

Assumes you already have **PX4-Autopilot**, **QGroundControl**, and **micro XRCE-DDS** installed.

```bash
git clone --recursive https://github.com/kimhoyun-robotair/PX4_Programming.git
cd PX4_Programming
colcon build --symlink-install
source install/setup.bash
```

---

## ▶️ Usage
Packages with launch files:

- px4_control_wgs84
- precision_landing
- px4-offboard

These can be launched using standard ROS 2 launch commands.

Package without launch file:

- px4_offboard_control

This package is designed to be run directly via ros2 run.

Refer to the launch/ directory and the setup.py file to understand how to run each node.
