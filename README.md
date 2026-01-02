# 🛡️ Robust RPLIDAR ROS 2 Driver (Industrial-Grade)

[![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-orange.svg?style=flat-square&logo=ros)](https://docs.ros.org/en/jazzy/)
[![C++17](https://img.shields.io/badge/C++-17-blue.svg?style=flat-square&logo=c%2B%2B)](https://isocpp.org/)
[![License](https://img.shields.io/badge/License-BSD--2--Clause-green.svg?style=flat-square)](https://opensource.org/licenses/BSD-2-Clause)
[![Build Status](https://img.shields.io/badge/Build-Passing-brightgreen.svg?style=flat-square)]()

> **"Because the official driver shouldn't crash just because you pulled the plug."**

This is a heavily refactored, **fault-tolerant** ROS 2 driver for Slamtec RPLIDAR. 
Designed with a **Lifecycle State Machine** and **Thread-Safe Architecture**, ensuring your robot keeps running even under hardware disconnection or permission failures.

---

## ⚡ Why Use This? (Table of Shame)

| Feature | Official Slamtec ROS 2 | **This Driver (frozenreboot)** |
| :--- | :---: | :---: |
| **Hot-plug Recovery** | ❌ Crash / Hang | **✅ Auto-reconnect via FSM** |
| **Permission Denied** | ❌ Silent Fail / Garbage Data | **✅ Explicit Diagnostics** |
| **Dynamic Reconfigure** | ❌ Restart Required | **✅ Runtime RPM/Mode Update** |
| **Zero-Copy Optimization**| ❌ N/A | **✅ Smart Pointer & Move Semantics** |
| **Architecture** | ❌ Tight SDK Coupling | **✅ Interface-based Abstraction** |

---

## 🧪 Call for Experiments: "Does it survive?"

I need your help to validate this driver on various robots! 
If you use this driver, please **stress-test** it (e.g., unplug USB while scanning, change RPM dynamically) and share your results.

### 📢 How to Submit a Report
Please open an issue with the title `[Experiment] Your_Robot_Name` and include:
1. **Lidar Model:** (e.g., A1, A2, S1...)
2. **Recovery Log:** (Copy paste the terminal output when you unplug/replug)
3. **Screenshot:** `rqt_graph` or `rviz2`

👉 [**Submit your Experiment Report Here**](https://github.com/frozenreboot/rplidar_ros2_driver/issues/new)

---

## 🚀 Getting Started

### 1. Installation
```bash
cd ~/ros2_ws/src
git clone https://github.com/frozenreboot/rplidar_ros2_driver.git
cd ..

# Install dependencies
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install
```

### 2. Quick Launch


```Bash
ros2 launch rplidar_ros2_driver rplidar.launch.py serial_port:=/dev/ttyUSB0
```

### 3. Dynamic Reconfigure (Runtime)

You can change the motor speed without killing the node:


```Bash

ros2 param set /rplidar_node rpm 1000
ros2 param set /rplidar_node scan_mode DenseBoost
```

---

## 🏗️ Architecture

This driver uses a **3-Layer Design** to decouple ROS 2 logic from the vendor SDK.

- **Node Layer:** Handles Lifecycle & Parameters.
    
- **Wrapper Layer:** Handles Threading & Mutex.
    
- **SDK Layer:** Raw data fetching.
    

![Architecture Diagram](./doc/architecture.png)

---

## 👤 Author & Maintainer

- **frozenreboot** - _Initial Refactoring & Architecture Design_
    
- Blog: [Tech Log](https://frozenreboot.github.io/)
    

Based on the original work by RoboPeak & Slamtec.

