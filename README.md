# 🛡️ Robust RPLIDAR ROS 2 Driver

[![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg?style=flat-square&logo=ros)](https://docs.ros.org/en/humble/)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-orange.svg?style=flat-square&logo=ros)](https://docs.ros.org/en/jazzy/)
[![ROS 2 Lyrical](https://img.shields.io/badge/ROS2-Lyrical-blueviolet.svg?style=flat-square&logo=ros)](https://docs.ros.org/en/lyrical/)
[![ROS 2 Rolling](https://img.shields.io/badge/ROS2-Rolling-lightgrey.svg?style=flat-square&logo=ros)](https://docs.ros.org/en/rolling/)
[![C++17](https://img.shields.io/badge/C++-17-blue.svg?style=flat-square&logo=c%2B%2B)](https://isocpp.org/)
[![License](https://img.shields.io/badge/License-BSD--2--Clause-green.svg?style=flat-square)](https://opensource.org/licenses/BSD-2-Clause)
[![ROS 2 CI](https://github.com/frozenreboot/rplidar_driver/actions/workflows/ros2_ci.yaml/badge.svg)](https://github.com/frozenreboot/rplidar_driver/actions/workflows/ros2_ci.yaml)

> **"Because the official driver shouldn't crash just because you pulled the plug."**

A **fault-tolerant** ROS 2 driver for Slamtec RPLIDAR, built around a
**Lifecycle State Machine** and a **thread-safe architecture**. Your robot
keeps running even under hardware disconnection or permission failures.

---

## ⚡ Why Use This?

Most existing RPLIDAR drivers assume the hardware never misbehaves. This
driver assumes the opposite. Compared to the reference implementation:

| Capability | Reference Driver | **This Driver** |
| :--- | :---: | :---: |
| **Hot-plug recovery** | Not handled | ✅ Auto-reconnect via lifecycle FSM |
| **Permission failure reporting** | Silent failure | ✅ Explicit diagnostics |
| **Runtime reconfiguration** | Restart required | ✅ Live RPM / scan-mode update |
| **Component composition** | Standalone only | ✅ `rclcpp_components` support |
| **SDK coupling** | Direct calls throughout | ✅ Interface-based abstraction layer |

---

## 🚀 Getting Started

### Option A — Binary install *(coming soon)*

This package has been submitted to the ROS 2 build farm. Once it syncs, you
will be able to install it directly:

```bash
sudo apt install ros-${ROS_DISTRO}-rplidar-driver
```

Supported distros: **Humble**, **Jazzy**, **Lyrical**, and **Rolling**.

### Option B — Build from source

```bash
cd ~/ros2_ws/src
git clone https://github.com/frozenreboot/rplidar_driver.git
cd ..

# Install dependencies
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install
```

### Quick Launch

```bash
ros2 launch rplidar_driver rplidar.launch.py serial_port:=/dev/ttyUSB0
```

### Dynamic Reconfigure (Runtime)

You can change the motor speed without killing the node:

```bash
ros2 param set /rplidar_node rpm 1000
ros2 param set /rplidar_node scan_mode DenseBoost
```

### Standby (Motor Control)

Park the motor without leaving the ACTIVE state. The serial connection, the
scan thread and the publishers all stay up, so waking costs only the motor
spin-up time:

```bash
ros2 service call /stop_motor std_srvs/srv/Trigger
ros2 service call /start_motor std_srvs/srv/Trigger
```

Or let the driver follow demand. With `auto_standby` the motor stops while
nobody subscribes to `scan` (or to `cloud`, when `publish_point_cloud` is
enabled) and restarts as soon as somebody does:

```bash
ros2 param set /rplidar_node auto_standby true
```

While `auto_standby` is enabled the two services are rejected, so the
subscriber count stays the single source of truth.

Diagnostics say *why* the motor is idle, which is usually the question:

```bash
ros2 topic echo /diagnostics    # "Standby (auto: no subscribers)" vs "Standby (motor off)"
```

---

## 🧪 Call for Experiments: "Does it survive?"

We need your help to validate this driver on various robots!
If you use this driver, please **stress-test** it (e.g., unplug USB while
scanning, change RPM dynamically) and share your results.

### 📢 How to Submit a Report
Please open an issue with the title `[Experiment] Your_Robot_Name` and include:
1. **Lidar Model:** (e.g., A1, A2, S1...)
2. **Recovery Log:** (Copy paste the terminal output when you unplug/replug)
3. **Screenshot:** `rqt_graph` or `rviz2`

👉 [**Submit your Experiment Report Here**](https://github.com/frozenreboot/rplidar_driver/issues/new)

---

## 🏗️ Architecture

This driver uses a **3-Layer Design** to decouple ROS 2 logic from the vendor SDK.

- **Node Layer:** Handles Lifecycle & Parameters.
- **Wrapper Layer:** Handles Threading & Mutex.
- **SDK Layer:** Raw data fetching.

![Architecture Diagram](./doc/architecture.png)

---

## 👥 Maintainers

- **frozenreboot** — Architecture & core development · [Tech Log](https://frozenreboot.github.io/)
- **cosmicog** ([@cosmicog](https://github.com/cosmicog)) — Co-maintainer
- **Błażej Sowa** ([@bjsowa](https://github.com/bjsowa)) — Release & packaging

Based on the original work by RoboPeak & Slamtec.

---

## 🤖 AI-Assisted Development Disclosure

In compliance with the [OSRF Policy on the Use of Generative AI in Contributions](https://github.com/openrobotics/osrf-policies-and-procedures/blob/main/OSRF%20Policy%20on%20the%20Use%20of%20Generative%20Tools%20(%E2%80%9CGenerative%20AI%E2%80%9D)%20in%20Contributions.md) (Effective May 2025), we explicitly disclose the use of Generative AI tools in the development of this driver.

* **Tools Used:**
    * **Anthropic Claude:** Release engineering support, refactoring suggestions, and documentation.
    * **Google Gemini:** Boilerplate code generation and test infrastructure scaffolding.
    * **OpenAI ChatGPT:** Refactoring suggestions and documentation formatting.
* **Verification:**
    * All AI-generated content has been **manually reviewed, tested, and verified** by the maintainers.
    * The logic, memory safety, and ROS 2 lifecycle state transitions have been rigorously checked to ensure system stability.
* **Note:**
    * Contributions follow the `Generated-by:` tag convention in commit messages as per the OSRF guidelines.
