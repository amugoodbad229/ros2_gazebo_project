# 🏎️ ROS 2 Jazzy Digital Twin: SLAM & Simulation

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue?logo=ros&logoColor=white)
![Ubuntu](https://img.shields.io/badge/OS-Ubuntu_24.04_(WSL)-E95420?logo=ubuntu&logoColor=white)
![Gazebo](https://img.shields.io/badge/Sim-Gazebo_Harmonic-orange)
![Build](https://img.shields.io/badge/Build-Passing-brightgreen)
![License](https://img.shields.io/badge/License-Apache_2.0-lightgrey)

> A complete **Digital Twin** simulation environment built for **ROS 2 Jazzy**. This project features a custom-built mobile robot equipped with Lidar, simulated in **Gazebo Harmonic**, and configured for real-time **SLAM** (Simultaneous Localization and Mapping) visualization in **RViz2**.

---

## 🌟 Features

*   **🤖 Custom Robot Model:** A physics-compliant differential drive robot (SDF) with inertia, collision, and wheel joints.
*   **🗺️ Real-Time SLAM:** Fully integrated `slam_toolbox` for asynchronous mapping.
*   **🏗️ Gazebo Harmonic:** Leverages the latest Gazebo (GZ) physics engine with `ros_gz_bridge`.
*   **📡 Sensor Fusion:** Lidar point clouds, Odometry, and TF transforms bridged seamlessly to ROS 2.
*   **🪟 WSL 2 Optimized:** Specifically tuned to run on Windows 11 via Windows Subsystem for Linux without GUI lag.

---

## 🛠️ Prerequisites

Before running the simulation, ensure you have the following installed:

*   **Operating System:** Ubuntu 24.04 (Noble Numbat)
*   **ROS Distribution:** [ROS 2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation.html)
*   **Environment:** Native Ubuntu or WSL 2 (Windows 11).

---

## 🚀 Installation Guide

### 1. Install Project Dependencies
Ensure all necessary simulation and navigation packages are installed on your system.

```bash
sudo apt update
sudo apt install -y ros-jazzy-desktop \
ros-jazzy-ros-gz \
ros-jazzy-slam-toolbox \
ros-jazzy-navigation2 \
ros-jazzy-nav2-bringup \
ros-jazzy-robot-state-publisher \
ros-jazzy-xacro
```

### 2. Clone the Repository
Create a workspace and clone this project.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/amugoodbad229/ros2_gazebo_project.git .
```

### 3. Build and Source
Compile the package using `colcon`.

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 🎮 How to Run

To run the simulation, you will need **two terminal windows**.

### Terminal 1: The Digital Twin
This launches the simulation environment, the robot, the bridge, SLAM, and the visualization tools.

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch slam_research_lab digital_twin.launch.py
```
> **What to expect:** Gazebo will open showing the robot in a research world. RViz2 will open showing the grid.

### Terminal 2: Robot Control
Open a new terminal to drive the robot using your keyboard.

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Controls:**
*   `i` : Move Forward
*   `j` : Turn Left
*   `l` : Turn Right
*   `k` : Stop

> **✨ Magic Moment:** The map in RViz will not appear immediately. **Drive the robot forward** slightly, and SLAM will begin building the map in real-time.

---

## 📂 Project Structure

```bash
~/ros2_ws/src/slam_research_lab
├── config
│   └── slam_params.yaml       # SLAM Toolbox configuration
├── launch
│   └── digital_twin.launch.py # Main launch file (Sim + Bridge + Nodes)
├── models
│   └── research_bot.sdf       # Robot description (Lidar, Wheels, Physics)
├── worlds
│   └── research_world.sdf     # Simulation environment
├── package.xml                # Dependencies
└── setup.py                   # Python package setup
```

---

## 🐛 Troubleshooting

### 1. RViz Topics are Empty (WSL 2 Users)
If you are running on WSL and do not see `/lidar` or `/map` in RViz, the Zenoh middleware might be failing to discover local topics. Revert to standard DDS for the session:

```bash
# Run this before the launch command
unset RMW_IMPLEMENTATION
ros2 launch slam_research_lab digital_twin.launch.py
```

### 2. "Package not found" Error
If ROS cannot find `slam_research_lab`, it means the local workspace isn't sourced.

```bash
# Run this inside ~/ros2_ws
source install/setup.bash
```

### 3. Map is not appearing
The map topic `/map` is only published **after** the Lidar detects changes in the environment. Ensure you drive the robot (`i` key) for at least 1 meter to initialize the map.
