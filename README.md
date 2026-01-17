# ROS 2 + Gazebo Project

A lightweight Python-based ROS 2 project for running robot nodes inside Gazebo simulations. This repository provides packages, launch files, and helper scripts to run simulations and robot bringups. Some components optionally use Zenoh for efficient bridging / mesh networking — Zenoh is optional and can be disabled if it causes instability.

Maintainer: amugoodbad229

---

## Table of contents

- [Overview](#overview)
- [Status](#status)
- [Requirements](#requirements)
- [Quickstart](#quickstart)
- [Build](#build)
- [Running the simulation](#running-the-simulation)
- [Disable Zenoh (safe options)](#disable-zenoh-safe-options)
- [Automatic fallback strategies (recommended)](#automatic-fallback-strategies-recommended)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

This repository contains ROS 2 Python packages and Gazebo integration to simulate and test robot behaviors locally. It is intended for development and small-scale simulation use. Zenoh may be used by certain nodes for optimized transport bridging; however, the project is designed so Zenoh can be disabled without breaking the rest of the system.

## Status

- Language: Python (100%)
- Basic simulation & launch scaffolding present (see `launch/` and `src/`).
- Zenoh integration is optional; see the "Disable Zenoh" section for safe options.

## Requirements

- Ubuntu 24.04 
- ROS 2 jazzy
- Gazebo 
- Python 3.8+
- colcon build tool
- (Optional) Zenoh libraries & Python bindings, only needed if you enable Zenoh features

## Quickstart

1. Clone the repository:
   ```bash
   git clone https://github.com/amugoodbad229/ros2_gazebo_project.git
   cd ros2_gazebo_project
   ```

2. Install ROS 2 and Gazebo following official instructions for your distro.

3. Optional Python deps:
   ```bash
   python3 -m pip install -r requirements.txt || true
   ```

4. Install package dependencies:
   ```bash
   sudo apt update
   rosdep install --from-paths src --ignore-src -r -y
   ```

## Build

Source your ROS 2 distribution and build:
```bash
source /opt/ros/<ros-distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
```
Replace `<ros-distro>` with your distro name (e.g., `jazzy`).

## Running the simulation

Typical usage examples (replace `<package>` and `<launch>` with actual names in this repo):

- With Zenoh enabled (default):
  ```bash
  source install/setup.bash
  ros2 launch <package> bringup.launch.py
  ```

- With Zenoh disabled (runtime parameter — if supported by the launch file):
  ```bash
  ros2 launch <package> bringup.launch.py use_zenoh:=false
  ```

If your repository's launch files don't yet expose a `use_zenoh` argument, see "Automatic fallback strategies" below for an implementation pattern.

## Disable Zenoh (safe options)

If Zenoh causes crashes, segfaults, or network instability, disable it using one of these safe approaches:

1) Runtime launch argument (preferred)
- Many launch files accept a boolean argument such as `use_zenoh` or `enable_zenoh`. Launch with:
  ```bash
  ros2 launch <package> bringup.launch.py use_zenoh:=false
  ```

2) Environment variable
- Set an environment variable before launching:
  ```bash
  export USE_ZENOH=false
  ros2 launch <package> bringup.launch.py
  ```
- Or one-liner:
  ```bash
  USE_ZENOH=false ros2 launch <package> bringup.launch.py
  ```

3) Edit launch/config files (persistent change)
- Edit `launch/*.py` or YAML config files and change the Zenoh-related parameter to `false`, then rebuild.

4) Remove/comment out Zenoh nodes
- If a dedicated Zenoh bridge node exists in a launch file (e.g., `zenoh_bridge`), comment it out as a temporary workaround.

## Automatic fallback strategies (recommended)

Make the code resilient by adding a runtime check and fallback. Two recommended patterns:

A) Launch-file argument (Python launch example)
```python
# launch/bringup.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    use_zenoh = LaunchConfiguration('use_zenoh', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_zenoh', default_value='true',
                               description='Enable Zenoh transport'),

        # Example node that only launches when use_zenoh is true
        Node(
            package='my_package',
            executable='zenoh_bridge_node',
            name='zenoh_bridge',
            condition=IfCondition(use_zenoh),
            parameters=[{'some_param': 1}]
        ),

        # Other nodes...
    ])
```

B) Node-level import fallback (Python runtime)
```python
# src/my_package/zenoh_helper.py
import os
import logging

logger = logging.getLogger('zenoh-helper')
USE_ZENOH = os.getenv('USE_ZENOH', 'true').lower() not in ('0', 'false', 'no')

zenoh_available = False
if USE_ZENOH:
    try:
        import zenoh  # or the specific zenoh Python binding this project uses
        zenoh_available = True
    except Exception as e:
        logger.warning("Zenoh disabled due to import error: %s", e)
        zenoh_available = False

def start_transport():
    if zenoh_available:
        # init zenoh transport
        pass
    else:
        # fallback to ROS2 DDS topics or local sockets
        pass
```

These patterns avoid runtime crashes when native bindings or system libs for Zenoh are incompatible.

## Troubleshooting

Symptoms Zenoh is causing trouble:
- Process crash / segfault referencing `zenoh` or `libzenoh`
- ImportError for zenoh Python bindings
- Network timeouts or message loss when Zenoh is enabled

What to try:
- Disable Zenoh via one of the safe options above to confirm the rest of the system works
- Check versions of Zenoh native libs and Python bindings; reinstall to match expected version
- Inspect logs with ROS 2 logging levels (e.g., `--ros-args --log-level DEBUG`)
- Verify environment variables (e.g., `ROS_DOMAIN_ID`) and network config if running distributed nodes
- If segfaults persist, run under gdb or capture a core dump and check native dependency mismatches

## Contributing

Contributions welcome. Suggested workflow:
- Fork the repo, create a feature branch
- Make changes and ensure the workspace builds
- Open a PR with a clear description and test notes

If you want, I can prepare a small PR to:
- Add a `use_zenoh` launch argument to bringup launch files
- Add a small `zenoh_helper` module that gracefully falls back if Zenoh isn't available
---
