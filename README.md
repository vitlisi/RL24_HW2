# NewRelease: Improved Codebase

## Overview

The **NewRelease** branch introduces a more readable and clearer version of the code, developed after the homework deadline. This update implements effort-based control for the manipulator across all four available trajectories.

---

## Setup Instructions

### 1. Clone the Repository
Download the repository from GitHub:
```bash
git clone -b REV_2 https://github.com/ferd-bot/RL_24_Homewrok_2_Robotics.git
```

### 2. Configure and Build the Workspace
To configure and build the workspace:
```bash
colcon build
source install/setup.bash
```

**Note**: The repository download includes extra files. Manually remove unnecessary files and move the required ones into the `src` folder.

---

## Launching the Manipulator

### 1. Control Modes
The new code supports controlling the manipulator in three modes:

- **Position Mode**: 
  Launch with the `iiwa_arm_controller` (with RViz):
  ```bash
  ros2 launch iiwa_bringup iiwa.launch.py command_interface:="position" robot_controller:="iiwa_arm_controller"
  ```

- **Velocity Mode**: 
  Launch with the `velocity_controller` (with RViz):
  ```bash
  ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
  ```

- **Effort Mode**: 
  Launch with the `effort_controller` (with Gazebo simulation):
  ```bash
  ros2 launch iiwa_bringup iiwa.launch.py command_interface:="effort" robot_controller:="effort_controller" use_sim:="true"
  ```

### 2. Launching Trajectories
To execute trajectories, open a new terminal and use the following commands based on the desired control mode:

- **Position Mode**:
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_node 0 --ros-args -p cmd_interface:=position
  ```

- **Velocity Mode**:
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_node 0 --ros-args -p cmd_interface:=velocity
  ```

- **Effort Mode**:
  ```bash
  ros2 run ros2_kdl_package ros2_kdl_node 0 --ros-args -p cmd_interface:=effort
  ```

---

## Available Trajectories

The following trajectories are supported, all with effort-based control:

1. **Linear with Trapezoidal Velocity Profile**:
   ```bash
   ros2 run ros2_kdl_package ros2_kdl_node 0 --ros-args -p cmd_interface:=effort
   ```

2. **Linear with Cubic Velocity Profile**:
   ```bash
   ros2 run ros2_kdl_package ros2_kdl_node 1 --ros-args -p cmd_interface:=effort
   ```

3. **Circular with Trapezoidal Velocity Profile**:
   ```bash
   ros2 run ros2_kdl_package ros2_kdl_node 2 --ros-args -p cmd_interface:=effort
   ```

4. **Circular with Cubic Velocity Profile**:
   ```bash
   ros2 run ros2_kdl_package ros2_kdl_node 3 --ros-args -p cmd_interface:=effort
   ```

---

## Key Updates

### What's Changed?
- The trajectories remain the same as the previous version.
- Control logic in `control.cpp` has been updated.
- Effort-based control is now implemented using the `idCntr` function.

---

## Notes

- For simplicity, only videos demonstrating **effort-based control with Gazebo** are attached.
