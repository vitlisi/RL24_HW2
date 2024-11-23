# RL24_HW2
Caricamento separato di iiwa per la grandezza dei file


## Instructions

### 1. Clone the repository:
Clone the repository into a local folder:
```bash
https://github.com/vitlisi/RL24_HW2.git
```

### 2. Configure and build the packages:
To configure and build all the packages in the workspace:
```bash
colcon build
source install/setup.bash
```

---

## Launching Packages

### 1. Launch the `iiwa` model with RViz:
To launch the IIWA model with RViz and the default command interface set to "position":
```bash
ros2 launch iiwa_bringup iiwa.launch.py
```

### 2. Launch the KDL node:
To launch the KDL node with the default command interface set to "position":
```bash
ros2 run ros2_kdl_package ros2_kdl_node
```

### 3. Use velocity commands:
To use velocity commands:
```bash
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=velocity
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
```

### 4. Use effort commands (with Gazebo):
To use effort commands with Gazebo simulation:
```bash
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=effort
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="effort" robot_controller:="effort_controller" use_sim:="true"
```

---

## Planner Selection

You can use the following commands to select and start a specific trajectory. Open another terminal and use these commands to set the planner:

### Available Planners:

- **Planner 0 (Trapezoidal Linear):**
  ```bash
  ros2 param set /ros2_kdl_node current_planner_index 0
  ```

- **Planner 1 (Cubic Linear):**
  ```bash
  ros2 param set /ros2_kdl_node current_planner_index 1
  ```

- **Planner 2 (Trapezoidal Circular):**
  ```bash
  ros2 param set /ros2_kdl_node current_planner_index 2
  ```

- **Planner 3 (Cubic Circular):**
  ```bash
  ros2 param set /ros2_kdl_node current_planner_index 3
  ```

- **Planner 4 (Home):**
  ```bash
  ros2 param set /ros2_kdl_node current_planner_index 4
  ```

**Note**: The possible sequence of trajectories is the selected trajectory followed by `home`. To start a new trajectory, you need to restart the KDL node.

---

## Topic Monitoring

You can monitor trajectory or torque errors using the following commands:

- **Trajectory error:**
  ```bash
  ros2 topic echo /trajectory_error
  ```

- **Torque error:**
  ```bash
  ros2 topic echo /torque_error
  ```

--- 

With this configuration, you can launch and test trajectories, monitor data, and configure the robot for different operational modes.
