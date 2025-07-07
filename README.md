# robot_position_pkg

`robot_position_pkg` is a ROS 2 package designed to control a simulated robot's linear position using an **Action Server** implemented as a `LifecycleNode`. The robot moves along a 1D axis between positions 0 and 100. Goals are defined by the desired target position and movement velocity. The node supports full lifecycle transitions, real-time feedback, goal preemption, and cancellation.

## Features

- Lifecycle Node implementation (`robot_position_server`)
- Custom ROS 2 Action interface: `robot_position_interface::action::RobotPosition`
- Lifecycle state support: `configure`, `activate`, `deactivate`, `cleanup`, `shutdown`
- Goal execution with real-time feedback
- Preemption and cancellation support
- Thread-safe execution and synchronized control

## Dependencies

Compatible with **ROS 2 Foxy, Humble, or newer**.

Required packages:

- `rclcpp`
- `rclcpp_lifecycle`
- `rclcpp_action`
- `lifecycle_msgs`
- `rcl_interfaces`
- `robot_position_interface` (custom action definition)

## Build and Run Instructions

Clone and build the package inside your ROS 2 workspace:

```bash
cd ~/ros2_ws/src
git clone https://github.com/Eduard0Castro/robot_position_pkg.git
cd ..
colcon build --packages-select robot_position
source install/setup.bash
```

## Node Descriptions

### robot_position_server.cpp

This file implements the **Action Server** as a `LifecycleNode`. It manages the robot's simulated position along a 1D axis (0–100), handling:

- Lifecycle transitions (`configure`, `activate`, `deactivate`, etc.)
- Goal validation (position range and node activation)
- Goal execution with velocity-based motion
- Real-time feedback publishing
- Support for **preemption** and **cancellation**

The server only accepts goals while in the `active` state. It ensures thread-safe goal handling and simulates robot motion with position updates and control logic.

---

### robot_position_client.cpp

This file implements a simple **Action Client** that connects to the server and sends position/velocity goals. It demonstrates how to:

- Create a goal request
- Send the goal to the correct topic (`/robot_position_<name>`)
- Receive feedback and result
- Handle success or failure

It's a useful interface for testing the server manually or in automated scripts.

---

### startup.cpp

This file implements a simple **Lifecycle Manager Node** responsible for automatically transitioning one or more lifecycle-enabled nodes through their initial states.

Key behaviours:

- Reads the parameter `node_name`, which is a list of node names to manage (e.g., `["robot_position"]`)
- For each node, it creates a client to call the `/change_state` service
- Automatically triggers:
  - `TRANSITION_CONFIGURE`: transitions node from `unconfigured` to `inactive`
  - `TRANSITION_ACTIVATE`: transitions node from `inactive` to `active`
- Waits 2 seconds between transitions for stability
- Prints the result of each service call (success or failure)

Example usage:

```bash
ros2 run robot_position startup
```
To customise which nodes are managed, you can set the parameter at runtime:

```bash
ros2 run robot_position startup --ros-args -p node_name:="['robot_position', 'some_other_node']"
```
---

### run_robot_position.launch.py

This launch file starts multiple instances of the `robot_position_server` 
node, each with a different name and robot identifier, as well as a centralized `startup` 
node that automatically manages their lifecycle transitions.

Launched components:

- `robot_position_a`: instance of the action server with `robot_name` parameter set to `"A"`
- `robot_position_b`: second instance with `robot_name` set to `"B"`
- `startup`: lifecycle controller node that sends `configure` and `activate` transitions to both servers

To launch the system:

```bash
ros2 launch robot_position run_robot_position.launch.xml
```

## Sending Goals

After the node is activated, you can send goals using the ROS 2 action interface:

```bash
ros2 action send_goal /<node_name> robot_position_interface/action/RobotPosition "{position: 20, velocity: 1}"
```

## Package Structure
```bash
robot_position/
├── launch/
│ └── run_robot_position.launch.py
├── src/
│ ├── robot_position_client.cpp
│ ├── robot_position_server.cpp
│ └── startup.cpp
├── CMakeLists.txt
├── LICENSE
└── package.xml
```

## Author

Developed by **Eduardo Castro**  
LinkedIn: [Eduardo Castro](https://www.linkedin.com/in/eduardo-castro-817059213)
