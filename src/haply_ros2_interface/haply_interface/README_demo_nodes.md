# Demo Nodes

## Table of Contents
- [state_subscriber_haply](#state_subscriber_haply)
- [state_subscriber_inverse3](#state_subscriber_inverse3)
- [state_subscriber_handle](#state_subscriber_handle)
- [target_position_sinus](#target_position_sinus)
- [target_position_input](#target_position_input)
- [haptic_ball](#haptic_ball)
- [PID_test](#pid_test)

## `state_subscriber_haply`

This minimal ROS2 node subscribes to the `haply_state` topic and prints the received state data (position, velocity, and orientation) of the Haply device to the console.

Useful for quick debugging or visualization of the current device state without using RViz.

To start the node, use the following command:

```bash
ros2 run haply_interface state_subscriber_haply
```

## `state_subscriber_inverse3`

This simple ROS2 node subscribes to the `inverse3_state` topic and logs the Inverse3 cursor position and velocity data.

Useful for debugging device position tracking.

To start the node, use the following command:

```bash
ros2 run haply_interface state_subscriber_inverse3
```

## `state_subscriber_handle`

This simple ROS2 node subscribes to the `handle_state` topic and logs the VerseGrip Stylus handle orientation (quaternion) and buttons data.

Useful for debugging device orientation tracking.

To start the node, use the following command:

```bash
ros2 run haply_interface state_subscriber_handle
```

## `target_position_sinus`

This ROS2 node publishes **sinusoidal target positions** to the `haply_target` topic using the `HaplyControl` message.  

Useful for quickly testing **position-based control** and PID tracking behavior of the Inverse3.

- X and Y are fixed (`x=0.03`, `y=-0.13`),  
- Z oscillates around `0.20 m` with amplitude `0.10 m`,  
- Publishes with 100 Hz.  

To start the node, use:

```bash
ros2 run haply_interface target_position_publisher
```

## `target_position_input`

This interactive ROS2 node lets you **manually send target positions** to the `haply_target` topic using the `HaplyControl` message.

- Prompts in the terminal for `x y z`, e.g. `0.05 -0.12 0.22`
- Publishes `target_position`

Useful for quick, manual testing of position-based control, PID tracking and for exploring the boundaries of the device’s reachable workspace.

**Run:**
```bash
ros2 run haply_interface target_position_input
```

**Example input:**
```bash
0.03 -0.13 0.20
```

## `haptic_ball`

This ROS2 demo node implements a virtual haptic sphere interaction.  
It continuously computes and applies forces to the Inverse3 device to simulate a spring-like boundary when the device enters the defined sphere volume.
It also publishes a sphere marker to RViz (`visualization_marker`) so that the simulated ball is also visible in the 3D scene.

**Parameters:**  
- `stiffness` → Controls how strong the restoring force is when the device penetrates the sphere boundary (higher value = harder surface).  
- `position_scale` → Scales the marker position and radius in RViz for better visibility (does not affect actual haptic behavior).  

**Run:**  

```bash
ros2 run haply_interface haptic_ball --ros-args -p stiffness:=200.0 -p position_scale:=10.0
```

## `PID_test`

This ROS2 node alternates between two predefined 3D positions and publishes them to the `haply_target` topic.  

- Useful for testing position-based control and system response to step changes.  
- Positions alternate at a configurable interval.  

### Parameters
- `interval` (default: `2.0`) → Time interval [s] between switching positions.  

### Usage

Run the node:

```bash
ros2 run haply_interface spring_damper_test --ros-args -p interval:=2.0
```