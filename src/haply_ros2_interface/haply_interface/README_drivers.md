# Driver nodes

This README contains detailed documentation for the available driver nodes including their functionality, published and subscribed topics, parameters, and usage instructions.

## Table of Contents
- [haply_driver_node](#haply_driver_node)
- [inverse3_driver_node](#inverse3_driver_node)
- [handle_driver_node](#handle_driver_node)

## `haply_driver_node`

This is the main driver node responsible for managing **both** the Inverse3 and the VerseGrip Stylus devices simultaneously.  

### Features

- **WebSocket connection**  
  Connects to the `haply-inverse-service` running on the host machine to exchange data with the hardware.

- **Real-time state acquisition**  
Listens to real-time data streams from both devices:
  - Cursor position and velocity (Inverse3)
  - Orientation and button states (VerseGrip Stylus)

- **ROS2 publishers**  
  Publishes device data to the following topics:
  - `inverse3_state` (`haply_msgs/Inverse3State`)  
    → Current position and velocity of the Inverse3.  
  - `handle_state` (`haply_msgs/HandleState`)  
    → Orientation and button states of the Handle.  
  - `haply_state` (`haply_msgs/HaplyState`)  
    → Unified topic with position, velocity, orientation, and button states.  

- **Force and position-based control**  
  Subscribes to control commands on topic:  
  - `haply_target` (`haply_msgs/HaplyControl`)  

  Two control modes are supported:
  1. **Force-based control**  
     - Incoming command specifies direct force values (`x`, `y`, `z`).  
     - These forces are applied immediately to the Inverse3 device.  

  2. **Position-based control**  
     - Incoming command specifies a target position.  
     - The driver computes corrective forces using a **PID controller**:  

        $F = K_p \cdot e + K_i \int e \cdot dt - K_d \cdot v$


        where:
          - $e = x_{target} - x$
          - $v$ is the measured velocity
          - $K_p, K_i, K_d$ are configurable controller gains

     - Computed forces are clamped to the maximum allowed per-axis force.  

- **Safety and timeouts**  
  - If no new force command is received within a configurable timeout, force output is set to zero.  
  - If no devices are detected for 2 seconds, the node shuts down automatically.  

- **Diagnostics and logging**  
  - Prints device information (ID, port, calibration, battery, readiness) at startup.  
  - Logs uptime and warnings if devices are missing or not awake.  

---

### Parameters

- `frequency` (default: `200.0`) → State publishing frequency [Hz]  
- `max_force` (default: `10.0`) → Maximum allowed per-axis force [N]  
- PID gains (fixed in code, can be tuned):  
  - $K_p = 30.0$ [N/m]  
  - $K_i = 5.0$ [N/(m·s)]  
  - $K_d = 0.9$ [N·s/m]  

---

### Usage

Run the driver node:

```bash
ros2 run haply_interface haply_driver_node --ros-args -p frequency:=200.0 -p max_force:=10.0
```

## `inverse3_driver_node`

This node is a dedicated ROS2 driver for the **Inverse3** device, allowing developers to monitor its state and control the applied forces independently of other hardware. 

---

### Features

- **WebSocket communication**  
  Connects to the `haply-inverse-service` to send and receive data from the Inverse3 device.  

- **Real-time state acquisition**  
  Continuously receives:  
  - Position  
  - Velocity  

- **ROS2 publishers**  
  - `inverse3_state` (`haply_msgs/Inverse3State`)  
    → Publishes the current position and velocity of the Inverse3 device.  

- **ROS2 subscriber**  
  - `haply_target` (`haply_msgs/HaplyControl`)  
    → Provides external commands for device control.  

- **Force and position-based control**  
  Subscribes to control commands on topic:  
  - `haply_target` (`haply_msgs/HaplyControl`)  

  Two control modes are supported:
  1. **Force-based control**  
     - Incoming command specifies direct force values (`x`, `y`, `z`).  
     - These forces are applied immediately to the Inverse3 device.  

  2. **Position-based control**  
     - Incoming command specifies a target position.  
     - The driver computes corrective forces using a **PID controller**:  

        $F = K_p \cdot e + K_i \int e \cdot dt - K_d \cdot v$

        where:
          - $e = x_{target} - x$
          - $v$ is the measured velocity
          - $K_p, K_i, K_d$ are configurable controller gains

     - Computed forces are clamped to the maximum allowed per-axis force.   

- **Safety and timeouts**  
  - If no new force command is received within a configurable timeout, force output is set to zero.  
  - If no devices are detected for 2 seconds, the node shuts down automatically.  

- **Diagnostics and logging**  
  - Prints device information (ID, port, calibration, readiness) at startup.  
  - Logs uptime and warnings if the device is missing.  

---

### Parameters

- `frequency` (default: `200.0`) → State publishing frequency [Hz]  
- `max_force` (default: `10.0`) → Maximum allowed per-axis force [N]  
- PID gains (fixed in code, can be tuned):  
  - $K_p = 30.0$ [N/m]  
  - $K_i = 5.0$ [N/(m·s)]  
  - $K_d = 0.9$ [N·s/m]  

---

### Usage

Run the driver node:

```bash
ros2 run haply_interface inverse3_driver_node --ros-args -p frequency:=200.0 -p max_force:=10.0
```

## `handle_driver_node`

This node is a dedicated ROS2 driver for the **VerseGrip Stylus** (also referred to as the Handle), allowing standalone monitoring of its orientation and button states without requiring the Inverse3 device.  

---

### Features

- **WebSocket communication**  
  Connects to the `haply-inverse-service` to send and receive data from the wireless VerseGrip Handle.  

- **Real-time state acquisition**  
  Continuously receives:  
  - Orientation (as quaternion)  
  - Button states 

- **ROS2 publishers**  
  - `handle_state` (`haply_msgs/HandleState`)  
    → Publishes the current orientation and button states of the Handle.  

- **Diagnostics and logging**  
  - Prints device information (ID, port, battery level, readiness) at startup.  
  - Logs warnings if the device is missing or not awake.  
  - Outputs uptime in the terminal.  

- **Keep-alive mechanism**  
  - Periodically sends `keep_alive` commands to the device to ensure it stays connected and awake.  

---

### Parameters

- `frequency` (default: `200.0`) → State publishing frequency [Hz]  

---

### Usage

Run the driver node:

```bash
ros2 run haply_interface handle_driver_node --ros-args -p frequency:=200.0
```