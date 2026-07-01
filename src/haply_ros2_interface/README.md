# Haply ROS2 Interface

This package provides a ROS2 interface for the **Haply Inverse3** and **VerseGrip Stylus (Handle)** haptic devices. It includes drivers, visualization tools, and control nodes to operate and test the devices individually or together within a ROS2 environment.

This implementation was tested on:
- Windows 11 (WSL2 Ubuntu 22.04 LTS (64-bit))
- Python 3.10.6


---

## Contents

- [Prerequisites](#prerequisites)
  - [Platform Recommendation](#platform-recommendation)
  - [ROS2 Version](#ros2-version)
  - [Required Python Packages](#required-python-packages)
  - [Communication Protocol](#communication-protocol)
  - [Other Recommendations](#other-recommendations)
- [Getting Started](#getting-started)
  - [Cloning the Repository](#cloning-the-repository)
  - [Set Up Haply Device](#set-up-haply-device)
  - [Connecting USB Devices to Linux (WSL 2)](#connecting-usb-devices-to-linux-wsl-2)
- [First Run: Two Plug-and-Play Demos](#first-run-two-plug-and-play-demos)
  - [`haptic_ball`](#haptic_ball)
  - [`target_position_sinus`](#target_position_sinus)
- [Repository Structure](#repository-structure)
  - [`haply_demos`](#haply_demos)
  - [`haply_interface`](#haply_interface)
  - [`haply_meshes`](#haply_meshes)
  - [`haply_msgs`](#haply_msgs)

## Prerequisites

### Platform Recommendation

This interface was developed and tested on **Ubuntu 22.04** running under **WSL**. It is therefore **recommended to use the same environment** for compatibility and stability.

If you don't have WSL installed yet, you can find detailed installation instructions here:  
[WSL Installation Guide](https://learn.microsoft.com/en-us/windows/wsl/install)

### ROS2 Version

This interface was developed and tested using **ROS2 Humble**. It is strongly recommended to use this version to ensure compatibility and stability.

You can find more information on how to install ROS2 Humble here:  
[ROS2 Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### Required Python Packages

The interface uses `websockets` and `orjson` in addition to standard ROS 2
Python packages such as `rclpy`. The project Docker image installs these during
the image build.

### Communication Protocol

This interface uses a WebSocket-based communication protocol to interact with the devices.

To enable this, the **Haply Inverse SDK Service** must be installed. This service acts as a WebSocket server and is required for the interface to function properly.

For installation and testing instructions, please refer to the official documentation:

[Haply Inverse SDK Service Guide](https://docs.haply.co/inverseSDK/)

### Other Recommendations

1. To verify that your system recognizes the Haply devices, we recommend installing the Haply Device Manager.  
   You can find installation instructions here:  
   [Haply Device Manager](https://docs.haply.co/inverseSDK/)

2. Additional demo codes can be found here:  
   [Haply Demos](https://gitlab.com/Haply/public/python_samples)  
   > **Note:** Please note that these examples use direct serial communication, not WebSocket as the ROS2 interface does.



## Getting started

### Cloning the repository

Navigate to the directory where you want to place the repository, then clone it:

```bash
git clone <repository url>
```

After cloning, go to the workspace root:

```bash
cd ~/haply_ros2_interface
```

and build it using `colcon`:
```bash
colcon build
```

Finally, source the setup script:

```bash
source install/setup.bash
```

### Set Up Haply Device

This interface is designed for the Haply Inverse3 and VerseGrip Stylus devices.

For setting up and calibrating the devices, please refer to the official Haply documentation:

[Haply Quick Start Guide](https://docs.haply.co/docs/quick-start)
 
It is recommended to recalibrate the device each time it is reconnected to ensure accurate tracking and operation.

### Connecting USB Devices to WSL 2

To use the Haply devices under WSL 2, it is necessary to make the USB connection available to your Linux distribution.

For first-time setup and detailed guidance, follow the official documentation:   [Microsoft Guide: Connect USB Devices to WSL](https://learn.microsoft.com/en-gb/windows/wsl/connect-usb)

#### After Initial Setup

After completing the installation steps described in the guide above, follow these commands each time you want to use the device:

1. Open PowerShell as Administrator, and list all connected USB devices:
   ```powershell
   usbipd list
   ```
2. Bind the USB device to allow forwarding to WSL. Replace <busid> with the Bus ID shown in the previous step (e.g., 4-4):
   ```powershell
   usbipd bind --busid <busid>
   ```
3. Attach the USB device to WSL
   ```powershell
   usbipd attach --wsl --busid <busid>
   ```
   > **Important:** Your Linux distribution (e.g., Ubuntu in WSL) **must be running** before performing the `attach` step. Simply open a WSL terminal before executing the `usbipd attach` command.

   > **Note:** If you receive the following warning:  
   > `usbipd: warning: The device appears to be used by Windows; stop the software using the device, or bind the device using the '--force' option.`  
   > It may be caused by a background process such as `haply-inverse-service.exe`.  
   > Open **Task Manager** and stop this process before attempting to attach the device.

4. When finished using the device, you can detach it:
   ```powershell
   usbipd detach --busid <busid>
   ```
> **Note:** The device will be inaccessible from Windows while attached to WSL.

## First Run: Two Plug-and-Play Demos

This section presents two simple demos that you can launch immediately after completing the steps above; they showcase the device’s capabilities and basic operation.

### `haptic_ball`

This demo implements a virtual haptic sphere interaction.  
It continuously computes and applies forces to the Inverse3 device to simulate a spring-like boundary when the device enters the defined sphere volume.
It also publishes a sphere marker to RViz (`visualization_marker`) so that the simulated ball is also visible in the 3D scene. 

**Run:**  

```bash
ros2 run haply_interface haptic_ball --ros-args -p stiffness:=200.0 -p position_scale:=10.0
```

### `target_position_sinus`

This demo moves the device sinusoidally along the z-axis with a 0.1 m amplitude. Meanwhile, it plots the desired and actual positions, allowing you to gauge the PID controller’s accuracy. 

To start the node, use:

```bash
ros2 run haply_interface target_position_publisher
```

## Repository Structure

### `haply_demos`  
This package contains the **launch files** that demonstrate different applications of the Haply devices.  
A detailed description of each launch file can be found in the [`launch_files_README.md`](src/haply_demos/launch_files_README.md).

---

### `haply_interface`  
This package contains the **ROS2 nodes** used by the system, organized into three categories:  
- **Driver nodes** → Handle the communication between the Haply devices and ROS2.
A detailed description of each node can be found in the [`drivers_README.md`](src/haply_interface/haply_interface/drivers/drivers_README.md).

- **Visualization nodes** → Provide 2D/3D visualization of device states and transforms (e.g., in RViz).  
A detailed description of each node can be found in the [`visualization_README.md`](src/haply_interface/haply_interface/visualize/visualization_README.md).

- **Demo nodes** → Demonstrate various capabilities and use-cases of the Haply devices.
A detailed description of each node can be found in the [`demo_nodes_README.md`](src/haply_interface/haply_interface/demo_nodes/demo_nodes_README.md).

---

### `haply_meshes`  
This package includes the **STL files** used for 3D visualization of the Handle (VerseGrip Stylus) in RViz.

---

### `haply_msgs`  
This package defines the **custom ROS2 message types** used by the ROS2 interface.
