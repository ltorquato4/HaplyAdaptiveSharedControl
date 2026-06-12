# Visualization

## Table of Contents
- [rviz_visualization_node](#rviz_visualization_node)
- [plotter_node](#plotter_node)

## `rviz_visualization_node`

This node is responsible for **visualizing the Haply device in RViz**.  
It subscribes to the `haply_state` topic and publishes TF transforms and visualization markers for real-time display.  

---

### Features

- **ROS2 subscription**  
  - `haply_state` (`haply_msgs/HaplyState`)  
    → Receives the current position, orientation, velocity, and button states from the driver node.  

- **TF broadcasting**  
  - Publishes a transform between: World frame (`world`) → Handle frame (`handle_link`)    

- **Marker publishing**  
  - Publishes a `visualization_msgs/Marker` of type `MESH_RESOURCE`.  
  - Uses the STL model `ArrowY.stl` from the `haply_meshes` package.  
  - Marker is positioned and oriented according to the Haply state.  

- **Diagnostics and logging**  
  - Prints node startup info with current parameters.  
  - Logs uptime continuously to the terminal.  

---

### Parameters

- `position_scale` (default: `10.0`) → Multiplier applied to the incoming Haply position before visualization.  
- `publish_frequency` (default: `100.0`) → Frequency [Hz] at which TFs and markers are published.  

---

### Published Topics

- `visualization_marker` (`visualization_msgs/Marker`)  
  → 3D marker for RViz visualization.  

- `/tf` (`tf2_msgs/TFMessage`)  
  → Dynamic transform between `world` and `handle_link`.  

---

### Usage

Run the visualization node:

```bash
ros2 run haply_interface rviz_visualization_node --ros-args -p position_scale:=10.0 -p publish_frequency:=100.0
```

## `plotter_node`

A node that displays the commanded target position, the actual position, and the percentage error between them.

---

### Features

- **Data sources**  
This ROS2 node subscribes to both:
  - `haply_target` (`haply_msgs/HaplyControl`) → the commanded target position 
  - `haply_state` (`haply_msgs/HaplyState`) → the measured actual device position

- **Real-time plotting**  
  Displays six diagrams in a single window:  
  - Left column → Actual vs Target position
  - Right column → Percentage error  
  - Row 1 → X-axis data  
  - Row 2 → Y-axis data  
  - Row 3 → Z-axis data  

- **Percentage error calculation**  
  Error is computed as:  

  $$
  \text{error} = \frac{x_{target} - x_{actual}}{x_{target}} \cdot 100
  $$



- **Rolling time window**  
  Only keeps the most recent N seconds (configurable with `plot_window` parameter).  

---

### Parameters

- `plot_window` (default: `30.0`) → Time window [s] for plotting  

---

### Usage

Run the node:

```bash
ros2 run haply_interface plotter_node --ros-args -p plot_window:=30.0
```
