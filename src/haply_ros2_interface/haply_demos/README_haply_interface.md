# Launch Files

## Table of Contents
- [demo_handle_state_read.launch.py](#demo_handle_state_readlaunchpy)
- [demo_haply_state_read.launch.py](#demo_haply_state_readlaunchpy)
- [demo_inverse3_state_read.launch.py](#demo_inverse3_state_readlaunchpy)
- [haptic_ball.launch.py](#haptic_balllaunchpy)
- [PID_test.launch.py](#pid_testlaunchpy)
- [rviz_visualization.launch.py](#rviz_visualizationlaunchpy)
- [target_position_input.launch.py](#target_position_inputlaunchpy)
- [target_position_sinus.launch.py](#target_position_sinuslaunchpy)

---

## `haply_state_read.launch.py`

Starts the driver and a subscriber node that logs the **combined Haply state** (Inverse3 + Handle).  
Useful for debugging the full system state.

**Nodes started:**
- `haply_driver_node`
- `state_subscriber_haply`

**Run with:**
```bash
ros2 launch haply_demos haply_state_read.launch.py
```

---

## `handle_state_read.launch.py`

Launches the driver and a subscriber node to monitor the **VerseGrip Handle** state (orientation and buttons).  
Useful for quickly checking handle state.

**Nodes started:**
- `haply_driver_node`
- `state_subscriber_handle`

**Run with:**
```bash
ros2 launch haply_demos handle_state_read.launch.py
```

---

## `inverse3_state_read.launch.py`

Runs the driver together with a subscriber node to read **Inverse3 position and velocity**.  
Intended for testing only the Inverse3 cursor tracking.

**Nodes started:**
- `haply_driver_node`
- `state_subscriber_inverse3`

**Run with:**
```bash
ros2 launch haply_demos inverse3_state_read.launch.py
```

---

## `haptic_ball.launch.py`

Starts the driver node together with the **haptic ball demo**, which simulates a virtual sphere that generates restoring forces.  
Also publishes a marker so the ball appears in RViz.

**Nodes started:**
- `haply_driver_node`
- `haptic_ball`
- `rviz_visualization_node`
- `rviz2`

**Run with:**
```bash
ros2 launch haply_demos haptic_ball.launch.py
```

---

## `PID_test.launch.py`

Launches the driver and the **PID step test demo**, which alternates between two predefined positions.  
Useful for testing system response and PID tuning.

**Nodes started:**
- `haply_driver_node`
- `PID_test`
- `plotter_node`

**Run with:**
```bash
ros2 launch haply_demos PID_test.launch.py
```

---

## `rviz_visualization.launch.py`

Starts the driver node together with the **RViz visualization node**.  
Shows the Haply device in RViz with TF frames and a 3D marker.

**Nodes started:**
- `haply_driver_node`
- `rviz_visualization_node`
- `rviz2`

**Run with:**
```bash
ros2 launch haply_demos rviz_visualization.launch.py
```

---

## `target_position_input.launch.py`

Runs the driver, an interactive **target input node**, and the **plotter node**.  
Allows manually entering positions while plotting actual vs target positions and errors.

**Nodes started:**
- `haply_driver_node`
- `target_position_input`
- `plotter_node`

**Run with:**
```bash
ros2 launch haply_demos target_position_input.launch.py
```

---

## `target_position_sinus.launch.py`

Starts the driver, a sinusoidal **target generator**, and the **plotter node**.  
Useful for testing continuous tracking and PID performance.

**Nodes started:**
- `haply_driver_node`
- `target_position_sinus`
- `plotter_node`

**Run with:**
```bash
ros2 launch haply_demos target_position_sinus.launch.py
```

