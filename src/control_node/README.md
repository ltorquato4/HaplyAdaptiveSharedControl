# control_node

An Adaptive Shared Control & Haptic Assistance ROS 2 package featuring dynamic rolling-horizon Model Predictive Control (MPC) and state-feedback (PD) control capabilities for cooperative human-robot manipulation.

---

## 1. Building & Running

### Dependencies

Ensure your workspace includes `haply_msgs`. Install Python packages:

```bash
pip install casadi pygame
```

### Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select control_node
source install/setup.bash
```

### Launch Debug Visulizer

If you wish to only launch the interactive Pygame-based debug visualizer:

```bash
ros2 run control_node test_control_node_output
```

The `control_node` itself is already included in the launch files. So for debugging purposes, launching this launch file only is enough.

### Launch Control Node & Debug Visualizer

To launch the headless control node in `DEBUG` alongside the interactive Pygame-based debug visualizer:

```bash
ros2 launch control_node control_debug_launch.py
```

When doing so, make sure the `control_node` is not running anywhere else.

---

## 2. Core Controller Schemes

### A. Model Predictive Control (MPC)

* **Optimization Engine:** Formulated using **CasADi** and resolved using the **IPOPT** interior-point solver.


* **Predictive Pipeline:** Uses a discrete state-space double integrator model ($x_{k+1} = A x_k + B u_k $) mapped over a configurable prediction horizon.


* **Cost Function Formulation:** Automatically balances human *Comfort* (minimizing control input authority), *Trajectory tracking* (closeness to path boundaries), and *Goal seeking* (terminal state convergence).



### B. State Feedback Controller

* **Implementation:** Standard closed-loop proportional-derivative (PD) controller mapping immediate positioning error ($e$) and estimated velocities ($\dot{e}$).


* **Active Boundary Protection:** Intercepts out-of-bound target velocities and clips matching assistance force projections to avoid destabilizing overshoot.



### C. Adaptive Shared Control Laws

In `adaptive` mode, controllers adapt in real-time to human haptic inputs:

1. **Progress-Based Interpolation:** Support efforts are heavily deployed at the start/end points of the point-to-point journey, giving the operator full free-play control in the middle region.


2. **Operator Impedance Tracking ($K_h$):** High human hand stiffness (increased rigor detected via `/estimation/K_h`) scale down assistant intervention factors ($\approx \tanh(\Vert{}K_h\Vert{})$), resolving control authority conflicts.



---

## 3. ROS 2 API Reference

### Subscribed Topics

* **`/study_controller_mode`** (`std_msgs/String`) — Dynamically switches operational modes between `adaptive` and `fixed`.


* **`/study_start_point`** & **`/study_end_point`** (`geometry_msgs/Point`) — Sets start/end point targets, triggering initialization loops.


* **`/experiment_cursor_position`** (`geometry_msgs/Point`) — High-frequency feed of haptic hand coordinates.


* **`/estimation/K_h`** (`std_msgs/Float64MultiArray`) — Estimated stiffness matrix of the human operator.


* **`/haply_state`** (`haply_msgs/HaplyState`) — Listens to physical button presses (holding Button A activates assistive control).



### Published Topics

* **`/control/U_a`** (`geometry_msgs/Vector3`) — Computed assistant control acceleration vectors.


* **`/control/K_a`** (`std_msgs/String`) — Runtime JSON serialization of active controller parameters and diagnostic weights.


* **`/haply_target`** (`haply_msgs/HaplyControl`) — Directly maps force vectors back to the physical actuator.



### Essential Launch Parameters

* **`use_mpc_controller`** (Boolean, default: `true`): Switches between MPC optimization and State Feedback (PD).


* **`adaptive_control_enabled`** (Boolean, default: `false`): Enables active $K_h$-dependent parameter adaptation.


* **`prediction_horizon`** (Integer, default: `5`): Set predictive lookahead steps.


* **`acceleration_to_force_factor`** (Double, default: `0.2`): Scales software-calculated control action into physical torque forces.



---

