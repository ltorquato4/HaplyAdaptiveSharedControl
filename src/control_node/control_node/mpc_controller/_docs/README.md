## Mathematical Formulation of the Adaptive MPC

The control framework is structured as a discrete-time, Finite-Horizon Optimal Control Problem (FHOCP) formulated over a prediction horizon $N$. At each sampling instant $k$, the controller minimizes a quadratic cost function subject to linear state-space dynamics and inequality constraints.

The optimization yields an optimal sequence of virtual control inputs $\mathbf{U}^* = [u_0^*, u_1^*, \dots, u_{N-1}^*]^T \in \mathbb{R}^{2N}$, from which only the first control vector $u_0^*$ is applied to the system.

---


### 1. State-Space Representation & Batch Prediction

The system is modeled as a discrete-time Linear Time-Invariant (LTI) system:

$$x_{k+1} = A x_k + B u_k $$

where $x_k \in \mathbb{R}^4$ represents the state vector $[x, \dot{x}, y, \dot{y}]^T$ (positions and velocities), $u_k \in \mathbb{R}^2$ is the control input acceleration vector $[u_x, u_y]^T$.

By recursively unrolling the system dynamics over the horizon $N$, the predicted state trajectory vector $\mathbf{X} = [x_1, x_2, \dots, x_N]^T \in \mathbb{R}^{4N}$ is calculated in a single batch matrix multiplication:

$$\mathbf{X} = \bar{A} x_0 + \bar{B} \mathbf{U}$$

where the batch propagation matrices $\bar{A} \in \mathbb{R}^{4N \times 4}$ and $\bar{B} \in \mathbb{R}^{4N \times 2N}$ are structured as:

$$\bar{A} = \begin{bmatrix} A \\ A^2 \\ \vdots \\ A^N \end{bmatrix}, \quad \bar{B} = \begin{bmatrix} B & 0 & \dots & 0 \\ AB & B & \dots & 0 \\ \vdots & \vdots & \ddots & \vdots \\ A^{N-1}B & A^{N-2}B & \dots & B \end{bmatrix}$$

---


### 2. Dynamic Reference Trajectory Generation

The reference trajectory $\mathbf{X}_{\text{ref}} = [x_{\text{ref}, 0}, \dots, x_{\text{ref}, N-1}]^T$ is synthesized at each time step by projecting the current actual state $x_0$ orthogonally onto the global path vector connecting the start position $\mathbf{p}_{\text{start}}$ and end position $\mathbf{p}_{\text{end}}$.

---


### 3. The Mathematical Cost Optimization & Constraints

The discrete optimization problem solved at every control interval is formulated as:

$$\min_{\mathbf{U}} \quad J(\mathbf{X}, \mathbf{U}) = \sum_{k=0}^{N-1} \left( \Vert{}x_k - x_{\text{ref}, k}\Vert{}_Q^2 + \Vert{}u_k\Vert{}_R^2 \right) + \Vert{}x_N - x_{\text{goal}}\Vert{}_P^2$$

$$\text{subject to: } \quad x_{k+1} = A x_k + B u_k + z_k$$

$$u_{\min} \leq u_k \leq u_{\max}$$

$$v_{\min} \leq \dot{x}_k, \dot{y}_k \leq v_{\max}$$

$$p_{\min} \leq x_k, y_k \leq p_{\max}$$

#### Time-Varying Weight Matrices

The penalty matrices $R$ (control effort/comfort), $Q$ (stage tracking deviation), and $P$ (terminal goal convergence) are diagonal matrices configured dynamically via the authority weightings:

$$R = \text{diag}\big(0.01 w_c, \, 0.01 w_c\big)$$

$$Q = \text{diag}\big(10 w_t, \, 0, \, 10 w_t, \, 0\big)$$

$$P = \text{diag}\big(100 w_g, \, 10 w_g, \, 100 w_g, \, 10 w_g\big)$$

---


## Mathematical Formulation of the Dynamic Adaptation Mechanism

At the core of the controller's intelligence is a continuous, online adaptation law that maps physical human behavior and task progress directly to the cost optimization parameters. Rather than using static weights ($w_{c,\text{base}}, w_{t,\text{base}}, w_{g,\text{base}}$), the MPC optimization surface is reshaped at every sampling interval $\Delta t$ by tracking a human-intent observer and a geometric progress metric.

---


### 1. The Human Intent & Authority Observer

The human authority model differentiates between two distinct physical modalities: **unconstrained driving intent** (high compliance) and **path stabilization/correction** (high guidance requirement). These behaviors are isolated in real time from the measured human stiffness matrix $K_h \in \mathbb{R}^{2 \times 2}$:

* **Translational Drive ($\kappa_c$):** Aggressive acceleration or locomotion along the principal Cartesian axes is captured by the diagonal components ($K_0$ and $K_6$), which indicate the user is actively pushing the system to translate.


* **Rotational/Stabilizing Correction ($\kappa_t$):** Finer guidance adjustments, path trimming, or stabilization efforts are captured by the off-diagonal coupled terms ($K_1$ and $K_7$), signaling cautious or corrective maneuvering.



$$\kappa_c = \frac{\vert{}K_0\vert{} + \vert{}K_6\vert{}}{2}, \quad \kappa_t = \frac{\vert{}K_1\vert{} + \vert{}K_7\vert{}}{2}$$

To determine which agent should dominate the control loop, the controller evaluates a **Normalized Dominance Difference** ($\Delta \kappa$):

$$\Delta \kappa = \text{clamp}\left( \frac{\kappa_c - \kappa_t}{\lambda}, \, -1.0, \, 1.0 \right)$$

Where $\lambda = 50.0 \text{ N/m}$ is a soft-normalization scale that prevents sensor noise or transient force spikes from destabilizing the optimizer.

---


### 2. Linear Scaling Law & Authority Blending

The calculated dominance difference $\Delta \kappa \in [-1, 1]$ directly modulates the balance between user compliance and robot track-keeping through an antagonistic linear scaling law:

```
  Human Dominates (Compliance) ◄──────────────┼──────────────► Robot Dominates (Guidance)
  Δκ = +1.0                                Δκ = 0.0                             Δκ = -1.0
  ────────────────────────────────────────────────────────────────────────────────────────
  w_c scales up (2.5x)                   Base Weights                    w_c drops to min (0.1x)
  w_t drops to min (0.1x)               (No Adaptation)                  w_t scales up (2.5x)

```

The time-varying stage weights are computed via:

$$w_c(\Delta \kappa) = \max\left( \delta_{\min}, \, 1.0 + \gamma \Delta \kappa \right) \cdot w_{c,\text{base}}$$

$$w_t(\Delta \kappa) = \max\left( \delta_{\min}, \, 1.0 - \gamma \Delta \kappa \right) \cdot w_{t,\text{base}}$$

* **Sensitivity Coefficient ($\gamma = 1.5$):** Amplifies the adaptation rate, allowing small variations in user grip stiffness to fluidly shift authority.


* **Safety Bound ($\delta_{\min} = 0.1$):** Acts as a mathematical safeguard. It guarantees that neither weight ever vanishes completely, ensuring that the optimization matrix remains strictly positive-definite ($\succ 0$) and stable.


---

### 3. Non-Linear Terminal Docking Adaptation

As the system approaches the goal, the controller must smoothly transition from a highly compliant interactive guide to a rigid positioning automaton to eliminate steady-state error. This is handled by a non-linear geometric override triggered by the path progress variable $t_{\text{current}}$:

$$\text{If } t_{\text{current}} > 0.85 \quad \text{or} \quad \mathcal{S}_{\text{docked}} = \text{True}$$

Upon crossing this threshold, a latching state variable $\mathcal{S}_{\text{docked}}$ is set to $\text{True}$. This ensures that even if user interaction temporarily deflects the state backward along the path ($t_{\text{current}} < 0.85$), the system remains locked in docking mode, avoiding controller chattering or boundary limit cycles.

The proximity intensity is governed by a quadratic scaling factor $\tau \in [0, 1]$:

$$\tau = \left( \frac{t_{\text{current}} - 0.85}{1.0 - 0.85} \right)^2$$

Using $\tau$, the cost parameters undergo a final non-linear transformation before being pushed to the solver:

$$\begin{aligned} w_c &\leftarrow w_c \cdot (1.0 - \alpha \tau) && \text{where } \alpha = 0.9 \\ w_t &\leftarrow w_t \cdot (1.0 + \beta \tau) && \text{where } \beta = 2.0 \\ w_g &\leftarrow w_g \cdot (1.0 + \eta \tau) && \text{where } \eta = 10^6 \end{aligned}$$
