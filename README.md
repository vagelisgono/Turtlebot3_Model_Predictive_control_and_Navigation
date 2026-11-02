# TurtleBot3 — System Identification and Model Predictive Control

A ROS Noetic / Gazebo 11 project using a **TurtleBot3 Burger**. The main idea is to identify a mathematical model from robot measurements, validate it, and use it for trajectory tracking and waypoint navigation.

This README focuses on **the mathematics behind each Python program**. Experimental plots, screenshots, and demonstrations can be added separately.

## 1. `collect_turtlebot_data.py` — Data collection

The program sends linear and angular velocity commands through `/cmd_vel` and records robot measurements from `/odom` in a CSV file. It tests rest, forward motion, rotation, stopping, and combined motion.

The recorded quantities are:

$$
u_{cmd} = \begin{bmatrix}v_{cmd}\\ \omega_{cmd}\end{bmatrix}, \qquad
X_{odom} = \begin{bmatrix}x & y & \theta & v & \omega\end{bmatrix}^{T}$$

where $x,y$ are position, $\theta$ is orientation, $v$ is forward velocity, and $\omega$ is angular velocity. Comparing the **commanded** and **measured** velocities makes it possible to estimate the robot's response.

## 2. `identify_turtlebot.py` — System identification

This program uses the recorded velocity step responses to estimate a **first-order dynamic model** for forward and angular motion:

$$\boxed{\tau_v\dot v+v=K_vv_{cmd}}$$

$$\boxed{\tau_\omega\dot\omega+\omega=K_\omega\omega_{cmd}}$$

- $K_v,K_\omega$: steady-state gains (how closely actual velocities follow commands).
- $\tau_v,\tau_\omega$: time constants (how quickly the robot responds).

For a constant input $u$, the solution is:

$$\boxed{y(t)=Ku+(y_0-Ku)e^{-t/\tau}}$$

Here $y_0$ is the measured initial velocity. The program finds $K$ and $\tau$ with **nonlinear least squares**:

$$\boxed{\min_{K,\tau}\sum_{i=1}^{n}(y_{pred}(t_i;K,\tau)-y_{meas}(t_i))^2}$$

The parameter values used later in the controllers are:

| Parameter | Estimated value |
|---|---:|
| $K_v$ | 1.0011045 |
| $\tau_v$ | 0.0774945 s |
| $K_\omega$ | 0.9942472 |
| $\tau_\omega$ | 0.032835 s |

**Note:** These short time constants are difficult to estimate reliably at low sampling rates; they should be treated as preliminary.

## 3. `turtlebot_full_identification.py` — Identification and validation in one experiment

This alternative program performs data collection, model fitting, prediction, and comparison in one workflow. Unlike the simple step-fitting script, it tracks command-change timestamps and handles piecewise-constant inputs.

For each interval of length $\Delta t$ with constant command $u_k$, the first-order model has the update:

$$\boxed{y_{k+1}=e^{-\Delta t/\tau}y_k+K(1-e^{-\Delta t/\tau})u_k}$$

It estimates $K_v,\tau_v,K_\omega,\tau_\omega$ from the training portion and keeps the combined-motion segment for validation. Position is reconstructed by integrating predicted velocities:

$$\dot x=v\cos\theta,\qquad \dot y=v\sin\theta,\qquad \dot\theta=\omega.$$

It then compares predicted and odometry measurements using the root mean squared error:

$$\boxed{\mathrm{RMSE}=\sqrt{\frac{1}{n}\sum_{i=1}^{n}(z_{pred,i}-z_{meas,i})^2}}$$

The separate `turtlebot_full_identification_commented.py` is the explanatory version of this workflow.

## 4. `model_validation.py` — Predicting robot motion

The model is tested by predicting a complete trajectory from a measured initial state and known commands, then comparing that prediction with `/odom`.

The five-state model is:

$$\boxed{X=\begin{bmatrix}x&y&\theta&v&\omega\end{bmatrix}^{T}}$$

$$\begin{aligned}
\dot x &=v\cos\theta, & \dot y &=v\sin\theta, & \dot\theta &=\omega,\\
\dot v &=(K_vv_{cmd}-v)/\tau_v, &
\dot\omega &=(K_\omega\omega_{cmd}-\omega)/\tau_\omega.
\end{aligned}$$

At each time step, predicted velocities are updated with the exponential first-order formula from Section 3. Position and heading are then approximated using average velocities and midpoint heading:

$$\begin{aligned}
\bar v_k&=\tfrac12(v_k+v_{k+1}), & \bar\omega_k&=\tfrac12(\omega_k+\omega_{k+1}),\\
\theta_{mid}&=\theta_k+\tfrac12\bar\omega_k\Delta t,\\
x_{k+1}&=x_k+\bar v_k\cos(\theta_{mid})\Delta t,\\
y_{k+1}&=y_k+\bar v_k\sin(\theta_{mid})\Delta t,\\
\theta_{k+1}&=\theta_k+\bar\omega_k\Delta t.
\end{aligned}$$

**Purpose:** Check whether the identified model predicts motion accurately enough to be used inside MPC.

## 5. `nonlinear_controller.py` — Nonlinear trajectory tracking

This controller makes the TurtleBot3 follow a desired **circular trajectory**. For the reference circle:

$$R=0.5\,\mathrm{m},\qquad v_r=0.1\,\mathrm{m/s},\qquad \omega_r=\frac{v_r}{R}=0.2\,\mathrm{rad/s}.$$

The reference pose is $(x_r,y_r,\theta_r)$. Position error is expressed in the **robot's local frame**:

$$\begin{aligned}
e_x &=\cos\theta(x_r-x)+\sin\theta(y_r-y),\\
e_y &=-\sin\theta(x_r-x)+\cos\theta(y_r-y),\\
e_\theta &=\operatorname{wrap}(\theta_r-\theta).
\end{aligned}$$

The nonlinear feedback commands are:

$$\boxed{v_{cmd}=v_r\cos(e_\theta)+k_xe_x}$$

$$\boxed{\omega_{cmd}=\omega_r+k_yv_re_y+k_\theta\sin(e_\theta)}$$

Here $k_x,k_y,k_\theta$ are controller gains. The first equation corrects forward error; the second corrects sideways and heading errors. This controller is based on **kinematics**, without explicitly predicting future states.

## 6. `mpc_controller.py` — Model Predictive Control

Unlike the nonlinear feedback controller, MPC uses the **identified five-state model** to predict the future and choose velocity commands.

For a prediction horizon $N=8$ and sampling time $\Delta t=0.1$ s, the optimizer searches for **8 pairs of commands** (16 variables):

$$\boxed{U=[u_0,u_1,\ldots,u_7],\qquad u_k=[v_{cmd,k},\omega_{cmd,k}]}$$

Starting with the measured state $X_0$, the model predicts:

$$X_{k+1}=f(X_k,u_k),\qquad k=0,\ldots,7.$$

This gives **8 future states over 0.8 seconds**. A typical trajectory-tracking cost adds the errors and command effort at each step:

$$\boxed{J(U)=\sum_{k=1}^{8}\left[q_p\|p_k-p_{r,k}\|^2+q_\theta e_{\theta,k}^2+r_v(v_{cmd,k}-v_r)^2+r_\omega(\omega_{cmd,k}-\omega_r)^2\right]}$$

where $p_k=[x_k,y_k]^T$. The $q$ and $r$ terms are weights that determine what matters most. The exact weights depend on the controller version.

The problem is:

$$\boxed{U^*=\arg\min_U J(U)}$$

subject to the allowed command limits. SciPy's **L-BFGS-B** optimizer evaluates candidate command sequences and searches for a low total cost.

**Important:** MPC does **not** execute all eight optimized commands. It executes only $u_0^*$, measures the new state from `/odom`, and optimizes another eight-step sequence. This is called **receding-horizon control**.

## 7. `mpc_waypoint_lidar.py` — Waypoints and obstacle-aware MPC

This program extends MPC with goals selected in RViz (`/move_base_simple/goal`) and obstacle measurements from LiDAR (`/scan`).

### Waypoint direction

For current position $p=[x,y]^T$ and waypoint $p_g=[x_g,y_g]^T$, the attractive unit vector is:

$$\boxed{a=\frac{p_g-p}{\|p_g-p\|}}$$

### LiDAR repulsion

For detected LiDAR distance $r_i$ and world-frame beam angle $\alpha_i$, nearby obstacles create a repulsive vector:

$$\boxed{r=-\frac{1}{5}\sum_i\left(1-\frac{r_i}{R_L}\right)^2
\begin{bmatrix}\cos\alpha_i\\\sin\alpha_i\end{bmatrix}}$$

Only valid points with $r_i<R_L$ are included. The combined direction and desired heading are:

$$d=a+k_{rep}r,\qquad \boxed{\theta_d=\operatorname{atan2}(d_y,d_x)}.$$

This is an **Artificial Potential Field (APF)**-inspired guidance method: move toward the goal while steering away from detected obstacles.

### Obstacle penalty

The MPC cost also penalizes predicted robot positions that get too close to LiDAR obstacle points $o_i$:

$$d_{ik}=\|p_k-o_i\|$$

$$\boxed{J_{obs,k}=w_{obs}\sum_i\max(0,d_{safe}-d_{ik})^2}$$

The complete cost implemented in this script is:

$$\boxed{\begin{aligned}
J(U)=\sum_{k=1}^{8}\big[&40\|p_g-p_k\|^2+3\,\operatorname{wrap}(\theta_d-\theta_k)^2\\
&+0.2(v_{cmd,k}-v_{desired})^2+0.03\omega_{cmd,k}^2+J_{obs,k}\big].
\end{aligned}}$$

It is minimized with the command bounds:

$$0\le v_{cmd}\le0.16\,\mathrm{m/s},\qquad -1\le\omega_{cmd}\le1\,\mathrm{rad/s}.$$

The controller stops when a detected obstacle is closer than `STOP_DISTANCE` (0.18 m in the current script). The LiDAR point locations assume the scanner is approximately centered and aligned with the robot. **This is a prototype:** APF can get stuck in local minima, and soft obstacle costs do not guarantee collision-free motion.

---

## Overall idea

```text
Velocity commands + /odom
          ↓
     Data collection
          ↓
   System identification
          ↓
  Mathematical robot model
          ↓
     Model validation
          ↓
   Nonlinear control / MPC
          ↓
 RViz waypoints + LiDAR MPC
          ↓
        TurtleBot3
```

*Plots, screenshots, experimental comparisons, and installation instructions can be added here as the project develops.*
