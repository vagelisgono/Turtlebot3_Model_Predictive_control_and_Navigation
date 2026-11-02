# TurtleBot3 — System Identification, Nonlinear Control and Model Predictive Control

This project studies the motion and control of a **TurtleBot3 Burger** in **ROS Noetic / Gazebo 11**. I first collected motion measurements, identified a mathematical model of the robot, tested the model against odometry, and then used it to build trajectory-tracking and waypoint-navigation controllers.

The main idea is:

**Measure the robot → identify its dynamics → predict its motion → design a controller → navigate using MPC and LiDAR.**

This README explains **what each Python program does, which equations it uses, and why those equations matter**. Screenshots, plots, videos and setup instructions can be added separately.

> **Viewing the equations:** This document uses GitHub's `math` blocks. The formatted equations should render on the repository's GitHub README page. If an editor shows the source instead, the plain-text descriptions beside each equation explain the same mathematics.

---

## Contents

- [1. Files and notation](#1-files-and-their-roles)
- [2. Data collection](#2-collect_turtlebot_datapy--collecting-motion-data)
- [3. Step-response identification](#3-identify_turtlebotpy--identifying-the-robots-velocity-dynamics)
- [4. Full identification and validation](#4-turtlebot_full_identificationpy--full-identification-and-validation-experiment)
- [5. Model validation](#5-model_validationpy--testing-the-identified-five-state-model)
- [6. Nonlinear trajectory tracking](#6-nonlinear_controllerpy--following-a-circle-with-nonlinear-feedback)
- [7. Model Predictive Control](#7-mpc_controllerpy--model-predictive-control-for-trajectory-tracking)
- [8. MPC waypoint/LiDAR navigation](#8-mpc_waypoint_lidarpy--mpc-navigation-with-rviz-goals-and-lidar)
- [9. How the programs connect](#9-how-all-the-programs-fit-together)
- [10. Accuracy and future additions](#10-notes-on-model-accuracy-and-future-additions)

---

## 1. Files and their roles

| Program | Purpose |
| --- | --- |
| `collect_turtlebot_data.py` | Send test velocity commands and record the robot's response. |
| `identify_turtlebot.py` | Estimate first-order velocity dynamics from the recorded data. |
| `turtlebot_full_identification.py` | Run a complete identification experiment, fit the model and validate it. |
| `turtlebot_full_identification_commented.py` | An educational, commented version of the full identification workflow. |
| `model_validation.py` | Predict the robot's motion using the identified model and compare it with odometry. |
| `nonlinear_controller.py` | Track a circular trajectory using nonlinear feedback control. |
| `mpc_controller.py` | Track a reference trajectory using Model Predictive Control (MPC). |
| `mpc_waypoint_lidar.py` | Combine MPC with RViz waypoints and LiDAR-based obstacle avoidance. |

### Mathematical notation used throughout

| Symbol | Meaning | Unit |
| --- | --- | --- |
| `x, y` | Robot position in the odometry frame | m |
| `theta` | Robot heading (yaw) | rad |
| `v` | Measured forward velocity | m/s |
| `omega` | Measured angular velocity | rad/s |
| `v_cmd` | Commanded forward velocity | m/s |
| `omega_cmd` | Commanded angular velocity | rad/s |
| `K_v, K_omega` | Velocity model gains | dimensionless |
| `tau_v, tau_omega` | Velocity response time constants | s |
| `dt` | Time between predictions | s |

The state of the robot is **not just its position**. For the control model used here it also includes orientation and actual velocities:

```math
X = \begin{bmatrix}x & y & \theta & v & \omega\end{bmatrix}^{T}
```

The control input has **two components**:

```math
u = \begin{bmatrix}v_{cmd} & \omega_{cmd}\end{bmatrix}^{T}
```

---

## 2. `collect_turtlebot_data.py` — Collecting motion data

### What the program does

This is the starting point of the project. It publishes velocity commands to **`/cmd_vel`**, subscribes to **`/odom`**, and saves the commanded and measured motion to `~/turtlebot_data.csv`.

The experiment contains several phases:

1. **Rest:** both commands are zero.
2. **Linear step:** command `v_cmd = 0.10 m/s`, with zero angular velocity.
3. **Linear stop:** set the forward command back to zero.
4. **Angular step:** command `omega_cmd = 0.60 rad/s`, with zero forward velocity.
5. **Angular stop:** stop rotating.
6. **Combined validation motion:** command `v_cmd = 0.08 m/s` and `omega_cmd = 0.35 rad/s`.
7. **Final stop.**

The basic script logs at approximately **20 Hz**, or one row every `0.05 s`, while the simulation is running.

### The mathematics

Every command is a two-dimensional vector:

```math
u(t) = \begin{bmatrix}v_{cmd}(t)\\\omega_{cmd}(t)\end{bmatrix}
```

The odometry message provides measured position and velocities. Heading is extracted from the odometry orientation quaternion. For quaternion components `(q_x, q_y, q_z, q_w)`, the yaw angle is:

```math
\theta = \operatorname{atan2}\!\left(2(q_w q_z+q_x q_y),\;1-2(q_y^2+q_z^2)\right)
```

For planar TurtleBot motion, the measured robot state can therefore be written as:

```math
X_{meas}(t) = [x(t),\;y(t),\;\theta(t),\;v(t),\;\omega(t)]^T
```

**Why collect both commands and measurements?** A velocity command is not automatically the velocity that the robot produces. For example, after a command changes from `0` to `0.1 m/s`, the measured velocity may take some time to rise. We want to model that difference.

A step input is particularly useful because it reveals two important things:

- **Steady-state response:** how close the final measured velocity is to the requested velocity.
- **Transient response:** how quickly the robot reaches that velocity.

**Output:** a CSV containing time, experiment phase, both commands, an odometry timestamp, robot pose and measured velocities. The timestamps are useful because a newly published command and the latest received odometry do not necessarily refer to exactly the same instant.

---

## 3. `identify_turtlebot.py` — Identifying the robot's velocity dynamics

### What the program does

This script reads the CSV from the first experiment, takes the linear and angular step-response segments, and fits a **first-order dynamic model** to each velocity.

It uses `scipy.optimize.least_squares` to find the parameters that make the model's predicted velocities as close as possible to the measured velocities. It also saves linear and angular step-response plots.

### A. Forward-velocity model

The model is:

```math
\tau_v\frac{dv}{dt}+v=K_v v_{cmd}
```

Equivalently:

```math
\frac{dv}{dt}=\frac{K_v v_{cmd}-v}{\tau_v}
```

This says that the rate of change of the forward velocity depends on the difference between **its target value** `K_v * v_cmd` and **its current value** `v`.

- `K_v` is the gain. If `K_v ≈ 1`, the measured steady-state speed is approximately equal to the command.
- `tau_v` is the time constant. A smaller `tau_v` means a faster response.

### B. Angular-velocity model

Rotation is modeled in the same way:

```math
\tau_{\omega}\frac{d\omega}{dt}+\omega=K_{\omega}\omega_{cmd}
```

Or:

```math
\frac{d\omega}{dt}=\frac{K_{\omega}\omega_{cmd}-\omega}{\tau_{\omega}}
```

This is an **empirical input/output model** of the robot's commanded and measured velocities, not a detailed model of its motors, wheels, friction or electrical components.

### C. Transfer functions

Assuming zero initial conditions, taking the Laplace transform gives:

```math
G_v(s)=\frac{V(s)}{V_{cmd}(s)}=\frac{K_v}{\tau_v s+1}
```

```math
G_{\omega}(s)=\frac{\Omega(s)}{\Omega_{cmd}(s)}=\frac{K_{\omega}}{\tau_{\omega}s+1}
```

These are **first-order transfer functions**. They describe how the robot responds to changes in a velocity command.

### D. Step-response equation

For a constant command `u` applied at time zero, with initial measured velocity `y_0`, a first-order system responds as:

```math
y(t)=Ku+(y_0-Ku)e^{-t/\tau}
```

Here `y` means either forward velocity or angular velocity.

- At `t = 0`, the exponential is `1`, so `y(0) = y_0`.
- As time becomes large, the exponential approaches `0`, so `y(∞) = Ku`.
- After one time constant `tau`, the response has covered about **63.2%** of the distance from its initial to its final value.

The script estimates `y_0` from the final samples of the preceding phase, rather than assuming the robot starts every step exactly at zero velocity.

### E. How the parameters are estimated

For trial values of `K` and `tau`, the equation above predicts the velocity at every recorded time. The prediction error at sample `i` is:

```math
e_i(K,\tau)=y_{pred}(t_i;K,\tau)-y_{meas}(t_i)
```

The algorithm minimizes the **sum of squared errors**:

```math
(K^*,\tau^*)=\underset{K,\tau}{\operatorname{argmin}}\sum_{i=1}^{n} e_i(K,\tau)^2
```

This is a **nonlinear least-squares identification problem**, because `tau` appears inside an exponential function.

The fit quality is summarized using **root mean squared error (RMSE)**:

```math
\operatorname{RMSE}=\sqrt{\frac{1}{n}\sum_{i=1}^{n}(y_{pred,i}-y_{meas,i})^2}
```

A smaller RMSE means that the predictions are closer to the recorded values, **for the data being evaluated**.

### Parameters used in the later controllers

| Parameter | Value | Meaning |
| --- | ---: | --- |
| `K_v` | `1.0011045` | Forward-velocity steady-state gain |
| `tau_v` | `0.0774945 s` | Forward-velocity time constant |
| `K_omega` | `0.9942472` | Angular-velocity steady-state gain |
| `tau_omega` | `0.032835 s` | Angular-velocity time constant |

**Important limitation:** These time constants are very short compared with a `20 Hz` logging interval (`0.05 s`). Timing delays, odometry filtering and discrete sampling make the fast transient difficult to estimate accurately. They should be treated as preliminary identified parameters.

---

## 4. `turtlebot_full_identification.py` — Full identification and validation experiment

### What the program does

This is a more complete version of the identification experiment. Instead of relying only on a CSV produced by another script, it:

1. Sends the test commands in Gazebo at approximately **25 Hz**.
2. Records odometry using odometry-message timestamps.
3. Records the times when command phases change.
4. Fits the linear and angular first-order models.
5. Predicts velocities and the resulting robot trajectory.
6. Validates the model using a **held-out combined-motion segment**.
7. Saves CSV predictions, plots and a text summary.

The program's math is similar to `identify_turtlebot.py`, but its **handling of time and changing commands** is more careful.

### A. Zero-order hold: command held constant between changes

ROS velocity commands are updated at discrete moments. Between those moments, the model assumes that the most recent command stays constant. This assumption is called a **zero-order hold (ZOH)**.

For the first-order model:

```math
\frac{dy}{dt}=\frac{Ku-y}{\tau}
```

if command `u_k` stays constant for a time interval `dt`, the exact velocity update is:

```math
y_{k+1}=e^{-dt/\tau}y_k+K(1-e^{-dt/\tau})u_k
```

Define:

```math
a=e^{-dt/\tau}
```

Then the same equation looks simpler:

```math
y_{k+1}=a y_k+K(1-a)u_k
```

**Meaning:** Part of the next velocity comes from the previous velocity (`a * y_k`), and the rest comes from the new command (`K * (1-a) * u_k`).

The function `first_order_response()` also splits a sampling interval if an input command changes inside it. This avoids assuming a command changed exactly when an odometry message arrived.

### B. Fitting using only the training data

The parameters are fitted with the familiar least-squares objective:

```math
\min_{K,\tau}\sum_{i\in\mathcal{T}}(y_{pred,i}(K,\tau)-y_{meas,i})^2
```

`T` here represents the **training samples**. The script deliberately excludes the `combined_validation` phase from parameter estimation.

Why? If we identified and judged the model on exactly the same motion, a good fit could simply reflect overfitting to that experiment. The combined-motion phase provides a more meaningful **unseen test** of the same identified dynamics.

### C. Predicting a path from velocities

After predicting forward and angular velocities, the program reconstructs robot motion using its **kinematic equations**:

```math
\dot{x}=v\cos\theta,\qquad\dot{y}=v\sin\theta,\qquad\dot{\theta}=\omega
```

Here `v` moves the robot in the direction of its current heading, while `omega` changes that heading.

To calculate the next position numerically, the script uses average velocities and a midpoint heading:

```math
\begin{aligned}
\bar v_k &= \frac{v_k+v_{k+1}}{2}\\
\bar\omega_k &= \frac{\omega_k+\omega_{k+1}}{2}\\
\theta_{mid,k} &= \theta_k+\frac{\bar\omega_k dt}{2}\\
x_{k+1} &= x_k+\bar v_k\cos(\theta_{mid,k})dt\\
y_{k+1} &= y_k+\bar v_k\sin(\theta_{mid,k})dt\\
\theta_{k+1} &= \theta_k+\bar\omega_k dt
\end{aligned}
```

Using midpoint heading is generally more accurate than using only the heading at the beginning of each time interval, especially during turns. This is still a **numerical approximation** of the pose trajectory.

### D. Comparing two predictions

The script makes two trajectory predictions:

- **Ideal-command prediction:** assume the actual velocities instantly equal the commanded velocities.
- **Identified-model prediction:** first calculate the velocities using the identified first-order dynamics, then integrate the motion.

Comparing both against odometry helps show whether modeling the velocity dynamics improves the simulated path.

The held-out **two-dimensional position RMSE** is:

```math
\operatorname{RMSE}_{position}=
\sqrt{\frac{1}{n}\sum_{i=1}^{n}\left[(x_{pred,i}-x_{odom,i})^2+(y_{pred,i}-y_{odom,i})^2\right]}
```

For the validation segment, the script resets the predicted trajectory to the measured position and heading **at the start of validation**. This isolates prediction error accumulated during that segment.

**Outputs:** `measurements.csv`, `predictions.csv`, `summary.txt`, velocity-response plots and a trajectory plot under `~/turtlebot_id_results/`.

**Caution:** `/odom` is a robot state estimate. It is useful for model checking but is not independent ground-truth motion measurement.

### `turtlebot_full_identification_commented.py`

This is the same overall identification/validation method with additional code explanations. It is included to make the modeling, fitting, timestamp handling and prediction steps easier to study. It does **not** introduce a different mathematical model.

---

## 5. `model_validation.py` — Testing the identified five-state model

### What the program does

This script takes the identified parameters, starts from a **measured initial state**, predicts future velocities and positions from recorded commands, and compares the predictions with the robot's odometry.

The goal is to answer an important question before building MPC:

**Is this mathematical model accurate enough to predict what the TurtleBot3 will do when I command it to move?**

### A. The complete state-space model

The state and input are:

```math
X=\begin{bmatrix}x\\y\\\theta\\v\\\omega\end{bmatrix},\qquad
u=\begin{bmatrix}v_{cmd}\\\omega_{cmd}\end{bmatrix}
```

The continuous-time model combines **kinematics** and **identified velocity dynamics**:

```math
\begin{aligned}
\dot{x} &= v\cos\theta\\
\dot{y} &= v\sin\theta\\
\dot{\theta} &= \omega\\
\dot{v} &= \frac{K_v v_{cmd}-v}{\tau_v}\\
\dot{\omega} &= \frac{K_{\omega}\omega_{cmd}-\omega}{\tau_{\omega}}
\end{aligned}
```

The first three equations describe how velocity changes the **pose** `(x, y, theta)`; the last two equations describe how commands change the **actual velocities** `(v, omega)`.

The model is nonlinear because of `cos(theta)` and `sin(theta)`. The velocity-response equations themselves are linear first-order models.

### B. Velocity prediction at each time step

For a time step `dt`, define:

```math
a_v=e^{-dt/\tau_v},\qquad a_{\omega}=e^{-dt/\tau_{\omega}}
```

The next velocities are:

```math
v_{k+1}=a_v v_k+K_v(1-a_v)v_{cmd,k}
```

```math
\omega_{k+1}=a_{\omega}\omega_k+K_{\omega}(1-a_{\omega})\omega_{cmd,k}
```

These velocity updates are **exact for each first-order model if the command is constant during the step**.

### C. Position and heading prediction

Using those two predicted velocities, the script then estimates the next pose with the midpoint integration equations from Section 4:

```math
\begin{aligned}
\bar v_k &= (v_k+v_{k+1})/2\\
\bar\omega_k &= (\omega_k+\omega_{k+1})/2\\
\theta_{mid,k} &= \theta_k+\bar\omega_k dt/2\\
x_{k+1} &= x_k+\bar v_k\cos(\theta_{mid,k})dt\\
y_{k+1} &= y_k+\bar v_k\sin(\theta_{mid,k})dt\\
\theta_{k+1} &= \theta_k+\bar\omega_k dt
\end{aligned}
```

Together, these steps define the **discrete prediction function**:

```math
X_{k+1}=f(X_k,u_k)
```

**One numerical prediction step.** Suppose the robot starts with `v = 0`, `omega = 0`, and receives `v_cmd = 0.10 m/s`, `omega_cmd = 0.20 rad/s` for `dt = 0.1 s`. With the identified parameters:

```math
\begin{aligned}
a_v &= e^{-0.1/0.0774945} \approx 0.2752 \\
a_{\omega} &= e^{-0.1/0.032835} \approx 0.0476 \\
v_1 &= 0.2752(0)+1.0011045(1-0.2752)(0.10) \\
&\approx 0.07256\;\mathrm{m/s} \\
\omega_1 &= 0.0476(0)+0.9942472(1-0.0476)(0.20) \\
&\approx 0.18939\;\mathrm{rad/s}
\end{aligned}
```

These are the **predicted actual velocities after the step**. The model then uses the average velocities during the step to estimate the new `x`, `y`, and `theta`. The MPC controller repeats exactly this prediction process for each of its eight candidate commands.

This function becomes the main mathematical component of the MPC controllers.

### D. Evaluating prediction errors

For any scalar quantity, such as forward velocity or heading, calculate:

```math
\operatorname{RMSE}(z)=\sqrt{\frac{1}{n}\sum_{i=1}^{n}(z_{pred,i}-z_{odom,i})^2}
```

Heading error should be handled carefully because `-pi` and `+pi` represent the same orientation. A wrapped angular difference is:

```math
\operatorname{wrap}(\Delta\theta)=\operatorname{atan2}(\sin\Delta\theta,\cos\Delta\theta)
```

In the earlier validation experiment, example results were approximately:

| Quantity | Reported RMSE |
| --- | ---: |
| Forward velocity | `0.00413 m/s` |
| Angular velocity, full test | `0.0733 rad/s` |
| Angular velocity, ignoring the first `0.25 s` | `0.00934 rad/s` |
| Position | `0.01234 m` |
| Heading | `0.05662 rad` |

These values belong to specific simulation runs; they are **not guaranteed performance across all trajectories**. In particular, the large early angular error deserves investigation before treating the identified time constants as reliable.

---

## 6. `nonlinear_controller.py` — Following a circle with nonlinear feedback

### What the program does

This script controls the robot so that it follows a **circular reference trajectory**. At each control cycle it reads the current pose, calculates where the robot should be, measures the tracking error, and turns that error into forward and angular velocity commands.

Unlike MPC, this controller does **not** optimize a sequence of future inputs. It uses the **current tracking error** in a feedback equation.

### A. Mathematical definition of the reference circle

For a circle with center `(x_c, y_c)`, radius `R`, and moving angle `phi(t)`, a counterclockwise reference trajectory can be written as:

```math
\begin{aligned}
\phi(t) &= \phi_0+\omega_r t\\
x_r(t) &= x_c+R\cos\phi(t)\\
y_r(t) &= y_c+R\sin\phi(t)\\
\theta_r(t) &= \phi(t)+\frac{\pi}{2}
\end{aligned}
```

The reference heading is tangent to the circle. The reference angular velocity and forward velocity are linked by:

```math
\omega_r=\frac{v_r}{R}
```

For this project:

```math
R=0.5\;\mathrm{m},\qquad v_r=0.1\;\mathrm{m/s},\qquad\omega_r=0.2\;\mathrm{rad/s}
```

### B. Position error in the robot's own coordinate frame

First calculate the difference between desired and actual positions in world coordinates:

```math
\Delta x=x_r-x,\qquad\Delta y=y_r-y
```

But the robot needs to know whether the target is **in front of it** or **to its side**. So we rotate the position error into the robot's local frame:

```math
\begin{bmatrix}e_x\\e_y\end{bmatrix}=
\begin{bmatrix}
\cos\theta & \sin\theta\\
-\sin\theta & \cos\theta
\end{bmatrix}
\begin{bmatrix}\Delta x\\\Delta y\end{bmatrix}
```

In expanded form:

```math
\begin{aligned}
e_x &= \cos\theta(x_r-x)+\sin\theta(y_r-y)\\
e_y &= -\sin\theta(x_r-x)+\cos\theta(y_r-y)
\end{aligned}
```

- `e_x > 0`: the reference point is in front of the robot.
- `e_y > 0`: the reference point is to the robot's left.

The heading error is:

```math
e_{\theta}=\operatorname{atan2}\left(\sin(\theta_r-\theta),\cos(\theta_r-\theta)\right)
```

Using `atan2(sin(...), cos(...))` ensures that the controller uses the shortest angular error, rather than incorrectly trying to turn almost `2*pi` radians when the heading crosses `-pi`/`+pi`.

### C. Nonlinear feedback law

The controller calculates its commands using:

```math
v_{cmd}=v_r\cos(e_{\theta})+k_xe_x
```

```math
\omega_{cmd}=\omega_r+k_yv_r e_y+k_{\theta}\sin(e_{\theta})
```

The feedback gains used in the earlier controller were:

```math
k_x=1,\qquad k_y=4,\qquad k_{\theta}=1.5
```

**What each term does:**

- `v_r * cos(e_theta)` gives forward motion aligned with the desired heading. If the heading error is large, this contribution becomes smaller.
- `k_x * e_x` accelerates or slows the robot to correct forward-position error.
- `omega_r` is the turning rate required simply to follow the reference circle.
- `k_y * v_r * e_y` adds a turn to correct sideways-position error.
- `k_theta * sin(e_theta)` corrects heading error.

The `cos` and `sin` terms are why the feedback law is **nonlinear**.

### D. Why this controller matters

It provides a useful **non-predictive baseline**: the robot continuously corrects its current position and heading errors. The MPC controller in the next section can then be evaluated on a similar reference trajectory.

A previous circular-trajectory experiment reported roughly **`0.0048 m` position RMSE**. This is a single-run simulation result, not a statistical performance guarantee.

---

## 7. `mpc_controller.py` — Model Predictive Control for trajectory tracking

### What the program does

This controller uses the five-state model identified earlier to decide which velocity commands to send. Instead of reacting only to the current error, it **predicts several possible future motions** and chooses the sequence with the lowest predicted cost.

It uses **Model Predictive Control (MPC)** with SciPy's **L-BFGS-B** optimizer.

### A. Prediction horizon and control variables

The controller uses:

```math
N=8,\qquad dt=0.1\;\mathrm{s}
```

So it predicts:

```math
T_{horizon}=N\cdot dt=8\cdot0.1=0.8\;\mathrm{s}
```

It starts from **one measured state**, `X_0`, and creates an initial guess for **eight future command pairs**:

```math
U=\{u_0,u_1,\ldots,u_7\},\qquad
u_k=\begin{bmatrix}v_{cmd,k}\\\omega_{cmd,k}\end{bmatrix}
```

Since each command pair contains two numbers, the optimizer adjusts **16 variables**. The controller's initial guess for circular tracking was eight repetitions of the reference command pair `(v_r, omega_r)`.

### B. Predicting eight future states

Each pair of commands is passed through the discrete model from Section 5:

```math
X_{k+1}=f(X_k,u_k)
```

The resulting prediction is:

```math
X_0\xrightarrow{u_0}X_1\xrightarrow{u_1}X_2
\xrightarrow{u_2}\cdots\xrightarrow{u_7}X_8
```

**Important distinction:** There are **8 control pairs** and **8 predicted future states**, but only **one current measured state** at the start of each optimization.

### C. What is the MPC cost function?

At each future step, MPC compares the predicted state with the desired reference trajectory. A typical tracking cost for this controller has four parts:

```math
\begin{aligned}
J_k ={}& q_p\left[(x_k-x_{ref,k})^2+(y_k-y_{ref,k})^2\right]\\
&+q_{\theta}\,e_{\theta,k}^2\\
&+r_v(v_{cmd,k-1}-v_{ref,k})^2\\
&+r_{\omega}(\omega_{cmd,k-1}-\omega_{ref,k})^2
\end{aligned}
```

Here the cost is evaluated at predicted state `X_k`, reached by applying control `u_(k-1)`.

- **Position cost:** penalizes being away from the desired path.
- **Heading cost:** penalizes facing the wrong direction.
- **Linear-command cost:** penalizes deviations from desired forward speed.
- **Angular-command cost:** penalizes deviations from desired turning rate.

The `q` and `r` coefficients determine how strongly each error matters. **This is the general tracking-cost structure; the exact numerical coefficients should match the version of `mpc_controller.py` in the repository.**

To avoid problems with angle wrapping, the heading error is:

```math
e_{\theta,k}=\operatorname{atan2}\left(\sin(\theta_{ref,k}-\theta_k),\cos(\theta_{ref,k}-\theta_k)\right)
```

### D. Total predicted cost

The optimizer does **not** minimize each step independently. It minimizes their **sum**:

```math
J(U)=J_1+J_2+\cdots+J_8=\sum_{k=1}^{8}J_k
```

The optimization problem is:

```math
U^*=\underset{U}{\operatorname{argmin}}\;J(U)
```

subject to command bounds such as:

```math
0\leq v_{cmd,k}\leq0.16\;\mathrm{m/s}
```

```math
-1\leq\omega_{cmd,k}\leq1\;\mathrm{rad/s}
```

**Why minimize the total and not every step separately?** An early turn can temporarily make the robot farther from the reference, but produce a much better position several steps later. MPC evaluates that trade-off over the whole horizon.

### E. What does the optimizer actually do?

`scipy.optimize.minimize(method="L-BFGS-B")` works approximately as follows:

1. Start with the current guess for the eight command pairs.
2. Predict the eight robot states using `predict()`.
3. Calculate the cost for all eight future steps using `cost()`.
4. Change the command sequence to seek a smaller **total** cost.
5. Repeat the prediction and cost calculation until a stopping condition is reached.
6. Return the best sequence found by this local optimizer.

The implementation does not supply an analytic Jacobian, so SciPy normally estimates derivatives numerically. L-BFGS-B is a **bounded, gradient-based local optimization method**; it does not guarantee finding the globally best trajectory.

### F. Receding-horizon control: why execute only one command?

This is the key MPC idea. After optimizing:

```math
U^*=\{u_0^*,u_1^*,\ldots,u_7^*\}
```

**only the first command pair** is published to the robot:

```math
u_{applied}=u_0^*
```

Approximately one control cycle later:

1. Read the **new actual** robot state from `/odom`.
2. Predict another eight future states from that new starting point.
3. Optimize eight new command pairs.
4. Execute only the new first pair.

This is called **receding-horizon control**. It makes the controller *closed-loop*: if the robot moves slightly differently from the model prediction, the next optimization starts from what actually happened.

The previous optimized commands are shifted to form the next initial guess:

```python
# From eight optimized pairs, discard the executed first pair.
# Repeat the last pair to keep eight pairs in the new guess.
U = np.r_[U[2:], U[-2:]]
```

This technique is called a **warm start**. It reduces unnecessary changes between successive optimization attempts.

A circular-path run previously produced about **`0.0041 m` position RMSE** (another run produced about `0.0028 m`). Repeated matched trials would be needed to establish whether MPC consistently improves on the nonlinear controller.

---

## 8. `mpc_waypoint_lidar.py` — MPC navigation with RViz goals and LiDAR

### What the program does

This is the most complete navigation controller in the project. It extends the identified-model MPC to move toward **waypoints selected in RViz** while taking **LiDAR measurements** into account.

Its main ROS interfaces are:

| Topic | Used for |
| --- | --- |
| `/odom` | Current state: `x, y, theta, v, omega` |
| `/scan` | Distances and directions of detected obstacles |
| `/move_base_simple/goal` | Waypoint positions selected with RViz 2D Nav Goal |
| `/cmd_vel` | Published linear and angular velocity commands |

Waypoints are accepted only when their frame is `odom`. The controller keeps a queue and works toward the first waypoint.

The mathematics has **three connected parts**: attractive direction to the waypoint, repulsive direction away from obstacles, and MPC optimization that considers both.

### A. Direction toward the waypoint

Let the current position and goal position be:

```math
p=\begin{bmatrix}x\\y\end{bmatrix},\qquad
p_g=\begin{bmatrix}x_g\\y_g\end{bmatrix}
```

The difference vector is:

```math
\Delta p=p_g-p=\begin{bmatrix}x_g-x\\y_g-y\end{bmatrix}
```

The Euclidean distance to the goal is:

```math
d_g=\sqrt{(x_g-x)^2+(y_g-y)^2}
```

Normalize the difference to obtain a unit vector pointing toward the goal:

```math
\mathbf a=\frac{p_g-p}{\|p_g-p\|}
```

This is the **attractive vector**. It tells the robot which direction it would prefer to travel if obstacles did not exist.

If the robot is already within **`GOAL_TOL = 0.12 m`**, the goal is marked reached rather than calculating this direction.

### B. Turning LiDAR ranges into obstacle points

A LiDAR scan contains ranges `r_i` and beam angles `beta_i` relative to the scanner. The code keeps every eighth beam for the optimization and uses nearby, valid points within **`LASER_RANGE = 0.9 m`**.

Assuming the sensor is at the robot center and aligned with the robot frame, convert the beam angle into the odometry frame:

```math
\alpha_i=\theta+\beta_i
```

Then approximate the position of each obstacle point:

```math
\begin{aligned}
o_{x,i}&=x+r_i\cos\alpha_i\\
o_{y,i}&=y+r_i\sin\alpha_i
\end{aligned}
```

So each detected obstacle is represented by a fixed point in the current odometry frame:

```math
o_i=\begin{bmatrix}o_{x,i}\\o_{y,i}\end{bmatrix}
```

**Approximation:** The program currently assumes the laser is centered and aligned with the robot. Accurate sensor placement would require the correct ROS TF transform. The obstacle points are also assumed stationary over one MPC prediction horizon.

### C. Repulsive vector: Artificial Potential Field idea

Nearby detected points generate vectors pointing **away** from the obstacles. Each point is weighted using:

```math
w_i=\left(1-\frac{r_i}{R_L}\right)^2,\qquad r_i<R_L
```

where `R_L = LASER_RANGE`.

The complete repulsive vector implemented in the script is:

```math
\mathbf r=-\frac{1}{5}\sum_{i=1}^{m}w_i
\begin{bmatrix}\cos\alpha_i\\\sin\alpha_i\end{bmatrix}
```

The negative sign makes the vector point **opposite** the measured obstacle directions. Close obstacles get larger weights than distant obstacles.

This is an **Artificial Potential Field (APF)-inspired** rule. It is a simple navigation heuristic, not a physically measured force.

### D. Combining the goal and obstacles

The controller combines attraction and repulsion:

```math
\mathbf d=\mathbf a+k_{rep}\mathbf r
```

with:

```math
k_{rep}=1.5
```

The desired navigation heading is the direction of the combined vector:

```math
\theta_d=\operatorname{atan2}(d_y,d_x)
```

**Interpretation:** The attractive vector points toward the waypoint, while the repulsive vector pushes the desired direction away from nearby walls or obstacles. The robot does not directly apply this vector as a velocity. Instead, **MPC receives its direction as a desired heading**.

### E. Desired forward speed

The navigation controller reduces speed when approaching the waypoint:

```math
v_{desired}=\min(0.12,\;0.7d_g)
```

When the goal is far away, the requested speed is limited to `0.12 m/s`. As the robot gets close, the desired speed decreases.

This speed reduction depends on **distance to the goal**, not directly on obstacle distance.

### F. MPC cost function for waypoint navigation

For each future step, the controller predicts a state using the **same five-state model** as the original MPC controller:

```math
X_{k+1}=f(X_k,u_k)
```

After prediction, it calculates four main penalties.

**1. Position error: move toward the waypoint**

```math
J_{position,k}=40\left[(x_g-x_k)^2+(y_g-y_k)^2\right]
```

**2. Heading error: follow the combined APF direction**

```math
e_{\theta,k}=\operatorname{atan2}(\sin(\theta_d-\theta_k),\cos(\theta_d-\theta_k))
```

```math
J_{heading,k}=3e_{\theta,k}^2
```

**3. Command penalty: prefer the desired speed and avoid unnecessary turning**

```math
J_{control,k}=0.2(v_{cmd,k-1}-v_{desired})^2+0.03\omega_{cmd,k-1}^{2}
```

**4. Obstacle penalty: discourage unsafe predicted positions**

For each obstacle point `o_i`, calculate the distance from the predicted robot position `p_k`:

```math
d_{ik}=\sqrt{(o_{x,i}-x_k)^2+(o_{y,i}-y_k)^2}
```

If the distance is larger than the safety threshold, there is no penalty. If it is smaller, the penalty increases quadratically:

```math
J_{obs,k}=w_{obs}\sum_{i=1}^{m}\left[\max(0,d_{safe}-d_{ik})\right]^2
```

With the current code constants:

```math
d_{safe}=0.30\;\mathrm{m},\qquad w_{obs}=200
```

**The full cost evaluated by the program** is therefore:

```math
\begin{aligned}
J(U)=\sum_{k=1}^{8}\Big[&40\left((x_g-x_k)^2+(y_g-y_k)^2\right)\\
&+3e_{\theta,k}^{2}\\
&+0.2(v_{cmd,k-1}-v_{desired})^2\\
&+0.03\omega_{cmd,k-1}^{2}\\
&+200\sum_{i=1}^{m}\max(0,0.30-d_{ik})^2\Big]
\end{aligned}
```

This formula is the mathematical version of the `cost()` function inside `mpc_waypoint_lidar.py`.

### G. Optimization and command limits

The optimization problem is:

```math
\underset{U}{\operatorname{minimize}}\;J(U)
```

subject to:

```math
0\leq v_{cmd,k}\leq0.16\;\mathrm{m/s},\qquad
-1\leq\omega_{cmd,k}\leq1\;\mathrm{rad/s}
```

It uses:

- **Prediction horizon:** `N = 8` steps.
- **Prediction time step:** `dt = 0.1 s`.
- **Optimizer:** SciPy `L-BFGS-B`.
- **Decision variables:** 8 linear and 8 angular velocity commands.
- **Initial guess:** repetitions of `(0.1 m/s, 0 rad/s)`, then a shifted warm start.

Just like the circular-tracking MPC, it publishes **only the first optimized command pair**, reads new odometry and LiDAR data, and solves the problem again.

### H. Safety checks and their limitations

The program stops if:

- No waypoint is available.
- Odometry or LiDAR data are missing or older than `0.6 s`.
- A valid LiDAR measurement is closer than **`STOP_DISTANCE = 0.18 m`**.
- The optimizer reports failure or an invalid solution.

The script warns if optimization lasts longer than `dt = 0.1 s`, because that may prevent consistent `10 Hz` execution.

**Important:** The obstacle term is a **soft penalty**, not a guaranteed collision-avoidance constraint. The model uses point obstacles, an approximate LiDAR pose and a short `0.8 s` horizon. Artificial Potential Fields can also get stuck in local minima or between symmetric obstacles. If the robot gets too close and triggers the stop condition, the current version stops without an automatic recovery maneuver.

---

## 9. How all the programs fit together

The mathematical connection between the programs is:

```text
collect_turtlebot_data.py
    Send commands and measure /odom
               |
               v
identify_turtlebot.py / turtlebot_full_identification.py
    Fit K_v, tau_v, K_omega, tau_omega
               |
               v
model_validation.py
    Predict X_next = f(X_current, command)
    Compare predicted states with odometry
               |
               +---------------------------+
               |                           |
               v                           v
nonlinear_controller.py              mpc_controller.py
    Correct current error                Predict 8 states
    using feedback                       Minimize total cost
               |                           |
               |                           v
               |                    mpc_waypoint_lidar.py
               |                        Add RViz waypoints
               |                        and LiDAR penalties
               +---------------------------+
                                           |
                                           v
                                   Publish /cmd_vel
                                           |
                                           v
                                        TurtleBot3
```

### The key mathematical difference between the controllers

**Nonlinear feedback controller:**

```math
u(t)=g(e_x(t),e_y(t),e_{\theta}(t),v_r(t),\omega_r(t))
```

It calculates a command from the **current error**.

**Model Predictive Controller:**

```math
\begin{aligned}
U^*&=\arg\min_U\sum_{k=1}^{N}J_k\\
\text{subject to }\;&X_{k+1}=f(X_k,u_k)\\
&u_k\in\mathcal U
\end{aligned}
```

It calculates a command by considering the **predicted cost of an entire future control sequence**.

Both are closed-loop controllers because they repeatedly obtain new robot measurements. Only MPC explicitly optimizes predicted future states.

---

## 10. Notes on model accuracy and future additions

The identified model provides a useful starting point for control, but its accuracy is limited by sampling, odometry estimation and the simplicity of the first-order dynamics. The controller comparisons cited above are preliminary simulation results. A fair comparison should use the same start states, reference trajectories, constraints and repeated trials, while also recording the optimization time.

The following material can be added later without changing the mathematical documentation:

- Identification and validation plots.
- Circular-trajectory comparisons between nonlinear control and MPC.
- RViz/Gazebo screenshots and navigation videos.
- Repeated-test RMSE, minimum obstacle distance and waypoint success rates.
- Installation and launch commands.

**Project status:** Simulation-based modeling and control study. The waypoint/LiDAR controller is a prototype and does not guarantee collision-free operation.
