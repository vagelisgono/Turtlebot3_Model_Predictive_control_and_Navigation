# TurtleBot3: System Identification, Nonlinear Control and Model Predictive Navigation

**A ROS/Gazebo mobile-robot control project built around experimentally identified dynamics, trajectory tracking, nonlinear model predictive control (NMPC), RViz waypoint navigation, and LiDAR-based obstacle avoidance.**

**Platform:** TurtleBot3 Burger · **Environment:** Ubuntu 20.04, ROS Noetic, Gazebo 11, RViz · **Language:** Python 3 · **Libraries:** NumPy, SciPy, Matplotlib

> **Project status:** The velocity models, open-loop validation, circular trajectory tracking, and circular MPC have been tested in Gazebo. RViz waypoint navigation with LiDAR/artificial-potential-field (APF) guidance and obstacle costs is implemented as a simulation prototype; reliable collision-free navigation has **not** yet been demonstrated.

## Contents

1. [Overview and objectives](#1-overview-and-objectives)
2. [System architecture](#2-system-architecture)
3. [ROS interface and software](#3-ros-interface-and-software)
4. [Experimental data collection](#4-experimental-data-collection)
5. [Mathematical modeling](#5-mathematical-modeling)
6. [System identification](#6-system-identification)
7. [Discrete-time prediction model](#7-discrete-time-prediction-model)
8. [Independent model validation](#8-independent-model-validation)
9. [Nonlinear trajectory-tracking controller](#9-nonlinear-trajectory-tracking-controller)
10. [Model predictive control](#10-model-predictive-control)
11. [RViz waypoint navigation](#11-rviz-waypoint-navigation)
12. [LiDAR and obstacle-aware MPC](#12-lidar-and-obstacle-aware-mpc)
13. [Experimental results](#13-experimental-results)
14. [Installation and running](#14-installation-and-running)
15. [Repository organization](#15-repository-organization)
16. [Limitations and future work](#16-limitations-and-future-work)

---

## 1. Overview and objectives

The goal of this project is to build a mobile-robot navigation system **from a mathematical model to a feedback controller**, rather than rely exclusively on a prebuilt navigation controller.

The development process is:

1. Excite the robot with known linear and angular velocity commands.
2. Record commanded velocities, odometry, pose, and timestamps.
3. Identify simple velocity-response dynamics from step-response data.
4. Validate the identified model using a different, combined-motion experiment.
5. Implement a nonlinear feedback controller for reference trajectory tracking.
6. Implement a nonlinear MPC using the identified model for future-state prediction.
7. Replace the fixed circular reference with waypoints selected interactively in RViz.
8. Introduce LiDAR-based repulsive guidance and predicted obstacle-clearance costs.

The main research question is whether **including experimentally identified velocity dynamics in predictive control** provides practical benefits compared with a simpler kinematic tracking controller. The current experiments provide a starting point; they do not yet establish a statistically reliable answer.

## 2. System architecture

```text
                   RViz: 2D Nav Goal
                          |
                          v
                    Waypoint queue
                          |
                          v
                    Attractive vector
                          |
LiDAR /scan ------> Repulsive vector ------> Desired heading
      |                                            |
      v                                            v
Obstacle points ------------------------------> MPC cost
                                                    ^
Odometry /odom ---> Current state -----------------|
                                                    |
                         Identified prediction model
                         x, y, theta, v, omega
                                                    |
                                       SciPy L-BFGS-B optimizer
                                                    |
                                        First optimized command
                                                    |
                                                /cmd_vel
                                                    |
                                                TurtleBot3
                                                    |
                                       Updated /odom and /scan
                                                    |
                                                Repeat
```

The classical circular MPC and nonlinear controller use the same robot platform but different control formulations. The final waypoint/LiDAR controller adds environmental information to the MPC objective.

## 3. ROS interface and software

| ROS topic | Message type | Usage |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | Linear and angular velocity commands |
| `/odom` | `nav_msgs/Odometry` | Robot pose, heading, and measured velocities |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR ranges and beam angles |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | RViz-selected waypoint positions |
| `/tf`, `/tf_static` | TF transforms | Coordinate relationships needed for visualization and sensor geometry |

The controller uses `linear.x` as forward velocity and `angular.z` as yaw rate. Orientation is extracted from the odometry quaternion as yaw using `euler_from_quaternion`.

## 4. Experimental data collection

### 4.1 Method

A ROS Python node publishes predefined command sequences on `/cmd_vel` while collecting `/odom` measurements. The original experimental sequence included:

| Phase | Duration | Linear command | Angular command | Purpose |
| --- | ---: | ---: | ---: | --- |
| Initial rest | 2 s | 0 | 0 | Establish an initial condition |
| Linear motion | 4 s | 0.10 m/s | 0 | Forward step response |
| Linear stop | 2 s | 0 | 0 | Observe deceleration |
| Angular motion | 4 s | 0 | 0.60 rad/s | Rotational step response |
| Angular stop | 2 s | 0 | 0 | Observe rotational deceleration |
| Combined motion | 4 s | 0.08 m/s | 0.35 rad/s | Independent validation segment |
| Final stop | 2 s | 0 | 0 | Stop robot |

A later version of the collection script may use slightly different phase labels/durations; the CSV should always be treated as the record of the actual experiment.

Recorded variables include:

```text
time, phase, v_cmd, omega_cmd, x, y, theta, v_measured, omega_measured
```

Some script versions use `w_cmd`, `v_meas`, `w_meas`, and add an `odom_stamp` column. Adapt offline analysis scripts to the actual CSV header.

The collected data serves two purposes:

- **Identification set:** The separate linear and angular step responses.
- **Validation set:** The combined translation-and-rotation response, which is not used to fit the parameters.

### 4.2 Data-quality checks

An early collection error assigned the X coordinate to both `x` and `y`:

```python
# Incorrect
x = msg.pose.pose.position.x
y = msg.pose.pose.position.x

# Correct
x = msg.pose.pose.position.x
y = msg.pose.pose.position.y
```

After correcting this error and repeating data collection, the model-validation position RMSE improved substantially. This illustrates why coordinate and logging checks must precede model assessment.

**Timing limitation:** Published commands and the latest received odometry sample may not have identical timestamps. The simulation odometry update rate was also changed during experimentation. Fast identified time constants can therefore be sensitive to sample rate, message timing, and phase transitions. Use message timestamps and synchronized measurements for more rigorous identification.

## 5. Mathematical modeling

### 5.1 Differential-drive kinematics

For planar mobile-robot motion, define pose:

$$
\mathbf q = \begin{bmatrix}x & y & \theta\end{bmatrix}^{T}
$$

where $x,y$ are the robot coordinates in the odometry frame and $\theta$ is its heading. With body-frame forward velocity $v$ and yaw rate $\omega$:

$$
\boxed{\begin{aligned}
\dot x &= v\cos\theta\\
\dot y &= v\sin\theta\\
\dot\theta &= \omega
\end{aligned}}
$$

The cosine and sine terms project the robot's forward velocity onto the world X and Y axes. For example, at $\theta=0$, forward movement is along positive X; at $\theta=\pi/2$, it is along positive Y.

### 5.2 Why add velocity dynamics?

A purely kinematic approximation assumes the robot immediately achieves its commanded speeds:

$$
v \approx v_{cmd},\qquad \omega \approx \omega_{cmd}.
$$

Instead, the project fits **first-order response models** to approximate how measured velocity follows commanded velocity:

$$
\boxed{\begin{aligned}
\tau_v\dot v + v &= K_v v_{cmd}\\
\tau_\omega\dot\omega + \omega &= K_\omega\omega_{cmd}.
\end{aligned}}
$$

- $K_v,K_\omega$ are dimensionless steady-state gains.
- $\tau_v,\tau_\omega$ are time constants in seconds.
- The gains indicate the eventual ratio between measured and commanded speeds.
- Smaller time constants imply a faster modeled response.

At steady state ($\dot v=\dot\omega=0$):

$$
v_{ss}=K_vv_{cmd},\qquad \omega_{ss}=K_\omega\omega_{cmd}.
$$

### 5.3 Five-state continuous-time model

Combining kinematics and identified velocity dynamics gives:

$$
\boxed{\mathbf X=\begin{bmatrix}x&y&\theta&v&\omega\end{bmatrix}^{T},\qquad
\mathbf u=\begin{bmatrix}v_{cmd}&\omega_{cmd}\end{bmatrix}^{T}}
$$

$$
\boxed{\dot{\mathbf X}=f(\mathbf X,\mathbf u)=
\begin{bmatrix}
v\cos\theta\\
v\sin\theta\\
\omega\\
(K_vv_{cmd}-v)/\tau_v\\
(K_\omega\omega_{cmd}-\omega)/\tau_\omega
\end{bmatrix}}
$$

This is a phenomenological input-to-velocity model, **not** a wheel-torque, motor-current, or full rigid-body dynamics identification.

## 6. System identification

### 6.1 Step-response equation

For a constant command $u$ applied to a first-order system,

$$
\tau\dot y+y=Ku,
$$

the analytical response from measured initial velocity $y_0$ is:

$$
\boxed{y(t)=Ku+(y_0-Ku)e^{-t/\tau}}.
$$

The Python identification script fits $(K,\tau)$ to the recorded response using SciPy nonlinear least squares, minimizing:

$$
\boxed{\min_{K,\tau}\sum_{i=1}^{n}\left[y_i-\hat y(t_i;K,\tau)\right]^2}.
$$

The fit is performed separately for linear and angular velocity. Parameter bounds prevent invalid negative gains or time constants.

### 6.2 Identified parameters

A reported fit after the higher-rate simulation experiments produced:

| Parameter | Identified value | Interpretation |
| --- | ---: | --- |
| $K_v$ | 1.0011045 | Linear steady-state gain |
| $\tau_v$ | 0.0774945 s | Estimated linear response time constant |
| $K_\omega$ | 0.9942472 | Angular steady-state gain |
| $\tau_\omega$ | 0.032835 s | Estimated angular response time constant |

The corresponding transfer functions are:

$$
\boxed{G_v(s)=\frac{1.0011045}{0.0774945s+1}}
$$

$$
\boxed{G_\omega(s)=\frac{0.9942472}{0.032835s+1}}.
$$

For example, a constant forward command of $0.08\,\mathrm{m/s}$ yields modeled steady-state speed:

$$
v_{ss}=1.0011045\times0.08\approx0.08009\,\mathrm{m/s}.
$$

**Interpretation caution:** The gains were consistently close to one, while estimated time constants varied with logging and simulation settings. These values are useful preliminary parameters, not definitive physical robot constants.

## 7. Discrete-time prediction model

MPC predicts the robot's state at discrete instants separated by $\Delta t$. The current controller uses:

```python
N = 8
DT = 0.1  # seconds
```

The total prediction horizon is $N\Delta t=0.8$ seconds.

### 7.1 Exact first-order velocity update under zero-order hold

Assuming each command is constant during one interval, define:

$$
a_v=e^{-\Delta t/\tau_v},\qquad a_\omega=e^{-\Delta t/\tau_\omega}.
$$

Then:

$$
\boxed{\begin{aligned}
v_{k+1}&=a_vv_k+K_v(1-a_v)v_{cmd,k}\\
\omega_{k+1}&=a_\omega\omega_k+K_\omega(1-a_\omega)\omega_{cmd,k}.
\end{aligned}}
$$

These are exact updates **for the fitted first-order velocity subsystems**, conditional on holding the command constant over the interval.

### 7.2 Approximate pose integration

The code approximates the velocity variation using endpoint averages:

$$
\bar v_k=\frac{v_k+v_{k+1}}{2},\qquad
\bar\omega_k=\frac{\omega_k+\omega_{k+1}}{2}.
$$

It estimates orientation halfway through the timestep:

$$
\theta_{mid}=\theta_k+\frac{\bar\omega_k\Delta t}{2}.
$$

Then predicts:

$$
\boxed{\begin{aligned}
x_{k+1}&=x_k+\bar v_k\cos(\theta_{mid})\Delta t\\
y_{k+1}&=y_k+\bar v_k\sin(\theta_{mid})\Delta t\\
\theta_{k+1}&=\theta_k+\bar\omega_k\Delta t.
\end{aligned}}
$$

This is a numerical approximation of pose evolution; averaging the endpoints is **not** the exact interval mean of an exponential velocity response.

### 7.3 One numerical prediction

Take the initial state $\mathbf X_0=[0,0,0,0,0]^T$ and command $v_{cmd}=0.10$ m/s, $\omega_{cmd}=0.20$ rad/s for $\Delta t=0.1$ s.

Using the fitted parameters:

$$
a_v\approx0.275,\qquad a_\omega\approx0.0476.
$$

The next predicted velocities are approximately:

$$
v_1\approx0.0726\,\mathrm{m/s},\qquad
\omega_1\approx0.1894\,\mathrm{rad/s}.
$$

With average velocities and midpoint orientation, the predicted next state is approximately:

$$
\boxed{\mathbf X_1\approx
\begin{bmatrix}
0.00363\\0.000017\\0.00947\\0.0726\\0.1894
\end{bmatrix}}.
$$

This same `predict()` operation is repeated across the MPC horizon.

## 8. Independent model validation

The fitted models were evaluated using the **combined-motion** segment rather than the step-response segments used for identification:

$$
v_{cmd}=0.08\,\mathrm{m/s},\qquad
\omega_{cmd}=0.35\,\mathrm{rad/s}.
$$

The validation program initializes predictions from the measured state at the start of the segment, then propagates the model **open-loop** using commanded inputs. It compares predicted versus recorded velocities, trajectory, and heading.

Position RMSE is computed as:

$$
\boxed{RMSE_{position}=\sqrt{\frac1n\sum_{i=1}^{n}
\left[(x_i-\hat x_i)^2+(y_i-\hat y_i)^2\right]}}.
$$

Heading error uses angle wrapping:

$$
e_{\theta,i}=\operatorname{atan2}
\left(\sin(\theta_i-\hat\theta_i),\cos(\theta_i-\hat\theta_i)\right).
$$

### Reported validation results

| Metric | Result |
| --- | ---: |
| Linear velocity RMSE | 0.004127 m/s |
| Angular velocity RMSE (full segment) | 0.073256 rad/s |
| Angular velocity RMSE (after first 0.25 s) | 0.009342 rad/s |
| Position RMSE | **0.012342 m (1.23 cm)** |
| Heading RMSE | 0.056622 rad (about 3.24°) |

The angular error was concentrated in an initial transient, including an unexpected negative angular-velocity spike. Its cause was not conclusively established; command/odometry timing and phase transitions are plausible contributors.

A supplementary comparison of instantaneous and first-order angular models *after* 0.25 s gave:

| Angular model | RMSE after 0.25 s |
| --- | ---: |
| Instantaneous $\omega=K_\omega\omega_{cmd}$ | 0.009341630 rad/s |
| First-order model | 0.009341826 rad/s |

The difference is negligible because both models have effectively reached steady state. **This does not show which model better captures the transient response.**

The comparisons use simulation odometry rather than independently measured ground-truth pose; therefore they are not evidence of millimeter-level physical navigation accuracy.

## 9. Nonlinear trajectory-tracking controller

### 9.1 Circular reference

An initial benchmark asks the robot to track a circle with:

$$
R=0.5\,\mathrm{m},\qquad v_r=0.1\,\mathrm{m/s},\qquad
\omega_r=\frac{v_r}{R}=0.2\,\mathrm{rad/s}.
$$

For initial pose $(x_0,y_0,\theta_0)$, the reference is constructed so that the robot starts on the desired trajectory:

$$
\theta_r(t)=\theta_0+\omega_rt,
$$

$$
\boxed{\begin{aligned}
x_r(t)&=x_0+R[\sin\theta_r(t)-\sin\theta_0]\\
y_r(t)&=y_0-R[\cos\theta_r(t)-\cos\theta_0].
\end{aligned}}
$$

The reference completes a circle after:

$$
T=\frac{2\pi}{\omega_r}\approx31.42\,\mathrm{s}.
$$

### 9.2 Robot-frame errors and feedback law

Let $\Delta x=x_r-x$, $\Delta y=y_r-y$. Transform global position error into the robot's local frame:

$$
\boxed{\begin{aligned}
e_x &= \cos\theta\,\Delta x+\sin\theta\,\Delta y\\
e_y &= -\sin\theta\,\Delta x+\cos\theta\,\Delta y\\
e_\theta &= \operatorname{wrap}(\theta_r-\theta).
\end{aligned}}
$$

The implemented nonlinear feedback law is:

$$
\boxed{\begin{aligned}
v_{cmd} &= v_r\cos e_\theta+k_xe_x\\
\omega_{cmd} &= \omega_r+k_yv_re_y+k_\theta\sin e_\theta.
\end{aligned}}
$$

The initial gains were $k_x=1$, $k_y=4$, and $k_\theta=1.5$, with command saturation. Forward correction reacts to longitudinal error, while angular correction reacts to lateral and heading errors.

This controller is a **kinematic feedback baseline**. It does not explicitly use the fitted $\tau_v,\tau_\omega$ inside its control law. A formal stability proof for the implemented, saturated controller has not yet been completed.

## 10. Model predictive control

### 10.1 Decision variables and prediction horizon

Unlike the direct feedback law, MPC searches for an entire sequence of future commands:

$$
\boxed{\mathbf U=
[v_{cmd,0},\omega_{cmd,0},\ldots,v_{cmd,N-1},\omega_{cmd,N-1}]^T}.
$$

For $N=8$, there are **16 optimization variables**, producing 8 predicted future states over 0.8 s.

The first iteration starts with a manually specified initial guess:

```python
U = np.tile([0.1, 0.0], N)
```

This initially represents eight straight-driving command pairs. **It is an optimizer starting point, not the executed plan.**

### 10.2 Circular-reference MPC objective

For circular tracking, the tested controller penalizes predicted position and heading error, velocity error, and command deviation:

$$
\begin{aligned}
J_{circle}(\mathbf U)=\sum_{k=1}^{N}\big[&60(e_{x,k}^2+e_{y,k}^2)
+3e_{\theta,k}^2\\
&+0.5(v_k-v_r)^2+0.2(\omega_k-\omega_r)^2\\
&+0.1(v_{cmd,k}-v_r)^2
+0.01(\omega_{cmd,k}-\omega_r)^2\big].
\end{aligned}
$$

**Implementation note:** The later simplified `mpc_controller.py` omitted the two measured-velocity error terms and retained the position, heading, and command penalties. The weights above document the earlier design; the precise executable cost is the one in the committed Python file.

### 10.3 Waypoint/LiDAR MPC objective

For the final navigation prototype, the reference is the current RViz waypoint $(x_g,y_g)$ and a LiDAR-influenced heading $\theta_d$. The `cost()` function implements:

$$
\boxed{\begin{aligned}
J(\mathbf U)=\sum_{k=1}^{N}\big[&40((x_g-x_k)^2+(y_g-y_k)^2)\\
&+3\,\operatorname{wrap}(\theta_d-\theta_k)^2\\
&+0.2(v_{cmd,k}-v_{desired})^2
+0.03\omega_{cmd,k}^2\\
&+J_{obs,k}\big].
\end{aligned}}
$$

The waypoint goal is held constant over a prediction horizon. This is **goal regulation with heading guidance**, not full time-parameterized reference-trajectory tracking.

### 10.4 Constraints

The implemented input limits are:

$$
\boxed{0\le v_{cmd,k}\le0.16\,\mathrm{m/s}}
$$

$$
\boxed{-1\le\omega_{cmd,k}\le1\,\mathrm{rad/s}}.
$$

They are passed as bounds to SciPy. The collision penalty is **soft**, not a hard guarantee that predicted positions remain clear of obstacles.

### 10.5 Optimization with SciPy

The controller calls:

```python
result = minimize(
    self.cost,
    U,
    args=(state, goal, heading, obstacles, speed),
    method="L-BFGS-B",
    bounds=bounds,
    options={"maxiter": 50, "ftol": 1e-5, "gtol": 1e-4}
)
```

`L-BFGS-B` is a bounded, gradient-based numerical optimizer. It evaluates the cost of candidate command sequences, estimates how changing the commands changes the cost, and searches for a lower-cost sequence. Because the motion model and obstacles make the problem nonlinear, the solver generally finds a **local solution**, not a guaranteed global optimum.

The optimization is **single shooting**: candidate controls are explicit decision variables, and the dynamics are enforced by calling `predict()` repeatedly inside `cost()`. The predicted states are calculated from the controls rather than optimized as independent decision variables.

### 10.6 Receding-horizon execution

Although the optimizer returns eight future command pairs, only the first one is sent:

```python
U = result.x
cmd.linear.x = float(U[0])
cmd.angular.z = float(U[1])
self.pub.publish(cmd)
```

At the next control cycle the robot's current state is measured again and a new eight-step problem is solved. The previous solution is shifted to initialize the next optimization (**warm start**):

```python
U = np.r_[U[2:], U[-2:]]
```

The logical cycle is:

```text
Measure state -> Guess 8 commands -> Predict 8 future states
       -> Sum 8 step costs -> Optimize all commands
       -> Execute first pair -> Measure again -> Replan
```

The optimizer minimizes the **total horizon cost**, not each step's cost independently. A temporarily higher cost at one step can be accepted if it improves the complete predicted sequence.

## 11. RViz waypoint navigation

The final prototype receives `PoseStamped` messages from RViz's **2D Nav Goal** tool on `/move_base_simple/goal`.

- Each selected $(x_g,y_g)$ is appended to a waypoint queue.
- The controller tracks the **first** waypoint.
- A waypoint is removed when the robot comes within `GOAL_TOL = 0.12 m`.
- The next waypoint becomes active.
- Goal orientation from the RViz arrow is currently ignored: only the clicked position is used.

The implementation expects RViz goals in the `odom` frame. A goal in another frame is rejected rather than transformed. For visualization, the `/scan` display also needs a valid TF path from its sensor frame (often `base_scan`) to `odom`.

The nominal forward-speed preference depends on remaining waypoint distance:

```python
speed = min(0.12, 0.7 * distance)
```

This is a **soft reference** in the cost function, not a hard speed cap. The actual linear command remains bounded by 0.16 m/s.

## 12. LiDAR and obstacle-aware MPC

### 12.1 Reading obstacle detections

The `/scan` callback receives laser ranges $d_i$ and beam directions $\alpha_i$. Invalid measurements are filtered; a subset (every eighth beam) is retained to reduce computational cost. The nearest valid range from the full scan is used for the emergency-stop check.

### 12.2 Transforming detections to obstacle points

Approximating the LiDAR as located at the robot center and aligned with its body frame, measured ranges are converted into odometry-frame points:

$$
\boxed{\begin{aligned}
x_{obs,i}&=x+d_i\cos(\theta+\alpha_i)\\
y_{obs,i}&=y+d_i\sin(\theta+\alpha_i).
\end{aligned}}
$$

**Limitation:** This approximation ignores the actual transform between `base_scan` and `base_link`, and uses the latest robot pose rather than pose synchronized to the scan timestamp. A TF-based transformation is a necessary improvement.

### 12.3 Attractive and repulsive vectors

The attractive direction toward the waypoint is normalized:

$$
\boxed{\mathbf F_{att}=\frac{1}{\sqrt{(x_g-x)^2+(y_g-y)^2}}
\begin{bmatrix}x_g-x\\y_g-y\end{bmatrix}}.
$$

For retained obstacles with $d_i<d_0$, the code assigns weights:

$$
w_i=\left(1-\frac{d_i}{d_0}\right)^2,
\quad d_0=\texttt{LASER\_RANGE}.
$$

The repulsive vector is:

$$
\boxed{\mathbf F_{rep}=-\frac15\sum_i w_i
\begin{bmatrix}\cos(\theta+\alpha_i)\\\sin(\theta+\alpha_i)\end{bmatrix}}.
$$

The factor $1/5$ is a heuristic scaling choice, not a physical law. The combined guidance is:

$$
\boxed{\mathbf F=\mathbf F_{att}+k_{rep}\mathbf F_{rep}},
\qquad
\boxed{\theta_d=\operatorname{atan2}(F_y,F_x)}.
$$

The vector is **not** added to $(v_{cmd},\omega_{cmd})$: the repulsive field changes the **desired heading** inside the MPC cost.

### 12.4 Direct obstacle penalty in predicted states

For predicted robot center $(x_k,y_k)$, the distance to retained LiDAR point $i$ is:

$$
d_{i,k}=\sqrt{(x_{obs,i}-x_k)^2+(y_{obs,i}-y_k)^2}.
$$

The code adds:

$$
\boxed{J_{obs,k}=k_{obs}\sum_i \max(0,d_{safe}-d_{i,k})^2}.
$$

This penalty is zero beyond the selected safety distance and increases as predicted positions approach detected points. It influences **all eight predicted steps**.

The latest available `mpc_waypoint_lidar.py` prototype uses:

| Setting | Value | Purpose |
| --- | ---: | --- |
| `LASER_RANGE` | 0.90 m | LiDAR range considered for MPC |
| `SAFE_DISTANCE` | 0.30 m | Obstacle-cost activation threshold |
| `STOP_DISTANCE` | 0.18 m | Emergency-stop threshold |
| `REP_GAIN` | 1.5 | APF repulsion multiplier |
| `OBSTACLE_WEIGHT` | 200 | Obstacle-cost weight |
| `GOAL_TOL` | 0.12 m | Waypoint arrival tolerance |

Larger margins and a turning-recovery heuristic were discussed during testing, but **are not treated here as tested changes**. Confirm the committed script before reporting different values.

### 12.5 Safety behavior and known failure mode

The code stops if required sensors are stale, if optimization fails, or if the nearest LiDAR return is below `STOP_DISTANCE`.

During testing, the robot was observed to approach obstacles too closely and then stop. This is consistent with the current reactive stop: after stopping, the robot can remain stuck because it does not implement a complete escape or global replanning behavior.

The APF can also have **local minima**, including symmetric obstacles directly ahead. The short 0.8 s horizon and an obstacle cost based on robot-center-to-point distance further limit avoidance performance. No collision-free guarantee is claimed.

## 13. Experimental results

### 13.1 System identification and validation

The velocity gains were close to one. The 4-second combined-motion test achieved about **1.23 cm open-loop predicted position RMSE** after correcting a data-collection coordinate bug. The angular-velocity error was much smaller after removing the first 0.25 s from evaluation, but that does **not** validate the initial transient.

### 13.2 Trajectory-tracking results

| Experiment | Reported position RMSE |
| --- | ---: |
| Nonlinear kinematic controller, circular reference | **0.0048 m (4.8 mm)** |
| Five-state MPC, circular reference — first reported run | **0.0028 m (2.8 mm)** |
| Five-state MPC, circular reference — later run | **0.0041 m (4.1 mm)** |

The earlier MPC run produced repeated solver convergence warnings. The later run used adjusted `L-BFGS-B` settings (`maxiter=50`, `ftol=1e-5`, `gtol=1e-4`) and completed without the reported warnings.

**These values are individual simulation results, not a controlled statistical comparison.** The baseline tracking experiment starts with almost zero initial reference error, while the controllers may differ in timing and other conditions. The numbers are not sufficient to claim that MPC generally outperforms nonlinear feedback control.

### 13.3 Obstacle-navigation status

RViz waypoint input and LiDAR-guided MPC have been implemented. An obstacle-related stopping issue was observed. No verified multi-scenario navigation success rate, collision rate, or minimum-clearance distribution is available yet.

### Recommended plots for the repository

Add project screenshots and plots under `docs/images/` when available:

- Linear and angular step-response plots, with measured vs fitted curves.
- Measured vs predicted XY trajectory from the independent validation set.
- Angular-velocity validation plot showing the initial transient.
- Circular-reference trajectory versus measured paths for both controllers.
- Tracking-error-versus-time and command-versus-time plots.
- RViz/Gazebo screenshots showing clicked waypoints and LiDAR detections.

## 14. Installation and running

### 14.1 Requirements

- Ubuntu 20.04
- ROS Noetic and a working catkin workspace
- Gazebo 11 and TurtleBot3 Gazebo/description packages
- TurtleBot3 Burger model
- Python 3 with `numpy`, `scipy`, `matplotlib`
- RViz for interactive waypoint selection

> ROS Noetic / Ubuntu 20.04 are legacy environments; the instructions document the project's development setup rather than recommending them for new production deployments.

### 14.2 Build and source workspace

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
export TURTLEBOT3_MODEL=burger
```

Make the scripts executable as needed:

```bash
chmod +x ~/catkin_ws/src/turtlebot_control_lab/scripts/*.py
```

### 14.3 Start TurtleBot3 simulation

```bash
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```

For obstacle experiments use an installed world with obstacles, for example:

```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

### 14.4 Run identification experiments

In another terminal:

```bash
source ~/catkin_ws/devel/setup.bash
rosrun turtlebot_control_lab data_collection.py
python3 ~/catkin_ws/src/turtlebot_control_lab/scripts/system_identification.py
python3 ~/catkin_ws/src/turtlebot_control_lab/scripts/model_validation.py
```

The original experiment scripts write/read `~/turtlebot_data.csv`. Use your **actual script filenames** if they differ from the names above. Do not run a data-collection node and a feedback controller simultaneously because both publish `/cmd_vel`.

### 14.5 Run circular trajectory tracking

Choose **one** controller at a time:

```bash
rosrun turtlebot_control_lab nonlinear_controller.py
```

or:

```bash
rosrun turtlebot_control_lab mpc_controller.py
```

### 14.6 Run RViz waypoint and LiDAR MPC

Launch RViz in a separate terminal:

```bash
source ~/catkin_ws/devel/setup.bash
rosrun rviz rviz
```

In RViz:

1. Set **Global Options → Fixed Frame** to `odom`.
2. Add **LaserScan** and set its topic to `/scan`.
3. Select **2D Nav Goal** and click a reachable location; this publishes `/move_base_simple/goal`.
4. Ensure the TF tree connects the scan frame to `odom` so LiDAR returns can be displayed.

Start the navigation controller:

```bash
rosrun turtlebot_control_lab mpc_waypoint_lidar.py
```

The robot waits for a goal, queues additional clicks, and attempts to reach the goals in order. **Only test obstacle avoidance in simulation at this stage.**

### 14.7 Useful debugging commands

```bash
rostopic list
rostopic hz /odom
rostopic hz /scan
rostopic echo -n 1 /scan/header
rostopic echo /move_base_simple/goal
rosrun tf tf_echo odom base_scan
```

If RViz reports a missing TF for `/scan`, inspect the actual `frame_id` from `/scan/header`, then verify a matching transform chain. Do not invent a static transform without checking the sensor's physical or URDF mounting transform.

## 15. Repository organization

One reasonable layout is:

```text
TurtleBot3-Model-Predictive-Control-and-Navigation/
├── README.md
├── .gitignore
├── CMakeLists.txt
├── package.xml
├── scripts/
│   ├── data_collection.py
│   ├── system_identification.py
│   ├── model_validation.py
│   ├── nonlinear_controller.py
│   ├── mpc_controller.py
│   └── mpc_waypoint_lidar.py
└── docs/
    └── images/                 # Optional screenshots and plots
```

**This is a suggested organization.** Include only files actually present in the repository; script names and CSV column labels varied between development versions.

## 16. Limitations and future work

1. **Validate identification timing.** Synchronize commands with odometry messages, repeat step tests at multiple amplitudes, and test transient fit on independent commands.
2. **Compare model structures.** Test instantaneous kinematics versus first-order velocity models and quantify whether identified dynamics improve multi-step prediction and control.
3. **Run controlled controller comparisons.** Use identical waypoints, initial errors, random seeds, and obstacle layouts; report repeated-trial mean, variance, and worst-case performance.
4. **Measure real-time feasibility.** Record optimization duration, missed deadlines, solver success, and actual closed-loop update rate.
5. **Improve obstacle geometry.** Use accurate TF transforms, scan timestamps, robot radius plus clearance, obstacle clustering, and robust handling of missing returns.
6. **Improve local planning.** Consider longer horizons, terminal costs, trajectory/control-rate smoothing, collision constraints and a global planner for APF dead ends.
7. **Add evidence.** Publish plots, RViz/Gazebo screenshots, a short demonstration video, and a reproducible set of launch/configuration files.
8. **Optional hardware evaluation.** After simulation and safety validation, assess transfer to a physical TurtleBot3.

---

## Summary

This project demonstrates a complete early-stage control-engineering workflow: **collect measurements → estimate a dynamical model → validate predictions → implement nonlinear feedback → optimize future motion with MPC → incorporate RViz goals and LiDAR obstacle information**.

The central technical result is an MPC that uses the experimentally identified five-state model during each prediction and optimization cycle. The main outstanding engineering challenge is reliable obstacle-aware navigation and stronger experimental validation.
