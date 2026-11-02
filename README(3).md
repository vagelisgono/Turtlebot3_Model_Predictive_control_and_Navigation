# TurtleBot3 — Explanation of Each Python Program and Its Mathematics

## 1. `collect_turtlebot_data.py`

### What the program does

This program collects experimental data from a TurtleBot3 in Gazebo. It sends forward and angular velocity commands on `/cmd_vel` and reads the robot's measured movement from `/odom`. Every recorded row contains the command, the position, the heading, and the measured velocities. The results are saved in `~/turtlebot_data.csv`.

The experiment runs through rest, forward movement, stopping, rotation, stopping again, and combined forward/rotational movement. The commands are intentionally simple so the response to each input can be studied separately.

### The mathematics

The **input** to the robot is a pair of commands:

```text
u(t) = [v_cmd(t), w_cmd(t)]

v_cmd = requested forward speed (m/s)
w_cmd = requested turning rate (rad/s)
```

The **measured state** is:

```text
X(t) = [x(t), y(t), theta(t), v(t), w(t)]

x, y  = robot position in the odometry frame (m)
theta = robot heading (rad)
v     = measured forward speed (m/s)
w     = measured turning rate (rad/s)
```

ROS stores orientation as a quaternion. The program converts it to a yaw angle using `euler_from_quaternion`. The planar yaw conversion is:

```text
theta = atan2(2 * (qw*qz + qx*qy),
              1 - 2 * (qy^2 + qz^2))
```

A **step input** is a command that changes suddenly and is then held constant. In the linear experiment, for example:

```text
v_cmd(t) = 0.00 m/s    before the step
v_cmd(t) = 0.10 m/s    during the step
```

The question is whether the measured `v(t)` follows that command immediately or takes time to reach it. The same idea is applied to turning, with `w_cmd = 0.60 rad/s`.

The script records approximately 20 samples per second:

```text
sampling interval ~= 1 / 20 = 0.05 s
```

It also records the odometry timestamp. This matters because the most recently received odometry measurement may be slightly older than the command recorded beside it. These measurements become the input to the identification scripts.

---

## 2. `identify_turtlebot.py`

### What the program does

This script reads `~/turtlebot_data.csv` and estimates how the robot's **actual linear and angular velocities** respond to the commands. It fits one first-order model for forward motion and another for rotation. It uses SciPy's `least_squares` function and creates plots comparing predicted and measured velocities.

### A. First-order velocity model

The model for forward movement is:

```text
tau_v * dv/dt + v = Kv * v_cmd

or:

dv/dt = (Kv * v_cmd - v) / tau_v
```

The model for turning is:

```text
tau_w * dw/dt + w = Kw * w_cmd

or:

dw/dt = (Kw * w_cmd - w) / tau_w
```

Here:

- `Kv` and `Kw` are **gains**: they determine the measured steady-state velocity for a constant command.
- `tau_v` and `tau_w` are **time constants**: they determine how quickly the measured velocity approaches its steady-state value.
- `dv/dt` and `dw/dt` are the rates of change of the actual velocities.

For example, if `Kv = 1`, a constant `v_cmd = 0.10 m/s` produces a steady-state speed of approximately `0.10 m/s`. A small `tau_v` means it approaches that speed quickly.

### B. Transfer functions

For zero initial conditions, the Laplace transforms of these equations are:

```text
Gv(s) = V(s) / Vcmd(s) = Kv / (tau_v*s + 1)

Gw(s) = W(s) / Wcmd(s) = Kw / (tau_w*s + 1)
```

These are **command-to-velocity transfer functions**. They model what happens between asking the robot to move and measuring its movement. They are not detailed equations for individual wheel motors or robot mass.

### C. Response to a constant command

For a command `u` held constant after time zero, the first-order equation has the solution:

```text
y(t) = K*u + (y0 - K*u) * exp(-t/tau)

where:
y(t) = predicted velocity at time t
y0   = measured velocity just before the step
K    = gain
tau  = time constant
u    = constant velocity command
```

This formula shows exactly how the velocity approaches `K*u`:

```text
at t = 0:             y(0) = y0
at t = tau:           63.2% of the change has occurred
as t becomes large:   y(t) approaches K*u
```

The code estimates `y0` from the last samples of the preceding experiment phase. It then uses this formula to predict the whole step response for trial values of `K` and `tau`.

### D. Estimating `K` and `tau`

For each measurement, the program calculates the difference between the predicted velocity and the recorded velocity:

```text
error_i = predicted_velocity_i - measured_velocity_i
```

It chooses the parameters that make the **sum of squared errors** as small as possible:

```text
(K_best, tau_best) = argmin over K,tau of:

    sum from i=1 to n [predicted_velocity_i(K,tau)
                      - measured_velocity_i]^2
```

This is **nonlinear least squares**, since `tau` is inside an exponential.

To measure the fit quality, the script calculates:

```text
RMSE = sqrt((1/n) * sum from i=1 to n (predicted_i - measured_i)^2)
```

RMSE has the same unit as the measured signal: `m/s` for forward velocity, `rad/s` for angular velocity. A smaller RMSE means closer agreement with those data.

The parameter values later used in the controllers are:

```text
Kv    = 1.0011045
tau_v = 0.0774945 s
Kw    = 0.9942472
tau_w = 0.032835 s
```

The angular time constant is shorter than the collector's `0.05 s` logging interval, so the data do not resolve that fast transient particularly well. The fitted values are useful working estimates, not exact physical constants.

---

## 3. `turtlebot_full_identification.py`

### What the program does

This is a more complete system-identification experiment. One script sends test commands, records odometry with timestamps, estimates the same four first-order parameters, predicts the robot's trajectory, and checks the predictions against a separate combined-motion part of the experiment.

It records command-change times independently of odometry times. This improves the treatment of input timing.

### A. Velocity prediction with zero-order hold

The input is treated as **constant between consecutive command changes**. This assumption is called a zero-order hold (ZOH).

Starting with:

```text
dy/dt = (K*u - y) / tau
```

if `u` stays constant for `dt` seconds, the exact velocity update is:

```text
y_next = K*u + (y_now - K*u) * exp(-dt/tau)
```

Equivalently, if:

```text
a = exp(-dt/tau)
```

then:

```text
y_next = a*y_now + K*(1-a)*u
```

The first term carries part of the previous velocity forward. The second term moves the predicted velocity toward the new command's steady-state value.

For forward and angular velocities separately:

```text
v_next = av*v + Kv*(1-av)*v_cmd
av     = exp(-dt/tau_v)

w_next = aw*w + Kw*(1-aw)*w_cmd
aw     = exp(-dt/tau_w)
```

The function `first_order_response()` even splits a time interval when a command changes *between* two odometry measurements. It applies the correct command over each smaller interval instead of pretending the command changed at an odometry timestamp.

### B. Parameter estimation and separate validation

The identification still minimizes squared velocity errors:

```text
J(K, tau) = sum over training samples:

            [velocity_predicted_i(K,tau) - velocity_measured_i]^2
```

The script fits the parameters using the early rest/step/stop measurements. The combined-motion portion is excluded from fitting. That allows the same identified model to be tested on a different combination of forward and turning commands.

### C. Converting predicted velocity into a predicted path

Velocity is not yet position. To predict where the robot moves, the program uses differential-drive **planar kinematics**:

```text
dx/dt     = v * cos(theta)
dy/dt     = v * sin(theta)
dtheta/dt = w
```

The forward speed `v` acts along the robot's heading `theta`. The turning rate `w` changes that heading.

For each recorded time interval, the code approximates the average velocities:

```text
v_avg = (v_now + v_next) / 2
w_avg = (w_now + w_next) / 2
```

It then estimates the heading halfway through the step:

```text
theta_mid = theta_now + w_avg*dt/2
```

Finally, it updates the position and heading:

```text
x_next     = x_now + v_avg*cos(theta_mid)*dt
y_next     = y_now + v_avg*sin(theta_mid)*dt
theta_next = theta_now + w_avg*dt
```

This is **midpoint-style numerical integration** for the pose. Unlike the first-order velocity update, the pose update is an approximation, not the exact solution of the full turning trajectory.

### D. Comparing predicted and measured trajectories

The program computes two paths:

- **Ideal-command path:** integrate `v_cmd` and `w_cmd` directly, as though the measured velocities instantly matched the commands.
- **Identified-model path:** predict `v` and `w` first, then integrate those predicted velocities.

To evaluate position during the held-out section, the code starts from the measured pose at that section's beginning and calculates:

```text
position_error_i^2 = (x_pred_i - x_odom_i)^2
                   + (y_pred_i - y_odom_i)^2

position_RMSE = sqrt((1/n) * sum from i=1 to n position_error_i^2)
```

Velocity RMSE is computed separately for `v` and `w`. The distinction matters: a model can predict speed well but still accumulate position error when integrated over time.

---

## 4. `turtlebot_full_identification_commented.py`

### What the program does

This is the **commented, educational version** of the full identification experiment. It explains the role of the ROS messages, command timing, numerical prediction, parameter fitting, trajectory integration, and validation inside the code.

### The mathematics

It does not introduce a new model. It uses the same chain of calculations as the previous program:

```text
1. Dynamic model:
   dy/dt = (K*u - y)/tau

2. Velocity prediction for a constant command:
   y_next = K*u + (y_now - K*u)*exp(-dt/tau)

3. Fit parameters:
   minimize sum of squared prediction errors over K and tau

4. Integrate planar movement:
   dx/dt = v*cos(theta)
   dy/dt = v*sin(theta)
   dtheta/dt = w

5. Compare with odometry:
   RMSE = sqrt(mean(prediction_error^2))
```

The purpose of this version is to make each mathematical calculation easier to follow in the Python implementation; it is not a separate control method.

---

## 5. `model_validation.py`

### What the program does

This program checks whether the identified model can reproduce **both the robot's velocities and its position** from recorded commands. It starts at a measured robot state, predicts how that state changes, and compares the predictions with odometry. The same prediction equations are later used by MPC.

### A. Five-state mathematical model

The state contains five numbers and the input contains two:

```text
X = [x, y, theta, v, w]
u = [v_cmd, w_cmd]
```

The continuous-time model combines kinematics and the identified first-order dynamics:

```text
dx/dt     = v * cos(theta)
dy/dt     = v * sin(theta)
dtheta/dt = w

dv/dt     = (Kv*v_cmd - v) / tau_v
dw/dt     = (Kw*w_cmd - w) / tau_w
```

The first three equations connect velocity to **position and orientation**. The last two connect **commanded velocities to measured velocities**. Together they form a five-state nonlinear model: the sine and cosine terms make the overall model nonlinear.

### B. Predicting velocities one step ahead

For each prediction interval `dt`:

```text
av = exp(-dt/tau_v)
aw = exp(-dt/tau_w)

v_next = av*v_now + Kv*(1-av)*v_cmd
w_next = aw*w_now + Kw*(1-aw)*w_cmd
```

These equations predict the velocity the robot will have *after* the command has acted for `dt` seconds. They do not simply assume `v_next = v_cmd` and `w_next = w_cmd`.

For example, with `dt = 0.1 s`, `v_now = 0`, and `v_cmd = 0.10 m/s`:

```text
av     = exp(-0.1/0.0774945) ~= 0.275
v_next = 0.275*0 + 1.0011045*(1 - 0.275)*0.10
       ~= 0.0726 m/s
```

So the model predicts that the robot has not yet fully reached the requested speed after the first step.

### C. Predicting position and orientation

The new velocities are used to calculate the average motion during the interval:

```text
v_avg     = (v_now + v_next)/2
w_avg     = (w_now + w_next)/2
theta_mid = theta_now + w_avg*dt/2

x_next     = x_now + v_avg*cos(theta_mid)*dt
y_next     = y_now + v_avg*sin(theta_mid)*dt
theta_next = theta_now + w_avg*dt
```

This makes one prediction function:

```text
X_next = f(X_now, u_now)
```

Calling the function repeatedly predicts a complete path:

```text
X0 --u0--> X1 --u1--> X2 --u2--> X3 ...
```

### D. Measuring model error

Predictions are compared with recorded odometry:

```text
RMSE(v) = sqrt(mean((v_pred - v_meas)^2))
RMSE(w) = sqrt(mean((w_pred - w_meas)^2))
```

For heading, an angular difference should be wrapped into the range from `-pi` to `pi` so that angles near `+pi` and `-pi` are treated as nearly identical:

```text
angle_error = atan2(sin(theta_pred - theta_meas),
                    cos(theta_pred - theta_meas))
```

Position error can be measured using the two-dimensional distance:

```text
position_error = sqrt((x_pred - x_meas)^2 + (y_pred - y_meas)^2)
```

The point of this program is to check the **prediction model itself**, independently of whether a controller can follow a path. The model must be able to predict movement before it can be useful inside MPC.

---

## 6. `nonlinear_controller.py`

### What the program does

This controller makes the TurtleBot3 follow a **moving circular reference trajectory**. At each control cycle, it computes where the robot should be, transforms the tracking error into the robot's coordinate frame, and calculates forward and angular velocity commands directly from that error.

It is a **feedback controller**, not an optimizer: it does not simulate eight future command choices like MPC.

### A. Generating the reference circle

A circular path of radius `R` and center `(xc, yc)` can be written as:

```text
phi(t)      = phi0 + w_ref*t
x_ref(t)    = xc + R*cos(phi(t))
y_ref(t)    = yc + R*sin(phi(t))
theta_ref(t)= phi(t) + pi/2
```

The heading `theta_ref` is tangent to the circle. The desired forward and angular velocities are related by:

```text
w_ref = v_ref/R
```

For the experiment:

```text
R     = 0.5 m
v_ref = 0.1 m/s
w_ref = 0.2 rad/s
```

### B. Calculating tracking error

First calculate the difference between desired and actual position:

```text
dx = x_ref - x
dy = y_ref - y
```

These differences are in the **world (odometry) frame**. The controller rotates them into the **robot's local frame**:

```text
ex =  cos(theta)*dx + sin(theta)*dy
ey = -sin(theta)*dx + cos(theta)*dy
```

This is the coordinate transformation:

```text
[ex]   [ cos(theta)   sin(theta)] [dx]
[ey] = [-sin(theta)   cos(theta)] [dy]
```

- `ex` tells the robot how far the target is **ahead or behind**.
- `ey` tells the robot how far the target is **to its left or right**.

Heading error is computed as:

```text
e_theta = atan2(sin(theta_ref - theta),
                cos(theta_ref - theta))
```

Wrapping the error with `atan2` avoids artificial jumps when angles cross `-pi` and `pi`.

### C. Nonlinear control law

The controller calculates:

```text
v_cmd = v_ref*cos(e_theta) + kx*ex

w_cmd = w_ref + ky*v_ref*ey + kth*sin(e_theta)
```

The gains used were:

```text
kx  = 1.0
ky  = 4.0
kth = 1.5
```

Each term has a purpose:

- `v_ref*cos(e_theta)` supplies the desired forward movement, reduced when the robot's direction differs from the reference.
- `kx*ex` corrects forward/backward position error.
- `w_ref` is the turn rate required by the reference circle.
- `ky*v_ref*ey` steers to reduce sideways position error.
- `kth*sin(e_theta)` corrects heading error.

The control law is nonlinear because of the `sin` and `cos` functions. It uses **the current tracking error** to decide the next command; it does not minimize a future multi-step cost.

---

## 7. `mpc_controller.py`

### What the program does

This controller follows a reference trajectory using **Model Predictive Control (MPC)**. Instead of calculating one command directly from the present error, it searches for a **sequence of future commands** that produces a low predicted tracking cost.

It predicts robot movement with the **five-state model** from `model_validation.py` and optimizes command values using SciPy's bounded `L-BFGS-B` method.

### A. What MPC optimizes

The prediction settings are:

```text
N  = 8 future steps
dt = 0.1 s per step

prediction horizon = N*dt = 0.8 s
```

Each control command has two values:

```text
u_k = [v_cmd_k, w_cmd_k]
```

The optimizer changes eight pairs, so there are **16 optimization variables**:

```text
U = [v_cmd_0, w_cmd_0,
     v_cmd_1, w_cmd_1,
     ...,
     v_cmd_7, w_cmd_7]
```

For the circular-tracking controller, the first initial guess is eight repetitions of `[v_ref, w_ref]`. An initial guess is just a starting point for the numerical optimizer, not the final command.

### B. How the model predicts eight future states

The current measured state comes from odometry:

```text
X0 = [x0, y0, theta0, v0, w0]
```

MPC calls the model once for each candidate command pair:

```text
X1 = f(X0, u0)
X2 = f(X1, u1)
X3 = f(X2, u2)
...
X8 = f(X7, u7)
```

The `f` function uses the velocity-update and midpoint-pose equations explained in `model_validation.py`.

This produces **eight predicted future states** at `+0.1 s`, `+0.2 s`, ..., `+0.8 s`. They are *predictions*, not eight new measurements.

### C. Cost of the predicted trajectory

For each predicted step, the controller evaluates how far the robot will be from the reference and how much control effort it uses. The tracking-cost structure is:

```text
step_cost_k =
    Qpos * [(x_pred_k - x_ref_k)^2 + (y_pred_k - y_ref_k)^2]
  + Qyaw * [heading_error_k]^2
  + Rv   * (v_cmd_k - v_ref_k)^2
  + Rw   * (w_cmd_k - w_ref_k)^2
```

The `Q` weights determine the importance of tracking errors, while the `R` weights penalize deviation from the desired input. The precise numeric weights depend on the saved version of `mpc_controller.py`.

The heading error should be a wrapped difference:

```text
heading_error_k = atan2(sin(theta_ref_k - theta_pred_k),
                        cos(theta_ref_k - theta_pred_k))
```

Every future step contributes to the **total cost**:

```text
J(U) = step_cost_1 + step_cost_2 + ... + step_cost_8
```

The goal is:

```text
U_best = argmin J(U), subject to velocity-command bounds
```

In this controller, the candidate commands are limited to:

```text
0.00 <= v_cmd_k <= 0.16 m/s
-1.0 <= w_cmd_k <= 1.0 rad/s
```

**Important:** The optimizer minimizes the **sum of all eight costs**, not every step's cost independently. A turn may increase the error at the first predicted step but greatly decrease it later. MPC can accept that trade-off when the total is better.

### D. How `minimize()` works

For a candidate `U`, the cost function:

1. Starts from the latest measured `X0`.
2. Predicts `X1` through `X8` using `f`.
3. Calculates the cost of each predicted step.
4. Returns the **sum** of those costs.

`scipy.optimize.minimize(..., method="L-BFGS-B")` repeatedly changes the 16 command values and reevaluates this cost. Because an analytical gradient is not supplied, SciPy normally estimates gradients numerically. It searches for a **locally low** cost while respecting the command limits; a globally optimal sequence is not guaranteed.

### E. Why only the first command is executed

After optimization, the program has:

```text
U_best = [u0_best, u1_best, ..., u7_best]
```

It publishes **only `u0_best`**, the *first pair* of forward and angular velocity commands. The other seven commands are predictions of potentially useful actions; they are not blindly executed.

At the next control cycle:

```text
1. Read the NEW measured robot state.
2. Start from that actual state, not the old predicted X1.
3. Optimize another eight-command sequence.
4. Publish only the new first command pair.
```

This is **receding-horizon control**. It makes MPC closed-loop: any difference between predicted and actual robot movement is accounted for at the next optimization.

The previous solution is shifted to make the next initial guess:

```python
# Drop the already applied first command pair.
# Repeat the final pair to keep the guess eight steps long.
U = np.r_[U[2:], U[-2:]]
```

This reuse of a previous solution is called a **warm start**.

---

## 8. `mpc_waypoint_lidar.py`

### What the program does

This program extends MPC to reach waypoints clicked in RViz while using LiDAR data to react to nearby obstacles. It reads:

- `/odom` for the current robot state.
- `/scan` for LiDAR distances.
- `/move_base_simple/goal` for goal positions (required to be in the `odom` frame).

It publishes the chosen motion commands on `/cmd_vel`. It processes the first waypoint in its queue before advancing to the next.

The mathematical additions are **goal direction**, **obstacle-point geometry**, **repulsive guidance**, and an **obstacle penalty inside the MPC cost**.

### A. Direction from the robot to the waypoint

If the robot is at `(x, y)` and the current waypoint is `(xg, yg)`, the displacement and distance are:

```text
dx = xg - x
dy = yg - y

d_goal = sqrt(dx^2 + dy^2)
```

The direction pointing toward the waypoint is normalized to unit length:

```text
attractive_x = dx / d_goal
attractive_y = dy / d_goal
```

Normalization makes this a **direction vector** rather than a distance-dependent force. The code only computes it while the waypoint is outside its goal tolerance, so `d_goal` is not zero here.

### B. From LiDAR ranges to obstacle positions

The LiDAR gives a distance `r_i` and a local bearing `alpha_i` for each valid return. The script samples one beam out of every eight to reduce the number of obstacle points used by the optimizer.

It approximates the LiDAR as being at the robot center and aligned with the robot heading `theta`. The absolute direction of a return is:

```text
angle_i = theta + alpha_i
```

Using trigonometry, the point detected by that beam is estimated in the odometry frame:

```text
obstacle_x_i = x + r_i*cos(angle_i)
obstacle_y_i = y + r_i*sin(angle_i)
```

These coordinates are used as fixed obstacle points over one MPC optimization. The approximation ignores the LiDAR's physical offset from the robot center and does not transform the scan with TF.

### C. Repulsion away from obstacles

Only sampled LiDAR returns closer than `LASER_RANGE` are used for the repulsive vector. The code uses:

```text
LASER_RANGE = 0.90 m

weight_i = (1 - r_i/LASER_RANGE)^2
```

Nearby objects receive a larger weight. The negative direction points **away** from each obstacle:

```text
repulsive_x = -(1/5) * sum_i [weight_i*cos(angle_i)]
repulsive_y = -(1/5) * sum_i [weight_i*sin(angle_i)]
```

This is an **artificial-potential-field-style** repulsive vector. The sum can become larger when there are more detected points; the factor `1/5` is the scaling used in this implementation, not a physical law.

### D. Combining the goal direction and obstacle repulsion

The program combines both vectors:

```text
REP_GAIN = 1.5

direction_x = attractive_x + REP_GAIN*repulsive_x
direction_y = attractive_y + REP_GAIN*repulsive_y
```

It converts this combined direction to a desired heading:

```text
desired_heading = atan2(direction_y, direction_x)
```

With no nearby obstacles, the repulsive vector is zero, so the heading points toward the waypoint. If obstacles are detected, the combined direction may turn away from them. This guides MPC; it is **not** an obstacle-free path guarantee.

### E. Choosing a desired speed

The reference forward speed decreases as the robot approaches the waypoint:

```text
speed = min(0.12, 0.7*d_goal)
```

Far from the goal, the desired speed is capped at `0.12 m/s`. Close to the goal, it is proportional to the remaining distance. If the distance is below `0.12 m`, the waypoint is considered reached.

### F. Exact cost terms in this program

The optimizer again predicts eight states using the same identified model:

```text
X_next = f(X_now, [v_cmd, w_cmd])
```

At each predicted step it calculates the distance to the goal:

```text
ex = xg - x_pred
ey = yg - y_pred
```

It also calculates the wrapped difference between the desired direction and predicted heading:

```text
e_theta = atan2(sin(desired_heading - theta_pred),
                cos(desired_heading - theta_pred))
```

The **goal, heading, and command cost** in the uploaded code is:

```text
J_base = 40*(ex^2 + ey^2)
       + 3*(e_theta^2)
       + 0.2*(v_cmd - speed)^2
       + 0.03*(w_cmd^2)
```

Meaning:

- `40*(ex^2 + ey^2)` penalizes predicted positions far from the goal.
- `3*e_theta^2` penalizes heading error relative to the goal/repulsion direction.
- `0.2*(v_cmd - speed)^2` penalizes deviations from the desired forward speed.
- `0.03*w_cmd^2` discourages unnecessarily large turns.

### G. Obstacle penalty in the MPC cost

For every predicted robot position and every detected obstacle point, the code calculates:

```text
d_i = sqrt((obstacle_x_i - x_pred)^2
         + (obstacle_y_i - y_pred)^2)
```

It adds a penalty only if that distance becomes smaller than the safe distance:

```text
SAFE_DISTANCE   = 0.30 m
OBSTACLE_WEIGHT = 200

J_obstacle = 200 * sum_i [max(0, 0.30 - d_i)^2]
```

If `d_i >= 0.30 m`, the penalty for that point is zero. If `d_i < 0.30 m`, the penalty grows quadratically as the predicted robot position gets closer to the obstacle.

The **total cost of one predicted step** is:

```text
J_step = J_base + J_obstacle
```

The **total eight-step MPC cost** is:

```text
J_total(U) = J_step_1 + J_step_2 + ... + J_step_8

U_best = argmin J_total(U)
```

This means an obstacle affects optimization through **all eight predicted positions**, not only the next one. But this is a **soft penalty**: a very high cost discourages unsafe proximity, without mathematically forbidding it.

The candidate commands are bounded by:

```text
0.00 <= v_cmd <= 0.16 m/s
-1.0 <= w_cmd <= 1.0 rad/s
```

The eight-step horizon is:

```text
8 * 0.1 = 0.8 seconds
```

### H. Command execution and stopping logic

Like the previous MPC script, the program executes **only the first optimized command pair**, then reads new odometry and LiDAR measurements and optimizes again.

In addition to the cost function, the code has a separate hard-stop check:

```text
if closest_LiDAR_distance < 0.18 m:
    publish zero linear and angular velocity
```

It also stops if measurements are stale or if there is no waypoint. This hard-stop rule is **separate from** the `0.30 m` soft obstacle penalty in the MPC cost.

Because the prediction horizon is only `0.8 s`, the laser geometry is approximate, and the obstacle cost is soft, the controller may still approach obstacles too closely or become stuck. Those are limitations of the method actually implemented, not behaviors that the mathematical optimization guarantees to avoid.
