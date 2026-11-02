#!/usr/bin/env python3

import math
import time
import numpy as np
import rospy

from scipy.optimize import minimize
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import euler_from_quaternion



Kv, tau_v = 1.0011045, 0.0774945 #vel model
Kw, tau_w = 0.9942472, 0.032835


N = 8     #horizon and prediction time
DT = 0.1


GOAL_TOL = 0.12  #nav settings
LASER_RANGE = 1.1
SAFE_DISTANCE = 0.3
STOP_DISTANCE = 0.1

REP_GAIN = 1.5
OBSTACLE_WEIGHT = 250

class MPCController:

    def __init__(self):
        rospy.init_node("mpc_waypoint_lidar")

        self.state = None
        self.odom_time = None
        self.scan_time = None
        self.closest = float("inf")

        self.scan_ranges = np.array([])
        self.scan_angles = np.array([])
        self.waypoints = []

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.Subscriber("/odom", Odometry, self.odom_callback)      #initilize nodes
        rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_callback)

        rospy.on_shutdown(self.stop)


    def odom_callback(self, msg):  #read robot state
        
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        theta = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        self.state = (
            p.x,
            p.y,
            theta,
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z
        )

        self.odom_time = rospy.get_time()


    def scan_callback(self, msg):   #callback for lazer
        
        ranges = np.asarray(msg.ranges)
        ids = np.arange(0, len(ranges), 8)

        valid = (
            np.isfinite(ranges)
            & (ranges >= msg.range_min)
            & (ranges <= msg.range_max)
        )

        self.closest = (
            float(np.min(ranges[valid]))
            if np.any(valid) else float("inf")
        )

        nearby = valid[ids] & (ranges[ids] < LASER_RANGE)

        self.scan_ranges = ranges[ids][nearby]
        self.scan_angles = msg.angle_min + ids[nearby] * msg.angle_increment

        self.scan_time = rospy.get_time()


    def goal_callback(self, msg):  #get goal from rviz
        
        if msg.header.frame_id.lstrip("/") != "odom":
            rospy.logwarn("Set RViz Fixed Frame to 'odom' before adding goals")
            return

        p = msg.pose.position

        self.waypoints.append((p.x, p.y))

        rospy.loginfo("Waypoint added: (%.2f, %.2f)", p.x, p.y)


    def stop(self):
        self.pub.publish(Twist())

   
   
    def predict(self, state, v_cmd, w_cmd):

        x, y, theta, v, w = state   #current state

        av = math.exp(-DT / tau_v)  #first order velocity model
        aw = math.exp(-DT / tau_w)

        v_next = av*v + Kv*(1-av)*v_cmd #avarage velocitoes
        w_next = aw*w + Kw*(1-aw)*w_cmd

        v_avg = (v + v_next) / 2
        w_avg = (w + w_next) / 2

        theta_mid = theta + w_avg*DT/2

        x += v_avg*math.cos(theta_mid)*DT
        y += v_avg*math.sin(theta_mid)*DT
        theta += w_avg*DT

        return x, y, theta, v_next, w_next

    def lidar_vectors(self, state):
        
        x, y, theta, _, _ = state

        angles = self.scan_angles + theta
        ranges = self.scan_ranges

        
        ox = x + ranges*np.cos(angles)
        oy = y + ranges*np.sin(angles)

        obstacles = np.column_stack((ox, oy))

        
        weights = (1 - ranges/LASER_RANGE)**2 #calc repulsive vector

        repulsive = -np.array([
            np.sum(weights*np.cos(angles)),
            np.sum(weights*np.sin(angles))
        ]) / 5.0

        return obstacles, repulsive

    def cost(self, U, state, goal, desired_heading, obstacles, speed):
        total = 0.0

        for k in range(N):
           
            v_cmd = U[2*k]   #candidate commadns
            w_cmd = U[2*k+1]

            
            state = self.predict(state, v_cmd, w_cmd) #preict next

            x, y, theta, _, _ = state

            
            ex = goal[0] - x   #waypoint errors
            ey = goal[1] - y

            e_theta = math.atan2(
                math.sin(desired_heading-theta),
                math.cos(desired_heading-theta)
            )

            
            total += (                 #penalties for error and control effort
                40*(ex**2 + ey**2)
                + 3*e_theta**2
                + 0.2*(v_cmd-speed)**2
                + 0.03*w_cmd**2
            )

            
            if len(obstacles):          #obsticle avoidance penalty
                distances = np.hypot(
                    obstacles[:, 0]-x,
                    obstacles[:, 1]-y
                )

                total += OBSTACLE_WEIGHT*np.sum(
                    np.maximum(0, SAFE_DISTANCE-distances)**2
                )

        return total

    def run(self):
        rate = rospy.Rate(10)

        U = np.tile([0.1, 0.0], N)
        bounds = [(0, 0.16), (-1, 1)] * N

        while not rospy.is_shutdown():

            now = rospy.get_time()

            
            if (self.state is None or self.scan_time is None
                    or now - self.scan_time > 0.6
                    or now - self.odom_time > 0.6):

                self.stop()
                rate.sleep()
                continue

            
            if not self.waypoints: #wait until waypoint is selected
                self.stop()
                rate.sleep()
                continue

            
            if self.closest < STOP_DISTANCE: #emergency stop if obstacel too close
                self.stop()
                rospy.logwarn_throttle(3, "Obstacle too close: stopping")
                rate.sleep()
                continue

            state = self.state
            x, y = state[0], state[1]

            
            goal = self.waypoints[0] #current waypoint

            dx = goal[0] - x
            dy = goal[1] - y

            distance = math.hypot(dx, dy)

            # Move to the next waypoint
            if distance < GOAL_TOL:
                rospy.loginfo(
                    "Waypoint reached: (%.2f, %.2f)", *goal
                )

                self.waypoints.pop(0)
                self.stop()

                U = np.tile([0.1, 0.0], N)

                rate.sleep()
                continue

            
            attractive = np.array([dx, dy]) / distance #attractive waypoint vector

            
            obstacles, repulsive = self.lidar_vectors(state) #repulsive lidar vector

           
            direction = attractive + REP_GAIN*repulsive #add vectors

            heading = math.atan2(
                direction[1], direction[0]
            )

            speed = min(0.12, 0.7*distance)

           
            begin = time.perf_counter()

            result = minimize(
                self.cost,
                U,
                args=(state, goal, heading, obstacles, speed),
                method="L-BFGS-B",
                bounds=bounds,
                options={
                    "maxiter": 50,
                    "ftol": 1e-5,
                    "gtol": 1e-4
                }
            )

            if time.perf_counter() - begin > DT:
                rospy.logwarn_throttle(5, "MPC slower than 10 Hz")

           
            if not result.success or not np.all(np.isfinite(result.x)):
                self.stop()

                rospy.logwarn_throttle(
                    5,
                    f"MPC optimization failed: {result.message}"
                )

                rate.sleep()
                continue

            U = result.x

            
            cmd = Twist() #exacute optimized command

            cmd.linear.x = float(U[0])
            cmd.angular.z = float(U[1])

            self.pub.publish(cmd)

           
            U = np.r_[U[2:], U[-2:]]

            rate.sleep()


if __name__ == "__main__":
    try:
        MPCController().run()

    except rospy.ROSInterruptException:
        pass