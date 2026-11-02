#!/usr/bin/env python3

import math
import numpy as np
import rospy

from scipy.optimize import minimize
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


Kv = 1.0011045         #model parameters
tau_v = 0.0774945

Kw = 0.9942472
tau_w = 0.032835

N = 8               #MPC horizon
DT = 0.1            #time between predictions


R = 0.5             #circle radius
v_ref = 0.1         #desired lienar vel
w_ref = v_ref / R   #desrired angular vel

class MPCController:

    def __init__(self):  #init node

        rospy.init_node("mpc_controller")

        self.state = None

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        rospy.on_shutdown(self.stop)


    def odom_callback(self, msg):   #get robot state

        
        p = msg.pose.pose.position

        
        q = msg.pose.pose.orientation
        theta = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z

        self.state = (p.x, p.y, theta, v, w)


    def stop(self):

        self.pub.publish(Twist())

    def reference(self, t):

        
        angle = self.theta0 + w_ref * t            #calculate desired possition on circle

        x = self.x0 + R * (math.sin(angle) - math.sin(self.theta0))
        y = self.y0 - R * (math.cos(angle) - math.cos(self.theta0))

        return x, y, angle

    def predict(self, state, v_cmd, w_cmd):

        x, y, theta, v, w = state #current predicted state

          
        av = math.exp(-DT / tau_v)  #first order velocity models
        aw = math.exp(-DT / tau_w)

        v_next = av * v + Kv * (1 - av) * v_cmd
        w_next = aw * w + Kw * (1 - aw) * w_cmd

        
        v_avg = (v + v_next) / 2 #avarge velocities
        w_avg = (w + w_next) / 2

        
        theta_mid = theta + w_avg * DT / 2 

        x += v_avg * math.cos(theta_mid) * DT
        y += v_avg * math.sin(theta_mid) * DT
        theta += w_avg * DT

        return x, y, theta, v_next, w_next

    def cost(self, U ,state , t):

        total = 0.0

        for k in range(N):

            
            v_cmd = U[2*k]     #candidate velocity commands
            w_cmd = U[2*k + 1]

            
            state = self.predict(state, v_cmd, w_cmd) #predict next state

            x, y, theta, _, _ = state

            
            xr, yr, tr = self.reference(t + (k+1)*DT) #desired possition

            
            ex = xr - x  #error tracking
            ey = yr - y

            etheta = math.atan2(
                math.sin(tr - theta),
                math.cos(tr - theta)
            )

            
            total += (                   #penilize error and control effort
                60 * (ex**2 + ey**2)
                + 3 * etheta**2
                + 0.1 * (v_cmd - v_ref)**2
                + 0.01 * (w_cmd - w_ref)**2
            )

        return total

    def run(self):

        rate = rospy.Rate(10)

        while self.state is None and not rospy.is_shutdown():
            rate.sleep()

        if rospy.is_shutdown():
            return

        self.x0, self.y0, self.theta0, _, _ = self.state #save initial robot possition

        start = rospy.get_time()

         
        U = np.tile([v_ref, w_ref], N)

        
        bounds = [(0, 0.16), (-1.0, 1.0)] * N #velocity constraints

        errors = []

        while not rospy.is_shutdown():

            t = rospy.get_time() - start

            
            if t >= 2 * math.pi / w_ref: #stop at one circle
                break

            
            state = self.state #read current state

            
            result = minimize(
              self.cost,
              U,
              args=(state, t),
              method="L-BFGS-B",
              bounds=bounds,
              options={
              "maxiter": 50,
              "ftol": 1e-5,
              "gtol": 1e-4
             }
              )

            if not np.all(np.isfinite(result.x)):
                rospy.logerr("MPC returned invalid commands")
                break

            if not result.success:
                rospy.logwarn_throttle(5, "MPC did not fully converge")

            U = result.x

            
            command = Twist()

            command.linear.x = float(U[0])
            command.angular.z = float(U[1])

            self.pub.publish(command)

            
            xr, yr, _ = self.reference(t) #calc tracking error

            errors.append(
                (xr - state[0])**2 +
                (yr - state[1])**2
            )

            
            U = np.r_[U[2:], U[-2:]]

            rate.sleep()

        self.stop()

        if errors:
            rmse = math.sqrt(sum(errors) / len(errors))
            rospy.loginfo("MPC Position RMSE: %.4f m", rmse)


if __name__ == "__main__":

    try:
        MPCController().run()

    except rospy.ROSInterruptException:
        pass
    




