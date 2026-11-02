#!/usr/bin/env python3

import rospy
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class NonlinearController:

    def __init__(self):                   #initilize node

        rospy.init_node("nonelinear_controller")

        self.pose = None

        self.pub = rospy.Publisher("/cmd_vel" ,Twist, queue_size=10)

        rospy.Subscriber("/odom", Odometry, self.odom_callback)

    def odom_callback(self ,msg):     #read robot possition and convert to euler angles

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        
        q = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.pose = (x, y, theta)    

    def stop_robot(self):

        self.pub.publish(Twist())

    def run(self):

        rate = rospy.Rate(20)

        while self.pose is None and not rospy.is_shutdown():  #wait for odom
            rate.sleep()

        if rospy.is_shutdown():
            return

        x0, y0, theta0 = self.pose #init pose

        R = 0.5     #trajectory parameters
        v_ref = 0.1
        w_ref = v_ref / R

        kx = 1.0 #controller gains
        ky = 4.0
        kth = 1.5

        start = rospy.get_time()
        errors = []

        while not rospy.is_shutdown():

            t = rospy.get_time() - start

            
            if t >= 2 * math.pi / w_ref: #stop after one circle
                break

            x, y, theta = self.pose

            
            theta_ref = theta0 + w_ref * t

            x_ref = x0 + R * (
                math.sin(theta_ref) - math.sin(theta0)
            )

            y_ref = y0 - R * (
                math.cos(theta_ref) - math.cos(theta0)
            )

            
            dx = x_ref - x #calc errors
            dy = y_ref - y

            
            ex = math.cos(theta) * dx + math.sin(theta) * dy
            ey = -math.sin(theta) * dx + math.cos(theta) * dy

            
            etheta = math.atan2(
                math.sin(theta_ref - theta),
                math.cos(theta_ref - theta)
            )

            
            v = v_ref * math.cos(etheta) + kx * ex  #nonelinear control law
            w = w_ref + ky * v_ref * ey + kth * math.sin(etheta)

           
            v = max(0.0, min(0.16, v)) #vel limits
            w = max(-1.0, min(1.0, w))

            
            command = Twist()  #publish cmd
            command.linear.x = v
            command.angular.z = w

            self.pub.publish(command)

            
            errors.append(dx**2 + dy**2) #store errors

            rate.sleep()

        self.stop_robot()

        
        if errors:
            rmse = math.sqrt(sum(errors) / len(errors)) #calc rmse
            rospy.loginfo("Position RMSE: %.4f m", rmse)


if __name__ == "__main__":

    try:
        controller = NonlinearController()
        controller.run()

    except rospy.ROSInterruptException:
        pass


