#!/usr/bin/env python3

import rospy
import csv
import os

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


class DataCollection:

    def __init__(self):

     rospy.init_node("data_collection") #initilize node

     self.odom_data = None #store last odom mesurment

     self.odom_counter = 0 

     self.cmd_pub = rospy.Publisher("/cmd_vel" ,Twist ,queue_size=10) #publisher sending velocity commands

     self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback) #subscriber for getting odometry mesurments

     rospy.on_shutdown(self.stop_robot) #stop when nodes shuts down

    def odom_callback(self, msg):
       
       x = msg.pose.pose.position.x #read the robots x coordinate

       y = msg.pose.pose.position.y #read the robots y coordinate

       q = msg.pose.pose.orientation #read orientation as a quaternion

       quaternion = [q.x, q.y, q.z, q.w]

       _, _, theta = euler_from_quaternion(quaternion) #convert quaternion to euler angels using only the angle around the z axis

       v_measured = msg.twist.twist.linear.x #mesure linear velocity

       omega_measured = msg.twist.twist.angular.z #mesure angular velocity

       timestamp = msg.header.stamp.to_sec() #read the timestamp of the odometry message

       self.odom_counter += 1 #update odom counter

       self.odom_data = (     #store all measurements together
            self.odom_counter,
            timestamp,
            x,
            y,
            theta,
            v_measured,
            omega_measured
        )
    def stop_robot(self): #stop the robot 

       command = Twist

       command.linear.x = 0.0 

       command.angular.z = 0.0

       self.cmd_pub.publish(command)

    def run(self):

       rate = rospy.Rate(100) #set control loop frequency at 20 Hz

       while not rospy.is_shutdown() and self.odom_data is None:
            rate.sleep()

       if rospy.is_shutdown():
            return

       experiments = [ #set the expirements

            #keep the robot stationary
            ("initial_rest", 2.0, 0.0, 0.0),

            #move forward
            ("linear_motion", 4.0, 0.1, 0.0),

            #stop after linear motion
            ("linear_stop", 2.0, 0.0, 0.0),

            #rotate in place
            ("angular_motion", 4.0, 0.0, 0.6),

            #stop after angular motion
            ("angular_stop", 2.0, 0.0, 0.0),

            #move forward while rotating
            ("combined_motion", 4.0, 0.08, 0.35),

            #final stop
            ("final_stop", 2.0, 0.0, 0.0)
        ]
       
       output_file = os.path.expanduser( #define output file
            "~/turtlebot_data.csv"
        )

       start_time = rospy.get_time() #record starting time

       last_recorded = 0

       with open(output_file, "w", newline="") as file: #open csv

            writer = csv.writer(file)

            writer.writerow([ #write column names
                "time",
                "phase",
                "v_cmd",
                "omega_cmd",
                "x",
                "y",
                "theta",
                "v_measured",
                "omega_measured"
            ])
            for name, duration, v, omega in experiments: #run the expirements

                if rospy.is_shutdown():
                    break

                rospy.loginfo("Starting experiment: %s", name)

                command = Twist()  #create velocity commands

                command.linear.x = v
                command.angular.z = omega

                phase_start = rospy.get_time()

                while not rospy.is_shutdown():

                    current_time = rospy.get_time()

                    elapsed = current_time - phase_start

                    if elapsed >= duration:
                        break

                    self.cmd_pub.publish(command) #send velocity command

                    data = self.odom_data #get latest odom data

                    if data is not None:

                        (
                            counter,
                            timestamp,
                            x,
                            y,
                            theta,
                            v_measured,
                            omega_measured
                        ) = data

                        
                        if counter != last_recorded:

                            
                            measurement_time = (
                                timestamp if timestamp > 0
                                else current_time
                            )

                            
                            t = measurement_time - start_time

                            if t >= 0:

                                writer.writerow([
                                    t,
                                    name,
                                    v,
                                    omega,
                                    x,
                                    y,
                                    theta,
                                    v_measured,
                                    omega_measured
                                ])

                            last_recorded = counter

                    rate.sleep()

       self.stop_robot()
       rospy.loginfo("Data collection completed.")
       rospy.loginfo("Data saved to: %s", output_file)

if __name__ == "__main__":

    try:

        collector = DataCollection()

        collector.run()

    except rospy.ROSInterruptException:
        pass

