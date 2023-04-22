#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64, Float32
from geometry_msgs.msg import Pose2D
import math

class Odometry:
    # Constructor
    def __init__(self):
        
        self.omega_left = 0.0
        self.omega_right = 0.0

        self.radius = 0.05
        self.lenght = 0.18

        self.sampling_time = 0.1

        self.velocity = 0.0
        self.angular_velocity = 0.0

        self.distance = 0.0
        self.theta = 0.0

        self.pose = Pose2D()
        rospy.init_node('odometry_publisher')
        self.odometry_publisher = rospy.Publisher('/odometry_publisher', Pose2D, queue_size=10)

        rospy.Subscriber('/wl', Float32, self.omega_left_Callback)
        rospy.Subscriber('/wr', Float32, self.omega_right_Callback)
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        
    def omega_left_Callback(self, msg):
        self.omega_left = msg.data

    def omega_right_Callback(self, msg):
        self.omega_right = msg.data

    def timer_callback(self, time):
        self.velocity = ((self.omega_right + self.omega_left)/2.0) * self.radius
        self.angular_velocity = ((self.omega_right - self.omega_left)/self.lenght) * self.radius

        self.distance += self.velocity*self.sampling_time
        self.theta += self.angular_velocity*self.sampling_time
        
        self.pose.theta = self.theta

        self.pose.x += math.cos(self.theta) * self.velocity*self.sampling_time
        self.pose.y += math.sin(self.theta) * self.velocity*self.sampling_time

        self.odometry_publisher.publish(self.pose)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    s = Odometry()
    s.run()