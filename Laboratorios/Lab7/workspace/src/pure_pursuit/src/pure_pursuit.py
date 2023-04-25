#!/usr/bin/env python

import rospy
import yaml
import rospkg
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Pose2D

class PurePursuit:
    def __init__(self):
        self.pose = Pose2D()
        self.pose.x = 0
        self.pose.y = 0
        self.pose.theta = 0

        self.lookahead = 0.5
        self.linear_velocity = 0.1
        self.angular_velocity = 0.0

        self.sampling_time = 0.1

        rospack = rospkg.RosPack()
        package_path = rospack.get_path("pure_pursuit")
        path_loc = package_path + '/config/waypoints.yaml'
        with open(path_loc, 'r') as doc:
            loc = yaml.load(doc)

        self.waypoints_x = loc['x1']
        self.waypoints_y = loc['y1']

        self.msg = Twist()
        self.msg.linear.x = 0
        self.msg.linear.y = 0
        self.msg.linear.z = 0
        self.msg.angular.x = 0
        self.msg.angular.y = 0
        self.msg.angular.z = 0

        rospy.init_node('pure_pursuit_node')
        self.odometry_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(self.sampling_time), self.timer_callback) 
        rospy.Subscriber('/odometry_publisher', Pose2D, self.odom_callback)
        rospy.on_shutdown(self.on_shutdown)

    def odom_callback(self, msg):
        self.pose = msg

    def closest_waypoint(self):
        dx = [self.pose.x - ix for ix in self.waypoints_x]
        dy = [self.pose.y - iy for iy in self.waypoints_y]
        euclidean_distance = np.hypot(dx, dy)
        closest_waypoint_index = np.argmin(euclidean_distance)
         
        if np.argmin(euclidean_distance) + 1 < len(self.waypoints_x):
           next_waypoint_index = closest_waypoint_index + 1
        else: 
           len(self.waypoints_y) - 1
        return next_waypoint_index

    def timer_callback(self, time):
        closest_index = self.closest_waypoint()
        next_x = self.waypoints_x[closest_index]
        next_y = self.waypoints_y[closest_index]

        alpha = np.arctan2(next_y - self.pose.y, next_x - self.pose.x) - self.pose.theta
        alpha = (alpha + np.pi) % (2 * np.pi) - np.pi
        
        sign = np.sign(alpha)

        rx = next_x*np.cos(alpha) - next_y*np.sin(alpha) 
        ry = next_x*np.sin(alpha) + next_y*np.cos(alpha)
        dx = (self.pose.x - rx)*sign 
        dy = ry - self.pose.y 
        distance = (dx + dy) 

        k = 2*distance/(self.lookahead**2)

        self.angular_velocity = k * self.linear_velocity

        self.msg.linear.x = self.linear_velocity
        self.msg.angular.z = self.angular_velocity
        self.odometry_pub.publish(self.msg)
        

    def on_shutdown(self):
        self.msg.linear.x = 0.0
        self.msg.linear.y = 0.0
        self.msg.linear.z = 0.0
        self.msg.angular.x = 0.0
        self.msg.angular.y = 0.0
        self.msg.angular.z = 0.0
        self.timer.shutdown()
        self.odometry_pub.publish(self.msg)

    def run(self):
        while not rospy.is_shutdown():
            dx = self.pose.x - self.waypoints_x[-1]
            dy = self.pose.y - self.waypoints_y[-1]
            goal_distance = np.hypot(dx,dy)
            if goal_distance < 0.1:
                self.on_shutdown()

if __name__ == '__main__':
    pure_pursuit = PurePursuit()
    pure_pursuit.run()

