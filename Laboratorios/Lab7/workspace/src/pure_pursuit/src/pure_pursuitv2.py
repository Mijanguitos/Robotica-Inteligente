#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose2D
import numpy as np
import yaml
import rospkg

def load_waypoints():
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('pure_pursuit')
    yaml_path = package_path + '/config/waypoints.yaml'
    
    with open(yaml_path, 'r') as f:
        data_yaml = yaml.safe_load(f)
        waypoints = np.column_stack((data_yaml['x4'], data_yaml['y4']))
    return waypoints

class PurePursuit:
    def __init__(self, waypoints):
        self.pose = Pose2D()
        self.pose.x = 0
        self.pose.y = 0
        self.pose.theta = 0

        self.distance = []
        self.next_point_index = 0

        self.waypoints = waypoints
        self.lookahead = 0.74
        self.max_velocity = 0.1
        self.sampling_time = 0.1
        self.angular_velocity = 0.0

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
        rospy.on_shutdown(self.stop)

    def odom_callback(self, msg):
        self.pose = msg
        
    def closest_waypoint(self):
        self.distance = np.hypot(self.waypoints[:,0] - self.pose.x, self.waypoints[:,1] - self.pose.y)
        closest_index = np.argmin(self.distance)
        self.next_point_index = closest_index + 1 if np.argmin(self.distance) + 1 < len(self.waypoints[:,0]) else len(self.waypoints[:,0]) - 1

    def timer_callback(self, timer):
        self.closest_waypoint()
        target_x = self.waypoints[self.next_point_index, 0]
        target_y = self.waypoints[self.next_point_index, 1]
        alpha = np.arctan2(target_y - self.pose.y, target_x - self.pose.x) - self.pose.theta

        x_rot = target_x * np.cos(alpha) - target_y * np.sin(alpha)
        y_rot = target_x * np.sin(alpha) + target_y * np.cos(alpha)

        new_distance = (self.pose.x - x_rot) + (y_rot - self.pose.y)
        k = (2*new_distance)/(self.lookahead**2)

        self.angular_velocity = k * self.max_velocity
        self.msg.linear.x = self.max_velocity
        self.msg.linear.y = 0.0
        self.msg.linear.z = 0.0
        self.msg.angular.x = 0.0
        self.msg.angular.y = 0.0
        self.msg.angular.z = self.angular_velocity
        self.odometry_pub.publish(self.msg)

    def stop(self):
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
            dx = self.pose.x - self.waypoints[-1, 0]
            dy = self.pose.y - self.waypoints[-1, 1]
            d = np.hypot(dx,dy)
            if d < 0.1:
                self.stop()

if __name__ == '__main__':
    waypoints = np.array(load_waypoints())
    pure_pursuit = PurePursuit(waypoints)
    pure_pursuit.run()
