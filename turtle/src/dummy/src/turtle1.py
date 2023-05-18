#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty

x = 0
y = 0
z = 0
theta = 0

xd = 0
yd = 0
zd = 0
thetad = 0

def dummyCallback(pose_message):
    global xd
    global yd
    global zd
    global thetad
    
    xd = pose_message.x
    yd = pose_message.y
    thetad = pose_message.theta

def poseCallback(pose_message):
    global x
    global y
    global z
    global theta
    
    x = pose_message.x
    y = pose_message.y
    theta = pose_message.theta

def orientate (xgoal, ygoal):
    global x
    global y
    global theta

    velocity_message = Twist()
    cmd_vel_topic = '/turtle1/cmd_vel'

    while(True):
        ka = 1.0
        desired_angle_goal = math.atan2(ygoal-y, xgoal-x)
        nuevoangulo= desired_angle_goal-theta 
        if nuevoangulo > math.pi:
            nuevoangulo -= 2*math.pi
        elif nuevoangulo < -math.pi:
            nuevoangulo += 2*math.pi 
       
        dtheta = nuevoangulo
       
        angular_speed = ka * (dtheta)

        velocity_message.linear.x = 0.0
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)
        print ('x=', x, 'y=', y)

        if (dtheta < 0.01):
            break

def go_to_goal (xgoal, ygoal):
    global x
    global y
    global theta

    velocity_message = Twist()
    cmd_vel_topic = '/turtle1/cmd_vel'

    while(True):

        colission = checkCollision()

        if  colission == True:
            kv = 0
        else:
            kv = 0.5				
        
        distance = abs(math.sqrt(((xgoal-x)**2)+((ygoal-y)**2)))
        linear_speed = kv * distance

        ka = 1.005
        desired_angle_goal = math.atan2(ygoal-y, xgoal-x)
        nuevoangulo= desired_angle_goal-theta 
        if nuevoangulo > math.pi:
            nuevoangulo -= 2*math.pi
        elif nuevoangulo < -math.pi:
            nuevoangulo += 2*math.pi 
       
        dtheta = nuevoangulo
       
            
        angular_speed = ka * (dtheta)

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed
        velocity_publisher.publish(velocity_message)
        print ('x=', x, 'y=', y)

        if (distance < 0.01):
            break

def checkCollision():
    global x
    global y
    global xd
    global yd
    
    range = 1

    hypot = math.hypot(x - xd, y - yd)
    hypot_publisher.publish(hypot)
    
    if hypot > range:
        return False
    else:
        return True

if __name__ == '__main__':
    try:
        rospy.init_node('turtlesim_motion_pose', anonymous = True)

        cmd_vel_topic = '/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size = 10)

        hypot_topic = '/hypot'
        hypot_publisher = rospy.Publisher(hypot_topic, Float32, queue_size = 10)


        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)
        time.sleep(2)

        dummy_topic = "/turtle_dummy/pose"
        pose_subscriber = rospy.Subscriber(dummy_topic, Pose, dummyCallback)
        time.sleep(2)

        while(1):
            time.sleep(3)
            orientate(5.5,10.0)
            time.sleep(3)
            go_to_goal(5.5,10.0)

            time.sleep(3)
            orientate(5.5,1.0)
            time.sleep(3)
            go_to_goal(5.5,1.0)


    except rospy.ROSInterruptException:        
        pass
