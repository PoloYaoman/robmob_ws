#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String

import math

K = 1

class VelNode:
    def __init__(self):
        rospy.init_node('vel_node', anonymous=True)

        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber('/cmd_vel_op', Twist, self.cmd_callback)
        rospy.Subscriber('/c_pose', Pose, self.pose_callback)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        rospy.spin()

    def cmd_callback(self, data):
        cmd_x = data.linear.x 
        cmd_y = data.linear.y 

        r = math.sqrt(cmd_x**2 + cmd_y**2)
        w = math.atan2(cmd_y, cmd_x) - math.pi/2

        twist_msg = Twist()
        twist_msg.linear.x = r 
        twist_msg.angular.z = w - self.theta

        print("Received command : ", cmd_x, cmd_y)
        print("Robot orientation: ", self.theta)
        print("cmd orientation: ", w)
        print("x speed = ", r, "theta speed = ", twist_msg.angular.z)

        # self.vel_publisher.publish(twist_msg)

    def pose_callback(self, data):
        self.x = data.position.x 
        self.y = data.position.y

        # rz = data.orientation.z 
        rw = data.orientation.w

        # if rz<0:
        #     self.theta = -2*math.acos(rw) - math.pi/2
        # else:
        #     self.theta = 2*math.acos(rz) - math.pi/2

        self.theta = -rw + math.pi/4

if __name__ == '__main__':
    try:
        vel_node = VelNode()
    except rospy.ROSInterruptException:
        pass