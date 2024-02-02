#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import String
import tf

import math

K = 1

class VelNode:
    def __init__(self):
        rospy.init_node('vel_node', anonymous=True)

        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        rospy.Subscriber('/cmd_vel_op', Twist, self.cmd_callback)
        rospy.Subscriber('/c_pose', Pose, self.pose_callback)

        self.tf_listener = tf.TransformListener()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        rospy.spin()

    def cmd_callback(self, data):
        cmd_x = data.linear.x 
        cmd_y = data.linear.y 

        r = 0
        w = 0
        w_tmp = 0

        if abs(cmd_x)>0.01 or abs(cmd_y)>0.01 :
            r = math.sqrt(cmd_x**2 + cmd_y**2)
            w_tmp = math.atan2(cmd_y, cmd_x) 
            w = w_tmp - self.theta
            if w>math.pi:
                w = math.pi - w
            elif w<-math.pi:
                w = -math.pi - w

        twist_msg = Twist()
        # if r>0.05:
        #     twist_msg.linear.x = 0.3
        # else:
        twist_msg.linear.x = r
        twist_msg.angular.z = w

        print("Received command : ", cmd_x, cmd_y)
        print("Robot orientation: ", self.theta)
        print("cmd orientation: ", w_tmp)
        print("x speed = ", r, "theta speed = ", twist_msg.angular.z)

        self.vel_publisher.publish(twist_msg)

    def pose_callback(self, data):
        self.x = data.position.x 
        self.y = data.position.y

        # rz = data.orientation.z 
        self.theta = data.orientation.w

        # if self.rz<0:
        #     self.theta = -2*math.acos(self.rw) - math.pi
        # else:
        #     self.theta = 2*math.acos(self.rw) - math.pi

        # (trans,rot) = self.tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        # (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(rot)

        # if yaw > math.pi:
        #     yaw = yaw%math.pi
        # elif yaw < - math.pi:
        #     yaw = -yaw%math.pi

        # self.theta = yaw

if __name__ == '__main__':
    try:
        vel_node = VelNode()
        
    except rospy.ROSInterruptException:
        pass