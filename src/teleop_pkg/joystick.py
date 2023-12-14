#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class TeleopRobot:
    def __init__(self):
        self.linear = 1
        self.angular = 2
        self.l_scale = 1.0
        self.a_scale = 1.0

        rospy.init_node('teleop_robot')

        rospy.Subscriber("joy", Joy, self.joy_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        rospy.spin()

    def joy_callback(self, joy):
        twist = Twist()
        twist.angular.z = self.a_scale * joy.axes[self.angular]
        twist.linear.x = self.l_scale * joy.axes[self.linear]
        self.vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        TeleopRobot()
    except rospy.ROSInterruptException:
        pass
