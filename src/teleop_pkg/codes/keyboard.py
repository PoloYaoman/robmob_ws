#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from pynput import keyboard

class TeleopRobot:
    def __init__(self):
        rospy.init_node('teleop_robot')

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.twist = Twist()
        self.linear_speed = 0.50
        self.angular_speed = 1.0

        self.key_listener = keyboard.Listener(on_press=self.on_key_press, on_release=self.on_key_release)
        self.key_listener.start()

        rospy.spin()

    def on_key_press(self, key):
        try:
            if key.char == 'z':
                self.twist.linear.x = self.linear_speed
            elif key.char == 's':
                self.twist.linear.x = -self.linear_speed
            elif key.char == 'd':
                self.twist.angular.z = -self.angular_speed
            elif key.char == 'q':
                self.twist.angular.z = self.angular_speed
        except AttributeError:
            pass

        self.vel_pub.publish(self.twist)

    def on_key_release(self, key):
        try:
            if key.char == 'z' or key.char == 's':
                self.twist.linear.x = 0.0
            elif key.char == 'q' or key.char == 'd':
                self.twist.angular.z = 0.0
        except AttributeError:
            pass

        self.vel_pub.publish(self.twist)

if __name__ == '__main__':
    try:
        TeleopRobot()
    except rospy.ROSInterruptException:
        pass
