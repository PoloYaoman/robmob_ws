#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy

def joystick_callback(data):
    rospy.loginfo("Axes: {}".format(data.axes))
    rospy.loginfo("Buttons: {}".format(data.buttons))

def joystick_listener():
    rospy.init_node('joystick_node', anonymous=True)

    rospy.loginfo("Joystick node is ready.")

    rospy.Subscriber("joy", Joy, joystick_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        joystick_listener()
    except rospy.ROSInterruptException:
        pass
