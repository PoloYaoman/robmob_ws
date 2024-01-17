#!/usr/bin/env python3
import roslib
import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('robot_tf_listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    rospy.sleep(1)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/base_link', '/map', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    print('trans', trans)
    print('rot', rot)