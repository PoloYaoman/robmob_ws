#!/usr/bin/env python3
import rospy
import math
import tf

if __name__ == '__main__':
    rospy.init_node('robot_tf_listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    rospy.sleep(1)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            rospy.sleep(2)
            tx = trans[0]
            ty = trans[1]
            tz = trans[2]
            rw = rot[3]
            
            print("translation : \n sur x:",tx, "\n sur y:",ty,"\n sur z", tz)
            print("rotation : \n",rw)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
    #Translation en [x,y,z]
    #Rotation en quaternion
