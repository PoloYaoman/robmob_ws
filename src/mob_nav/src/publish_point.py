#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped

def point_callback(data):
    x = data.point.x
    y = data.point.y
    z = data.point.z
    rospy.loginfo("coordinates:x=%f y=%f z=%f" %(x,y,z))

if __name__ == '__main__':
    rospy.init_node('publish_point_listener')
    rospy.Subscriber("clicked_point", PointStamped, point_callback)
    rospy.spin()