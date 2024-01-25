#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PointStamped



if __name__ == '__main__':
    rospy.init_node('publish_point_listener')
    rospy.Subscriber("clicked_point", PointStamped, point_callback)