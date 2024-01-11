#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from a_star import AStarPlanner

import numpy as np


GRID_SIZE = 1
ROBOT_RADIUS = 0.3

GX = -13
GY = -3

K = 1
#flag = True


class NavNode:
    def __init__(self):
        rospy.init_node('nav_node', anonymous=True)

        # Subscribers
        rospy.Subscriber('/map', OccupancyGrid, self.occupancy_grid_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Publisher
        self.vel_cmd_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Initialize other variables or setup here
        self.grid = []

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_rz = 0.0

        self.path = []

        self.vel = Twist()

    def occupancy_grid_callback(self, data):
        # Process occupancy grid data here
        #if flag:

        rospy.loginfo("Received occupancy grid data")

        # Extract grid information
        width = data.info.width
        height = data.info.height
        grid_tmp = data.data

        # Convert 1D data array into a 2D array
        self.grid = [[0 for _ in range(width)] for _ in range(height)]

        for i in range(height):
            for j in range(width):
                index = i * width + j
                self.grid[i][j] = grid_tmp[index]
                #if grid_tmp[index] > -1:
                    #rospy.loginfo("obstacle point!")

        self.trim_edges()

        #rospy.loginfo(self.grid)
            #flag = False
        
    def trim_edges(self):
        for row,i in enumerate(self.grid):
            if np.max(row)<0:
                self.grid = np.delete(self.grid, i, axis=0)
        

    def odom_callback(self, data):
        # Process odometry data here
        rospy.loginfo("Received odometry data")

        self.odom_x = data.pose.pose.position.x 
        self.odom_y = data.pose.pose.position.y 
        self.odom_rz = data.pose.pose.orientation.w

        # Example: Publish a sample Twist message
        twist_msg = Twist()
        # twist_msg.linear.x = 0.1  # Example linear velocity
        # twist_msg.angular.z = 0.2  # Example angular velocity

        twist_msg = self.get_vel()

        self.vel_cmd_publisher.publish(twist_msg)

    def get_vel(self):
        if not self.path:
            self.get_path()

        v1 = self.vel.linear.x - K * (self.path[0][0] - self.odom_x)
        v2 = self.vel.linear.y - K * (self.path[0][1] - self.odom_y)

        twist_msg = Twist()
        twist_msg.linear.x = v1
        twist_msg.linear.y = v2

        self.vel.linear.x = v1
        self.vel.linear.y = v2

        self.path.pop(0)

        return twist_msg

    def get_path(self):
        ox, oy = [], []

        for x,row in enumerate(self.grid):
            for y,column in enumerate(row):
                if column > 0:
                    ox.append(x)
                    oy.append(y)

        a_star = AStarPlanner(ox, oy, GRID_SIZE, ROBOT_RADIUS)
        rx, ry = a_star.planning(self.odom_x, self.odom_y, GX, GY)

        for x,i in enumerate(rx):
            self.path.append([x,ry[i]])

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Perform any other processing or tasks here
            rate.sleep()

if __name__ == '__main__':
    try:
        your_node = NavNode()
        your_node.run()
    except rospy.ROSInterruptException:
        pass
