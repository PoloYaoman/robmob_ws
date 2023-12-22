#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from a_star import AStarPlanner


GRID_SIZE = 1
ROBOT_RADIUS = 0.3

GX = -13
GY = -3

K = 1


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
        rospy.loginfo("Received occupancy grid data")

        self.grid = data.data

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

        for x,y in rx,ry:
            self.path.append([x,y])

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
