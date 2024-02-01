#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, Pose, PoseStamped, PointStamped
import tf

#from a_star import AStarPlanner

import numpy as np
import matplotlib.pyplot as plt
import random
import math
import time



GRID_SIZE = 1
CALC_RES = 8
ROBOT_RADIUS = 15

K = 0.5

NXT = 2

show_animation = True



class NavNode:
    def __init__(self):
        rospy.init_node('nav_node', anonymous=True)
        self.PUBPOINT = True

        #GOAL POINT
        self.GX = -13
        self.GY = -3
        
        # Subscribers
        while self.PUBPOINT:
            rospy.Subscriber("clicked_point", PointStamped, self.point_callback)
            continue
        rospy.Subscriber('/map', OccupancyGrid, self.occupancy_grid_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Publisher
        self.vel_cmd_publisher = rospy.Publisher('/cmd_vel_op', Twist, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.01), self.timer_callback)

        self.pose_publisher = rospy.Publisher('/c_pose', Pose, queue_size=10)
        self.path_publisher = rospy.Publisher('/path', Path, queue_size=10)

        self.tf_listener = tf.TransformListener()

        # Initialize other variables or setup here
        self.grid = []
        self.og_width = 0
        self.og_height = 0
        self.res = 1

        self.min_x = 0.0
        self.min_y = 0.0
        self.max_x = 0.0
        self.max_y = 0.0


        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_rz = 0.0
        self.odom_time = rospy.get_rostime()

        self.vel = Twist()

        self.rx = []
        self.ry = []

        #ORIGINE MAP
        self.orx = 0.0
        self.ory = 0.0
        self.orht = 0.0

    class Node_:
        cost = 0
        x = 0
        y = 0
        parent_index = 0

        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.cost) + "," + str(self.parent_index)

    def point_callback(self,data):
        if self.PUBPOINT == True:
            self.GX = data.point.x
            self.GY = data.point.y
            self.PUBPOINT = False

    
    def timer_callback(self,timer):
        #rospy.loginfo("Entering timer callback")
        # Example: Publish a sample Twist message
        twist_msg = Twist()
        twist_msg.linear.x = 0
        twist_msg.linear.y = 0

        if len(self.grid)>0:
            twist_msg = self.main_planning()

        self.vel_cmd_publisher.publish(twist_msg)
        #rospy.loginfo("Exiting timer callback")

    def occupancy_grid_callback(self, data):
        # Process occupancy grid data here
        #rospy.loginfo("Received occupancy grid data")

        # Extract grid information
        self.og_width = data.info.width
        self.og_height = data.info.height
        grid_tmp = data.data
        self.orx = data.info.origin.position.x
        self.ory = data.info.origin.position.y
        # self.orth = data.info.origin.orientation.w
        print("origine: \n x:",self.orx,"\n y: ", self.ory)

        self.res = data.info.resolution

        # Convert 1D data array into a 2D array
        self.grid = [[0 for _ in range(self.og_width)] for _ in range(self.og_height)]

        for i in range(self.og_height):
            for j in range(self.og_width):
                index = i * self.og_width + j
                self.grid[i][j] = grid_tmp[index]


    def odom_callback(self, data):
        # Process odometry data here
        # rospy.loginfo("Received odometry data")
        # now = rospy.get_rostime()
        # odom_y = data.pose.pose.position.y
        # odom_x = data.pose.pose.position.x

        # if self.odom_time.secs > 0:
        #     self.vel.linear.x = (odom_x - self.odom_x) / (self.odom_time.secs)
        #     self.vel.linear.x = (odom_y - self.odom_y) / (self.odom_time.secs)

        # self.odom_x = odom_x
        # self.odom_y = odom_y
        # self.odom_rw = data.pose.pose.orientation.w

        # pose_msg = Pose()
        # pose_msg.position.x = odom_x 
        # pose_msg.position.y = odom_y 
        # pose_msg.orientation.z = data.pose.pose.orientation.z
        # pose_msg.orientation.w = self.odom_rw 
        # self.pose_publisher.publish(pose_msg)

        # self.odom_time = rospy.get_rostime() - now

        (trans,rot) = self.tf_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
        (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(rot)
        self.odom_x = trans[0]
        self.odom_y = trans[1]

        pose_msg = Pose()
        pose_msg.position.x = self.odom_x 
        pose_msg.position.y = self.odom_y 
        pose_msg.orientation.w = yaw
        self.pose_publisher.publish(pose_msg)

    def main_planning(self):
        #rospy.loginfo("Entering main planning")
        #print(__file__ + " start!!")

        # start and goal position
        #sx = self.odom_x  # [m]
        #sy = self.odom_y  # [m]
        sx = self.odom_x#/self.res 
        sy = self.odom_y#/self.res
        gx = self.GX/self.res  # [m]
        gy = self.GY/self.res # [m]
        grid_size = CALC_RES # [m]
        robot_radius = ROBOT_RADIUS          # [m]


        #print("Position Actuelle : ", sx, ",", sy)

        pos_prec_x = sx 
        pos_prec_y = sy-1

        # set obstacle positions
        occupancyMap2D = self.grid
        
        ox, oy = [], []
        for x,row in enumerate(self.grid):
            for y,column in enumerate(row):
                if column > 0:
                    ox.append(y + self.orx/self.res)
                    oy.append(x + self.ory/self.res) 

                    # ox.append(y)
                    # oy.append(x) 
        
        self.resolution = grid_size
        self.rr = robot_radius
        self.motion = self.get_motion_model()
        if (len(ox) != 0) and (len(oy) != 0):
            self.calc_obstacle_map(ox, oy)

        if show_animation:  # pragma: no cover
            plt.plot(ox, oy, ".k")
            plt.plot(sx, sy, "og")
            plt.plot(gx, gy, "xb")
            plt.grid(True)
            plt.axis("equal")

        if len(self.rx) <= 2:
            self.rx, self.ry = self.planning(sx, sy, gx, gy)

            self.rx.reverse()
            self.ry.reverse()

            self.rx = [x*self.res for x in self.rx]
            self.ry = [y*self.res for y in self.ry]

            path_msg = Path()
            for x,y in zip(self.rx,self.ry):
                pose_msg_tmp = PoseStamped()
                pose_msg_tmp.pose.position.x = x
                pose_msg_tmp.pose.position.y = y
                path_msg.poses.append(pose_msg_tmp)

                path_msg.header.frame_id= "/map"
            
            self.path_publisher.publish(path_msg)

        v1 = 0
        v2 = 0

        if len(self.rx)>1:
            print("Path length: ", len(self.rx))
            # print([x*self.res for x in self.rx[0:10]])
            # print([y*self.res for y in self.ry[0:10]])

            v1 = K * (self.rx[NXT] - self.odom_x)
            v2 = K * (self.ry[NXT] - self.odom_y)

            if v1>5 or v2>5:
                v1 = 0
                v2 = 0

        print("Calculated velocity : ", v1, v2)

        twist_msg = Twist()
        twist_msg.linear.x = v1
        twist_msg.linear.y = v2
        
        dist_x = 100
        dist_y = 100

        if len(self.rx)>1:
            dist_x = abs(self.odom_x-self.rx[NXT])
            dist_y = abs(self.odom_y-self.ry[NXT])

            print("Current pose: ", self.odom_x, self.odom_y)
            print("Target pose: ", self.rx[NXT], self.ry[NXT])
            print("Distance to next target: ",dist_x, dist_y)

        if len(self.rx)>0 and dist_x<0.2 and dist_y<0.2:
            self.rx.pop(0)
            self.ry.pop(0)

        return twist_msg
    
    def planning(self, sx, sy, gx, gy):
        """
        A star path search

        input:
            s_x: start x position [m]
            s_y: start y position [m]
            gx: goal x position [m]
            gy: goal y position [m]

        output:
            rx: x position list of the final path
            ry: y position list of the final path
        """
        #rospy.loginfo("Entering path planning")

        start_node = self.Node_(self.calc_xy_index(sx, self.min_x),
                               self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node_(self.calc_xy_index(gx, self.min_x),
                              self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node
        while True:
            if len(open_set) == 0:
                #print("Open set is empty..")
                break

            c_id = min(
                open_set,
                key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node,
                                                                     open_set[
                                                                         o]))
            current = open_set[c_id]

            # show graph
            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                # for stopping simulation with the esc key.
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            if current.x == goal_node.x and current.y == goal_node.y:
                #print("Find goal")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from the open set
            del open_set[c_id]

            # Add it to the closed set
            closed_set[c_id] = current

            # expand_grid search grid based on motion model
            for i, _ in enumerate(self.motion):
                node = self.Node_(current.x + self.motion[i][0],
                                 current.y + self.motion[i][1],
                                 current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                # If the node is not safe, do nothing
                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node  # discovered a new node
                else:
                    if open_set[n_id].cost > node.cost:
                        # This path is the best until now. record it
                        open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.og_width*self.res + (node.x - self.min_x)
    
    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """
        calc grid position

        :param index:
        :param min_position:
        :return:
        """
        #self.get_logger().info('entering calc_grid_position()')

        pos = index * self.resolution + min_position
        return pos
    
    @staticmethod
    def get_motion_model():
        #rospy.loginfo("Entering motion model")
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion

    def calc_obstacle_map(self, ox, oy):
        #self.get_logger().info('entering calc_obstacle_map()')
        #rospy.loginfo("Calculating obstacle map")

        self.min_x = round(min(ox))
        self.min_y = round(min(oy))
        self.max_x = round(max(ox))
        self.max_y = round(max(oy))
        #print("min_x:", self.min_x)
        #print("min_y:", self.min_y)
        #print("max_x:", self.max_x)
        #print("max_y:", self.max_y)

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)
        #print("x_width:", self.x_width)
        #print("y_width:", self.y_width)

        # obstacle map generation
        self.obstacle_map = [[False for _ in range(self.y_width)]
                             for _ in range(self.x_width)]
        for ix in range(0, self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(0, self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for iox, ioy in zip(ox, oy):
                    d = math.hypot(iox - x, ioy - y)
                    if d <= self.rr:
                        self.obstacle_map[ix][iy] = True
                        break

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x:
            return False
        elif py < self.min_y:
            return False
        elif px >= self.max_x:
            return False
        elif py >= self.max_y:
            return False

        # collision check
        if node.x < self.x_width and node.y < self.y_width:
            if self.obstacle_map[node.x][node.y]:
                return False

        return True

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry



    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Perform any other processing or tasks here
            rate.sleep()

if __name__ == '__main__':
    try:
        NavNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
