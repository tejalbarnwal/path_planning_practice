#!/usr/bin/env python

import heapq as heap
import os
import sys
# sys.path.append(os.path.abspath("/home/teju/catkin_ws/src/path_planning_practice"))
from grid_n_transforms import grid
from node import Node
import numpy as np
import cv2
import time
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math


TOLERANCE = 0.2

class PathPlanner:
    def __init__(self):
        print("MAPPING INITIALIZING")
        self.grid = grid()
        
        # cv2.imshow("original2", self.img_matrix.astype("float"))
        # self.grid.add_obstacle()
        
        # list1 = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14]
        # making nodes
        self.nodes_matrix = np.full((self.grid.world_grid_length , self.grid.world_grid_width) , Node(0,0))
        for x in range(0, self.grid.world_grid_length):
            for y in range(0, self.grid.world_grid_length):
                self.nodes_matrix[x][y] = Node(-x,-y)

        # add_neighbours
        for x in range(0, self.grid.world_grid_length):
            for y in range(0, self.grid.world_grid_length):
                self.nodes_matrix[x][y].add_neighbours(self.grid)

        # print("neighbours")
        # for i in self.nodes_matrix[1][0].neighbours:
        #     print(i.x , i.y , "done")

        self.start = self.nodes_matrix[0][0]
        # self.theta = theta
        self.goal = self.nodes_matrix[14][14]
        print("MAP DONE! PLANNING INITIALIZING")
        self.path = None

    def a_star(self , start , end , grid):
        # it takes three inputs -> start , goal and map, start and end are in world coordinates

        # before stating a_star checkout if the goal is obstacle free
            # check if goal is valid and take input as grid map
        if end.is_obstacle:
            print("GOAL LOCATION INVALID")
            return None
        print("GOAL IS VALID! :)")
        # if goal is valid go ahead with applying the algorithm
        openList = []
        closedList = []

        final = None

        heap.heappush(openList , (0.0 , start))
        #print("OPENLIST:",openList)
        #print("STARTING LOOP")
        #print("                            ")
        self.iterator = 0

        while openList:
            self.img_matrix = grid.grid.copy()
            #print("---------------------------------------------------------")
            # q is node object with x,y theta
            current = heap.heappop(openList)[1]
            #print("cureent node ",current.x , current.y)

            #print("OPENLIST:",openList)
            #print("                                              ")


            if (current.x == end.x and current.y == end.y):
                print("PATH PLANNED")
                self.path = []
                while current != None:
                    self.path.append(current)
                    current = current.parent
                print("YAYAYAYAYAYAYYAYAYYA!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
                print("                     ")
                for i in self.path:
                    print(i.x , i.y)
                    spot_px = i.tf_gazebo_to_px(grid.meter_per_pixel)
                    # self.img_matrix[spot_px[0],spot_px[1]] = 200/255.
                    self.img_matrix[spot_px[1]: grid.meter_per_pixel + spot_px[1]
                                                          , spot_px[0]: grid.meter_per_pixel + spot_px[0]] = 210/255.

                    #cv2.imshow("path" , self.img_matrix.astype("float"))
                    #cv2.waitKey(1)
                    # time.sleep(1)
                #cv2.waitKey(0)
                return self.path
                

            current.add_neighbours(self.grid)
            for neighbour in current.neighbours:
                #print("neighbour coord" ,neighbour.x ,neighbour.y)
                #print(neighbour.is_obstacle)
                if ((neighbour not in closedList) and (not neighbour.is_obstacle)):
                    temp_g = current.g + current.manhattan_dist(neighbour.node_gazebo_list)
                    # frind the distance to neighbour to current node

                    newPath_to_node = False

                    if (neighbour in openList):
                        if temp_g < neighbour.g:
                            neighbour.g = temp_g
                            newPath_to_node = True
                            neighbour.h = neighbour.manhattan_dist(end.node_gazebo_list)
                            neighbour.f = neighbour.g + neighbour.h
                            neighbour.parent = current
                    else:
                        neighbour.g = temp_g
                        newPath_to_node = True
                        neighbour.h = neighbour.manhattan_dist(end.node_gazebo_list)
                        neighbour.f = neighbour.g + neighbour.h
                        neighbour.parent = current
                        heap.heappush(openList , (neighbour.f , neighbour))

                    #print("did you find new path to node" , newPath_to_node)
                    #print("OPEN-LIST:",openList)

            closedList.append(current)
        
           
            #print("CLOSED-LIST:",closedList)
            

            print(len(openList))
            for spot_gz in openList:
                spot_px = spot_gz[1].tf_gazebo_to_px(grid.meter_per_pixel)
                # self.img_matrix[spot_px[0],spot_px[1]] = 100/255.
                self.img_matrix[spot_px[1]: grid.meter_per_pixel + spot_px[1]
                                                      , spot_px[0]: grid.meter_per_pixel + spot_px[0]] = 100/255.
                #print("spot in px",spot_px)
                # print("SHAPPPPPPEEEE",self.img_matrix.shape)
            # for spot_gz in closedList:
            #     spot_px = spot_gz.tf_gazebo_to_px(grid.meter_per_pixel)
            #     # self.img_matrix[spot_px[0],spot_px[1]] = 200/255.
            #     self.img_matrix[spot_px[1]: grid.meter_per_pixel + spot_px[1]
            #                                           , spot_px[0]: grid.meter_per_pixel + spot_px[0]] = 150/255.

            # img = cv2.resize(self.img_matrix.astype("float"))
            cv2.imshow("open" , cv2.resize(self.img_matrix.astype("float"),(100,100)))
            #print(neighbour.node_gazebo_list)
            # time.sleep(1)
            self.path = []
            while current != None:
                self.path.append(current)
                current = current.parent
            for i in self.path:
                #print(i.x , i.y)
                spot_px = i.tf_gazebo_to_px(grid.meter_per_pixel)
                # self.img_matrix[spot_px[0],spot_px[1]] = 200/255.
                self.img_matrix[spot_px[1]: grid.meter_per_pixel + spot_px[1]
                                                      , spot_px[0]: grid.meter_per_pixel + spot_px[0]] = 210/255.

                cv2.imshow("path" , cv2.resize(self.img_matrix.astype("float"),(100,100)))


            if cv2.waitKey(1) == ord("q"):
               cv2.destroyAllWindows()
            # time.sleep(1)
            #print(self.iterator)
            #print("---------------------------------------------------------")
            self.iterator = self.iterator + 1

yo = PathPlanner()
#print(yo.nodes_matrix[0][0].is_obstacle)
path = yo.a_star(yo.start , yo.goal , yo.grid)
path_goals = []
for i in reversed(path):
    path_goals.append([i.x , i.y])


class controller:
    def __init__(self , path_goals):
        rospy.init_node("teju")

        self.vel_publisher = rospy.Publisher("/ubiquity_velocity_controller/cmd_vel" , Twist , queue_size = 1)

        rospy.Subscriber("/exact_pose" , Odometry , self.odom_callback)

        self.x = 0.0
        self.y = 0.0

        self.theta = 0.0

        #rpy=000 , xyzw=0001
        #rpy=00 90 , xyzw=00 0.707 0.707
        #rpy=00 -90 , xyzw=0 0 -0.707 0.707
        self.path_goals = path_goals

        self.vel_cmd = Twist()
        self.vel_cmd.linear.x = 0.0
        self.vel_cmd.linear.x = 0.0
        self.vel_cmd.linear.x = 0.0
        self.vel_cmd.angular.x = 0.0
        self.vel_cmd.angular.x = 0.0
        self.vel_cmd.angular.x = 0.0

        self.rate = rospy.Rate(20)


    def odom_callback(self , msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
       
        (__ ,__ , self.theta) = euler_from_quaternion([msg.pose.pose.orientation.x , msg.pose.pose.orientation.y
                                                            , msg.pose.pose.orientation.z , msg.pose.pose.orientation.w])
        

    def control_it(self):

        for goal in path_goals:
            print("goal for now", goal)
            while goal[0] - self.x <= -0.001 or goal[1] - self.y <= -0.001:
                    print("A")
                    while abs(self.theta - (math.atan2(goal[1] - self.y , goal[0] - self.x))) <= 0.001:
                        print("B")
                        if (goal[1] - self.y) * (goal[0] - self.x) < 0:
                            self.vel_cmd.angular.z = -0.05
                            print("C")

                        else:
                            self.vel_cmd.angular.z = 0.05
                            print("D")

                        self.vel_cmd.linear.x = 0.0

                        self.vel_publisher.publish(self.vel_cmd)
                        self.rate.sleep()

                    self.vel_cmd.linear.x = 0.0
                    self.vel_cmd.angular.z = 0.0
                    print("E")


                    while goal[0] - self.x <= -0.01:
                        self.vel_cmd.linear.x = (goal[0] - self.x) * 0.01
                        self.vel_cmd.angular.x = 0.0
                        print("F1")

                        self.vel_publisher.publish(self.vel_cmd)
                        self.rate.sleep()

                    while goal[1] - self.y <= -0.01:
                        self.vel_cmd.linear.x = (goal[1] - self.y) * 0.01
                        self.vel_cmd.angular.x = 0.0
                        print("F2")

                        self.vel_publisher.publish(self.vel_cmd)
                        self.rate.sleep()

                    self.vel_cmd.linear.x = 0.0
                    self.vel_cmd.angular.z = 0.0
                    print("G")


teju_control = controller(path_goals)
teju_control.control_it()













