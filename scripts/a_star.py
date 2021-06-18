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


TOLERANCE = 0.2

class PathPlanner:
    def __init__(self):
        print("MAPPING INITIALIZING")
        self.grid = grid()
        self.img_matrix = self.grid.grid.copy()
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
        self.goal = self.nodes_matrix[199][199]
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
        print("OPENLIST:",openList)
        print("STARTING LOOP")
        print("                            ")
        self.iterator = 0

        while openList:
            print("---------------------------------------------------------")
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

                    cv2.imshow("path" , self.img_matrix.astype("float"))
                    cv2.waitKey(1)
                    # time.sleep(1)
                cv2.waitKey(0)
                break

            current.add_neighbours(self.grid)
            for neighbour in current.neighbours:
                #print("neighbour coord" ,neighbour.x ,neighbour.y)
                print(neighbour.is_obstacle)
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
            


            for spot_gz in openList:
                spot_px = spot_gz[1].tf_gazebo_to_px(grid.meter_per_pixel)
                # self.img_matrix[spot_px[0],spot_px[1]] = 100/255.
                self.img_matrix[spot_px[1]: grid.meter_per_pixel + spot_px[1]
                                                      , spot_px[0]: grid.meter_per_pixel + spot_px[0]] = 100/255.
                #print("spot in px",spot_px)
                # print("SHAPPPPPPEEEE",self.img_matrix.shape)
            for spot_gz in closedList:
                spot_px = spot_gz.tf_gazebo_to_px(grid.meter_per_pixel)
                # self.img_matrix[spot_px[0],spot_px[1]] = 200/255.
                self.img_matrix[spot_px[1]: grid.meter_per_pixel + spot_px[1]
                                                      , spot_px[0]: grid.meter_per_pixel + spot_px[0]] = 150/255.

            # img = cv2.resize(self.img_matrix.astype("float"))
            cv2.imshow("open&close" , self.img_matrix.astype("float"))
            #print(neighbour.node_gazebo_list)
            # time.sleep(1)
            self.path = []
            while current != None:
                self.path.append(current)
                current = current.parent
            for i in self.path:
                print(i.x , i.y)
                spot_px = i.tf_gazebo_to_px(grid.meter_per_pixel)
                # self.img_matrix[spot_px[0],spot_px[1]] = 200/255.
                self.img_matrix[spot_px[1]: grid.meter_per_pixel + spot_px[1]
                                                      , spot_px[0]: grid.meter_per_pixel + spot_px[0]] = 210/255.

                cv2.imshow("path" , self.img_matrix.astype("float"))


            if cv2.waitKey(1) == ord("q"):
                cv2.destroyAllWindows()
            # time.sleep(1)
            print(self.iterator)
            print("---------------------------------------------------------")
            self.iterator = self.iterator + 1

yo = PathPlanner()
#print(yo.nodes_matrix[0][0].is_obstacle)
yo.a_star(yo.start , yo.goal , yo.grid)


