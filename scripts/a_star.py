#!/usr/bin/env python

import heapq as heap
import os
import sys
# sys.path.append(os.path.abspath("/home/teju/catkin_ws/src/path_planning_practice"))
from grid_n_transforms import grid
from node import Node
import numpy as np

TOLERANCE = 0.2

class PathPlanner:
    def __init__(self):
        print("MAPPING INITIALIZING")
        self.grid = grid()
        
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

        self.start = self.nodes_matrix[0][0]
        # self.theta = theta
        self.goal = self.nodes_matrix[5][5]
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

        while (final == None ) and openList:
            # q is node object with x,y theta
            current = heap.heappop(openList)[1]

            if (current == end):
                print("PATH PLANNED")
                self.path = []
                while current != None:
                    self.path.append(current)
                    current = current.parent

            for neighbour in current.neighbours:
                print(neighbour.is_obstacle)
                if ((neighbour not in closedList) and (not neighbour.is_obstacle)):
                    temp_g = current.g + current.manhattan_dist(neighbour)
                    # frind the distance to neighbour to current node

                    newPath_to_node = False

                    if (neighbour in openList):
                        if temp_g < neighbour.g:
                            neighbour.g = temp_g
                            newPath_to_node = True
                            neighbour.h = neighbour.manhattan_dist(end)
                            neighbour.f = neighbour.g + neighbour.h
                            neighbour.parent = current
                    else:
                        neighbour.g = temp_g
                        newPath_to_node = True
                        neighbour.h = neighbour.manhattan_dist(end)
                        neighbour.f = neighbour.g + neighbour.h
                        neighbour.parent = current
                        heap.heappush(openList , (neighbour.f , neighbour))

            closedList.append(current)

            for spot_gz in openList:
                spot_px = spot_gz[1].tf_gazebo_to_px(grid.meter_per_pixel)
                grid[spot_px[0],spot_px[1]] = 100

            for spot_gz in closedList:
                spot_px = spot_gz[1].tf_gazebo_to_px(grid.meter_per_pixel)
                grid[spot_px[0],spot_px[1]] = 200


yo = PathPlanner()
#print(yo.nodes_matrix[0][0].is_obstacle)
yo.a_star(yo.start , yo.goal , yo.grid)




