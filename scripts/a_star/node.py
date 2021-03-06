#!/usr/bin/env python

import math
from grid_n_transforms import grid
import numpy as np

SIMILARITY_THRESHOLD = 0.1
SAFETY_OFFSET = 5  #no.of pixels away from the wall the robot should remain


grid = grid()

class Node:
  def __init__(self , x , y , parent=None):

    self.x = x
    self.y = y

    self.node_gazebo_list = [x,y]
    self.node_pixel_list = self.tf_gazebo_to_px(grid.meter_per_pixel)

    # self.theta = theta
    self.parent = parent

    self.f = 0.0
    self.g = 0.0
    self.h = 0.0

    self.neighbours = []  
    # self.add_neighbours(grid)

    # print(grid[self.node_pixel_list[0] , self.node_pixel_list[1]])
    for i in range(self.node_pixel_list[0] , self.node_pixel_list[0] + grid.meter_per_pixel):
      for j in range(self.node_pixel_list[1] , self.node_pixel_list[1] + grid.meter_per_pixel):
        if grid[j,i] == 0:
          self.is_obstacle = True
          break
        else:
          self.is_obstacle = False


  def tf_gazebo_to_px(self , meter_per_pixel):

    scaled_px_list = [-1 * i for i in reversed(self.node_gazebo_list)]
    #print("scaled px list" , scaled_px_list)
    px_list = [meter_per_pixel * i for i in scaled_px_list]
    #print("px list" ,px_list)
    return px_list

  def tf_px_to_gazebo(self , meter_per_pixel):

    scaled_px_list = [int(i/meter_per_pixel) for i in self.node_pixel_list]
    #print("scaled" , scaled_px_list)
    gazebo_list = [-1 * i for i in reversed(scaled_px_list)]
    return gazebo_list

  def tf_gazeboTH_to_pxTH(self):
    pass

  def euclidean_dist(self , point_2):
    return math.dist(self.node_gazebo_list , point_2)

  def manhattan_dist(self ,point_2):
    # print(point_2)
    # print("yo")
    return abs(self.x - point_2[0]) + abs(self.y - point_2[1])

  def add_neighbours(self , grid):
    i = self.x
    j = self.y
    # print("gazebo coord" , i , j)

    if (-i < grid.world_grid_length - 1):
      # nextnode_pixel_list = [i+1 , j]
      # nextnode_gazebo_list = self.tf_px_to_gazebo(grid.meter_per_pixel)
      # print("case1" , nextnode_gazebo_list)
      self.neighbours.append(Node(i-1 , j))
      # print("1")

    if (-i > 0):
      # nextnode_pixel_list = [i-1 , j]
      # nextnode_gazebo_list = self.tf_px_to_gazebo(grid.meter_per_pixel)
      #print("case1" , nextnode_gazebo_list)
      self.neighbours.append(Node(i+1 , j))
      # print("2")

    if (-j < grid.world_grid_width - 1):
      # nextnode_pixel_list = [i , j+1]
      # nextnode_gazebo_list = self.tf_px_to_gazebo(grid.meter_per_pixel)
      # print("case1" , nextnode_gazebo_list)
      self.neighbours.append(Node(i , j-1))
      # print("3")

    if (-j > 0):
      # nextnode_pixel_list = [i , j-1]
      # nextnode_gazebo_list = self.tf_px_to_gazebo(grid.meter_per_pixel)
      # print("case1" , nextnode_gazebo_list)
      self.neighbours.append(Node(i , j+1))
      # print("4")



#node = Node(10,10)
# print(node.is_obstacle)

# self.grid_length = 500
# self.grid_width = 500

# self.world_grid_length = 10
# self.world_grid_width = 10





# list1 = [0,1,2,3,4,5,6,7,8,9]
#         # making nodes
# nodes_matrix = np.full((10,10) , Node(0,0))
# for x in list1:
#     for y in list1:
#         nodes_matrix[x][y] = Node(-x,-y)
# print("##############################################################")
# # print(nodes_matrix[0][0].node_pixel_list)

# for x in list1:
#   for y in list1:
#       nodes_matrix[x][y].add_neighbours(grid)

# print("neighbours")
# for i in nodes_matrix[1][0].neighbours:
#   print(i.x , i.y , "done")