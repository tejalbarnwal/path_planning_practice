#!/usr/bin/env python

import math
from grid_n_transforms import grid

SIMILARITY_THRESHOLD = 0.1
SAFETY_OFFSET = 5  #no.of pixels away from the wall the robot should remain


grid = grid()

class Node:
	def __init__(self , coordinate_list , parent=None):

		self.x = coordinate_list[0]
		self.y = coordinate_list[1]

    self.node_gazebo_list = coordinate_list
    self.node_pixel_list = self.tf_gazebo_to_px(self.node_gazebo_list , grid.meter_per_pixel)

		# self.theta = theta
		self.parent = parent

		self.f = 0.0
		self.g = 0.0
		self.h = 0.0

    self.neighbours = []  
    self.add_neighbours()

    if grid[self.node_pixel_list[0] , self.node_pixel_list[1]] == 1:
      self.is_obstacle = False
    else:
      self.is_obstacle = True


	def tf_gazebo_to_px(self , gazebo_list , meter_per_pixel):

    scaled_px_list = [-1 * i for i in reversed(gazebo_list)]
    print(scaled_px_list)
    px_list = [meter_per_pixel * i for i in scaled_px_list]
    print(px_list)
    return px_list

  def tf_px_to_gazebo(self , px_list , meter_per_pixel):

    scaled_px_list = [int(i/meter_per_pixel) for i in px_list]
    print("scaled" , scaled_px_list)
    gazebo_list = [-1 * i for i in reversed(scaled_px_list)]
    return gazebo_list

  def tf_gazeboTH_to_pxTH(self):
    pass

  def euclidean_dist(self , point_2):
    return math.dist(self.node_gazebo_list , point_2)

  def manhattan_dist(self ,point_2):
    return abs(self.node_gazebo_list[0] - point_2[0]) + abs(self.node_gazebo_list[0] - point_2[0])

  def add_neighbours(self , grid):
    i = self.node_pixel_list[0]
    j = self.node_pixel_list[1]

    if (i < grid.grid_length - 1):
      nextnode_pixel_list = [i+1 , j]
      nextnode_gazebo_list = self.tf_px_to_gazebo(nextnode_pixel_list , grid.meter_per_pixel)
      self.neighbours.append(nextnode_gazebo_list)

    if (i > 0):
      nextnode_pixel_list = [i-1 , j]
      nextnode_gazebo_list = self.tf_px_to_gazebo(nextnode_pixel_list , grid.meter_per_pixel)
      self.neighbours.append(nextnode_gazebo_list)

    if (j < grid.grid_width - 1):
      nextnode_pixel_list = [i , j+1]
      nextnode_gazebo_list = self.tf_px_to_gazebo(nextnode_pixel_list , grid.meter_per_pixel)
      self.neighbours.append(nextnode_gazebo_list)

    if (j > 0):
      nextnode_pixel_list = [i , j-1]
      nextnode_gazebo_list = self.tf_px_to_gazebo(nextnode_pixel_list , grid.meter_per_pixel)
      self.neighbours.append(nextnode_gazebo_list)