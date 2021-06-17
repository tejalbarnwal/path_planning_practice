#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import math

class grid:
  def __init__(self):
    
    self.grid_length = 500
    self.grid_width = 500

    self.world_grid_length = 10
    self.world_grid_width = 10

    self.meter_per_pixel = int(self.grid_length / self.world_grid_length)

    self.grid = np.ones((self.grid_length , self.grid_width))
    #print(self.grid)


  def add_obstacles(self):

    self.obstacle_1_ul_lr = [(1,1) , (4,4)]
    self.obstacle_2_ul_lr = [(5,0) , (8,3)]
    self.obstacle_3_ul_lr = [(4,4) , (9,7)]
    self.obstacle_4_ul_lr = [(0,5) , (4,7)]
    
    #print(obstacle_1_ul_lr[0][0] , obstacle_1_ul_lr[1][0])
    self.grid[meter_per_pixel * obstacle_1_ul_lr[0][1]: meter_per_pixel * obstacle_1_ul_lr[1][1] 
                                                      , meter_per_pixel * obstacle_1_ul_lr[0][0]: meter_per_pixel * obstacle_1_ul_lr[1][0]] = 0
    
    self.grid[meter_per_pixel * obstacle_2_ul_lr[0][1]: meter_per_pixel * obstacle_2_ul_lr[1][1] 
                                                      , meter_per_pixel * obstacle_2_ul_lr[0][0]: meter_per_pixel * obstacle_2_ul_lr[1][0]] = 0

    self.grid[meter_per_pixel * obstacle_3_ul_lr[0][1]: meter_per_pixel * obstacle_3_ul_lr[1][1] 
                                                      , meter_per_pixel * obstacle_3_ul_lr[0][0]: meter_per_pixel * obstacle_3_ul_lr[1][0]] = 0

    self.grid[meter_per_pixel * obstacle_4_ul_lr[0][1]: meter_per_pixel * obstacle_4_ul_lr[1][1] 
                                                      , meter_per_pixel * obstacle_4_ul_lr[0][0]: meter_per_pixel * obstacle_4_ul_lr[1][0]] = 0

 
  def show_grid(self):

    cv2.imshow("grid_window" , self.grid.astype("float"))
    if cv2.waitKey(0) == ord("q"):
      cv2.destroyAllWindows()



  


# grid1 = grid()
# grid1.show_grid()
# list1 = [250,0]
# print(grid1.tf_px_to_gazebo(list1))
# print(grid1.tf_gazebo_to_px([-1,0]))



