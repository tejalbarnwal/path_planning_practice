#!/usr/bin/env python

import rospy
import numpy as np
import cv2
import math

class grid:
  def __init__(self):
    
    self.grid_length = 500
    self.grid_width = 500

    self.world_grid_length = 200
    self.world_grid_width = 200

    self.meter_per_pixel = int(self.grid_length / self.world_grid_length) # it is actually pixels per meter shitttt mannn

    self.grid = np.ones((self.grid_length , self.grid_width))
    self.add_obstacles()
    cv2.imshow("original",self.grid.astype("float"))
    #print(self.grid)

  def __getitem__(self , pixel_list):
    return self.grid[pixel_list[0]][pixel_list[1]]


  def add_obstacles(self):

    obstacle_1_ul_lr = [(10,10) , (40,40)]
    obstacle_2_ul_lr = [(50,0) , (80,30)]
    obstacle_3_ul_lr = [(40,40) , (80,90)]
    obstacle_4_ul_lr = [(0,50) , (30,70)]
    
    #print(obstacle_1_ul_lr[0][0] , obstacle_1_ul_lr[1][0])
    self.grid[self.meter_per_pixel * obstacle_1_ul_lr[0][1]: self.meter_per_pixel * obstacle_1_ul_lr[1][1] 
                                                      , self.meter_per_pixel * obstacle_1_ul_lr[0][0]: self.meter_per_pixel * obstacle_1_ul_lr[1][0]] = 0
    
    self.grid[self.meter_per_pixel * obstacle_2_ul_lr[0][1]: self.meter_per_pixel * obstacle_2_ul_lr[1][1] 
                                                      , self.meter_per_pixel * obstacle_2_ul_lr[0][0]: self.meter_per_pixel * obstacle_2_ul_lr[1][0]] = 0

    self.grid[self.meter_per_pixel * obstacle_3_ul_lr[0][1]: self.meter_per_pixel * obstacle_3_ul_lr[1][1] 
                                                      , self.meter_per_pixel * obstacle_3_ul_lr[0][0]: self.meter_per_pixel * obstacle_3_ul_lr[1][0]] = 0

    self.grid[self.meter_per_pixel * obstacle_4_ul_lr[0][1]: self.meter_per_pixel * obstacle_4_ul_lr[1][1] 
                                                      , self.meter_per_pixel * obstacle_4_ul_lr[0][0]: self.meter_per_pixel * obstacle_4_ul_lr[1][0]] = 0

 
  def show_grid(self , window_name):
    img = cv2.resize(self.grid.astype("float") , (400,400))
    cv2.imshow(window_name , img)
    



  


# grid1 = grid()
# #grid1.show_grid("1")
# if cv2.waitKey(0) == ord("q"):
#       cv2.destroyAllWindows()
# list1 = [250,0]
# print(grid1.tf_px_to_gazebo(list1))
# print(grid1.tf_gazebo_to_px([-1,0]))



