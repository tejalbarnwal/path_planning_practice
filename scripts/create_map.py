#!/usr/bin/env python

import rospy
import numpy as np
import cv2

def map_creator():

# create a grid of height , width
# make its all elements as 1
# at random make its some cells as 0
# define a relation for conversion of pixel to world coordinates => X1,Y1 (IN GAZEBO) = -y , -x in img

  grid_length = 500
  grid_width = 500
  meter_per_pixel = 50

  grid = np.ones((grid_length , grid_width))
  # grid = np.random.rand(grid_length , grid_width)
  # grid = grid < 0.7
  print(grid)

  ## ADDING OBSTACLES
  # assume all obstacles to be rectangular
  obstacle_1_ul_lr = [(1,1) , (4,4)]
  obstacle_2_ul_lr = [(5,0) , (8,3)]
  obstacle_3_ul_lr = [(4,4) , (9,7)]
  obstacle_4_ul_lr = [(0,5) , (4,7)]
  
  #print(obstacle_1_ul_lr[0][0] , obstacle_1_ul_lr[1][0])
  grid[meter_per_pixel * obstacle_1_ul_lr[0][1]: meter_per_pixel * obstacle_1_ul_lr[1][1] 
                                                    , meter_per_pixel * obstacle_1_ul_lr[0][0]: meter_per_pixel * obstacle_1_ul_lr[1][0]] = 0
  
  grid[meter_per_pixel * obstacle_2_ul_lr[0][1]: meter_per_pixel * obstacle_2_ul_lr[1][1] 
                                                    , meter_per_pixel * obstacle_2_ul_lr[0][0]: meter_per_pixel * obstacle_2_ul_lr[1][0]] = 0

  grid[meter_per_pixel * obstacle_3_ul_lr[0][1]: meter_per_pixel * obstacle_3_ul_lr[1][1] 
                                                    , meter_per_pixel * obstacle_3_ul_lr[0][0]: meter_per_pixel * obstacle_3_ul_lr[1][0]] = 0

  grid[meter_per_pixel * obstacle_4_ul_lr[0][1]: meter_per_pixel * obstacle_4_ul_lr[1][1] 
                                                    , meter_per_pixel * obstacle_4_ul_lr[0][0]: meter_per_pixel * obstacle_4_ul_lr[1][0]] = 0

  cv2.imshow("hi" , grid.astype("float"))
  if cv2.waitKey(0) == ord("q"):
    cv2.destroyAllWindows()


map_creator()