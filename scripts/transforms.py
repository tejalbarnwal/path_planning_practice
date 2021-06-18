#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from math import sqrt

# WORLD FRAME - X ,Y
# IMAGE FRAME - x,y

def img_to_world(px_list):
	# first scale the img coordinates to world coordinates
	# tranform it to get wrt gazebo x and y 
	scaled_coordinated = meter_per_pixel * x , meter_per_pixel * y
	gazebo_frame = -scaled_coordinated[1] , -scaled_coordinated[0]
	pass


def world_to_img():
	to_be_scaled = [-gazebo_frame[1] , -gazebo_frame[0]]
	pixel_coor = to_be_scale/10
	pass

def worldHeaing_to_pixelHeading():
	pass

def euclidean_distance(a ,b):
	return sqrt( (a[0] - b[0]) ** 2 + (a[1] - b[1])**2 )