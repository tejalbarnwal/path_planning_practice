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
            if  abs(goal[0] - self.x) >= 0.01:
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


                    while abs(goal[0] - self.x) >= 0.001:
                        self.vel_cmd.linear.x = (goal[0] - self.x) * 1.0
                        self.vel_cmd.angular.x = 0.0
                        print("F1")

                        self.vel_publisher.publish(self.vel_cmd)
                        self.rate.sleep()

                    self.vel_cmd.linear.x = 0.0
                    self.vel_cmd.angular.z = 0.0
                    print("G")

            if abs(goal[1]-self.y) >= 0.01:
                print("A11")
                print("theta" , self.theta)
                print("tan" ,math.atan2(goal[1] - self.y , goal[0] - self.x))
                while abs(self.theta - (math.atan2(goal[1] - self.y , goal[0] - self.x))) >= 0.01:
                    print("B11")
                    print("theta" , self.theta)
                    print("tan" ,math.atan2(goal[1] - self.y , goal[0] - self.x))
                    self.vel_cmd.angular.z = (self.theta - (math.atan2(goal[1] - self.y , goal[0] - self.x))) * -0.1
                    print("D11")

                    self.vel_cmd.linear.x = 0.0

                    self.vel_publisher.publish(self.vel_cmd)
                    self.rate.sleep()

                self.vel_cmd.linear.x = 0.0
                self.vel_cmd.angular.z = 0.0
                print("E11")


                while abs(goal[1] - self.y) >= 0.001:
                    self.vel_cmd.linear.x = (goal[1] - self.y) * 0.5
                    self.vel_cmd.angular.x = 0.0
                    print("F11")

                    self.vel_publisher.publish(self.vel_cmd)
                    self.rate.sleep()

                self.vel_cmd.linear.x = 0.0
                self.vel_cmd.angular.z = 0.0
                print("G11")


            


path_goals = [[0,1]]
teju_control = controller(path_goals)
teju_control.control_it()
