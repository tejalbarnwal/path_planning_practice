#!/usr/bin/env python

import heapq as heap
from grid_n_transform import grid
from node import Node


TOLERANCE = 0.2

class PathPlanner:
	def __init__(self , start , theta , goal):
		print("mapping initializing")
		self.grid = grid()
		self.grid.add_obstacle()
		self.start = start #
		self.theta = theta #
		self.goal = goal #
		print("map done! Planning initializing")

		# making nodes
		# add_neighbours
	def a_star(self , start , end , grid):
		# it takes three inputs -> start , goal and map, start and end are in world coordinates

		# before stating a_star checkout if the goal is obstacle free
			# check if goal is valid and take input as grid map
		if not end.is_valid(grid):
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
			current = heap.heappop(opened)[1]

			if (current == end):
				print("PATH PLANNED")

			for neighbour in current.neighbours:
				if ((neighbour not in closedList) and (not neighbour.is_obstacle)):
					temp_g = current.g + 

