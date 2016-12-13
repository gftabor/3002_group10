#!/usr/bin/env python

from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
import rospy, tf, numpy, math


def path_callback(input_path):
	global pointpub
	global path 
	global index 
	global length
	print "saw path"
	length =  len(input_path.poses)
	print length
	#flip list around
	path = list(input_path.poses)
	path.reverse()
	path = optimize_path(path)
	#publish first goal in list
	pointpub.publish(path[0])
	index = 1
	print"new point"
def optimize_path(shitty_path):
	good_path = []
	good_path.append(shitty_path[0])
	for i, PoseStamped in enumerate(shitty_path):
		usefulPoint = 1
		if(i is not len(shitty_path)-1 and i is not 0):
			next = shitty_path[i+1]
			last = shitty_path[i-1]
			if(str(next.pose.position.x - PoseStamped.pose.position.x) is str(PoseStamped.pose.position.x - last.pose.position.x)):
				usefulPoint = 0
		if(usefulPoint):
			good_path.append(PoseStamped)
	good_path.append(shitty_path[-1])
	return good_path

#Expands obstacles in the grid to allow easier navagation.
def obstacleExpansion(grid):
	global mappub

	#Put all cells in grid in a global array.
	cells = grid.data

	#This array is used to designate the index of cells that will become obstacles.
	toBeObstacle = []

	print "started"
	#Runs through all cells in the grid.
	for i in range(0,len(cells)):
		#If a cell is an obstacle the loop will make the four adjacent cells obstacles.

		if (cells[i] > 40):
			#Calculates edge case for the cell left of the given obstacle cell.
			if(i%grid.info.width == 0):
				left = i
			else:
				left = i-1;

			#Calculates edge case for the cell right of the given obstacle cell.
			if(i%grid.info.width == grid.info.width-1):
				right = i
			else:
				right = i+1

			#Calculates edge case for the cell top of the given obstacle cell.
			if(i-grid.info.width < 0):
				top = i
			else:
				top = i-grid.info.width

			#Calculates edge case for the cell bottom of the given obstacle cell.
			if(i+grid.info.width > grid.info.width*grid.info.height-1):
				bottom = i
			else:
				bottom = i+grid.info.width

			#Adds the location of the given cells to the array to be changed.
			toBeObstacle.append(left)
			toBeObstacle.append(right)
			toBeObstacle.append(top)
			toBeObstacle.append(bottom)

	#Loops through the array and changes each value in the array of cells to determine an obstacle.
	l = list(grid.data)
	for j in range(0,len(toBeObstacle)):
		l[toBeObstacle[j]] = 100

	newGrid = OccupancyGrid()
	newGrid.info = grid.info
	newGrid.data = l

	#Publishes the grid as a changed map if expanded enough, else expands again.
	if(checkTimesExpanded(grid.info.resolution)):
		mappub.publish(newGrid)
	else:
		obstacleExpansion(newGrid)

def checkTimesExpanded(resolution):
	global timesExpanded

	print timesExpanded

	timesExpanded = timesExpanded+1

	if(timesExpanded >= 0.35/resolution):
		timesExpanded=0
		return True
	else:
		return False

	


def waypoint_callback():
	global pointpub
	global startFlag
	global path
	global index
	if(startFlag == 1):
		if(index<length):
			pointpub.publish(path[index])
		index = index + 1
	startFlag = 1


#Recursive function. Takes in a grid and starting cell location and returns the numerical location 
#in the list of cells of the closest frontier, -1 if none exist in the enclosed space.
def getClosestFronteir(grid, cellLoc):
	#Define the list of cells as the grid's cells.
	cells = grid.data

	#BASE CASE: If the cell is a fronteir, return the location of the cell.
	if(cells[cellLoc] == -1):
		return cellLoc

	#RECURSIVE CASE: If the cell is known run recursion on all four nearby cells.
	elif(cells[cellLoc] == 0):
		#If left cell exists run on it.
		if(not(cellLoc%grid.info.width == 0)):
			getClosestFronteir(grid, cellLoc-1)
		#If right cell exists run on it.
		if(not(cellLoc%grid.info.width == grid.info.width-1)):
			getClosestFronteir(grid, cellLoc+1)
		#If top cell exists run on it.
		if(not(cellLoc-grid.info.width < 0)):
			getClosestFronteir(grid, cellLoc-grid.info.width)
		#if bottom cell exists run on it.
		if(not(cellLoc+grid.info.width > grid.info.width*grid.info.height-1)):
			getClosestFronteir(grid, cellLoc+grid.info.width)

	#If there's nothing left to run on, return -1.
	elif():
		return -1


def run():

	

	global pointpub
	global mappub
	rospy.init_node('move_robot', anonymous=True)
	path_sub = rospy.Subscriber('/totes_path', Path, path_callback, queue_size=1) #change topic for best results
	pointpub = rospy.Publisher("way_point", PoseStamped, queue_size=100)
	mappub = rospy.Publisher("/map_real", OccupancyGrid, queue_size=1)

	mapsub = rospy.Subscriber('/map',OccupancyGrid,obstacleExpansion, queue_size=1)
    # spin() simply keeps python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':

	global timesExpanded
	timesExpanded = 0

	global mapGrid


	rospy.set_param('/move_base/global_costmap/inflation_layer/inflation_radius', '20')

#/move_base/global_costmap/inflation_layer/inflation_radius

	run()
    