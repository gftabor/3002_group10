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
	path = reversed(input_path.poses)
	#publish first goal in list
	pointpub.publish(path[0])
	index = 1
	print"new point"

#Expands obstacles in the grid to allow easier navagation.
def obstacleExpansion(grid):

	#TODO put all cells in grid in a global array.
	cells = grid.data

	#This array is used to designate cells that will become obstacles.
	toBeObstacle = []

	#Runs through all cells in the grid.
	for i in cells:
		#If a cell is not an obstacle and is too close to an obstacle add it to the array that will make designated cells obstacles.
		if (cells[i] < 20) and isCloseToObstacle(cells[i], 0):
			toBeObstacle.append(cells[i])

	#Makes cells in toBeObstacle obstacles.
	for j in toBeObstacle:
		toBeObstacle.cells[j].isObstacle = True

#Recursive function returns true if the cell passed is an obstacle, false if far away cells are not obstacles.
def isCloseToObstacle(cell, startingLoc, distFromStart):
	#BASE1: If cell reached is an obstacle return true.
	if(cell.isObstacle):
		return True
	#BASE2: If distance from original cell is too long return false.
	elif(distFromStart>3):
		return False
	#Check all adjacent cells if no base case is met.
	else: #TODO Math to get cell locations of top and bottom cells.
		return isCloseToObstacle(cells[startingLoc-1],startingLoc-1,distFromStart+1) and isCloseToObstacle(cells[startingLoc+1],startingLoc+1,distFromStart+1) and isCloseToObstacle() and isCloseToObstacle()



def waypoint_callback():
	global pointpub
	global path
	global index
	if(index<length)
		pointpub.publish(path[index])
	index = index + 1

def run():
	global pointpub
	rospy.init_node('move_robot', anonymous=True)
	path_sub = rospy.Subscriber('totes_path', Path, path_callback, queue_size=1) #change topic for best results
	pointpub = rospy.Publisher("way_point", PoseStamped, queue_size=100)
    # spin() simply keeps python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':

	global mapGrid



	run()
    