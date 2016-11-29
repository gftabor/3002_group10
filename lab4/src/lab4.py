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
	path = optimize_path(reversed(input_path.poses))
	#publish first goal in list
	pointpub.publish(path[0])
	index = 1
	print"new point"
def optimize_path(shitty_path):
	good_path = []
	good_path.append(shitty_path[0])
	for i, PoseStamped in enumerate(shitty_path):
	usefulPoint = 1
		if(i is not len(shitty_path) and i is not 0):
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

	#TODO put all cells in grid in a global array.
	cells = grid.data

	#This array is used to designate cells that will become obstacles.
	toBeObstacle = []

	#Runs through all cells in the grid.
	for i in cells:
		#If a cell is an obstacle the loop will make the four adjacent cells obstacles.
		if (cells[i] > 40):
			#Calculates edge case for the cell left of the given obstacle cell.
			if(i%grid.data.width == 0):
				left = i
			else:
				left = i-1;

			#Calculates edge case for the cell right of the given obstacle cell.
			if(i%grid.data.width == grid.data.width-1):
				right = i
			else:
				right = i+1

			#Calculates edge case for the cell top of the given obstacle cell.
			if(i-grid.data.width < 0):
				top = i
			else:
				top = i-width

			#Calculates edge case for the cell bottom of the given obstacle cell.
			if(i+grid.data.width > grid.data.width*grid.data.height-1):
				bottom = i
			else:
				bottom = i+grid.data.width

			cells[left] = cells[i]
			cells[right] = cells[i]
			cells[top] = cells[i]
			cells[bottom] = cells[i]


def waypoint_callback():
	global pointpub
	global path
	global index
	if(index<length):
		pointpub.publish(path[index])
	index = index + 1

def run():
	global pointpub
	rospy.init_node('move_robot', anonymous=True)
	path_sub = rospy.Subscriber('totes_path', Path, path_callback, queue_size=1) #change topic for best results
	pointpub = rospy.Publisher("way_point", PoseStamped, queue_size=100)
	mappub = rospy.Publisher("/map_real", OccupancyGrid, queue_size=1)

	mapsub = rospy.Subscriber('/map',OccupancyGrid,obstacleExpansion, queue_size=1)
    # spin() simply keeps python from exiting until this node is stopped
	rospy.spin()


if __name__ == '__main__':

	global mapGrid

	rospy.set_param('/move_base/global_costmap/inflation_layer/inflation_radius', '20')

#/move_base/global_costmap/inflation_layer/inflation_radius

	run()
    