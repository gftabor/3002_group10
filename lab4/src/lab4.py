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
	run()
    