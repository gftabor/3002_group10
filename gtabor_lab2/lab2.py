#!/usr/bin/env python

import rospy, tf, numpy, math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped

from tf.transformations import euler_from_quaternion
# Add additional imports for each of the message types used


#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):
    global pose
    # subscribe to rviz button click
    # change button properties to desired topic name
    # subscribe to topic using PoseStamped
    # calculate rotation needed
    # calculate and drive the distance
    # find difference between current and PoseStamped orientation and ROTATE
    print "got msg"
    goalX = goal.pose.position.x
    goalY = goal.pose.position.y
    odomW=goal.pose.orientation
    q = [odomW.x, odomW.y, odomW.z, odomW.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    goaltheta = math.degrees(yaw)

    initialX = pose.position.x
    initialY = pose.position.y
    yDist = initialY-goalY
    xDist = initialX-goalX

    faceAngle(math.degrees(math.atan2(yDist,xDist)))
    driveStraight(.2,math.sqrt(math.pow(xDist,2) + math.pow(yDist,2)))
    faceAngle(math.degrees(goaltheta))



#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
    driveStraight(.2,0.6)
    rotate(90)
    driveStraight(.2,0.45)
    rotate(-135)




#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
    global pub

    lin_vel = ((u1+u2)/2)*.038;
    ang_vel = (u2-u1)/.25;    

    twist_msg = Twist();
    stop_msg = Twist();

    twist_msg.linear.x = lin_vel
    twist_msg.angular.z = ang_vel
    stop_msg.linear.x = 0
    stop_msg.angular.z = 0

    now = rospy.Time.now().secs
    while (rospy.Time.now().secs - now <= time and not rospy.is_shutdown()):
        pub.publish(twist_msg)
    pub.publish(stop_msg)



#This function publishes a Twist message
def publishTwist(linearVelocity, angularVelocity):
    global pub
    msg = Twist()
    msg.linear.x = linearVelocity
    msg.angular.z = angularVelocity
    pub.publish(msg)

#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):

    global pose
    initialX = pose.position.x
    initialY = pose.position.y
    atTarget = False
    while (not atTarget and not rospy.is_shutdown()):
        currentX = pose.position.x
        currentY = pose.position.y
        currentDistance = math.sqrt(math.pow(currentX-initialX,2) + math.pow(currentY-initialY,2))
        if(currentDistance >= distance):
            atTarget = True
            publishTwist(0,0)
        else:
            publishTwist(speed,0)
            rospy.sleep(0.15)
        print currentDistance
        
#Accepts an angle and makes the robot rotate around it.
def faceAngle(angle):
    global odom_list
    global pose
    while(math.fabs(angle)>180):
        angle = angle - math.copysign(360,angle)
    #get angle into useful range

    error = angle-math.degrees(pose.orientation.z)
    while ((math.fabs(error) >= 2) and not rospy.is_shutdown()):
        error = angle-math.degrees(pose.orientation.z)
        while(math.fabs(error)>180):
            error = error - math.copysign(360,error)
            #get error into useful range every time step
        publishTwist(0,math.copysign(.8,error))
        print math.degrees(pose.orientation.z)
        
def rotate(angle):
    global odom_list
    global pose
    faceAngle(math.degrees(pose.orientation.z)-angle)
    #-angle is to get rotate to turn right by default

#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    pass  # Delete this 'pass' once implemented

#Bumper Event Callback function
def readBumper(msg):
    if (msg.state == 1 and msg.bumper==1):
        # What should happen when the bumper is pressed?
        print "Bumper is pressed"
        executeTrajectory()

def timerCallback(event):
   
    global pose
    global xPosition
    global yPosition
    global theta
    
    odom_list.waitForTransform('odom','base_footprint', rospy.Time(0), rospy.Duration(1.0))
    (position, orientation) = odom_list.lookupTransform('odom','base_footprint', rospy.Time(0)) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)
    pose.position.x=position[0]
    pose.position.y=position[1]

    odomW=orientation
    q = [odomW[0], odomW[1], odomW[2], odomW[3]]
    roll, pitch, yaw = euler_from_quaternion(q)

    pose.orientation.z = yaw
    theta = math.degrees(yaw)


# This is the program's main function
if __name__ == '__main__':
    # Change this node name to include your username
    rospy.init_node('gtabor_lab2')


    # These are global variables. Write "global <variable_name>" in any other function
    #  to gain access to these global variables
    global pose
    global pub
    global odom_tf
    global odom_list
    global sub
    pose = Pose()

    
    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist,queue_size=10) # Publisher for commanding robot motion
    bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
    goal_sub = rospy.Subscriber('/move_base_simple/goal',PoseStamped, navToPose, queue_size=1) # Callback function to handle bumper events

    odom_list = tf.TransformListener()
    
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))

    print "Starting Lab 2"

    #make the robot keep doing something...
   
    rospy.Timer(rospy.Duration(.01), timerCallback)

    
    rospy.spin()        

    print "Lab 2 complete!"