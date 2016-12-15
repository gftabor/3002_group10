#!/usr/bin/env python

import rospy, tf, numpy, math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String, Header
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():

    print "driving Straight"
    driveStraight(.1, .6) #speed = .1 m/s and distance is in meters
    publishTwist(0,0)
    rotate(90)
    publishTwist(0,0)
    driveStraight(.1, .45)
    publishTwist(0,0)
    rotate(-135)
    publishTwist(0,0) #stop moving

#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
    global pub

    lin_vel = ((u1+u2)/2)*.038; #linear velocity is the average of the wheel velocities times the wheel radius
    ang_vel = (u2-u1)/.25;    #angular velocity is difference of wheel velocity over wheel base
    #create twist messages
    twist_msg = Twist();
    stop_msg = Twist();

    #initialize twist messages
    twist_msg.linear.x = lin_vel
    twist_msg.angular.z = ang_vel
    stop_msg.linear.x = 0 #velocity should be zero when you don't wanna move
    stop_msg.angular.z = 0

    now = rospy.Time.now().secs
    while (rospy.Time.now().secs - now <= time and not rospy.is_shutdown()):
        pub.publish(twist_msg) #publish messages to move
    pub.publish(stop_msg)

#This function publishes a Twist message using angular and linear velocities
def publishTwist(linearVelocity, angularVelocity):
    global pub
    msg = Twist()
    msg.linear.x = linearVelocity
    msg.angular.z = angularVelocity
    pub.publish(msg)

def navToPose(goal):
    #use global x,y,and theta
    global header_pub
    print 'saw pos'
    global xPosition
    global yPosition
    global theta
    global move_pub
    #get desired positions from goal from rviz click
    desiredY = goal.pose.position.y
    desiredX = goal.pose.position.x

    quat = goal.pose.orientation
    q = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(q)
    desiredT = math.degrees(yaw)
    move_pub.publish(goal)
    distance =1
    startTime = rospy.Time.now().secs
    while(math.fabs(distance)>0.1 and rospy.Time.now().secs-startTime < 15):
        rospy.sleep(0.05)
        #move_pub.publish(goal)
        distance = math.sqrt((desiredX - xPosition)**2 + (desiredY - yPosition)**2)
        #print distance
    newHeader = Header()
    header_pub.publish(newHeader)
    print 'request new waypoint'
    rospy.sleep(.15)

#This function accepts a speed and a distance for the robot to move in straight line
def driveStraight(speed, distance):

    global pose
    global currentDistance
    initialX = pose.position.x
    initialY = pose.position.y

    atTarget = False
    #if the robot hasn't travelled the full distance and python is still going
    while (not atTarget and not rospy.is_shutdown()):
        currentX = pose.position.x
        currentY = pose.position.y
        currentDistance = math.sqrt(math.pow(currentX-initialX,2) + math.pow(currentY-initialY,2)) #use distance formula to calculate current distance from targets
        if(currentDistance >= distance): #if you have reached the target, stop moving
            atTarget = True
            publishTwist(0,0)
        else: #move at designated speed
            publishTwist(speed,0)
            rospy.sleep(0.15)
        
#Accepts an angle and makes the robot rotate to it in global fram
def faceAngle(angle):
    global odom_list
    global pose

    while(math.fabs(angle)>180):
        angle=angle - math.copysign(360, angle) #make angle in readable range

    error = angle - math.degrees(pose.orientation.z) #calculate error
     
    while(math.fabs(error) >=5 and not rospy.is_shutdown()): #if you haven't reached the goal and you're still trying to
        error = angle - math.degrees(pose.orientation.z) #recalculate error since you've probably moved
        while(math.fabs(error)>180): 
            error = error - math.copysign(360,error) #if you ended up turned 180, fix your error calculation 
        publishTwist(0,math.copysign(1.2,error)) #set angular velocity to degrees to specified angle    

#rotates a certain number of degrees, angle, regardless of global positioning
def rotate(angle):
    global odom_list
    global pose
    faceAngle(math.degrees(pose.orientation.z)-angle)
    
#move in an arc of a certain radius at a certain speed to a certain angle (arc measurment)
def driveArc(radius, speed, angleArc):
    w = speed / radius #calculate angular speed

    global odom_list
    global pose
    vel = Twist();  
 
    calcAngle = math.degrees(pose.orientation.z) + angleArc
    #put angle in a sensical range
    if (calcAngle > 360):
        calcAngle -= 360
    if (calcAngle > 180):
        calcAngle = (-360 + calcAngle)

    #calculates error, difference in desired angle and actual angle
    error = calcAngle-math.degrees(pose.orientation.z)

    #while you haven't reached the target
    while ((abs(error) >= 5) and not rospy.is_shutdown()):
        publishTwist(speed, w) #move with desired angular velocity w and given linear speed, speed
        rospy.sleep(0.15)    
        error = calcAngle-math.degrees(pose.orientation.z)   
    publishTwist(0,0)


#Bumper Event Callback function
def readBumper(msg):
    if (msg.state == 1):
        print "Bumper is pressed"
        #executeTrajectory() #execute trajectory function when the bumper is pressed

#timer callback, executes every .01s in rospy.Timer in main
def timerCallback(event):
   
    global pose
    global xPosition
    global yPosition
    global theta
    #obtain odometry data from frame 'odom' to frame 'base_footprint'
    odom_list.waitForTransform('map','base_footprint', rospy.Time(0), rospy.Duration(100.0))
    #store the information in position and orientation
    (position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0))
    pose.position.x=position[0]
    pose.position.y=position[1]
    #these are repetitive but it helps to have them stored globally (navToPose uses them)
    xPosition=position[0]
    yPosition=position[1]

    odomW=orientation
    q = [odomW[0], odomW[1], odomW[2], odomW[3]]
    roll, pitch, yaw = euler_from_quaternion(q) #get angles from the orientation odometry data

    pose.orientation.z = yaw
    theta = math.degrees(yaw)


# This is the program's main function
if __name__ == '__main__':
    rospy.init_node('aemrick_lab2')
    print "lab2 init"
    #global variables
    global pub
    global pose
    global odom_tf
    global odom_list
    global header_pub
    global move_pub
    pose = Pose()
    
    #Publishers and Subscribers
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, None, queue_size=10) # Publisher for commanding robot motion
    bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
    goal_sub = rospy.Subscriber('/way_point', PoseStamped, navToPose, queue_size=100)
    header_pub = rospy.Publisher('way_point_success', Header,queue_size=1)
    move_pub = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1)

    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    
    rospy.Timer(rospy.Duration(.01), timerCallback) #update odometry data captured in timerCallback every .01ms
    rospy.sleep(.15)

    print "Starting Lab 2"
    
    faceAngle(0)
    print '1'
    rospy.sleep(1.5)
    faceAngle(90)
    rospy.sleep(1.5)
    print '2'
    faceAngle(180)
    rospy.sleep(1.5)
    faceAngle(-90)
    rospy.sleep(1.5)
    print '3'
    faceAngle(0)
    faceAngle(90)
    faceAngle(180)
    print '4'
    faceAngle(-90)
    faceAngle(0)
    print '5'
    rospy.sleep(1.5)
    faceAngle(90)
    rospy.sleep(1.5)
    print '6'
    faceAngle(180)
    rospy.sleep(1.5)
    faceAngle(-90)
    rospy.sleep(1.5)
    print '7'
    faceAngle(0)

    newHeader = Header()
    header_pub.publish(newHeader)
    faceAngle(90)
    
    while (not rospy.is_shutdown()):
        rospy.spin()

    print "Lab 2 complete!"

