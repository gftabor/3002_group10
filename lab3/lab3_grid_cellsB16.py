#!/usr/bin/env python

from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
import rospy, tf, numpy, math, heapq

def dist(x1, y1, x2, y2):
    return math.sqrt((x2-x1)**2 + (y2-y1)**2)

def createWay(processed):
    currentX = goalX
    currentY = goalY
    global my_path
    global pub
    global startX
    global startY
    my_path = Path()
    pose = PoseStamped()
    
    while (currentX is not startPosX or currentY is not startPosY):
        pose.pose.position.x = (currentX*resolution)+offsetX + (1.5 * resolution)
        pose.pose.position.y = (currentY*resolution)+offsetY - (.5 * resolution)
        pose.pose.orientation.w = 1
        
        my_path.poses.append(pose)
        
        for i in range (0,len(processed)):
            if (processed[i].nodeX is currentX and processed[i].nodeY is currentY):
                currentX = processed[i].parentX
                currentY = processed[i].parentY
                break
                
                
    pub.publish(my_path)


# reads in global map
def mapCallBack(data):
    global mapData
    global width
    global height
    global mapgrid
    global resolution
    global offsetX
    global offsetY
    global seenMap
    seenMap = 1
    mapgrid = data
    resolution = data.info.resolution
    mapData = data.data
    width = data.info.width
    height = data.info.height
    offsetX = data.info.origin.position.x
    offsetY = data.info.origin.position.y
    #print data.info

def readGoal(goal):
    global seenGoal
    global goalX
    global goalY
    goalX = int((goal.pose.position.x - offsetX)/resolution -1.5)
    goalY = int((goal.pose.position.y - offsetY)/resolution +0.5)
    #print goal.pose
    seenGoal = 1

def readStart(startPos):
    global seenStart
    global startPosX
    global startPosY
    startPosX = int((startPos.pose.pose.position.x - offsetX)/resolution -1.5)

    startPosY = int((startPos.pose.pose.position.y - offsetY)/resolution +0.5)
    #print startPos.pose.pose
    seenStart = 1

class node:
    def __init__(self, x1, y1, x2, y2, costs):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.costs = costs

    def getX(self):
        return self.x1

    def getY(self):
        return self.y1
    
    def getCost(self):
        return self.costs

def astar():
    fringe = []
    processed = []

    n1 = node(startPosX,startPosY,startPosX,startPosY,0)
    
    heapq.heappush(fringe,(0, n1))

    while(1 and not rospy.is_shutdown()):
        if len(fringe) == 0:
            print "no path"
            break
        processingNode = heapq.heappop(fringe)[1]

        if(processingNode.getX() is goalX and processingNode.getY() is goalY): #if processing goal or fringe empty
            print "cost is %d" % processingNode.getCost()
            break
    
        wasProcessed=0
     
        for n in processed:
            if(int(n.getX()) is int(processingNode.getX()) and int(processingNode.getY()) is int(n.getY())):
                print "removed"
                wasProcessed=1
                break
        
        if(wasProcessed is 1):
            continue
        #print "processing %d,%d to find goal %d,%d" % (processingNode[0], processingNode[1], goalX, goalY)
        print "processing"
        for i in range(0,4):
            x = processingNode.getX()
            y = processingNode.getY()
            if(i==0):
                x += (1)
            elif(i==1):
                y += (1)
            elif(i==2):
                x -= (1)
            elif(i==3):
                y -= (1)
            if(x <1 or y <1 or x>width or y >height):
                continue
            if(mapData[(x+1 + (y-1)*width)]==100): #if occupied
                continue

           
            #print len(fringe)
            cost = dist(x,y,goalX,goalY) + processingNode.getCost() +1
            n2 = node(x,y,processingNode.getX(),processingNode.getY(),processingNode.getCost() + 1)
            heapq.heappush(fringe,(cost, n2))
        processed.append(processingNode)
        print "processed"
        print len(processed)
        publishCells(processed)
    createWay(processed)
        


def publishCells(nodes):
    global pub
    global pubway

    # resolution and offset of the map
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution 
    cells.cell_height = resolution

    
    for node in nodes: #width should be set to width of grid
        point=Point()
        point.x=(node.getX()*resolution)+offsetX + (1.5 * resolution) # added secondary offset 
        point.y=(node.getY()*resolution)+offsetY - (.5 * resolution)
        point.z=0
        cells.cells.append(point)
    pubway.publish(cells)           

#Main handler of the project
def run():
    global pub
    global pubway
    global mapData
    global resolution
    global offsetX
    global offsetY
    global width
    global height
    global my_path
    global seenGoal
    global goalX
    global goalY
    global seenStart
    global startPosX
    global startPosY
    global seenMap
    seenGoal = 0
    seenStart = 0
    seenMap = 0
    resolution =0
    offsetX = 0
    offsetY = 0
    width = 0
    height = 0 
    mapgrid = OccupancyGrid()
    #initiliaze variables so they mapcallback not being run does not crash program

    rospy.init_node('lab3')
    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)  
    pubpath = rospy.Publisher("/path", GridCells, queue_size=1) # you can use other types if desired
    pubway = rospy.Publisher("/waypoints", GridCells, queue_size=1)
    goal_sub = rospy.Subscriber('move_base_simple/goal', PoseStamped, readGoal, queue_size=1) #change topic for best results
    start_sub = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, readStart, queue_size=1) #change topic for best results

    # wait a second for publisher, subscribers, and TF
    rospy.sleep(1)

    while (1 and not rospy.is_shutdown()):
        #publishCells(mapData) #publishing map data every 2 seconds
        #rospy.sleep(2)  
        if(seenStart and seenGoal and seenMap):
            astar()
            print("ASTAR")
    


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
