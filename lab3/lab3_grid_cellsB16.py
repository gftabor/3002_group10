#!/usr/bin/env python

from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
import rospy, tf, numpy, math

def createWay(processed):
    currentX = goalX
    currentY = goalY
    global my_path
    global pub
    my_path = Path()
    
    while (currentX is not starX or currentY is not startY):
        pose.pose.position.x = (currentX*resolution)+offsetX + (1.5 * resolution)
        pose.pose.position.y = (currenty*resolution)+offsetY - (.5 * resolution)
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
    print data.info

def readGoal(goal):
    global seenGoal
    global goalX
    global goalY
    goalX= goal.pose.position.x
    goalY= goal.pose.position.y
    print goal.pose
    seenGoal = 1

def readStart(startPos):
    global seenStart
    global startPosX
    global startPosY
    startPosX = startPos.pose.pose.position.x
    startPosY = startPos.pose.pose.position.y
    print startPos.pose.pose
    seenStart = 1

class node(object):
    def __init__(self, x1, y1, x2, y2, costs):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.costs = costs

def aStar(start,goal):
    fringe = []
    processed = []

    n1 = node(startX,startY,startX,startY,0)
    
    heapq.heappush(fringe,(0, n1))

    while(1):
    
        if len(fringe) == 0:
            print "no path"
            break
        processingNode = heapq.heappop(fringe)

        if(processingNode.nodeX is goalX and processingNode.nodeY is goalY): #if processing goal or fringe empty
            print "cost is %d" % processingNode.realCost
            break
    
        print "processing %d,%d to find goal %d,%d" % (processingNode.nodeX, processingNode.nodeY, goalX, goalY)

        for i in range(0,4):
            x = processingNode.nodeX
            y = processingNode.nodeY
            if(i==0):
                x = x+1
            elif(i==1):
                y = y+1
            elif(i==2):
                x = x-1
            elif(i==3):
                y = y-1
        if(x <1 or y <1 or x>width or y >height):
            continue
        if(mapData.data[(x+1 + (y-1)*width)]==100): #if occupied
            continue

        wasProcessed=false
     
        for j in range(0,len(processed)):
            if(processed[j].nodeX == x and processed[j].nodeY == y):
                wasProcessed=true
            break
        
        if(wasProcessed):
            continue

        cost = dist(x,y,goalX,goalY) + processingNode.realCost +1
        n2 = node(x,y,processingNode.nodeX,processingNode.nodeY,processingNode.realCost + 1)
      
        heapq.heappush(fringe,(cost, n2))
        processed.append(processingNode)
        publishCells(processed)
    createWay(processed)
        


def publishCells(grid):
    global pub
    global pubway
    print "publishing"

    # resolution and offset of the map
    k=0
    cells = GridCells()
    cells.header.frame_id = 'map'
    cells.cell_width = resolution 
    cells.cell_height = resolution

    for i in range(1,height): #height should be set to hieght of grid
        k=k+1
        for j in range(1,width): #width should be set to width of grid
            k=k+1
            #print k # used for debugging
            if (grid[k] == 100):
                point=Point()
                point.x=(j*resolution)+offsetX + (1.5 * resolution) # added secondary offset 
                point.y=(i*resolution)+offsetY - (.5 * resolution)
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
    resolution =0
    offsetX = 0
    offsetY = 0
    width = 0
    height = 0 
    mapData = OccupancyGrid()
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
        if(seenStart and seenGoal and seenMap)
            #call Astar
        print("Complete")
    


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
