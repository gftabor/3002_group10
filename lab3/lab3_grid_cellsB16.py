#!/usr/bin/env python

import rospy
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from kobuki_msgs.msg import BumperEvent
import tf
import numpy
import math 
import rospy, tf, numpy, math

def createWay(processed):
    currentX = goalX
    currentY = goalY
    global my_path
    my_path = Path()
    
    while (currentX not starX or currentY not startY):
        pose.pose.position.x = (currentX*resolution)+offsetX + (1.5 * resolution)
        pose.pose.position.y = (currenty*resolution)+offsetY - (.5 * resolution)
        pose.pose.orientation.w = 1
        
        my_path.poses.append(pose)
        
        for i in range (0,len(processed)):
            if (processed[i].nodeX == currentX and processed[i].nodeY == currentY):
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
    mapgrid = data
    resolution = data.info.resolution
    mapData = data.data
    width = data.info.width
    height = data.info.height
    offsetX = data.info.origin.position.x
    offsetY = data.info.origin.position.y
    print data.info

def readGoal(goal):
    global goalX
    global goalY
    goalX= goal.pose.position.x
    goalY= goal.pose.position.y
    print goal.pose
    # Start Astar


def readStart(startPos):

    global startPosX
    global startPosY
    startPosX = startPos.pose.pose.position.x
    startPosY = startPos.pose.pose.position.y
    print startPos.pose.pose

def aStar(start,goal):
    def Astar():
    fringe = [];
    processed = [];

    n1 = node(startX,startY,startX,startY,0)
    
    heapq.heappush(fringe,(0, n1))

  while(1):
    
    if len(fringe) == 0:
      print "no path"
      break
    processingNode = heapq.heappop(fringe)


    if(processingNode.nodeX == goalX && processingNode.nodeY == goalY): #if processing goal or fringe empty
      print "cost is %d" % processingNode.realCost
      break
    
    print "processing %d,%d to find goal %d,%d" % (processingNode.nodeX, processingNode.nodeY, goalX, goalY)

    for i in range(0,4):
      x = processingNode.nodeX;
      y = processingNode.nodeY;
      if(i==0):
          x++;
      elif(i==1):
          y++;
      elif(i==1):
          x--;
      elif(i==1):
          y--;
      if(x <1 || y <1 || x>width || y >height):
        continue;
      if(grid.data[(x+1 + (y-1)*width)]==100): // if occupied
        continue;

      wasProcessed=false;
      for(j=0; j<processed.size();j++):
        if(processed[j].nodeX==x&&processed[j].nodeY==y){
          wasProcessed=true;
          break;
        }
      }
      if(wasProcessed)
        continue;
      int cost = dist(x,y,goalX,goalY) + processingNode.realCost +1;
      node newNode(x,y,cost,processingNode.nodeX,processingNode.nodeY,processingNode.realCost +1);
      fringe.push_back(newNode); std::push_heap (fringe.begin(),fringe.end(),Comp());
    }
    processed.push_back(processingNode);
  }
}

def publishCells(grid):
    global pub
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
                point.y=(i*resolution)+offsetY - (.5 * resolution) # added secondary offset ... Magic ?
                point.z=0
                cells.cells.append(point)
    pub.publish(cells)           

#Main handler of the project
def run():
    global pub
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
        publishCells(mapData) #publishing map data every 2 seconds
        rospy.sleep(2)  
        print("Complete")
    


if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass
