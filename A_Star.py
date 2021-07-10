#!/usr/bin/env python
import rospy
from grid_map_msgs.msg import GridMap
import math
import numpy as np
import matplotlib.pyplot as plt
import a_star 


def search(height, gridData):

    occupied=[]
    for i in range (len(gridData)):
        if not math.isnan(gridData[i]):
            if (abs(gridData[i]-height)<=1e-2):
                occupied.append(i)
    return occupied      


def heuristic(position, goal_position):
  h=abs(position[0]-goal_position[0])+abs(position[1]-goal_position[1])
  return h  

def createOccupancyGrid(occupiedCellPositions,r):
    XCells=int(r.info.length_x/r.info.resolution) #number of cells along the x axis i.e. the number of columns
    YCells=int(r.info.length_y/r.info.resolution) #number of cells along the y axis i.e. the number of rows
    OccupancyGrid=np.zeros((YCells,XCells)) #initialising a numpy array containing zeros of the size same as that of the grid map

#if pos is the position of a cell in the grid map then let the corresponding indices of the cell be (x,y)-> y=pos % XCells and x= pos // XCells
#since positioning of the cells happens along the x axis.
   
    for pos in occupiedCellPositions:
        y= pos % XCells
        x= pos // XCells
        OccupancyGrid[x][y]=1 # note that x is the row and y is the column
    return OccupancyGrid

def callback(r):
    #r.data[0].data is the array to pass to the search function.  
    height=float(input("Enter the Height:"))
    occupiedCellPositions=search(height,r.data[0].data)
    OccupancyGrid=createOccupancyGrid(occupiedCellPositions,r)
    print("RViZ Axes convention.. x axis: ->, y axis: ^")
    # plt.imshow(OccupancyGrid,cmap='Greys')
    

    plt.imshow(OccupancyGrid, cmap='Greys', )
  

    # #coords (vertical axis, horizontal axis)
    start=(1,3)
    goal=(13,15)
    path, cost, p = a_star.a_star(OccupancyGrid, heuristic, start, goal)
    print(p)
    y,x=zip(*p)
    plt.plot(x,y)
    #plt.grid()
    plt.show()



def subToGridMap():
    rospy.init_node("OccupancyGridFromGridMap", anonymous=True)
    rospy.Subscriber( '/grid_map_pcl_loader_node/grid_map_from_raw_pointcloud', GridMap, callback)
    rospy.spin()




if __name__=='__main__':
    subToGridMap()   





