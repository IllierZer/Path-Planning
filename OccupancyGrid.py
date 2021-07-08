#!/usr/bin/env python
import rospy
from grid_map_msgs.msg import GridMap
import math
import numpy as np
import matplotlib.pyplot as plt

DRONE_OFFSET=0.1 # drone has a well defined geometry and hence its dimension along the z-axis cannot be ignored.

def search(height, gridData):
    drone_offset=DRONE_OFFSET
    occupied=[]
    for i in range (len(gridData)):
        if not math.isnan(gridData[i]):
            if (abs(gridData[i]+drone_offset>=height)): #if the drone is below the obstacle height (inclusive of drone offset) then the obstacle will be included in the occupancy grid.
                occupied.append(i)
    return occupied


def callback(r):
    #r.data[0].data is the array to pass to the search function.  
   
    XCells=int(r.info.length_x/r.info.resolution) #number of cells along the x axis i.e. the number of columns
    YCells=int(r.info.length_y/r.info.resolution) #number of cells along the y axis i.e. the number of rows
    height=float(input("Enter the Height:"))
    occupiedCellPositions=search(height,r.data[0].data)
    OccupancyGrid=np.zeros((YCells,XCells)) #initialising a numpy array containing zeros of the size same as that of the grid map

#if pos is the position of a cell in the grid map then let the corresponding indices of the cell be (x,y)-> y=pos % XCells and x= pos // XCells
#since positioning of the cells happens along the x axis.
   
    for pos in occupiedCellPositions:
        y = pos % XCells # since y represents the column, it should lie between [0,XCells), hence pos % XCells
        x = pos // XCells # since x represents the row, and positioning is along XCells, x = pos // XCells
        OccupancyGrid[x][y]=1 # note that x is the row and y is the column
    print("RViZ Axes convention.. x axis: ->, y axis: ^")
    
    # visualise the Occupancy Grid. 
    plt.imshow(OccupancyGrid,cmap='Greys')
    plt.show()


def subToGridMap():
    rospy.init_node("OccupancyGridFromGridMap", anonymous=True) #initialise the subscriber node
    rospy.Subscriber( '/grid_map_pcl_loader_node/grid_map_from_raw_pointcloud', GridMap, callback) #subscribe to the grid map pcl topic 
    rospy.spin()

if __name__=='__main__':
    subToGridMap()   


# this is a subscriber node which subscribes to the grid map being published and then converts the grid map data into an occupancy grid
# make sure to publish a grid map while running this node.