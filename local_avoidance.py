#!/usr/bin/env python
import rospy
from sensor_msgs.msg import *
import numpy as np
import ros_numpy
import math
import time
from utils.offboard import uav
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from tf.transformations import euler_from_quaternion
k = 5
NEXT_WAYPOINT = (15, 0)
class detector:
    def __init__(self):
        self.CAMERA_SPAN = 180.0 #span of depth camera in degrees
        self.NUM_SECTORS = 16 #number of sectors 
        self.MAX_DEPTH = 18.
        self.ALPHA = 90/self.NUM_SECTORS
        self.altitude = 1.5
        self.obstacleDensity = []
        self.tomove = Point()
        self.vel = Point()
        self.angular = Point() #new
        self.q = Quaternion() #new
        self.globalvel = Point()
        self.loc = Point()
        self.x_hat = 0
        self.y_hat = 0
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.loc_pose)
        rospy.Subscriber("/camera_front/depth/image_raw", Image, self.frontCallback)
        rospy.Subscriber("/camera_right/depth/image_raw", Image, self.rightCallback)
        rospy.Subscriber("/camera_left/depth/image_raw", Image, self.leftCallback)

    def loc_pose(self, data):
        self.loc.x = data.pose.position.x
        self.loc.y = data.pose.position.y
        self.loc.z = data.pose.position.z
        self.q.x = data.pose.orientation.x
        self.q.y = data.pose.orientation.y
        self.q.z = data.pose.orientation.z
        self.q.w = data.pose.orientation.w
    
    def give_yaw(self): #returns yaw in local frame 
        quatern = (self.q.x , self.q.y, self.q.z,self.q.w)
        euler = euler_from_quaternion(quatern)
        yaw = euler[2]
        return yaw

    def allZeros(self, arr):
        ref = 0
        same = True
        for i in range (0,len(arr)):
            if arr[i] != ref:
                same = False
                break
        return same
    
    def allOnes(self,arr):
        ref = 1
        same = True
        for i in range (0,len(arr)):
            if arr[i] != ref:
                same = False
                break
        return same
    
    def goodToGo(self,arr):
        gTGo = False
        length = len(arr)
        centre = length//2
        if self.allZeros(arr[centre-(length//4):centre+(length//4)]):
            gTGo = True
        return gTGo

    def frontCallback(self,data):
        self.frontImage = ros_numpy.numpify(data)
        self.width = data.width
        self.Fimage = []        
        sectorNumber = 0
        while sectorNumber < self.NUM_SECTORS:
            self.obstacleDepth = 0
            #for i in range (21,25):
            for j in range((self.width/self.NUM_SECTORS)*sectorNumber,(self.width/self.NUM_SECTORS)*(sectorNumber+1)): #ensure that widthImage is a factor of NUM_SECTORS
                self.obstacleDepth += self.frontImage[24][j]
            # print ("Obstacle Depth for the sector number ",sectorNumber, " is :", self.obstacleDepth)
            self.Fimage.append(self.obstacleDepth/100)
            sectorNumber += 1

        for i in range(len(self.Fimage)):
            if (self.Fimage[i]  <= 0.2) : # can adjust this to include the distance of avoidance # 4.5 to avoid obstacle at a distance of ~5 m, found manually. 
                self.Fimage[i] = 1
            else :
                self.Fimage[i] = 0
        # print ("Front Image:", self.Fimage)

    def rightCallback(self,data):
        self.rightImage = ros_numpy.numpify(data)
        self.width = data.width
        self.Rimage = []        
        sectorNumber = 0
        while sectorNumber < self.NUM_SECTORS:
            self.obstacleDepth = 0
            #for i in range (21,25):
            for j in range((self.width/self.NUM_SECTORS)*sectorNumber,(self.width/self.NUM_SECTORS)*(sectorNumber+1)): #ensure that widthImage is a factor of NUM_SECTORS
                self.obstacleDepth += self.rightImage[24][j]
            # print ("Obstacle Depth for the sector number ",sectorNumber, " is :", self.obstacleDepth)
            self.Rimage.append(self.obstacleDepth/100)
            sectorNumber += 1

        for i in range(len(self.Rimage)):
            if (self.Rimage[i]  <= 0.2) : # can adjust this to include the distance of avoidance # 4.5 to avoid obstacle at a distance of ~5 m, found manually. 
                self.Rimage[i] = 1
            else :
                self.Rimage[i] = 0
        self.Rimage = self.Rimage[0:8]
        # print ("Right Image:", self.Rimage)

    def leftCallback(self,data):
        self.leftImage = ros_numpy.numpify(data)
        self.width = data.width
        self.Limage = []        
        sectorNumber = 0
        while sectorNumber < self.NUM_SECTORS:
            self.obstacleDepth = 0
            #for i in range (21,25):
            for j in range((self.width/self.NUM_SECTORS)*sectorNumber,(self.width/self.NUM_SECTORS)*(sectorNumber+1)): #ensure that widthImage is a factor of NUM_SECTORS
                self.obstacleDepth += self.leftImage[24][j]
            # print ("Obstacle Depth for the sector number ",sectorNumber, " is :", self.obstacleDepth)
            self.Limage.append(self.obstacleDepth/100)
            sectorNumber += 1

        for i in range(len(self.Limage)):
            if (self.Limage[i]  <= 0.2) : # can adjust this to include the distance of avoidance # 4.5 to avoid obstacle at a distance of ~5 m, found manually. 
                self.Limage[i] = 1
            else :
                self.Limage[i] = 0
            
        self.Limage = self.Limage[-8:] #can be generalized for any span, this is to make three 90 degree cameras obtain a 180 degree span. 
        # print ("Left Image:", self.Limage)
        
    def clear(self,a):
        clear  = False
        ref = 0
        l = len(a)
        for i in range(l):
            streak = 0
            if a[i]==ref:
                for j in range(i,l):
                    if a[j] == a[i]:
                        streak += 1
                    else:
                        i = j
                        break
                    if streak == 6:
                        clear = True
        return clear 

    # def crowded(self,array):
    #     crowded = False





                

    def fusion(self):
        self.total_array = self.Limage + self.Fimage + self.Rimage #cameras are at 90 degrees hence no overlap
        print ("Total Vision:", self.total_array)

        if self.clear(self.total_array):
            self.feasible = 0
            self.maxNumberOfContinuousFreeCells = []
            self.indexOfFirstFreeCell = []
            i=0
            i_prev = 1000
            while i <= (len(self.total_array)-1):
                if self.total_array[i] == self.feasible:
                    counter = 0
                    if i_prev != i:
                        self.indexOfFirstFreeCell.append(i)
                        j = i
                        while j <= (len(self.total_array)-1):
                            if self.total_array[j] == self.feasible:
                                counter += 1
                                j += 1
                                i = j
                            else:
                                i=j
                                break
                        self.maxNumberOfContinuousFreeCells.append(counter)
                        i_prev = i
                else:
                    i += 1
                    continue

            self.choose = max(self.maxNumberOfContinuousFreeCells)
            index = self.maxNumberOfContinuousFreeCells.index(self.choose)
            newIndex = self.indexOfFirstFreeCell[index]
            # print ("Max no of continuous free cells:",self.maxNumberOfContinuousFreeCells)
            # print ("Starting Index:",self.indexOfFirstFreeCell)
            yaw = self.give_yaw()
            self.theta = (180 - self.CAMERA_SPAN)/2 + (newIndex + float(self.choose)/2)*self.ALPHA - (yaw*180/math.pi)
            # print ("Index corresponding to max continuous self.feasible cells:",newIndex)
            # print ("Max number of continuos self.feasible cells",self.choose)
            print ("Theta :",self.theta)
            self.x_hat = math.sin(math.radians(self.theta))
            self.y_hat = math.cos(math.radians(self.theta))
            distanceToMove = abs(2.0/(math.cos(90-self.theta)))
            self.tomove.x = distanceToMove*self.x_hat
            self.tomove.y = distanceToMove*self.y_hat
            self.tomove.z = self.altitude
            veltomove = 0.5
            yaw = self.give_yaw()
            # ref = math.atan2((NEXT_WAYPOINT[1]-self.loc.y),(NEXT_WAYPOINT[0]-self.loc.x))
            self.vel.x = veltomove*self.x_hat
            self.vel.y = 3*veltomove*self.y_hat
            self.vel.z = 0
            self.angular.x = 0
            self.angular.y = 0
            self.angular.z = 0
            #print ("Yaw:",yaw)
            # if abs(yaw-ref) <= 0.0001:
            #     self.vel.x = 1*veltomove*x_hat
            #     self.vel.y = 2*veltomove*y_hat
            #     self.vel.z = 0
            # else :
            #     x_hat = math.sin(yaw+8)
            #     y_hat = math.cos(yaw+8)
            #     self.vel.x = veltomove*x_hat
            #     self.vel.y = 2*veltomove*y_hat
            #     self.vel.z = 0                

        else:
            self.vel.x = 0
            self.vel.y = 0
            self.vel.z = 0
            self.angular.x = 0
            self.angular.y = 0
            self.angular.z = 0.3
            yaw = self.give_yaw()
            #print ("Yaw:", yaw)
            # self.theta = (180 - self.CAMERA_SPAN)/2 + (newIndex + float(self.choose)/2)*self.ALPHA - yaw 



    # def callback(self, data):
    #     self.frontImage = ros_numpy.numpify(data)
    #     self.widhtImage = data.width

    # def computewp(self):
    #     self.fusion()
    #     # self.obstacledeptharr = []
    #     # sectorNumber = 0
        
    #     # while sectorNumber < self.NUM_SECTORS:
    #     #     self.obstacleDepth = 0
    #     #     #for i in range (21,25):
    #     #     for j in range((self.widhtImage/self.NUM_SECTORS)*sectorNumber,(self.widhtImage/self.NUM_SECTORS)*(sectorNumber+1)): #ensure that widthImage is a factor of NUM_SECTORS
    #     #         self.obstacleDepth += self.frontImage[24][j]
    #     #     # print ("Obstacle Depth for the sector number ",sectorNumber, " is :", self.obstacleDepth)
    #     #     self.obstacledeptharr.append(self.obstacleDepth/100)
    #     #     sectorNumber += 1
    #     # print ("Obstacle Depth:",self.obstacledeptharr)
    #     # for i in range(len(self.obstacledeptharr)):
    #     #     if (self.obstacledeptharr[i]  <= 0.2) : # can adjust this to include the distance of avoidance # 4.5 to avoid obstacle at a distance of ~5 m, found manually. 
    #     #         self.obstacledeptharr[i] = 1
    #     #     else :
    #     #         self.obstacledeptharr[i] = 0
    #     # print ("Sector Feasibility:",self.obstacledeptharr)
    #     print ("Sector Feasibility:", self.total_array)


if __name__ == '__main__':
    rospy.init_node("GP", anonymous = True )
    altitude = 1.5
    mvc = uav()
    det = detector()
    mvc.setarm(1)
    time.sleep(2)
    mvc.offboard()
    # mvc.gotoposeonce(0,0,altitude)
    # mvc.gotopose(NEXT_WAYPOINT[0],NEXT_WAYPOINT[1],altitude)
    det.fusion()
    while True:
        det.fusion()
        absolutedist = np.sqrt((NEXT_WAYPOINT[0]-det.loc.x)**2 + (NEXT_WAYPOINT[1]-det.loc.y)**2 )
        yaw = det.give_yaw()
        if not det.goodToGo(det.total_array) :
            det.fusion()
            mvc.getvelBody(det.vel.x, det.vel.y, det.vel.z,det.angular.x,det.angular.y, det.angular.z)
            # if det.freeSpaceVisible == False:
            #     mvc.getvelBody(2,0,0,0,0,0) # if u have yawed then move forward a bit before moving in the direction of the goal
            yaw = det.give_yaw()
        else:
            yaw = det.give_yaw()
            delta = math.atan2((NEXT_WAYPOINT[1]-det.loc.y),(NEXT_WAYPOINT[0]-det.loc.x)) - yaw
            print ("Error in theta_Z:", delta)
            # print ("Current yaw:", yaw)
            # print ("Theta to goal: ",delta)
            error = k*delta
            while abs(error) >= 0.1:
                error = k*delta
                print ("To yaw:", delta)
                mvc.getvelBody(0,0,0,0,0,error)
                yaw = det.give_yaw()                
                print ("Yaw after one iteration:", yaw)
                delta = math.atan2((NEXT_WAYPOINT[1]-det.loc.y),(NEXT_WAYPOINT[0]-det.loc.x)) - yaw
            if not det.goodToGo(det.total_array):
              continue
            else:
                det.fusion()   
                mvc.getvelBody((NEXT_WAYPOINT[0]-det.loc.x)/absolutedist,(NEXT_WAYPOINT[1]-det.loc.y)/absolutedist,0,0,0,0)
            # mvc.getvelLocal((NEXT_WAYPOINT[0]-det.loc.x)/absolutedist,(NEXT_WAYPOINT[1]-det.loc.y)/absolutedist,0, delta)

