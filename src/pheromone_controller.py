#!/usr/bin/env python2.7


import rospy

from std_msgs.msg import String, Int16MultiArray, MultiArrayDimension, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, PointCloud
from nav_msgs.msg import Odometry, OccupancyGrid

# from map import Map
from robot import Robot

import random
import math
import timeit
import time
import numpy as np
from numpy import linalg as LA
# import transformations as tf

class pheromoneController:
    def __init__(self, robotWidth = 1, robotHeight = 1):


        self.robotWidth = robotWidth
        self.robotHeight = robotHeight
        self.localStigmergyMap = np.full(1,1,dtype=np.uint16)
        self.mapRecieved = False
        self.xWeight = 0
        self.yWeight = 0
        self.theta = 0
        self.robotNameSpace = rospy.get_namespace()        

        # how often to publish the local maps of each robot (Hz)
        np.set_printoptions(threshold=np.nan)

    def setUp( self):

        print "My name space is: " + self.robotNameSpace

        self.robot = Robot(self.robotWidth, self.robotHeight, self.robotNameSpace)
        self.publisher_rate = rospy.get_param('~publisher_rate', 10)

        self.pubTopicName_pherHeading = str(self.robotNameSpace) + "pheromoneHeading"
        self.pub = rospy.Publisher(self.pubTopicName_pherHeading, Float32, queue_size = 1)
        print "Publishing to the topic: " + self.pubTopicName_pherHeading

        self.listenerTopicName_pheromones = '/ground/localPheromone' + str(self.robotNameSpace)
        rospy.Subscriber(self.listenerTopicName_pheromones, Int16MultiArray, self.callBackStigmergy)
        print "Subscribed to the topic: " + self.listenerTopicName_pheromones



    #This calculates the norm between the robot and a new point x,y
    def norm( self, x_pos, y_pos):
      return math.sqrt( x_pos*x_pos + y_pos*y_pos)

    #This calculates the rotation needed to move to a new point x,y from the current frame of reference
    def calcRotation( self, x_pos, y_pos):
      return math.atan2(y_pos, x_pos);

    def grabCentre( self, localStigmergyMap):

        size = localStigmergyMap.shape
        res = np.zeros((2,size[0]))
        midpoint = (size[0]/2, size[1]/2)
        for y in range(0,size[0]):
            for x in range(0,size[1]):
                if localStigmergyMap[y,x] == 0:
                    res[1,y] = res[1,y] + 1
                    res[0,x] = res[0,x] + 1
        xWeight = 0
        yWeight = 0
        for i in range(0,size[0]):
            xWeight = xWeight + res[0,i]* (i-midpoint[0])
            yWeight = (yWeight + res[1,i]* (i-midpoint[1]))
        yWeight = yWeight * -1
        return (xWeight, yWeight)


    def formatMessage( self, data):
        arraySize = data.layout.dim[0].size
        arrayDim = data.layout.dim[0].stride
        arrayHeight = int(arraySize/arrayDim)
        arrayData = data.data
        arrayData = np.asarray(arrayData)
        arrayData = np.reshape(arrayData, (arrayDim,arrayHeight))
        return arrayData

    def rotateCoordinate(self, px, py, angle):
        ##Rotate a point counterclockwise by a given angle around a given origin.
        ##The angle should be given in radians.
        qx =  math.cos(angle) * (px) - math.sin(angle) * (py)
        qy =  math.sin(angle) * (px) + math.cos(angle) * (py)
        return qx, qy

    def calculateNewHeading( self):
        self.theta = self.calcRotation( self.xWeight, self.yWeight)
        self.robot.originalTheta = self.robot.eularAngles[2]
        self.robot.goalTheta = self.theta  - self.robot.originalTheta
        x = math.cos(self.robot.goalTheta)
        y = math.sin(self.robot.goalTheta)
        turn = self.calcRotation(x , y)
        return turn

    def checkHeading( self):
        turn = self.calculateNewHeading()
        return (abs(turn) > self.turn_threshold)


    def calcThetaDegree( self, radians):
        res = math.degrees(radians)
        res = res - 90
        return res


    def normaliseTheta( self, theta):
        if theta < 0:
                theta = 360 + theta
        return theta


    def callBackStigmergy( self, data):
        self.localStigmergyMap = self.formatMessage( data)
        (self.xWeight, self.yWeight) = self.grabCentre( self.localStigmergyMap)
        self.theta = self.calcRotation( self.xWeight, self.yWeight)
        print self.theta
        self.mapRecieved = True


    def publisher(self):
        self.pub.publish(self.theta)

    def listener(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('pheromoneController', anonymous=True)
        self.setUp()
        while not rospy.is_shutdown():

            self.publisher()
            r = rospy.Rate(self.publisher_rate) # 10hz
            r.sleep()
