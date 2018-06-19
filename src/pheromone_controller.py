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
        self.listenerTopicName_command = 'cmd_vel'
        self.listenerTopicName_robotOdom = 'odom'

        self.N_DISTANCES = 6  # do not modify
        self.MAX_LASER_DIST = 20
        self.STEER_THRESHOLD = 0.8
        self.map_initiated = False
        # distances measured by lasers:
        self.distance_obstacle = np.full(self.N_DISTANCES, -1, dtype=np.float64)
        self.walls_bool = np.zeros(self.N_DISTANCES-1)
        self.obstacleAhead = False
        self.localStigmergyMap = np.full(1,1,dtype=np.uint16)
        self.mapRecieved = False
        self.victimFound = False
        self.pub = rospy.Publisher(self.listenerTopicName_command, Twist, queue_size = 1)
        self.xWeight = 0
        self.yWeight = 0
        self.avoidValue = 0.0
        self.turn_threshold = 0.5
        self.robotNameSpace = rospy.get_namespace()        

        # how often to publish the local maps of each robot (Hz)
        np.set_printoptions(threshold=np.nan)

    def setUp( self):

        print "My name space is: " + self.robotNameSpace

        self.robot = Robot(self.robotWidth, self.robotHeight, self.robotNameSpace)
        self.MIN_LASER_DIST = rospy.get_param('~distance_to_walls', 1)
        self.publisher_rate = rospy.get_param('~update_rate', 10)
        self.listenerTopicName_pheromones = '/ground/localPheromone' + str(self.robotNameSpace)


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
        arrayData = data.data
        arrayData = np.asarray(arrayData)
        arrayData = np.reshape(arrayData, (arrayDim,arrayDim))
        return arrayData


    def rotateCoordinate(self, px, py, angle):
        ##Rotate a point counterclockwise by a given angle around a given origin.
        ##The angle should be given in radians.
        qx =  math.cos(angle) * (px) - math.sin(angle) * (py)
        qy =  math.sin(angle) * (px) + math.cos(angle) * (py)
        return qx, qy


    def callBackStigmergy( self, data):
        self.localStigmergyMap = self.formatMessage( data)
        (self.xWeight, self.yWeight) = self.grabCentre( self.localStigmergyMap)
        self.theta = self.calcRotation( self.xWeight, self.yWeight)
        self.mapRecieved = True
        # print "ROBOT " + str(self.robot.ID) + " has recieved the stigmergy map"
        # print self.localStigmergyMap


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

    def callBackOdom( self, data):
        self.robot.x = data.pose.pose.position.x
        self.robot.y = data.pose.pose.position.y
        self.robot.quaternion = data.pose.pose.orientation
        self.robot.quaternion = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        ax, ay, az = tf.euler_from_quaternion_rounded(0.005, self.robot.quaternion)
        self.robot.eularAngles = [ax, ay, az]

    def normaliseTheta( self, theta):
        if theta < 0:
                theta = 360 + theta
        return theta

    def publisher(self):
        self.pub.publish(self.robot.twist)

    def listener(self):

        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('robotController', anonymous=True)
        self.setUp()
        rospy.Subscriber(self.listenerTopicName_pheromones, Int16MultiArray, self.callBackStigmergy)
        rospy.Subscriber(self.listenerTopicName_robotOdom, Odometry, self.callBackOdom)

        while not rospy.is_shutdown():

            self.publisher()
            r = rospy.Rate(self.publisher_rate) # 10hz
            r.sleep()
