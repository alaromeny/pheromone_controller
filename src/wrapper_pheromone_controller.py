#!/usr/bin/env python2.7
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#

import rospy

from std_msgs.msg import String, Int16MultiArray, MultiArrayDimension
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid

# from map import Map
from robot import Robot
from pheromone_controller import pheromoneController

import math
import timeit
import numpy as np
from numpy import linalg as LA

if __name__ == '__main__':
    message = "Pheromone WRAPPER!!!"
    rospy.logdebug(message)
    myController = pheromoneController(0.5,0.5)

    myController.listener()


