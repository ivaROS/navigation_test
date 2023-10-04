#!/usr/bin/env python
from __future__ import print_function
import rospy
import rospkg
import roslaunch

import test_driver
from gazebo_driver_v2 import Prototype
import std_msgs.msg
import sys
import time
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from gazebo_msgs.srv import GetModelState
import csv
import math




def gazebo_setup():
    print("Gazebo setup")
    gazebo_driver.shutdown()
    global returnedPosition
    x = [0, -1, -2]
    y = [3, 2, 1]
    gazebo_driver.moveBarrelsTest(3, x, y)

    # Unpause physics?


def gazebo_bridge():
    print("Gazebo Bridge")
    gazebo_driver.resetRobot()
    gazebo_driver.resetBarrels(3)


def gazebo_teardown():
    print("Gazebo teardown")
    gazebo_driver.delete_barrels(3)
    # Pause physics?




def main():

    gazebo_setup()


if __name__ == "__main__":
    gazebo_driver = Prototype()
    main()
