#!/usr/bin/env python
from __future__ import print_function
from builtins import str
from builtins import range
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
from geometry_msgs.msg import Transform
import csv
import math
import random

## CHANGE THIS
num_test = 5

launch = None
launch_static = None

controller_name =[]
#x, y, z, roll pitch yaw
section_0 = [8, 8, 0, 0 ,0, -3.14]
section_1 = [8, 1, 0, 0, 0, -3.14]
section_2 = [8, -7.5, 0, 0, 0, 3.14]
section_3 = [0, 7, 0, 0, 0, -1.59]
section_4 = [0, 0, 0, 0, 0, 3.14]
section_5 = [0, -7.5, 0, 0, 0, 3.14]
section_6 = [-5, 6, 0, 0, 0, -1.57]
section_7 = [-5, 1, 0, 0, 0, 1.57]
section_8 = [-5, -7, 0, 0, 0, 1.57]
total_section = [section_0, section_1, section_2, section_3, section_4, section_5, section_6, section_7, section_8]
testNum = 1
testResult = [0, 0]
testTime = [0 ,0]
startSection = [0]
endSection = [0]
controller_name = []
numController = 2
    # Unpause physics?
def move_robot(pose):
    gazebo_driver.resetRobotImpl(pose)

def launch_controller(controller_name):
    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    path = None
    if (controller_name == "Lpips_togo.launch"):
        path = rospack.get_path("pips_dwa_implementation")
    else:
	path = rospack.get_path("pips_test")
    # We'll assume Gazebo is launched are ready to go
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    print(path)
    global launch
    launch = roslaunch.parent.ROSLaunchParent(
        uuid, [path + "/launch/" + controller_name])
    launch.start()

    # Give it time to boot up
    time.sleep(20)
    
def launch_staticTF(number):
    rospack = rospkg.RosPack()
    path = None
    path = rospack.get_path("pips_test")
    # We'll assume Gazebo is launched are ready to go
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    global launch_static
    launch_static = roslaunch.parent.ROSLaunchParent(
        uuid, [path + "/launch/static_transformations/static_" + str(number) + ".launch"])
    launch_static.start()
    time.sleep(3)

def kill_controller():
    # This kills the launch, not the script! Beautiful.
    global launch
    launch.shutdown()
    global launch_static
    launch_static.shutdown()
    # print "I can still do stuff here"


def reset_odom(pub):
    pub.publish()



def writeCSVfile():
  global testNum
  outputfile_name = 'test_data/data_' + \
    str(testNum) + '.csv'
  with open(outputfile_name, 'wb') as csvfile:
        datawriter = csv.writer(csvfile, delimiter=' ',
                                quoting=csv.QUOTE_MINIMAL)
        testNumber = ['test ' + str(testNum)]
        datawriter.writerow(testNumber)
        positions = []
        datawriter.writerow(['LPIPS Result', testResult[0]])
        datawriter.writerow(['Time Taken', testTime[0]])
        datawriter.writerow(['TEB Result', testResult[1]])
        datawriter.writerow(['Time Taken', testTime[1]])
        datawriter.writerow(['Start Section', startSection[0]])
        datawriter.writerow(['End Section', endSection[0]])


def callService():
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp1 = gms(model_name="mobile_base", relative_entity_name="world")
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

def getState(goal_index):
    data = callService()
    current_x = data.pose.position.x
    print(("My Position is ", current_x))
    current_y = data.pose.position.y
    print(("My Position is ", current_y))
    diff_x = (total_section[goal_index][0] - current_x) * (total_section[goal_index][0] - current_x)
    diff_y = (total_section[goal_index][1] - current_y) * (total_section[goal_index][1] - current_y)
    diff_total = math.sqrt(diff_x + diff_y)
    if (diff_total < 2.0):
        return True
    else:
        return False
      
def callService():
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp1 = gms(model_name="mobile_base", relative_entity_name="world")
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

    
def main(controller_name):
    odom_pub = rospy.Publisher(
        '/mobile_base/commands/reset_odometry', std_msgs.msg.Empty, queue_size=10)
    for segment in range(100):
      	new_posGoal = gazebo_driver.getRandInt(0, 8)
	start_pose = total_section[new_posGoal[0]]
	end_pose = total_section[new_posGoal[1]]
	startSection[0] = new_posGoal[0]
	endSection[0] = new_posGoal[1]
	for i in range(numController):
	  reset_odom(odom_pub)
	  time.sleep(1)
	  move_robot(start_pose)

	  start = time.time()
	  launch_controller(controller_name[i])
	  time.sleep(5)
	  output = test_driver.run_testImpl(end_pose)
	  testTime[i] = time.time() - start
	  if (output == True):
	    output2 = getState(new_posGoal[1])
	    if(output2):
	      testResult[i] = 'success'
	    else:
	      testResult[i] = 'fail_bumped'
	  else:
	    testResult[i] = 'failt_noFinding'
	global testNum
	testNum = testNum + 1
	writeCSVfile()

if __name__ == "__main__":
    gazebo_driver = Prototype()
    global controller_name
    controller_name.append("Lpips_togo.launch")
    controller_name.append("teb_controller.launch")
    main(controller_name)

