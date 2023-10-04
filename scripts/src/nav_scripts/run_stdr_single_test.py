#!/usr/bin/env python3

# import quadruped_nav_scripts.movebase_driver as test_driver
from stdr_testing_scenarios import TestingScenarios
from stdr_testing_scenarios import SectorScenario, CampusScenario, FourthFloorScenario, SparseScenario
import rospy
import os
import multiprocessing as mp
import roslaunch
import rospkg
import subprocess

from stdr_master import *

seed = 3

num_barrels = 50
min_obstacle_spacing = 0.75


task = {'seed': 5, 'fov':360, 'robot':'nonholonomic', 'controller':'bgap_mpc_casadi', 'world':'dense_laser', 'taskid':0}

rospy.init_node('test_driver', anonymous=True)

print(task["world"])
trans = [0, 0]

if task["world"] == "sector_laser":
    scenario = SectorScenario(task, "world")
    trans[0] = 10.217199
    trans[1] = 10.1176
elif task["world"] == "office_laser":
    scenario = FourthFloorScenario(task, "world")
    trans[0] = 43.173023
    trans[1] = 30.696842
elif task["world"] == "dense_laser":
    scenario = SparseScenario(task, "world")
    trans[0] = 9.955517
    trans[1] = 9.823917
elif task["world"] == "campus_laser":
    scenario = CampusScenario(task, "world")
    # trans[0] = 14.990204
    # trans[1] = 13.294787
    trans[0] = 15.002400
    trans[1] = 13.295538

start = scenario.getStartingPose()
goal = scenario.getGoal()
start.position.x += trans[0]
start.position.y += trans[1]
# goal.pose.position.x += trans[0]
# goal.pose.position.y += trans[1]

cmd_launch= str('roslaunch bezier_gap_benchmark stdr_' + task['robot'] + '_' + task['world'] + '_world.launch' \
                    + ' fov:=' + str(task['fov']) + ' rbtx:=' + str(start.position.x) + ' rbty:=' + str(start.position.y) \
                    + ' map_num:=' + str(task['seed']))
subprocess.Popen(cmd_launch, shell=True)

rospy.Rate(5).sleep()

run_test(goal_pose=goal, record=False, taskid=task["taskid"])

subprocess.call('killall -9 stdr_server_node robot_state_publisher static_transform_publisher map_server map_transform_publisher costmap_2d_node', shell=True)