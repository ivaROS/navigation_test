#!/usr/bin/env python

import test_driver
from testing_scenarios import TestingScenarios
import rospy


seed = 16

num_barrels = 6
min_obstacle_spacing = 1.5

task = {'seed':seed, 'scenario':'corridor_zigzag_door', 'num_obstacles':num_barrels, 'min_obstacle_spacing':min_obstacle_spacing}

task = {'seed':seed, 'scenario':'dense'}

rospy.init_node('test_driver', anonymous=True)

scenarios = TestingScenarios()
scenario = scenarios.getScenario(task)

rospy.Rate(1).sleep()

scenario.setupScenario()
#result = test_driver.run_test(goal_pose=scenario.getGoal())
