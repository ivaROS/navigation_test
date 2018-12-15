#!/usr/bin/env python

import test_driver
from testing_scenarios import TestingScenarios
import rospy


seed = 0

num_barrels = 50
min_obstacle_spacing = 1.8

starting_pose = [-5,0,0]
goal = [15,15,0]

# book chapter testing
task = {'seed':seed, 'scenario':'fourth_floor', 'num_barrels':num_barrels, 'target_id':4, 'min_obstacle_spacing':min_obstacle_spacing}

# task = {'seed':seed, 'scenario':'campus', 'num_barrels':num_barrels, 'target_id':1}

# task = {'seed':seed, 'scenario':'sector'}

rospy.init_node('test_driver', anonymous=True)

#test_driver.reset_costmaps()

#rospy.Rate(1).sleep()

scenarios = TestingScenarios()
scenario = scenarios.getScenario(task)

rospy.Rate(1).sleep()

scenario.setupScenario()



result = test_driver.run_test(goal_pose=scenario.getGoal())
