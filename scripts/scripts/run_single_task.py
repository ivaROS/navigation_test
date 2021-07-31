#!/usr/bin/env python

import nav_scripts.movebase_driver as test_driver
from testing_scenarios import TestingScenarios
import rospy


seed = 50

num_barrels = 50
min_obstacle_spacing = 0.75


task = {'seed': 0, 'controller':'egoteb', 'scenario':'dense', 'min_obstacle_spacing': min_obstacle_spacing}

rospy.init_node('test_driver', anonymous=True)

scenarios = TestingScenarios()
scenario = scenarios.getScenario(task)

rospy.Rate(1).sleep()

scenario.setupScenario()

