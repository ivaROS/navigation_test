#!/usr/bin/env python

import test_driver
from testing_scenarios import TestingScenarios
import rospy


seed = 20

num_barrels = 50
min_obstacle_spacing = 1.5

task = {'seed':seed, 'scenario':'corridor_zigzag_door', 'num_obstacles':num_barrels, 'min_obstacle_spacing':min_obstacle_spacing}

task = {'seed':seed, 'scenario':'dense'}





starting_pose = [20,24,0]
goal = [29,24,0]

starting_pose = [20,24.25,0]
goal = [32,24.25,0]

starting_pose = [-1,0,0]
goal = [15,15,0]

# book chapter testing
task = {'init_pose':starting_pose, 'goal':goal, 'world':"none"}
task = {'seed':seed, 'scenario':'campus', 'target_id':1, 'num_barrels':num_barrels}

rospy.init_node('test_driver', anonymous=True)

#test_driver.reset_costmaps()

#rospy.Rate(1).sleep()

scenarios = TestingScenarios()
scenario = scenarios.getScenario(task)

rospy.Rate(1).sleep()

scenario.setupScenario()



result = test_driver.run_test(goal_pose=scenario.getGoal())
