#!/usr/bin/env python

import test_driver
from testing_scenarios import TestingScenarios
import rospy


seed = 28

num_barrels = 150
min_obstacle_spacing = 0.5

starting_pose = [-9,0,0]
goal = [9,4,0]

# book chapter testing
task = {'seed':seed, 'scenario':'fourth_floor', 'num_obstacles':num_barrels, 'target_id':4, 'min_obstacle_spacing':min_obstacle_spacing}

task = {'seed':seed, 'scenario':'campus', 'num_obstacles':num_barrels, 'min_obstacle_spacing':min_obstacle_spacing}

#task= {'scenario': 'fourth_floor', 'controller':'pips_egocylindrical_dwa', 'seed':seed, 'robot':'quadrotor', 'min_obstacle_spacing': min_obstacle_spacing, 'num_obstacles': num_barrels}
# task= {'scenario': 'fourth_floor', 'controller':'dwa', 'seed':seed, 'robot':'pioneer', 'min_obstacle_spacing': min_obstacle_spacing, 'num_obstacles': num_barrels}

# task= {'scenario': 'sector_laser', 'controller':'dwa', 'seed':seed, 'robot':'turtlebot', 'min_obstacle_spacing': min_obstacle_spacing, 'num_obstacles': num_barrels}
task= {'scenario': 'full_sector_extra', 'controller':'egocylindrical_pips_dwa', 'seed':seed, 'robot':'turtlebot', 'min_obstacle_spacing': min_obstacle_spacing, 'num_obstacles': num_barrels}
#
# task = {'seed':seed, 'scenario':'campus_obstacle', 'num_obstacles':num_barrels, 'min_obstacle_spacing':min_obstacle_spacing}
# task = {'seed':seed, 'scenario':'fourth_floor_obstacle', 'num_obstacles':num_barrels, 'min_obstacle_spacing':min_obstacle_spacing}

# task = {'seed':seed, 'scenario':'sector'}

seed = 7
task= {'scenario': 'dense', 'controller':'teb', 'seed':seed, 'robot':'turtlebot', 'min_obstacle_spacing':0.75}

#task = {'seed':seed, 'scenario':'full_fourth_floor_obstacle', 'num_obstacles':50, 'min_obstacle_spacing':min_obstacle_spacing}


rospy.init_node('test_driver', anonymous=True)

#test_driver.reset_costmaps()

#rospy.Rate(1).sleep()

scenarios = TestingScenarios()
scenario = scenarios.getScenario(task)

rospy.Rate(1).sleep()

scenario.setupScenario()



result = test_driver.run_test(goal_pose=scenario.getGoal())

#print(result)