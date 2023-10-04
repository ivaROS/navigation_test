#!/usr/bin/env python3

from __future__ import print_function
from builtins import str
from nav_scripts.movebase_driver import run_test
from nav_scripts.testing_scenarios import TestingScenarios
import rospy


seed = 71

num_barrels = 500
min_obstacle_spacing = 1

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

task = {'seed':seed, 'scenario':'full_sector_laser', 'min_obstacle_spacing':min_obstacle_spacing, 'num_obstacles':num_barrels}

task = {'seed':seed, 'scenario':'full_campus_obstacle', 'min_obstacle_spacing':min_obstacle_spacing, 'num_obstacles':num_barrels}
task = {'seed':seed, 'scenario':'full_fourth_floor_obstacle', 'num_obstacles':50, 'min_obstacle_spacing':min_obstacle_spacing}
task= {'scenario': 'dense', 'controller':'teb_multigoal', 'seed':0, 'robot':'turtlebot', 'min_obstacle_spacing':1}

seed=74
task = {'seed':seed, 'scenario':'full_campus_obstacle', 'min_obstacle_spacing':min_obstacle_spacing, 'num_obstacles':num_barrels}

task= {'scenario': 'dense', 'controller':'teb', 'seed':7, 'robot':'turtlebot', 'min_obstacle_spacing':1.0}
#task = {'seed':3, 'scenario':'fourth_floor_obstacle', 'min_obstacle_spacing':min_obstacle_spacing}

task= {'scenario': 'dense', 'controller':'teb', 'seed':32, 'robot':'turtlebot', 'min_obstacle_spacing':0.75}

task= {'scenario': 'dense', 'controller':'teb', 'seed':56, 'robot':'turtlebot', 'min_obstacle_spacing':0.5}

task = {'seed':74, 'scenario':'full_campus_obstacle', 'min_obstacle_spacing':min_obstacle_spacing, 'num_obstacles':num_barrels}
task= {'scenario': 'dense', 'controller':'teb', 'seed':56, 'robot':'turtlebot', 'min_obstacle_spacing':0.625}

task= {'scenario': 'dense', 'controller':'teb', 'seed':10, 'robot':'turtlebot', 'min_obstacle_spacing':0.625}

task= {'scenario': 'sector_extra', 'controller':'teb', 'seed':10, 'robot':'turtlebot', 'min_obstacle_spacing': 1, 'num_obstacles':50}

task= {'scenario': 'fourth_floor_obstacle', 'controller':'teb', 'seed':59, 'robot':'turtlebot', 'min_obstacle_spacing': 1, 'num_obstacles':500}
task= {'scenario': 'fourth_floor_obstacle', 'controller':'teb', 'seed':7, 'robot':'turtlebot', 'min_obstacle_spacing': 1, 'num_obstacles':500}

task= {'scenario': 'campus_obstacle', 'controller':'teb', 'seed':28, 'robot':'turtlebot', 'min_obstacle_spacing': 1, 'num_obstacles':500}   #91, 14
task= {'scenario': 'campus_obstacle', 'controller':'teb', 'seed':73, 'robot':'turtlebot', 'min_obstacle_spacing': 1, 'num_obstacles':500}   #51
task= {'scenario': 'campus_obstacle', 'controller':'teb', 'seed':63, 'robot':'turtlebot', 'min_obstacle_spacing': 1, 'num_obstacles':500}   #9
task= {'scenario': 'campus_obstacle', 'controller':'ego_teb_multi', 'seed':87, 'robot':'turtlebot', 'min_obstacle_spacing': 1, 'num_obstacles':500}   #9

task= {'scenario': 'dense', 'controller':'ego_teb', 'seed':35, 'robot':'turtlebot', 'min_obstacle_spacing':0.625}
task= {'scenario': 'dense', 'controller':'ego_teb_multi', 'seed':81, 'robot':'turtlebot', 'min_obstacle_spacing':0.625}

task= {'scenario': 'dense', 'controller':'ego_teb_multi', 'seed':91, 'robot':'turtlebot', 'min_obstacle_spacing':0.5}
task= {'scenario': 'dense', 'controller':'ego_teb_multi', 'seed':0, 'robot':'turtlebot', 'min_obstacle_spacing':0.5}


task= {'scenario': 'dense', 'controller':'ego_teb_multi', 'seed':0, 'robot':'turtlebot', 'min_obstacle_spacing':1}

task= {'scenario': 'dense', 'controller':'ego_teb_multi', 'seed':5, 'robot':'turtlebot', 'min_obstacle_spacing':0.5}

task= {'scenario': 'dense', 'controller':'ego_teb_multi', 'seed':68, 'robot':'turtlebot', 'min_obstacle_spacing':0.5}

#task= {'scenario': 'fourth_floor_obstacle', 'controller':'teb', 'seed':59, 'robot':'turtlebot', 'min_obstacle_spacing': 1, 'num_obstacles':500}
#task= {'scenario': 'campus_obstacle', 'controller':'ego_teb_multi', 'seed':20, 'robot':'turtlebot', 'min_obstacle_spacing': 1, 'num_obstacles':500}   #9


rospy.init_node('test_driver', anonymous=True)

#test_driver.reset_costmaps()

rospy.Rate(1).sleep()

scenarios = TestingScenarios()
scenario = scenarios.getScenario(task)

rospy.Rate(1).sleep()

import time
start_time = time.time()
scenario.setupScenario()
end_time = time.time()

print(str(end_time-start_time))
#print scenario.getGoalMsg()
result = run_test(goal_pose=scenario.getGoalMsg(), record=False)

print(result)
