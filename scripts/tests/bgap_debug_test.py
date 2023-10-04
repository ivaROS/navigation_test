#!/usr/bin/env python3

from nav_scripts.gazebo_master import MultiMasterCoordinator
import time



start_time = time.time()
num_instances = 3
master = MultiMasterCoordinator(num_instances, save_results=True)
# master.fieldnames.extend(
#     ['seed'])
master.start()

min_obstacle_spacing = 1
controller = 'bgap_mpc_casadi' # 'teb', 'bgap_mpc_casadi'

use_bezier = True
use_pose_controller = False
use_keyhole = True
use_ni = True
use_po = True
has_feedforward = True
v_des = 0.4

seed_list = range(2,25)
seed_list = [ele for ele in seed_list for i in range(4)]

def getTasks():
    for scenario in ["dense"]: # "sector_laser", "dense", "campus_obstacle", "fourth_floor_obstacle"
        for seed in seed_list:
            if scenario != "fourth_floor_obstacle" and scenario != "campus_obstacle":
                far_feasible = True
            else:
                far_feasible = False
            task = {'scenario': scenario, 'seed': seed, 'controller': controller, 'robot': 'turtlebot', 'robot_impl': 'turtlebot', 
            'controller_args': {'far_feasible': far_feasible, 'use_bezier': use_bezier, 'use_pose_controller': use_pose_controller,
                                'use_keyhole': use_keyhole, 'use_ni': use_ni, 'use_po': use_po, 'has_feedforward' : has_feedforward, 'v_des' : v_des,
                                'enable_vis': True}, 
            'min_obstacle_spacing': min_obstacle_spacing, 'timeout': 600,
            'record': True, 'stat_record': False, 'world_args': {'gazebo_gui': False}, 'robot_args': {'odom_topic': '/odom'}}

            if scenario != 'sector_laser' and scenario != "dense":
                task['num_obstacles'] = 50

            yield task


master.add_tasks(tasks=getTasks())
master.wait_to_finish()
master.shutdown()
end_time = time.time()
print("Total time: " + str(end_time - start_time))
