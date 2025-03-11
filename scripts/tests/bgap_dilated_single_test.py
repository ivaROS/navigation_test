#!/usr/bin/env python3

from nav_scripts.gazebo_master import MultiMasterCoordinator
import time



start_time = time.time()
num_instances = 1
master = MultiMasterCoordinator(num_instances, save_results=False, use_existing_roscore=True)
# master.fieldnames.extend(
#     ['seed'])
master.start()

min_obstacle_spacing = 1
controller = 'bgap_mpc_casadi_dilation' # 'teb', 'bgap_mpc_casadi'

use_bezier = True
use_pose_controller = True
use_keyhole = True # inflating gap is not compatible with pg
use_ni = False
use_po = False
has_feedforward = True
v_des = 0.3

scneario_only = False

seed_list = range(1)
seed_list = [ele for ele in seed_list for i in range(1)]
seed_list = [7]

def getTasks():
    for scenario in ["sector_laser"]: # "dense", "campus_obstacle", "fourth_floor_obstacle", "sector_laser"
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
            'record': False, 'stat_record': False, 'timing_record': False, 'world_args': {'gazebo_gui': False}, 'robot_args': {'odom_topic': '/odom'},
            'scenario_only': scneario_only}

            if scenario != 'sector_laser' and scenario != "dense":
                task['num_obstacles'] = 50

            yield task


master.add_tasks(tasks=getTasks())

if not scneario_only:
    master.wait_to_finish()
    master.shutdown()
else:
    while True:
        try:
            pass
        except KeyboardInterrupt:
            master.shutdown()
            sys.exit()

end_time = time.time()
print("Total time: " + str(end_time - start_time))
