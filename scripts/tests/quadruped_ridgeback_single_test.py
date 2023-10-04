#!/usr/bin/env python3

from nav_scripts.gazebo_master import MultiMasterCoordinator
import time



start_time = time.time()
num_instances = 1
master = MultiMasterCoordinator(num_instances, save_results=False, use_existing_roscore=True)
# master.fieldnames.extend(
#     ['seed'])
master.start()

min_obstacle_spacing = 1.5
controller = 'gpf_pg_hybrid_newlaser_generic_model_depth_ego' # 'potential_gap_hybrid_newlaser_generic_model_depth_ego', 'potential_gap_generic_model_depth_ego', 'gpf_pg_hybrid_newlaser_generic_model_depth_ego'

def getTasks():
    for scenario in ["dense"]: # "sector_laser", "dense", "campus_obstacle", "fourth_floor_obstacle"
        for seed in [3]:
            if scenario != "fourth_floor_obstacle":
                far_feasible = True
            else:
                far_feasible = False
            task = {'scenario': scenario, 'seed': seed, 'controller': controller, 'robot': 'ridgeback', 
            'controller_args': {'far_feasible': far_feasible}, 'min_obstacle_spacing': min_obstacle_spacing, 'timeout': 600,
            'record': False, 'world_args': {'gazebo_gui': True}, 'robot_args': {'odom_topic': '/ground_truth/state'}}

            if scenario != 'sector_laser' and scenario != "dense":
                task['num_obstacles'] = 50

            yield task


master.add_tasks(tasks=getTasks())
master.wait_to_finish()
master.shutdown()
end_time = time.time()
print("Total time: " + str(end_time - start_time))
