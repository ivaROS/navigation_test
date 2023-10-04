#!/usr/bin/env python3

from nav_scripts.gazebo_master import MultiMasterCoordinator
import time



start_time = time.time()
num_instances = 2
master = MultiMasterCoordinator(num_instances, save_results=True)
# master.fieldnames.extend(
#     ['seed'])
master.start()

min_obstacle_spacing = 2.
controller = 'potential_gap_generic_model_depth_ego' # 'potential_gap_hybrid_newlaser_generic_model_depth_ego', 'potential_gap_generic_model_depth_ego'

def getTasks():
    for scenario in ["fourth_floor_obstacle"]: # "sector_laser", "dense", "campus_obstacle", "fourth_floor_obstacle"
        for seed in [1,2,6,7,9,10,14,18,31,32,38,39,40,41]:
            if scenario != "fourth_floor_obstacle":
                far_feasible = True
            else:
                far_feasible = False
            task = {'scenario': scenario, 'seed': seed, 'controller': controller, 'robot': 'ridgeback', 
            'controller_args': {'far_feasible': far_feasible}, 'min_obstacle_spacing': min_obstacle_spacing, 'timeout': 600,
            'record': False, 'world_args': {'gazebo_gui': False}, 'robot_args': {'odom_topic': '/ground_truth/state'}}

            if scenario != 'sector_laser' and scenario != "dense":
                task['num_obstacles'] = 50

            yield task


master.add_tasks(tasks=getTasks())
master.wait_to_finish()
master.shutdown()
end_time = time.time()
print("Total time: " + str(end_time - start_time))
