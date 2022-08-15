#!/usr/bin/env python3

from nav_scripts.gazebo_master import MultiMasterCoordinator
import time



start_time = time.time()
num_instances = 1
master = MultiMasterCoordinator(num_instances, save_results=True)
master.fieldnames.extend(
    ['max_number_classes', 'laser_fov', 'ec_radius', 'cost_factor', 'selection_obst_cost_scale',
     'enable_standard_costs', 'selection_alternative_time_cost', 'repeat', 'sim_speed', 'gap_criteria',
     'add_initial_plan', 'switching_blocking_period',
     'gap_merge_max_size', 'gap_merge_min_radius', 'gap_merge_offset', 'gap_merge_scale',
     'enable_gap_axial_conversion', 'enable_radial_gap_conversion', 'gap_conversion_rotation_angle',
     'num_traj_type_radial', 'num_traj_type_none', 'num_traj_type_axial', 'gap_conversion_rotation_angle',
     'initial_trajectory_factor', 'use_projection_operator', 'mpc_update_period', 'max_replan_period',
     'replan_min_tte', 'replan_min_ttc', 'min_goal_dist', 'replan_exclusion_radius',
     'gap_goal_min_dist', 'transformed_egocircle', 'narrow_gap_threshold_factor',
     'goal_sampling_des_extension_dist', 'goal_sampling_safety_dist', 'narrow_gap_prevent_replan_dist_factor',
     'turtlebot_description', 'controller_logging', 'gazebo_recording', 'error_details'])
master.start()


def getTasks():
    for seed in range(20):
        task = {'scenario': 'sparse', 'controller': 'teb', 'robot': 'turtlebot', 'world_args': {'gazebo_gui': True}, 'robot_impl': 'turtlebot'}
        yield task


master.add_tasks(tasks=getTasks())

master.wait_to_finish()
master.shutdown()
end_time = time.time()
print("Total time: " + str(end_time - start_time))
