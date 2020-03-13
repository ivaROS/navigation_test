#!/usr/bin/env python

import time
from nav_scripts.gazebo_master import MultiMasterCoordinator
if __name__ == "__main__":

    start_time = time.time()
    num_instances = 2
    master = MultiMasterCoordinator(num_instances)
    master.start()

    def getTasks():
        for [scenario, min_obstacle_spacing] in [['medium', 2], ['dense' ,0.5], ['campus_obstacle' ,1], ['fourth_floor_obstacle' ,1]]:
            for robot in ['turtlebot']:
                for seed in range(2):
                    for global_planning_freq in [0 ,1]:
                        for controller in ['dwa','teb']:
                                task = {'controller': controller, 'seed': seed, 'scenario': scenario, 'robot': robot, 'min_obstacle_spacing': min_obstacle_spacing,
                                        'controller_args': {'global_planning_freq': global_planning_freq}, 'world_args':{'gazebo_gui':'true'}}
                                yield task


    master.add_tasks(tasks=getTasks())

    # master.singletask()
    master.wait_to_finish()
    # rospy.spin()
    master.shutdown()
    end_time = time.time()
    print "Total time: " + str(end_time - start_time)
