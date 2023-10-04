#!/usr/bin/env python

from __future__ import print_function
from builtins import str
from builtins import range
from nav_scripts.gazebo_master import MultiMasterCoordinator
from nav_scripts.testing_scenarios import TestingScenarios, GeneralScenario
from nav_scripts.controller_launcher import ControllerLauncher
import time

def run_auto_tests(show_gazebo=False):

    start_time = time.time()
    master = MultiMasterCoordinator(num_masters=1,save_results=False)
    master.start()

    def getTasks():
        controller_freq = 5
        for scenario in ['pregen_dense']:
            for seed in range(0 ,10):
                for global_planning_freq in [1]:
                    for controller in ['teb']:
                        task = {'controller': controller, 'seed': seed, 'scenario': scenario, 'robot': 'turtlebot',
                                "min_obstacle_spacing": 0.5,
                             'record': False, 'world_args': {"gazebo_gui":show_gazebo},
                                'controller_args': {'global_planning_freq': global_planning_freq,
                                                    'controller_freq': controller_freq}}

                        yield task



    master.add_tasks(tasks=getTasks())
    master.wait_to_finish()
    master.shutdown()
    end_time = time.time()
    print("Total time: " + str(end_time - start_time))


if __name__ == "__main__":

    run_auto_tests(show_gazebo=True)

