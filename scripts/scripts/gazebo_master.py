#!/usr/bin/env python

import subprocess
import multiprocessing as mp
import os
import sys
import rospkg
import roslaunch
import time
import test_driver
from gazebo_driver_v2 import GazeboDriver
import rosgraph
import threading
import Queue
from ctypes import c_bool

import signal

import socket
import contextlib

from testing_scenarios import TestingScenarios

from actionlib_msgs.msg import GoalStatusArray
from nav_msgs.msg import Odometry
import rospy
import csv
import datetime


def port_in_use(port):
    with contextlib.closing(socket.socket(socket.AF_INET, socket.SOCK_STREAM)) as sock:
        if sock.connect_ex(('127.0.0.1', port)) == 0:
            print "Port " + str(port) + " is in use"
            return True
        else:
            print "Port " + str(port) + " is not in use"
            return False



class MultiMasterCoordinator:
    def __init__(self):
        signal.signal(signal.SIGINT, self.signal_shutdown)
        signal.signal(signal.SIGTERM, self.signal_shutdown)
        self.children_shutdown = mp.Value(c_bool, False)
        self.soft_shutdown = mp.Value(c_bool, False)

        self.should_shutdown = False


        self.num_masters = 4

        self.save_results = True
        self.task_queue_capacity = 2000 #2*self.num_masters
        self.task_queue = mp.JoinableQueue(maxsize=self.task_queue_capacity)
        self.result_queue_capacity = 2000 #*self.num_masters
        self.result_queue = mp.JoinableQueue(maxsize=self.result_queue_capacity)
        self.gazebo_masters = []
        self.result_list = []
        self.gazebo_launch_mutex = mp.Lock()

        self.fieldnames = ["controller"]
        self.fieldnames.extend(TestingScenarios.getFieldNames())
        self.fieldnames.extend(["pid","result","time","path_length","robot"])
        self.fieldnames.extend(["sim_time"])


    def start(self):
        self.startResultsProcessing()
        self.startProcesses()

    def startResultsProcessing(self):
        self.result_thread = threading.Thread(target=self.processResults,args=[self.result_queue])
        self.result_thread.daemon=True
        self.result_thread.start()

    def startProcesses(self):
        self.ros_port = 11311
        self.gazebo_port = self.ros_port + 100
        for ind in xrange(self.num_masters):
            self.addProcess()


    def addProcess(self):
        while port_in_use(self.ros_port):
            self.ros_port += 1

        while port_in_use(self.gazebo_port):
            self.gazebo_port += 1

        gazebo_master = GazeboMaster(self.task_queue, self.result_queue, self.children_shutdown, self.soft_shutdown, self.ros_port,
                                     self.gazebo_port, gazebo_launch_mutex=self.gazebo_launch_mutex)
        gazebo_master.start()
        self.gazebo_masters.append(gazebo_master)

        self.ros_port += 1
        self.gazebo_port += 1

        time.sleep(1)


    def processResults(self,queue):

        # outputfile_name = "~/Documents/dl3_gazebo_results_" + str(datetime.datetime.now())
        outputfile_name = "/data/fall2018/chapter_experiments/chapter_experiments_" + str(datetime.datetime.now())
        outputfile_name = os.path.expanduser(outputfile_name)

        with open(outputfile_name, 'wb') as csvfile:
            seen = set()
            fieldnames = [x for x in self.fieldnames if not (x in seen or seen.add(x))] #http://www.martinbroadhurst.com/removing-duplicates-from-a-list-while-preserving-order-in-python.html

            datawriter = csv.DictWriter(csvfile, fieldnames=fieldnames, restval='', extrasaction='ignore')
            datawriter.writeheader()

            while not self.should_shutdown: #This means that results stop getting saved to file as soon as I try to kill it
                try:
                    task = queue.get(block=False)

                    result_string = "Result of ["
                    for k,v in task.iteritems():
                        #if "result" not in k:
                            result_string+= str(k) + ":" + str(v) + ","
                    result_string += "]"

                    print result_string

                    if "error" not in task:
                        self.result_list.append(result_string)
                        if self.save_results:
                            datawriter.writerow(task)
                            csvfile.flush()
                    else:
                        del task["error"]
                        self.task_queue.put(task)
                        self.addProcess()

                    #print "Result of " + task["world"] + ":" + task["controller"] + "= " + str(task["result"])
                    queue.task_done()
                except Queue.Empty, e:
                    #print "No results!"
                    time.sleep(1)

    def signal_shutdown(self,signum,frame):
        self.shutdown()

    def shutdown(self):
        with self.children_shutdown.get_lock():
            self.children_shutdown.value = True

        for process in mp.active_children():
            process.join()

        self.should_shutdown = True

        #for process in self.gazebo_masters:
        #    if process.is_alive():
        #        process.join()
        #sys.exit(0)

    def waitToFinish(self):
        print "Waiting until everything done!"
        self.task_queue.join()
        print "All tasks processed!"
        with self.soft_shutdown.get_lock():
            self.soft_shutdown.value = True

        #The problem is that this won't happen if I end prematurely...
        self.result_queue.join()
        print "All results processed!"

        for result in self.result_list:
            print result

    # This list should be elsewhere, possibly in the configs package
    def addTasks(self):
        controllers = ["dwa", "teb", "pips_dwa", "pips_ni" ]  # "rl_single", "rl_goal",
        '''
        for a in range(0, 50):
            for controller in controllers: #controllers:
                for repetition in range(1):
                    task = {'scenario': 'sector', 'controller': controller, 'seed': a}
                    self.task_queue.put(task)
        
        for a in range(0, 50):
            for controller in ["pips_dwa_propagated", "rl_single"]:
                for repetition in range(1):
                    task = {'scenario': 'sector', 'controller': controller, 'seed': a}
                    self.task_queue.put(task)
        


        for a in range(0, 50):
            for controller in ['regression_goal', 'regression_goal_propagated','pips_ni', 'multiclass_propagated', 'rl_goal', 'rl_single','multiclass', "pips_dwa_propagated", "pips_dwa", 'pips_ni_propagated']: #'rl_goal'
                for repetition in range(1):
                    task = {'scenario': 'sector', 'controller': controller, 'seed': a}
                    self.task_queue.put(task)
        '''

        '''
        for a in range(0, 50):
            for controller in ['pips_ni', 'multiclass_propagated', 'rl_goal', 'rl_single', 'regression_goal', 'regression_goal_propagated']: #'rl_goal'
                for repetition in range(1):
                    task = {'scenario': 'sector', 'controller': controller, 'seed': a}
                    self.task_queue.put(task)

        for a in range(0, 50):
            for controller in ['multiclass', "pips_dwa_propagated", "pips_dwa"]:
                for repetition in range(1):
                    task = {'scenario': 'sector', 'controller': controller, 'seed': a}
                    self.task_queue.put(task)
        '''
        '''
        for a in range(50, 150):
            for controller in ['pips_ni', 'multiclass_propagated', 'rl_goal', 'rl_single', 'regression_goal', 'regression_goal_propagated', 'multiclass', "pips_dwa_propagated", "pips_dwa", 'teb', 'dwa']:  # 'rl_goal'
                for repetition in range(1):
                    task = {'scenario': 'sector', 'controller': controller, 'seed': a}
                    self.task_queue.put(task)
        '''

        '''
        for a in [a for a in range(52,97) if a not in [57,58,59,62,66,63,65,67,79,80,87,88,90]]:
            for controller in ['egocylindrical_pips_dwa']:
                for repetition in range(1):
                    task = {'scenario': 'sector', 'controller': controller, 'seed': a}
                    self.task_queue.put(task)
        '''
        '''
        for a in range(52,97):
            for controller in ['depth_pips_dwa']:
                for repetition in range(1):
                    task = {'scenario': 'sector', 'controller': controller, 'seed': a}
                    self.task_queue.put(task)
        '''

        '''
        for a in range(1, 100):
            for controller in ['dwa']: #, 'teb', 'egocylindrical_pips_dwa', 'depth_pips_dwa']:
                for repetition in range(1):
                    task = {'scenario': 'sector', 'controller': controller, 'seed': a}
                self.task_queue.put(task)
        '''

        '''
        for a in range(52, 97):
            for controller in ['pips_ec_rh']:  # , 'teb', 'egocylindrical_pips_dwa', 'depth_pips_dwa']:
                for repetition in range(1):
                    task = {'scenario': 'sector', 'controller': controller, 'seed': a}
                self.task_queue.put(task)
        '''

        '''
        for a in range(0,100):
            for controller in ['pips_ec_rh','depth_pips_dwa','egocylindrical_pips_dwa','dwa','teb']:  # , 'teb', 'egocylindrical_pips_dwa', 'depth_pips_dwa']:
                for repetition in range(1):
                    task = {'scenario': 'sector', 'controller': controller, 'seed': a}
                self.task_queue.put(task)
        '''
        '''
        for scenario in ['campus', 'sector']:
            for a in range(0, 20):
                for controller in ['egocylindrical_pips_dwa', 'egocylindrical_pips_dwa_no_recovery', 'dwa', 'dwa_no_recovery']:  # , 'teb', 'egocylindrical_pips_dwa', 'depth_pips_dwa']:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a, 'num_barrels':20} #, 'controller_args':['use_recovery_behaviors:=' + recovery_behaviors]}
                        self.task_queue.put(task)

        for scenario in ['campus', 'sector']:
            for a in range(20, 100):
                for controller in ['egocylindrical_pips_dwa', 'egocylindrical_pips_dwa_no_recovery', 'dwa',
                                   'dwa_no_recovery']:  # , 'teb', 'egocylindrical_pips_dwa', 'depth_pips_dwa']:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a,
                                'num_barrels': 20}  # , 'controller_args':['use_recovery_behaviors:=' + recovery_behaviors]}
                        self.task_queue.put(task)
        '''

        # for scenario in ['sector']:
        #     for a in range(64,100):
        #         for controller in ['egocylindrical_pips_dwa', 'egocylindrical_pips_dwa_no_recovery', 'dwa', 'dwa_no_recovery']:  # , 'teb', 'egocylindrical_pips_dwa', 'depth_pips_dwa']:
        #             for repetition in range(1):
        #                 task = {'scenario': scenario, 'controller': controller, 'seed': a}  # , 'controller_args':['use_recovery_behaviors:=' + recovery_behaviors]}
        #                 self.task_queue.put(task)

        '''
        for scenario in ['campus']:
            for num_barrels in [0,10]:
                for a in range(0,100):
                    for controller in ['teb','egocylindrical_pips_dwa', 'egocylindrical_pips_dwa_no_recovery', 'dwa',
                                       'dwa_no_recovery']:  # , 'teb', 'egocylindrical_pips_dwa', 'depth_pips_dwa']:
                        for repetition in range(1):
                            task = {'scenario': scenario, 'controller': controller, 'seed': a,
                                    'num_barrels': num_barrels}  # , 'controller_args':['use_recovery_behaviors:=' + recovery_behaviors]}
                            self.task_queue.put(task)
        '''

        '''
        for scenario in ['campus']:
            for num_barrels in [10]:
                for a in range(36,100):
                    for controller in ['teb','egocylindrical_pips_dwa', 'egocylindrical_pips_dwa_no_recovery', 'dwa',
                                       'dwa_no_recovery']:  # , 'teb', 'egocylindrical_pips_dwa', 'depth_pips_dwa']:
                        for repetition in range(1):
                            task = {'scenario': scenario, 'controller': controller, 'seed': a,
                                    'num_barrels': num_barrels}  # , 'controller_args':['use_recovery_behaviors:=' + recovery_behaviors]}
                            self.task_queue.put(task)

        for scenario in ['campus','sector']:
            for num_barrels in [20]:
                for a in range(0, 100):
                    for controller in ['teb']:
                        for repetition in range(1):
                            task = {'scenario': scenario, 'controller': controller, 'seed': a,
                                    'num_barrels': num_barrels}  # , 'controller_args':['use_recovery_behaviors:=' + recovery_behaviors]}
                            self.task_queue.put(task)
                            
        '''

        '''
        #I stopped the above just before it finished the last one
        task = {'scenario': 'sector', 'controller': 'teb', 'seed': 2,
                'num_barrels': 20}
        self.task_queue.put(task)
        '''
        '''
        for scenario in ['sector']:
            for num_barrels in [20]:
                for a in range(52,97):
                    for controller in ['depth_pips_dwa']:
                        for repetition in range(1):
                            task = {'scenario': scenario, 'controller': controller, 'seed': a,
                                    'num_barrels': num_barrels}  # , 'controller_args':['use_recovery_behaviors:=' + recovery_behaviors]}
                            self.task_queue.put(task)
        '''

        '''
        for scenario in ['sector']:
            for num_barrels in [20]:
                for a in range(52, 97):
                    for controller in ['egocylindrical_pips_dwa']:
                        for repetition in range(1):
                            task = {'scenario': scenario, 'controller': controller, 'seed': a,
                                    'num_barrels': num_barrels}  # , 'controller_args':['use_recovery_behaviors:=' + recovery_behaviors]}
                            self.task_queue.put(task)
        '''

        '''
        #rerunning cases that failed in both prior sets
        for scenario in ['sector']:
            for num_barrels in [20]:
                for a in [53,57,67,74,79]:
                    for controller in ['egocylindrical_pips_dwa']:
                        for repetition in range(1):
                            task = {'scenario': scenario, 'controller': controller, 'seed': a,
                                    'num_barrels': num_barrels}  # , 'controller_args':['use_recovery_behaviors:=' + recovery_behaviors]}
                            self.task_queue.put(task)
        '''

        '''
        #These didn't finish
        for scenario in ['campus']:
            for num_barrels in [0,10,20]:
                for a in range(0,100):
                    for controller in ['egocylindrical_pips_dwa']:
                        for repetition in range(1):
                            task = {'scenario': scenario, 'controller': controller, 'num_barrels': num_barrels, 'seed': a}
                            self.task_queue.put(task)
        '''
        '''
        for scenario in ['sector']:
            for a in range(0,100):
                for controller in ['egocylindrical_pips_dwa','pips_ec_rh','egocylindrical_pips_dwa_no_recovery','pips_ec_rh_no_recovery']:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a}
                        self.task_queue.put(task)

        #Except for the first 3, these didn't run
        for scenario in ['campus']:
            for num_barrels in [0, 10, 20]:
                for a in range(0, 100):
                    for controller in ['egocylindrical_pips_dwa','pips_ec_rh','egocylindrical_pips_dwa_no_recovery','pips_ec_rh_no_recovery']:
                        for repetition in range(1):
                            task = {'scenario': scenario, 'controller': controller,
                                    'num_barrels': num_barrels, 'seed': a}
                            self.task_queue.put(task)
        '''

        '''
        for scenario in ['sector']:
            for a in range(0,40):
                for controller in ['egocylindrical_pips_dwa']:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a}
                        self.task_queue.put(task)
        '''
        '''
        for scenario in ['sector']:
            for a in range(40, 100):
                for controller in ['egocylindrical_pips_dwa']:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a}
                        self.task_queue.put(task)
        '''
        '''
        for scenario in ['sector']:
            for a in range(40, 100):
                for controller in ['pips_ec_rh']:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a}
                        self.task_queue.put(task)
        '''
        '''
        for scenario in ['sector']:
            for a in range(33, 40):
                for controller in ['egocylindrical_pips_dwa', 'pips_ec_rh', 'egocylindrical_pips_dwa_no_recovery',
                                   'pips_ec_rh_no_recovery', 'teb', 'dwa']:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a}
                        self.task_queue.put(task)

            for a in range(40, 61):
                for controller in [
                                   'egocylindrical_pips_dwa_no_recovery',
                                   'pips_ec_rh_no_recovery', 'teb', 'dwa']:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a}
                        self.task_queue.put(task)
        '''
        '''
        for scenario in ['sector']:
            for a in range(61, 100):
                for controller in [
                    'egocylindrical_pips_dwa_no_recovery',
                    'pips_ec_rh_no_recovery', 'teb', 'dwa']:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a}
                        self.task_queue.put(task)
            for a in range(94, 100):
                for controller in ['pips_ec_rh']:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a}
                        self.task_queue.put(task)
                        
        '''
        '''
        for scenario in ['sector']:
            for a in range(0, 100):
                for controller in ['pips_ec_rh'
                    'baseline_rl_goal',
                    'goal_regression', 'multiclass'
                                   ]:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a}
                        self.task_queue.put(task)
        '''

        '''
        #Switched to circles for this one
        for scenario in ['sector']:
            for a in range(0, 100):
                for controller in ['pips_ec_rh'
                    #'baseline_rl_goal',
                    #'goal_regression', 'multiclass'
                                   ]:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a}
                        self.task_queue.put(task)
        #new circle model here
        for scenario in ['sector']:
            for a in range(0, 100):
                for controller in ['rl_goal'
                                   # 'baseline_rl_goal',
                                   # 'goal_regression', 'multiclass'
                                   ]:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a}
                        self.task_queue.put(task)
        '''
        '''
        for scenario in ['sector']:
            for a in range(18, 50):
                for controller in ['pips_ec_rh'
                                   # 'baseline_rl_goal',
                                   # 'goal_regression', 'multiclass'
                                   ]:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a}
                        self.task_queue.put(task)

        for scenario in ['sector']:
            for a in range(0, 50):
                for controller in ['rl_single'
                                   # 'baseline_rl_goal',
                                   # 'goal_regression', 'multiclass'
                                   ]:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a}
                        self.task_queue.put(task)
        '''
        '''
        for scenario in ['sector']:
            for a in range(0, 100):
                for controller in ['rl_goal'
                                    ,'multiclass'
                                    ,'regression_goal'
                                   ]:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a}
                        self.task_queue.put(task)
        '''
        '''
        for scenario in ['sector']:
            for a in range(0, 50):
                for controller in ['rl_goal_no_recovery'
                                    ,'multiclass_no_recovery'
                                    ,'rl_single_no_recovery'
                                    ,'pips_ec_rh_no_recovery'
                                   ]:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a}
                        self.task_queue.put(task)
        '''
        
        '''
        for scenario in ['sector']:
            for a in range(0, 50):
                for controller in [
                                    'pips_ec_rh_no_recovery_5'
                                    ,'pips_ec_rh_no_recovery_25'
                                   ]:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a}
                        self.task_queue.put(task)
        '''
        '''
        for scenario in ['sector']:
            for a in range(0, 50):
                for controller in [
                                    'pips_ec_rh_no_recovery_9'
                                    ,'pips_ec_rh_no_recovery_15'
                                    ,'pips_ec_rh_no_recovery_19'
                                   ]:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a}
                        self.task_queue.put(task)
        '''

        '''
        for scenario in ['sparse','dense']:
            for a in range(0, 50):
                for controller in [
                    'rl_goal_no_recovery'
                    ,'multiclass_no_recovery'
                    ,'dwa_no_recovery'
                    ,'depth_pips_dwa_no_recovery'
                    ,'pips_ec_rh_no_recovery'
                    ,'regression_goal_no_recovery'
                    ,'rl_single_no_recovery'
                    ,'rl_goal_sat_no_recovery'
                    ,'rl_sat_single_no_recovery'
                ]:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a}
                        self.task_queue.put(task)
        '''

        '''
        for scenario in ['dense']:
            for a in range(0, 50):
                for controller in [
                    'baseline_to_goal_no_recovery'
                    , 'teb_no_recovery'
                ]:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a}
                        self.task_queue.put(task)

        for scenario in ['sparse']:
            for a in range(0, 50):
                for controller in [
                    'baseline_to_goal_no_recovery'
                    , 'teb_no_recovery'
                ]:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a}
                        self.task_queue.put(task)
        '''
        '''
        for scenario in ['sparse','dense']:
            for a in range(0, 50):
                for controller in [
                    'pips_ec_rh_no_recovery_5'
                ]:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a}
                        self.task_queue.put(task)
        '''
        '''
        for scenario in ['medium']:
            for a in range(0, 50):
                for controller in [
                    'rl_goal_no_recovery'
                    ,'multiclass_no_recovery'
                    ,'dwa_no_recovery'
                    ,'depth_pips_dwa_no_recovery'
                    ,'pips_ec_rh_no_recovery'
                    ,'regression_goal_no_recovery'
                    ,'rl_single_no_recovery'
                    ,'rl_goal_sat_no_recovery'
                    ,'rl_sat_single_no_recovery'
                    ,'pips_ec_rh_no_recovery_5'
                    ,'baseline_to_goal_no_recovery'
                    ,'teb_no_recovery'
                ]:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a}
                        self.task_queue.put(task)
        '''
        '''
        for scenario in ['medium','dense']: #'sparse',
            for a in range(0, 50):
                for controller in [
                    'rl_goal_no_recovery'
                    ,'multiclass_no_recovery'
                    ,'dwa_no_recovery'
                    ,'depth_pips_dwa_no_recovery'
                    ,'pips_ec_rh_no_recovery'
                    ,'regression_goal_no_recovery'
                    ,'rl_single_no_recovery'
                    ,'rl_goal_sat_no_recovery'
                    ,'rl_sat_single_no_recovery'
                    ,'pips_ec_rh_no_recovery_5'
                    ,'baseline_to_goal_no_recovery'
                    ,'teb_no_recovery'
                ]:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a}
                        self.task_queue.put(task)
        '''
        '''
        for scenario in ['corridor_zigzag','corridor_zigzag_door']:  # 'sparse',
            for a in range(0, 50):
                for controller in [
                    'rl_goal_no_recovery'
                    , 'multiclass_no_recovery'
                    , 'dwa_no_recovery'
                    , 'depth_pips_dwa_no_recovery'
                    , 'pips_ec_rh_no_recovery'
                    , 'regression_goal_no_recovery'
                    , 'rl_single_no_recovery'
                    , 'rl_goal_sat_no_recovery'
                    , 'rl_sat_single_no_recovery'
                    , 'pips_ec_rh_no_recovery_5'
                    , 'baseline_to_goal_no_recovery'
                    , 'teb_no_recovery'
                ]:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a, 'num_obstacles':6, 'min_obstacle_spacing':1.5}
                        self.task_queue.put(task)
        '''
        '''
        for scenario in ['corridor_zigzag_door']:  # 'sparse',
            for a in range(39, 50):
                for controller in [
                    'rl_goal_no_recovery'
                    , 'multiclass_no_recovery'
                    , 'dwa_no_recovery'
                    , 'depth_pips_dwa_no_recovery'
                    , 'pips_ec_rh_no_recovery'
                    , 'regression_goal_no_recovery'
                    , 'rl_single_no_recovery'
                    , 'rl_goal_sat_no_recovery'
                    , 'rl_sat_single_no_recovery'
                    , 'pips_ec_rh_no_recovery_5'
                    , 'baseline_to_goal_no_recovery'
                    , 'teb_no_recovery'
                ]:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a, 'num_obstacles': 6,
                                'min_obstacle_spacing': 1.5}
                        self.task_queue.put(task)
        '''
        '''
        for scenario in ['dense','medium','sparse']:
            for a in range(0, 50):
                for controller in [
                    'rl_goal_no_recovery'
                    , 'multiclass_no_recovery'
                    , 'pips_ec_rh_no_recovery'
                    , 'regression_goal_no_recovery'
                    , 'rl_single_no_recovery'
                    , 'rl_goal_sat_no_recovery'
                    , 'rl_sat_single_no_recovery'
                    , 'pips_ec_rh_no_recovery_5'
                    , 'baseline_to_goal_no_recovery'
                ]:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a}
                        self.task_queue.put(task)
        
        
        for scenario in ['corridor_zigzag','corridor_zigzag_door']:
            for a in range(0, 50):
                for controller in [
                    'rl_goal_no_recovery'
                    , 'multiclass_no_recovery'
                    , 'pips_ec_rh_no_recovery'
                    , 'regression_goal_no_recovery'
                    , 'rl_single_no_recovery'
                    , 'rl_goal_sat_no_recovery'
                    , 'rl_sat_single_no_recovery'
                    , 'pips_ec_rh_no_recovery_5'
                    , 'baseline_to_goal_no_recovery'
                ]:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a, 'num_obstacles': 6,
                                'min_obstacle_spacing': 1.5}
                        self.task_queue.put(task)
        '''
        '''
        for scenario in ['sparse']:
            for a in range(44, 50):
                for controller in [
                    'rl_goal_no_recovery'
                    , 'multiclass_no_recovery'
                    , 'pips_ec_rh_no_recovery'
                    , 'regression_goal_no_recovery'
                    , 'rl_single_no_recovery'
                    , 'rl_goal_sat_no_recovery'
                    , 'rl_sat_single_no_recovery'
                    , 'pips_ec_rh_no_recovery_5'
                    , 'baseline_to_goal_no_recovery'
                ]:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a}
                        self.task_queue.put(task)
        '''
        '''
        for scenario in ['corridor_zigzag','corridor_zigzag_door']:
            for a in range(0, 50):
                for controller in [
                        'rl_goal_combo_no_recovery'
                        ,'rl_goal_combo_no_recovery_3_5'
                        ,'rl_goal_combo_no_recovery_5_3'
                        ,'rl_goal_combo_no_recovery_3_3'
                     ]:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a, 'num_obstacles': 6,
                                'min_obstacle_spacing': 1.5}
                        self.task_queue.put(task)
                        
        for scenario in ['medium','sparse']:
            for a in range(0, 50):
                for controller in [
                        'rl_goal_combo_no_recovery'
                        ,'rl_goal_combo_no_recovery_3_5'
                        ,'rl_goal_combo_no_recovery_5_3'
                        ,'rl_goal_combo_no_recovery_3_3'
                     ]:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a}
                        self.task_queue.put(task)
        '''
        '''
        for scenario in ['corridor_zigzag', 'corridor_zigzag_door']:
            for a in range(40, 50):
                for controller in [
                    'rl_goal_no_recovery'
                    , 'multiclass_no_recovery'
                    , 'dwa_no_recovery'
                    , 'depth_pips_dwa_no_recovery'
                    , 'pips_ec_rh_no_recovery'
                    , 'regression_goal_no_recovery'
                    , 'rl_single_no_recovery'
                    , 'rl_goal_sat_no_recovery'
                    , 'rl_sat_single_no_recovery'
                    , 'pips_ec_rh_no_recovery_5'
                    , 'baseline_to_goal_no_recovery'
                    , 'teb_no_recovery'
                ]:
                    for repetition in range(1):
                        task = {'scenario': scenario, 'controller': controller, 'seed': a, 'num_obstacles': 6,
                                'min_obstacle_spacing': 1.5}
                        self.task_queue.put(task)
        '''


        # for scenario in ['sector']:
        #     for controller in ['dwa']:
        #         for seed in range(0, 50):
        #             task= {'scenario': scenario, 'controller':controller, 'seed':seed, 'robot':'pioneer'}
        #             self.task_queue.put(task)

        # for scenario in ['campus']:
        #     for seed in range(0, 50):
        #         for controller in ['dwa', 'egocylindrical_pips_dwa']:
        #             task= {'scenario': scenario, 'controller':controller, 'seed':seed, 'robot':'pioneer', 'min_obstacle_spacing': 1.5, 'num_obstacles': 50}
        #             self.task_queue.put(task)

        # for scenario in ['sector_laser','sector_extra','campus_obstacle','fourth_floor_obstacle']:

        for scenario in ['sector_extra','sector_laser']:
            for controller in ['egocylindrical_pips_dwa']:
                for seed in range(0, 50):
                    task= {'scenario': scenario, 'controller':controller, 'seed':seed, 'robot':'turtlebot', 'min_obstacle_spacing': 1, 'num_obstacles': 30, 'controller_args':{'sim_time':2}}
                    self.task_queue.put(task)

        for scenario in ['campus_obstacle','fourth_floor_obstacle']:
            for controller in ['egocylindrical_pips_dwa']:
                for seed in range(0, 50):
                    task= {'scenario': scenario, 'controller':controller, 'seed':seed, 'robot':'turtlebot', 'min_obstacle_spacing': 1, 'num_obstacles': 50, 'controller_args':{'sim_time':2}}
                    self.task_queue.put(task)

        for scenario in ['sector_extra','sector_laser']:
            for controller in ['egocylindrical_pips_dwa']:
                for seed in range(0, 50):
                    task= {'scenario': scenario, 'controller':controller, 'seed':seed, 'robot':'box_turtle', 'min_obstacle_spacing': 1.2, 'num_obstacles': 30, 'controller_args':{'sim_time':2}}
                    self.task_queue.put(task)

        for scenario in ['campus_obstacle','fourth_floor_obstacle']:
            for controller in ['egocylindrical_pips_dwa']:
                for seed in range(0, 50):
                    task= {'scenario': scenario, 'controller':controller, 'seed':seed, 'robot':'box_turtle', 'min_obstacle_spacing': 1.2, 'num_obstacles': 50, 'controller_args':{'sim_time':2}}
                    self.task_queue.put(task)

        # for scenario in ['stereo_sector_extra','stereo_sector_laser']:
        #     for controller in ['dwa','stereo_egocylindrical_pips_dwa']:
        #         for seed in range(0, 50):
        #             task= {'scenario': scenario, 'controller':controller, 'seed':seed, 'robot':'turtlebot', 'min_obstacle_spacing': 1, 'num_obstacles': 30, 'controller_args':{'sim_time':2}}
        #             self.task_queue.put(task)
        #
        # for scenario in ['stereo_campus_obstacle','stereo_fourth_floor_obstacle']:
        #     for controller in ['dwa','stereo_egocylindrical_pips_dwa']:
        #         for seed in range(0, 50):
        #             task= {'scenario': scenario, 'controller':controller, 'seed':seed, 'robot':'turtlebot', 'min_obstacle_spacing': 1, 'num_obstacles': 50, 'controller_args':{'sim_time':2}}
        #             self.task_queue.put(task)
        #
        # for scenario in ['stereo_sector_extra', 'stereo_sector_laser']:
        #     for controller in ['dwa','stereo_egocylindrical_pips_dwa']:
        #         for seed in range(0, 50):
        #             task = {'scenario': scenario, 'controller': controller, 'seed': seed, 'robot': 'box_turtle',
        #                     'min_obstacle_spacing': 1.2, 'num_obstacles': 30,
        #                     'controller_args': {'sim_time': 2}}
        #             self.task_queue.put(task)
        #
        # for scenario in ['stereo_campus_obstacle', 'stereo_fourth_floor_obstacle']:
        #     for controller in ['dwa','stereo_egocylindrical_pips_dwa']:
        #         for seed in range(0, 50):
        #             task = {'scenario': scenario, 'controller': controller, 'seed': seed, 'robot': 'box_turtle',
        #                     'min_obstacle_spacing': 1.2, 'num_obstacles': 50,
        #                     'controller_args': {'sim_time': 2}}
        #             self.task_queue.put(task)


        # task = {'scenario': 'sector_laser', 'controller': 'egocylindrical_pips_dwa', 'seed': 0, 'robot': 'turtlebot'}
        # self.task_queue.put(task)
        #
        # for sim_time in [1,2,3]:
        #     task = {'scenario': 'sector_laser', 'controller': 'egocylindrical_pips_dwa', 'seed': 0, 'robot': 'turtlebot', 'controller_args':{'sim_time':sim_time}}
        #     self.task_queue.put(task)

        # for scenario in ['stereo_sector_extra', 'stereo_sector_laser']:
        #     for sim_time in [1, 2, 3, 4]:
        #         for controller in ['stereo_egocylindrical_pips_dwa']:
        #             for seed in range(0, 50):
        #                 task = {'scenario': scenario, 'controller': controller, 'seed': seed, 'robot': 'turtlebot',
        #                         'min_obstacle_spacing': 1, 'num_obstacles': 30, 'controller_args':{'sim_time':sim_time}}
        #                 self.task_queue.put(task)
        #
        # for scenario in ['stereo_campus_obstacle', 'stereo_fourth_floor_obstacle']:
        #     for sim_time in [1, 2, 3, 4]:
        #         for controller in ['stereo_egocylindrical_pips_dwa']:
        #             for seed in range(0, 50):
        #                 task = {'scenario': scenario, 'controller': controller, 'seed': seed, 'robot': 'turtlebot',
        #                         'min_obstacle_spacing': 1, 'num_obstacles': 50, 'controller_args':{'sim_time':sim_time}}
        #                 self.task_queue.put(task)


    #This list should be elsewhere, possibly in the configs package
    def addTasks10(self):
        controllers = ["pips_dwa", "pips_dwa_propagated"] #,"dwa", "teb"["brute_force"] #
        barrel_arrangements = [3,5,7]

        for a in range(0,500):
            for num_barrels in barrel_arrangements:
                for controller in controllers:
                    for repetition in range(1):
                        task = {'scenario': 'trashcans', 'num_barrels': num_barrels, 'controller': controller, 'seed': a}
                        self.task_queue.put(task)

    def addTasks1(self):
        controllers = ["pips_dwa", "octo_dwa", "teb"]
        barrel_arrangements = [3,5,7]

        for a in range(100):
            for num_barrels in barrel_arrangements:
                for controller in controllers:
                    for repetition in range(4):
                        task = {'scenario': 'trashcans', 'num_barrels': num_barrels, 'controller': controller, 'seed': a}
                        self.task_queue.put(task)

    def addTasks3(self):
        task = {'scenario': 'trashcans', 'num_barrels': 3, 'controller': 'octo_dwa', 'seed': 54}
        for _ in range(5):
            self.task_queue.put(task)

    def addTasks1(self):
        controllers = ["pips_dwa", "octo_dwa", "teb"]

        for i in range(100):
            for j in range(0,7): #[1,2,5,6]:
                for controller in controllers:

                    task = {'scenario': 'campus', 'num_barrels': 20, 'controller': controller, 'seed': 25+ i, 'target_id': j}

                    self.task_queue.put(task)


class GazeboMaster(mp.Process):
    def __init__(self, task_queue, result_queue, kill_flag, soft_kill_flag, ros_port, gazebo_port, gazebo_launch_mutex, **kwargs):
        super(GazeboMaster, self).__init__()
        self.daemon = False

        self.task_queue = task_queue
        self.result_queue = result_queue
        self.ros_port = ros_port
        self.gazebo_port = gazebo_port
        self.gazebo_launch_mutex = gazebo_launch_mutex
        self.core = None
        self.gazebo_launch = None
        self.controller_launch = None
        self.gazebo_driver = None
        self.current_world = None
        self.kill_flag = kill_flag
        self.soft_kill_flag = soft_kill_flag
        self.is_shutdown = False
        self.had_error = False

        self.gui = True


        print "New master"

        self.ros_master_uri = "http://localhost:" + str(self.ros_port)
        self.gazebo_master_uri = "http://localhost:" + str(self.gazebo_port)
        os.environ["ROS_MASTER_URI"] = self.ros_master_uri
        os.environ["GAZEBO_MASTER_URI"]= self.gazebo_master_uri

        if self.gui==False:
            if 'DISPLAY' in os.environ:
                del os.environ['DISPLAY']   #To ensure that no GUI elements of gazebo activated
        else:
            if 'DISPLAY' not in os.environ:
                os.environ['DISPLAY']=':0'



    def run(self):
        while not self.is_shutdown and not self.had_error:
            self.process_tasks()
            time.sleep(5)
            if not self.is_shutdown:
                print >> sys.stderr, "(Not) Relaunching on " + str(os.getpid()) + ", ROS_MASTER_URI=" + self.ros_master_uri
        print "Run totally done"

    def process_tasks(self):
        self.roslaunch_core()
        rospy.set_param('/use_sim_time', 'True')
        rospy.init_node('test_driver', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        scenarios = TestingScenarios()

        self.had_error = False

        while not self.is_shutdown and not self.had_error:
            # TODO: If fail to run task, put task back on task queue
            try:

                task = self.task_queue.get(block=False)

                scenario = scenarios.getScenario(task)

                if scenario is not None:


                    self.roslaunch_gazebo(scenario.getGazeboLaunchFile(task["robot"])) #pass in world info
                    #time.sleep(30)

                    if not self.gazebo_launch._shutting_down:

                        controller_args = task["controller_args"] if "controller_args" in task else {}

                        try:

                            scenario.setupScenario()
                            self.roslaunch_controller(task["robot"], task["controller"], controller_args)
                            task.update(controller_args)    #Adding controller arguments to main task dict for easy logging

                            print "Running test..."

                            #master = rosgraph.Master('/mynode')

                            #TODO: make this a more informative type
                            result = test_driver.run_test(goal_pose=scenario.getGoal())

                        except rospy.ROSException as e:
                            result = "ROSException: " + str(e)
                            task["error"]= True
                            self.had_error = True

                        self.controller_launch.shutdown()

                    else:
                        result = "gazebo_crash"
                        task["error"] = True
                        self.had_error = True

                else:
                    result = "bad_task"

                if isinstance(result, dict):
                    task.update(result)
                else:
                    task["result"] = result
                task["pid"] = os.getpid()
                self.return_result(task)

                if self.had_error:
                    print >> sys.stderr, result


            except Queue.Empty, e:
                with self.soft_kill_flag.get_lock():
                    if self.soft_kill_flag.value:
                        self.shutdown()
                        print "Soft shutdown requested"
                time.sleep(1)


            with self.kill_flag.get_lock():
                if self.kill_flag.value:
                    self.shutdown()

        print "Done with processing, killing launch files..."
        # It seems like killing the core should kill all of the nodes,
        # but it doesn't
        if self.gazebo_launch is not None:
            self.gazebo_launch.shutdown()

        if self.controller_launch is not None:
            self.controller_launch.shutdown()

        print "GazeboMaster shutdown: killing core..."
        self.core.shutdown()
        #self.core.kill()
        #os.killpg(os.getpgid(self.core.pid), signal.SIGTERM)
        print "All cleaned up"


    def start_core(self):

        #env_prefix = "ROS_MASTER_URI="+ros_master_uri + " GAZEBO_MASTER_URI=" + gazebo_master_uri + " "

        my_command = "roscore -p " + str(self.ros_port)

        #my_env = os.environ.copy()
        #my_env["ROS_MASTER_URI"] = self.ros_master_uri
        #my_env["GAZEBO_MASTER_URI"] = self.gazebo_master_uri

        print "Starting core..."
        self.core = subprocess.Popen(my_command.split()) # preexec_fn=os.setsid
        print "Core started! [" + str(self.core.pid) + "]"


    def roslaunch_core(self):

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        #roslaunch.configure_logging(uuid)

        self.core = roslaunch.parent.ROSLaunchParent(
            run_id=uuid, roslaunch_files=[],
            is_core=True, port=self.ros_port
        )
        self.core.start()

    def roslaunch_controller(self, robot, controller_name, controller_args={}):

        #controller_path =

        rospack = rospkg.RosPack()
        path = rospack.get_path("nav_scripts")

        # We'll assume Gazebo is launched are ready to go

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
        #roslaunch.configure_logging(uuid)
        #print path

        #Remapping stdout to /dev/null
        sys.stdout = open(os.devnull, "w")

        for key,value in controller_args.items():
            var_name = "GM_PARAM_"+ key.upper()
            value = str(value)
            os.environ[var_name] = value
            print("Setting environment variable [" + var_name + "] to '" + value + "'")

        self.controller_launch = roslaunch.parent.ROSLaunchParent(
            run_id=uuid, roslaunch_files=[path + "/launch/" + robot + "_" + controller_name + "_controller.launch"],
            is_core=False, port=self.ros_port #, roslaunch_strs=controller_args
        )
        self.controller_launch.start()

        sys.stdout = sys.__stdout__


    def roslaunch_gazebo(self, world):
        if world == self.current_world:
            if not self.gazebo_launch._shutting_down:
                return
            else:
                print "Gazebo crashed, restarting"

        if self.gazebo_launch is not None:
            self.gazebo_launch.shutdown()

        self.current_world = world


        # This will wait for a roscore if necessary, so as long as we detect any failures
        # in start_roscore, we should be fine
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
        #roslaunch.configure_logging(uuid) #What does this do?
        #print path

        #Without the mutex, we frequently encounter this problem:
        # https://bitbucket.org/osrf/gazebo/issues/821/apparent-transport-race-condition-on
        with self.gazebo_launch_mutex:
            self.gazebo_launch = roslaunch.parent.ROSLaunchParent(
                run_id=uuid, roslaunch_files=[world],
                is_core=False, port=self.ros_port
            )
            self.gazebo_launch.start()

        try:
            msg = rospy.wait_for_message("/odom", Odometry, 30)
        except rospy.exceptions.ROSException:
            print "Error! odometry not received!"
            return False

        return True

    def shutdown(self):
        self.is_shutdown = True

    # TODO: add conditional logic to trigger this
    def task_error(self, task):
        self.task_queue.put(task)
        self.task_queue.task_done()
        self.shutdown()

    def return_result(self,result):
        print "Returning completed task: " + str(result)
        self.result_queue.put(result)
        self.task_queue.task_done()




if __name__ == "__main__":

    start_time = time.time()
    master = MultiMasterCoordinator()
    master.start()
    master.addTasks()
    master.waitToFinish()
    master.shutdown()
    end_time = time.time()
    print "Total time: " + str(end_time - start_time)

