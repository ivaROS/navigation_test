#!/usr/bin/env python3

from __future__ import print_function
from future import standard_library
standard_library.install_aliases()
from builtins import str
from builtins import range
from builtins import object
import multiprocessing as mp
import os
import sys
import rospkg
import roslaunch
import time
import nav_scripts.movebase_driver as test_driver
import threading
import queue
from ctypes import c_bool

import signal
import itertools
import socket
import contextlib

from nav_scripts.testing_scenarios import TestingScenarios, TestingScenarioError
from nav_scripts.controller_launcher import ControllerLauncher
from nav_scripts.ros_launcher_helper import GazeboLauncher, RobotLauncher, RoscoreLauncher, LauncherErrorCatcher, RosLauncherMonitor
from nav_scripts.task_pipeline import TaskProcessingPipeline, ResultRecorder, Worker, TaskProcessingException, ExceptionLevels

import rospy
import csv
import datetime
import traceback

class ConsoleResultRecorder(ResultRecorder):
    def __init__(self):
        super(ConsoleResultRecorder, self).__init__()
        self.result_list = []

    def process_task(self, task):
        result_string = "Result of ["
        for k, v in list(task.items()):
            result_string += str(k) + ":" + str(v) + ","
        result_string += "]"
        print(result_string)

        self.result_list.append(result_string)


class CSVResultRecorder(ConsoleResultRecorder):

    def __init__(self):
        super(CSVResultRecorder, self).__init__()

    def run(self):
        outputfile_name = "~/simulation_data/results_" + str(datetime.datetime.now())
        outputfile_name = os.path.expanduser(outputfile_name)

        with open(outputfile_name, 'w') as self.csvfile:
            seen = set()
            fieldnames = [x for x in self.fieldnames if not (x in seen or seen.add(
                x))]  # http://www.martinbroadhurst.com/removing-duplicates-from-a-list-while-preserving-order-in-python.html

            self.datawriter = csv.DictWriter(self.csvfile, fieldnames=fieldnames, restval='', extrasaction='ignore')
            self.datawriter.writeheader()
            super(CSVResultRecorder, self).run()

    def process_task(self, task):
        super(CSVResultRecorder, self).process_task(task=task)
        self.datawriter.writerow(task)
        self.csvfile.flush()



class MultiMasterCoordinator(TaskProcessingPipeline):
    def __init__(self, num_masters=1, save_results = True, use_existing_roscore=False):
        super(MultiMasterCoordinator, self).__init__()

        self.started = False

        if num_masters != 1 and use_existing_roscore:
            raise ValueError("You cannot run more than 1 instance of GazeboMaster when use_existing_roscore is enabled!")

        self.num_masters = num_masters
        self.use_existing_roscore = use_existing_roscore
        self.save_results = save_results

        self.fieldnames = ["controller"]
        self.fieldnames.extend(TestingScenarios.getFieldNames())
        self.fieldnames.extend(["pid","result","time","path_length","robot","total_rotation"])
        #self.fieldnames.extend(["sim_time", "obstacle_cost_mode", "sum_scores"])
        self.fieldnames.extend(["bag_file_path", 'global_planning_freq', 'controller_freq', 'num_inferred_paths', 'num_paths', 'enable_cc', 'gazebo_gui', 'record', 'global_potential_weight', 'timeout', 'bash_source_file']) #,'converter', 'costmap_converter_plugin', 'global_planning_freq', 'feasibility_check_no_poses', 'simple_exploration', 'weight_gap', 'gap_boundary_exponent', 'egocircle_early_pruning', 'gap_boundary_threshold', 'gap_boundary_ratio', 'feasibility_check_no_tebs', 'gap_exploration', 'gap_h_signature', ])

        super(MultiMasterCoordinator,self).setup_stages(num_workers=num_masters)


    def start(self):
        #TODO: find a cleaner way to ensure that the task_input and result_recorder are consistent in fieldnames
        self.result_recorder.fieldnames = self.fieldnames
        super(MultiMasterCoordinator, self).start()

    def get_worker(self, num):
        return GazeboMaster(num=num, use_existing_roscore=self.use_existing_roscore)

    def get_result_recorder(self):
        return CSVResultRecorder() if self.save_results else ConsoleResultRecorder()

    #TODO: maybe replace all 'wait_for_finish' functions with 'wait_to_finish' to avoid confusion
    def wait_to_finish(self, source="Unknown"):
        super(MultiMasterCoordinator, self).wait_for_finish(source=source)

    def add_tasks(self, tasks):
        return super(MultiMasterCoordinator, self).add_tasks(tasks=tasks)

        if False:
            if(not self.started):
                self.add_task_fieldnames(tasks=tasks)

                print("Adding tasks asynchronously in separate thread...")
                self.task_thread = threading.Thread(target=self.add_tasks_impl,args=[tasks])
                self.task_thread.daemon=True
                self.task_thread.start()
            else:
                print("Adding tasks...")
                self.add_tasks_impl(tasks=tasks)

    def add_tasks_impl(self, tasks):
        warned_keys = set()
        num_skipped = 0

        def good_keys(dictionary):
            good_task = True

            for key in dictionary:
                if isinstance(dictionary[key],dict):
                    if not good_keys(dictionary[key]):
                        good_task = False
                elif key not in self.fieldnames:
                    good_task = False
                    if key not in warned_keys:
                        warned_keys.add(key)
                        print("Key [" + key + "] is not in fieldnames! Skipping all tasks that include it", file=sys.stderr)
            return good_task

        for task in tasks:
            if good_keys(task):
                self.task_queue.put(task)
            else:
                num_skipped +=1

        print("Finished adding tasks. Skipped [" + str(num_skipped) + "] tasks.")

    #TODO:
    # tasks must be iterable and finite length
    def add_task_fieldnames(self, tasks):
        if self.started:
            print("Warning! You cannot add task fieldnames once the MultiMasterCoordinator has been started! No fieldnames will be added!", file=sys.stderr)
            return

        added_keys = set()

        def add_keys(dictionary):
            for key in dictionary:
                if isinstance(dictionary[key], dict):
                    add_keys(dictionary[key])
                elif key not in self.fieldnames:
                    if key not in added_keys:
                        added_keys.add(key)

        for task in tasks:
            add_keys(task)

        print("Finished adding missing task fieldnames:" + str(added_keys))


class GazeboMaster(Worker):
    def __init__(self, num, use_existing_roscore):
        super(GazeboMaster, self).__init__(num=num)
        self.use_existing_roscore = use_existing_roscore

    ##NOTE: Due to the use of certain class variables and writing of environment variables, this needs to happen in the new process and can't be done in __init__
    def setup(self):
        self.roscore = RoscoreLauncher(use_existing_roscore=self.use_existing_roscore)
        self.robot_launcher = RobotLauncher()
        self.gazebo_launcher = GazeboLauncher(robot_launcher=self.robot_launcher)
        self.controller_launcher = ControllerLauncher()
        self.monitor = RosLauncherMonitor(
            launchers=[self.roscore, self.robot_launcher, self.gazebo_launcher, self.controller_launcher])

        self.gui = True

        print("New master")

        # if 'SIMULATION_RESULTS_DIR' in os.environ:

        # Disabling all GUI elements of Gazebo decreases simulation load, but also disables cameras. However, it is not currently clear if worlds without cameras benefit
        if self.gui == False:
            if 'DISPLAY' in os.environ:
                del os.environ['DISPLAY']  # To ensure that no GUI elements of gazebo activated
        else:
            if 'DISPLAY' not in os.environ:
                os.environ['DISPLAY'] = ':0'

    def run(self):
        #TODO: Add top level loop to allow restarting if roscore dies or something like that
        self.setup()
        self.monitor.append(self.interrupt_monitor)

        #Starts roscore if needed; does not launch anything else, but ensures they all get shutdown properly
        with self.roscore, self.gazebo_launcher, self.robot_launcher:
            rospy.set_param('/use_sim_time', 'True')
            rospy.init_node('test_driver', anonymous=True, disable_signals=True, disable_rosout=True)
            # rospy.on_shutdown(self.shutdown)
            self.scenarios = TestingScenarios()
            self.had_error = False
            super(GazeboMaster, self).run()

        print("Run totally done")


    def process_task(self, task):
        rospy.loginfo("Got next task [" + str(task) + "]")
        try:
            result = self.task_result_func(task=task)
        except TaskProcessingException as e:
            result = {"result": "ERROR", "error_details": str(e)}
        except Exception as e:
            print(str(e))
            result = {"result": "UNEXPECTED_ERROR", "error_details": str(e)}
        finally:
            task["worker"] = self.name

            if isinstance(result, dict):
                task.update(result)
            else:
                task["result"] = result
            task["pid"] = os.getpid()

            self.output_queue.put(task)



    def task_result_func(self, task):
        with get_scenario_helper(self, task) as scenario: #Currently no reason to use as context manager
            with scenario.launch_gazebo() as gazebo:  #start it if needed, close it if error
                gazebo.launch()
                with scenario.launch_robot() as robot: #same here
                    robot.launch()
                    with scenario.setup():
                        with scenario.launch_controller() as controller:
                            controller.launch()
                            with scenario.setup():
                                result = scenario.run()
                                return result


def get_scenario_helper(self, task):
    class LauncherArgHelper(object):
        def __init__(myself):
            myself.name = type(myself).name
            myself.launcher = type(myself).launcher
            myself.init_value()
            myself.init_args()
            myself.update_task_args()

        def init_value(myself):
            try:
                myself.value = task[myself.name]
            except KeyError as e:
                raise TestingScenarioError(msg="Task is missing required key! " + str(e), exc_level=ExceptionLevels.BAD_CONFIG) from e

        def init_args(myself):
            myself.arg_name = myself.name + "_args"
            try:
                myself.args = task[myself.arg_name]
            except KeyError:
                myself.args = {}

        def update_task_args(myself):
            try:
                task[myself.arg_name].update(myself.args)
            except KeyError:
                task[myself.arg_name] = myself.args

        def launch(myself, **kwargs):
            base_kwargs = {myself.name: myself.value, myself.arg_name: myself.args}
            base_kwargs.update(kwargs)
            myself.launcher.launch(**base_kwargs)
            #myself.launcher.launch(myself.value, myself.args)

        def __enter__(myself):
            #myself.launcher.__enter__()
            return myself

        #The way using this system, don't want to close launchers on exit
        def __exit__(myself, exc_type, exc_val, exc_tb):
            if exc_type is not None and issubclass(exc_type, type(myself.launcher).exc_type):
                print("Caught error from [" + str(myself.launcher.name) + "], shutting it down. Error was:\n" + str(exc_val) + "\n" + ''.join(
                    traceback.format_exception(None, value=exc_val, tb=exc_tb)))
                myself.launcher.shutdown()

    class ScenarioHelper(object):

        def __init__(myself):
            myself.scenario = self.scenarios.getScenario(task)

        def __enter__(myself):
            return myself

        def __exit__(myself, exc_type, exc_val, exc_tb):
            pass

        def launch_gazebo(myself):
            class GazeboHelper(LauncherArgHelper):
                name = "world"
                launcher = self.gazebo_launcher

                def init_value(gzself):
                    try:
                        gzself.value = myself.scenario.getGazeboLaunchFile()
                    except Exception as e:
                        raise TestingScenarioError(msg="Unable to get gazebo launch file for scenario!", exc_level=ExceptionLevels.BAD_CONFIG, task=task)

            myself.gazebo = GazeboHelper()
            return myself.gazebo

        def launch_robot(myself):
            class RobotHelper(LauncherArgHelper):
                name = "robot"
                launcher = self.robot_launcher

            myself.robot = RobotHelper()
            return myself.robot

        def launch_controller(myself):
            class ControllerHelper(LauncherArgHelper):
                name = "controller"
                launcher = self.controller_launcher

                def launch(gzself):
                    gzself.launcher.launch(robot=myself.robot.value, controller_name=gzself.value, controller_args=gzself.args)

                def __exit__(gzself, exc_type, exc_val,
                             exc_tb):  # NOTE: could these be replaced by 'args' and/or 'kwargs'?
                    gzself.launcher.__exit__(exc_type=exc_type, exc_val=exc_val, exc_tb=exc_tb)

            myself.controller = ControllerHelper()
            return myself.controller

        def setup(myself):
            class ScenarioSetup(object):

                def __enter__(gzself):
                    myself.scenario.setupScenario()

                def __exit__(gzself, exc_type, exc_val, exc_tb):  # NOTE: could these be replaced by 'args' and/or 'kwargs'?
                    if isinstance(exc_val, (rospy.ROSException, rospy.ServiceException)):
                        raise TestingScenarioError(str(exc_val)) from exc_val
                    elif exc_val is None:
                        try:
                            myself.scenario.cleanup()
                        except (rospy.ROSException, rospy.ServiceException) as e:
                            """
                            NOTE: In this situation, we would ideally shut some things down, but we don't want to 
                            lose the result. One option might be to place the result in the exception, 
                            and check for it when handling the exception
                            """
                            #raise TestingScenarioError(str(e)) from e
                            print(str(e))

            return ScenarioSetup()

        def run(myself):
            timeout = task["timeout"] if "timeout" in task else None
            record = task["record"] if "record" in task else False
            result = test_driver.run_test(goal_pose=myself.scenario.getGoalMsg(), record=record, timeout=timeout,
                                          monitor=self.monitor)
            return result

    return ScenarioHelper()