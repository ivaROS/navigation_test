#!/usr/bin/env python

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

from nav_scripts.testing_scenarios import TestingScenarios
from nav_scripts.controller_launcher import ControllerLauncher

from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock as ClockMsg
from sensor_msgs.msg import JointState
import rospy
import csv
import datetime

from roslaunch.node_args import _rospack


def port_in_use(port):
    with contextlib.closing(socket.socket(socket.AF_INET, socket.SOCK_STREAM)) as sock:
        if sock.connect_ex(('127.0.0.1', port)) == 0:
            print("Port " + str(port) + " is in use")
            return True
        else:
            print("Port " + str(port) + " is not in use")
            return False



class MultiMasterCoordinator(object):
    def __init__(self, num_masters=1, save_results = True, use_existing_roscore=False):
        signal.signal(signal.SIGINT, self.signal_shutdown)
        signal.signal(signal.SIGTERM, self.signal_shutdown)
        self.children_shutdown = mp.Value(c_bool, False)
        self.soft_shutdown = mp.Value(c_bool, False)

        self.should_shutdown = False
        self.started = False

        if num_masters != 1 and use_existing_roscore:
            raise ValueError("You cannot run more than 1 instance of GazeboMaster when use_existing_roscore is enabled!")

        self.num_masters = num_masters
        self.use_existing_roscore = use_existing_roscore

        self.save_results = save_results
        self.task_queue_capacity = 2000 #2*self.num_masters
        self.task_queue = mp.JoinableQueue(maxsize=self.task_queue_capacity)
        self.result_queue_capacity = 2000 #*self.num_masters
        self.result_queue = mp.JoinableQueue(maxsize=self.result_queue_capacity)
        self.gazebo_masters = []
        self.result_list = []
        self.gazebo_launch_mutex = mp.Lock()

        self.fieldnames = ["controller"]
        self.fieldnames.extend(TestingScenarios.getFieldNames())
        self.fieldnames.extend(["pid","result","time","path_length","robot","total_rotation"])
        #self.fieldnames.extend(["sim_time", "obstacle_cost_mode", "sum_scores"])
        self.fieldnames.extend(["bag_file_path", 'global_planning_freq', 'controller_freq', 'num_inferred_paths', 'num_paths', 'enable_cc', 'gazebo_gui', 'record', 'global_potential_weight', 'timeout', 'bash_source_file']) #,'converter', 'costmap_converter_plugin', 'global_planning_freq', 'feasibility_check_no_poses', 'simple_exploration', 'weight_gap', 'gap_boundary_exponent', 'egocircle_early_pruning', 'gap_boundary_threshold', 'gap_boundary_ratio', 'feasibility_check_no_tebs', 'gap_exploration', 'gap_h_signature', ])


    def start(self):
        self.started=True
        self.startResultsProcessing()
        self.startProcesses()

    def startResultsProcessing(self):
        self.result_thread = threading.Thread(target=self.processResults,args=[self.result_queue])
        self.result_thread.daemon=True
        self.result_thread.start()

    def startProcesses(self):
        self.ros_port = 11311
        self.gazebo_port = self.ros_port + 100
        for ind in range(self.num_masters):
            self.addProcess()


    def addProcess(self):
        if not self.use_existing_roscore:
            while port_in_use(self.ros_port):
                self.ros_port += 1

        while port_in_use(self.gazebo_port):
            self.gazebo_port += 1

        gazebo_master = GazeboMaster(self.task_queue, self.result_queue, self.children_shutdown, self.soft_shutdown, self.ros_port,
                                     self.gazebo_port, self.use_existing_roscore, gazebo_launch_mutex=self.gazebo_launch_mutex)
        gazebo_master.start()
        self.gazebo_masters.append(gazebo_master)

        self.ros_port += 1
        self.gazebo_port += 1

        time.sleep(1)


    def processResults(self,my_queue):

        outputfile_name = "~/simulation_data/results_" + str(datetime.datetime.now())
        outputfile_name = os.path.expanduser(outputfile_name)

        with open(outputfile_name, 'w') as csvfile:
            seen = set()
            fieldnames = [x for x in self.fieldnames if not (x in seen or seen.add(x))] #http://www.martinbroadhurst.com/removing-duplicates-from-a-list-while-preserving-order-in-python.html

            datawriter = csv.DictWriter(csvfile, fieldnames=fieldnames, restval='', extrasaction='ignore')
            datawriter.writeheader()

            while not self.should_shutdown: #This means that results stop getting saved to file as soon shutdown is commanded
                try:
                    task = my_queue.get(block=False)

                    result_string = "Result of ["
                    for k,v in list(task.items()):
                        #if "result" not in k:
                            result_string+= str(k) + ":" + str(v) + ","
                    result_string += "]"

                    print(result_string)

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
                    my_queue.task_done()
                except queue.Empty as e:
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

    def wait_to_finish(self):
        print("Waiting until everything done!")
        self.task_queue.join()
        print("All tasks processed!")
        with self.soft_shutdown.get_lock():
            self.soft_shutdown.value = True

        #The problem is that this won't happen if things end prematurely...
        self.result_queue.join()
        print("All results processed!")

    def add_tasks(self, tasks):
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


class GazeboMaster(mp.Process):
    def __init__(self, task_queue, result_queue, kill_flag, soft_kill_flag, ros_port, gazebo_port, use_existing_roscore, gazebo_launch_mutex, **kwargs):
        super(GazeboMaster, self).__init__()
        self.rospack = rospkg.RosPack()
        self.rospack_caches = {}

        self.daemon = False

        self.task_queue = task_queue
        self.result_queue = result_queue
        self.use_existing_roscore = use_existing_roscore
        self.ros_port = ros_port
        self.gazebo_port = gazebo_port
        self.gazebo_launch_mutex = gazebo_launch_mutex
        self.core = None
        self.gazebo_launch = None
        self.controller_launch = None
        self.robot_launch = None
        self.current_robot = None
        self.gazebo_driver = None
        self.current_world = None
        self.current_world_args = None
        self.kill_flag = kill_flag
        self.soft_kill_flag = soft_kill_flag
        self.is_shutdown = False
        self.had_error = False

        self.gui = True


        print("New master")

        self.ros_master_uri = "http://localhost:" + str(self.ros_port)
        self.gazebo_master_uri = "http://localhost:" + str(self.gazebo_port)
        os.environ["ROS_MASTER_URI"] = self.ros_master_uri
        os.environ["GAZEBO_MASTER_URI"]= self.gazebo_master_uri

        #if 'SIMULATION_RESULTS_DIR' in os.environ:

        # Disabling all GUI elements of Gazebo decreases simulation load, but also disables cameras. However, it is not currently clear if worlds without cameras benefit
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
                print("(Not) Relaunching on " + str(os.getpid()) + ", ROS_MASTER_URI=" + self.ros_master_uri, file=sys.stderr)
        print("Run totally done")

    def process_tasks(self):
        if not self.use_existing_roscore:
            self.roslaunch_core()
        rospy.set_param('/use_sim_time', 'True')
        rospy.init_node('test_driver', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        scenarios = TestingScenarios()

        self.had_error = False

        while not self.is_shutdown and not self.had_error:
            # TODO: If fail to run task, put task back on task queue
            try:
                rospy.loginfo("Trying to get next task...")
                task = self.task_queue.get(block=False)
                rospy.loginfo("Got next task [" + str(task) + "]")

                scenario = scenarios.getScenario(task)

                if scenario is not None:

                    #TODO: handle failure to launch gazebo
                    world_args = task["world_args"] if "world_args" in task else {}
                    world_args.update(scenario.getWorldArgs())
                    self.roslaunch_gazebo(scenario.getGazeboLaunchFile(), world_args=world_args) #pass in world info
                    if world_args is not None:
                        task.update(world_args)
                    if "robot" in task and task['robot'] is not None:
                        robot_args = task["robot_args"] if "robot_args" in task else {}
                        self.roslaunch_robot(task["robot"], robot_args=robot_args)
                        task.update(robot_args)

                        if task["controller"] is None:
                            result = "nothing"
                        elif not self.gazebo_launch._shutting_down:

                            controller_args = task["controller_args"] if "controller_args" in task else {}

                            try:

                                scenario.setupScenario()
                                self.roslaunch_controller(task["robot"], task["controller"], controller_args)
                                task.update(controller_args)    #Adding controller arguments to main task dict for easy logging

                                print("Running test...")

                                #master = rosgraph.Master('/mynode')

                                record = task["record"] if "record" in task else False
                                timeout = task["timeout"] if "timeout" in task else None
                                #TODO: make this a more informative type
                                result = test_driver.run_test(goal_pose=scenario.getGoalMsg(), record=record, timeout=timeout)
                                scenario.cleanup()
                            except rospy.ROSException as e:
                                result = "ROSException: " + str(e)
                                task["error"]= True
                                self.had_error = True
                            except roslaunch.RLException as e:
                                result = "RLException: " + str(e)

                            if self.controller_launch is not None:
                                self.controller_launch.shutdown()

                        else:
                            result = "gazebo_crash"
                            task["error"] = True
                            self.had_error = True
                    else:
                        result = "bad_robot"
                else:
                    result = "bad_scenario"

                if isinstance(result, dict):
                    task.update(result)
                else:
                    task["result"] = result
                task["pid"] = os.getpid()
                self.return_result(task)

                if self.had_error:
                    print(result, file=sys.stderr)


            except queue.Empty as e:
                with self.soft_kill_flag.get_lock():
                    if self.soft_kill_flag.value:
                        self.shutdown()
                        print("Soft shutdown requested")
                time.sleep(1)


            with self.kill_flag.get_lock():
                if self.kill_flag.value:
                    self.shutdown()

        print("Done with processing, killing launch files...")
        # It seems like killing the core should kill all of the nodes,
        # but it doesn't
        if self.gazebo_launch is not None:
            self.gazebo_launch.shutdown()

        if self.robot_launch is not None:
            self.robot_launch.shutdown()

        if self.controller_launch is not None:
            self.controller_launch.shutdown()

        if self.core is not None:
            print("GazeboMaster shutdown: killing core...")
            self.core.shutdown()
        #self.core.kill()

        print("All cleaned up")


    def roslaunch_core(self):
        rospy.loginfo("Launching roscore with port [" + str(self.ros_port) + "]")
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        #roslaunch.configure_logging(uuid)

        self.core = roslaunch.parent.ROSLaunchParent(
            run_id=uuid, roslaunch_files=[],
            is_core=True, port=self.ros_port
        )
        self.core.start()

    def roslaunch_controller(self, robot, controller_name, controller_args=None):
        rospy.loginfo("Launching controller [" + controller_name + "] for robot [" + str(robot) + "] with args [" + str(controller_args) + "]")

        if controller_args is None:
            controller_args = {}

        old_environ = os.environ

        global _rospack

        def clear_roslaunch_rospack_cache():
            ''' roslaunch.node_args.create_local_process_args() caches a RosPack instance, so we clear it here
                to ensure that the correct environment is used. create_local_process_args() accepts an 'env'
                argument, but there is no way to pass it in without modifying roslaunch.nodeprocess.py
            '''

            _rospack = None

        # From https://stackoverflow.com/a/7198338
        def source_workspace2(bash_file):
            import os, subprocess as sp, json

            source = 'source ' + bash_file
            dump = '/usr/bin/python -c "import os, json;print json.dumps(dict(os.environ))"'
            pipe = sp.Popen(['/bin/bash', '-c', '%s && %s' % (source, dump)], stdout=sp.PIPE)
            env = json.loads(pipe.stdout.read())
            os.environ = env

        # Alternative (untested) approach, from https://stackoverflow.com/a/3505826
        def source_workspace(bash_file):
            import os
            import pprint
            import shlex
            import subprocess

            command = shlex.split("env -i bash -c 'source" + bash_file + " && env'")
            proc = subprocess.Popen(command, stdout=subprocess.PIPE)
            for line in proc.stdout:
                (key, _, value) = line.partition("=")
                os.environ[key] = value
            proc.communicate()

            pprint.pprint(dict(os.environ))

        bash_source_file = None

        controller_path = None
        if os.path.isfile(controller_name):
            controller_path = controller_name
        elif ControllerLauncher.contains(controller_name):
            controller_path = ControllerLauncher.getPath(name=controller_name)
            bash_source_file = ControllerLauncher.getEnvironment(name=controller_name)

        #bash_source_file = controller_args['bash_source_file'] if 'bash_source_file' in controller_args else None
        if 'bash_source_file' in controller_args:
            arg_bash_file = controller_args['bash_source_file']
            if arg_bash_file is not None and arg_bash_file != bash_source_file:
                print("Warning, environment provided in controller args will overwrite the environment provided by controller config")
                bash_source_file = arg_bash_file

        if bash_source_file is not None:
            if os.path.isfile(bash_source_file):
                source_workspace2(bash_file = bash_source_file)
            else:
                print("Error! Attempting to source file [" + str(bash_source_file) + "], but the file does not exist!")
                return False

        if bash_source_file not in self.rospack_caches:
            self.rospack_caches[bash_source_file] = rospkg.RosPack()

        rospack = self.rospack_caches[bash_source_file]

        #Save old
        old_rospack = _rospack
        _rospack = rospack

        if controller_path is None:
            path = rospack.get_path("nav_scripts")
            controller_path = path + "/launch/" + robot + "_" + controller_name + "_controller.launch"

        if not os.path.isfile(controller_path):
            print("Error! Attempting to load controller from path [" + str(controller_path) + "], but the file does not exist!")
            return False

        # We'll assume Gazebo is launched and ready to go

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
        #roslaunch.configure_logging(uuid)
        #print path

        #Remapping stdout to /dev/null
        sys.stdout = open(os.devnull, "w")

        # Set environment variables to specify controller arguments
        for key,value in list(controller_args.items()):
            var_name = "GM_PARAM_"+ key.upper()
            value = str(value)
            os.environ[var_name] = value
            print("Setting environment variable [" + var_name + "] to '" + value + "'")

        #clear_roslaunch_rospack_cache()

        self.controller_launch = roslaunch.parent.ROSLaunchParent(
            run_id=uuid, roslaunch_files=[controller_path],
            is_core=False, port=self.ros_port #, roslaunch_strs=controller_args
        )
        self.controller_launch.start()

        #Restore usual environment
        if old_environ is not None:
            os.environ = old_environ
            #clear_roslaunch_rospack_cache()

        _rospack = old_rospack

        sys.stdout = sys.__stdout__

    def roslaunch_robot(self, robot, robot_args=None):
        rospy.loginfo("Launching Gazebo Robot [" + robot + "] with args [" + str(robot_args) + "]")

        if robot == self.current_robot:
            if not self.robot_launch._shutting_down:
                return
            else:
                print("Error with robot, restarting")

        if self.robot_launch is not None:
            self.robot_launch.shutdown()

        self.current_robot = robot

        if robot_args is None:
            robot_args = {}

        if os.path.isfile(robot):
            robot_path = robot
        else:
            robot_path = self.rospack.get_path("nav_configs") + "/launch/spawn_" + robot + ".launch"

        # This will wait for a roscore if necessary, so as long as we detect any failures
        # in roslaunch_core, we should be fine
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
        #roslaunch.configure_logging(uuid) #What does this do?
        #print path

        for key,value in list(robot_args.items()):
            var_name = "GM_PARAM_"+ key.upper()
            value = str(value)
            os.environ[var_name] = value
            print("Setting environment variable [" + var_name + "] to '" + value + "'")

        self.robot_launch = roslaunch.parent.ROSLaunchParent(
            run_id=uuid, roslaunch_files=[robot_path],
            is_core=False, port=self.ros_port
        )
        self.robot_launch.start()

        odom_topic = "/odom"
        if 'odom_topic' in robot_args:
            odom_topic = robot_args['odom_topic']
        # Wait for gazebo simulation to be running
        try:
            msg = rospy.wait_for_message(odom_topic, Odometry, 30)
        except rospy.exceptions.ROSException:
            print("Error! /odom not received!")
            return False

        return True


    def roslaunch_gazebo(self, world, world_args=None):
        rospy.loginfo("Launching Gazebo World [" + world + "] with args [" + str(world_args) + "]")
        if world == self.current_world and world_args == self.current_world_args:
            if not self.gazebo_launch._shutting_down:
                return
            else:
                print("Gazebo crashed, restarting")

        if self.gazebo_launch is not None:
            self.gazebo_launch.shutdown()
            self.current_robot = None   # Ensures a new robot will be spawned

        self.current_world = world
        self.current_world_args = world_args


        # This will wait for a roscore if necessary, so as long as we detect any failures
        # in roslaunch_core, we should be fine
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
        #roslaunch.configure_logging(uuid) #What does this do?
        #print path

        if world_args is None:
            world_args = {'gazebo_gui':'false'}

        # Set environment variables to specify world arguments
        for key,value in list(world_args.items()):
            var_name = "GM_PARAM_"+ key.upper()
            value = str(value)
            os.environ[var_name] = value
            print("Setting environment variable [" + var_name + "] to '" + value + "'")

        #Without the mutex, we frequently encounter this problem:
        # https://bitbucket.org/osrf/gazebo/issues/821/apparent-transport-race-condition-on
        with self.gazebo_launch_mutex:
            self.gazebo_launch = roslaunch.parent.ROSLaunchParent(
                run_id=uuid, roslaunch_files=[world],
                is_core=False, port=self.ros_port
            )
            self.gazebo_launch.start()

        # Wait for gazebo simulation to be running
        try:
            msg = rospy.wait_for_message("/clock", ClockMsg, 30)
        except rospy.exceptions.ROSException:
            print("Error! clock not received!")
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
        print("Returning completed task: " + str(result))
        rospy.loginfo("Returning completed task [" + str(result) + "]...")
        self.result_queue.put(result)
        rospy.loginfo("Returned completed task, marking task done [" + str(result) + "]...")
        self.task_queue.task_done()
        rospy.loginfo("Marked task done [" + str(result) + "]")




if __name__ == "__main__":

    start_time = time.time()
    master = MultiMasterCoordinator(2)
    master.start()

    def getTasks():
        controller_freq = 5
        for [scenario, min_obstacle_spacing] in [['dense', 1], ['sector_laser',1], ['sector_extra',1], ['campus_obstacle',1], ['fourth_floor_obstacle',1]]:
            for seed in range(1,20):
                for global_planning_freq in [1]:
                    for controller in ['dwa', 'teb', 'ego_teb', 'p2d', 'p2d_local_global']:
                        task = {'controller': controller, 'seed': seed, 'scenario': scenario, 'robot': 'turtlebot',
                                'min_obstacle_spacing': min_obstacle_spacing,
                                'controller_args': {'global_planning_freq': global_planning_freq,
                                                    'controller_freq': controller_freq}}

                        if scenario == 'sector_extra':
                            task['num_obstacles']=50

                        yield task

                    for num_paths in [5]:
                        for controller in ['informed_pips_dwa_multiclass', 'informed_pips_dwa_rl']:
                            task = {'controller': controller, 'seed': seed, 'scenario': scenario, 'robot': 'turtlebot', 'min_obstacle_spacing': min_obstacle_spacing,
                                    'controller_args': {'global_planning_freq': global_planning_freq, 'controller_freq': controller_freq, 'num_inferred_paths': num_paths}}
                            yield task
                        for controller in ['informed_pips_dwa_bruteforce']:
                            task = {'controller': controller, 'seed': seed, 'scenario': scenario, 'robot': 'turtlebot', 'min_obstacle_spacing': min_obstacle_spacing,
                                    'controller_args': {'global_planning_freq': global_planning_freq, 'controller_freq': controller_freq, 'num_paths': num_paths}}
                            yield task

                    for controller in ['informed_pips_dwa_to_goal',  'informed_pips_dwa_regression_goal']:
                        for enable_cc in ['true']:
                            task = {'controller': controller, 'seed': seed, 'scenario': scenario, 'robot': 'turtlebot', 'min_obstacle_spacing': min_obstacle_spacing,
                                    'controller_args': {'enable_cc': enable_cc, 'global_planning_freq': global_planning_freq, 'controller_freq': controller_freq, }}
                            yield task



    master.add_tasks(tasks=getTasks())
    
    #master.singletask()
    master.wait_to_finish()
    #rospy.spin()
    master.shutdown()
    end_time = time.time()
    print("Total time: " + str(end_time - start_time))



