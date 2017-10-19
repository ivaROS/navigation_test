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
        self.is_shutdown = mp.Value(c_bool,False)

        self.num_masters = 8
        self.task_queue_capacity = 20 #2*self.num_masters
        self.task_queue = mp.JoinableQueue(maxsize=self.task_queue_capacity)
        self.result_queue_capacity = 20 #*self.num_masters
        self.result_queue = mp.JoinableQueue(maxsize=self.result_queue_capacity)
        self.gazebo_masters = []
        self.result_list = []


    def start(self):
        self.startResultsProcessing()
        self.startProcesses()
        self.addTasks()

    def startResultsProcessing(self):
        self.result_thread = threading.Thread(target=self.processResults,args=[self.result_queue])
        self.result_thread.daemon=True
        self.result_thread.start()

    def startProcesses(self):
        ros_port = 11311
        gazebo_port = ros_port + 100
        for ind in xrange(self.num_masters):

            while port_in_use(ros_port):
                ros_port += 1

            while port_in_use(gazebo_port):
                gazebo_port += 1

            gazebo_master = GazeboMaster(self.task_queue, self.result_queue, self.is_shutdown, ros_port, gazebo_port)
            gazebo_master.start()
            self.gazebo_masters.append(gazebo_master)

            ros_port +=1
            gazebo_port +=1

            time.sleep(1)


    def processResults(self,queue):
        while not self.is_shutdown.value:
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
                else:
                    del task["error"]
                    self.task_queue.put(task)

                #print "Result of " + task["world"] + ":" + task["controller"] + "= " + str(task["result"])
                queue.task_done()
            except Queue.Empty, e:
                #print "No results!"
                time.sleep(1)

    def signal_shutdown(self,signum,frame):
        self.shutdown()

    def shutdown(self):
        with self.is_shutdown.get_lock():
            self.is_shutdown.value = True

        for process in self.gazebo_masters:
            if process.is_alive():
                process.join()
        #sys.exit(0)

    def waitToFinish(self):
        print "Waiting until everything done!"
        self.task_queue.join()
        print "All tasks processed!"
        self.result_queue.join()
        print "All results processed!"

        for result in self.result_list:
            print result



    #This list should be elsewhere, possibly in the configs package
    def addAllTasks(self):

        controllers = ["dwa", "eband", "teb"]
        barrel_arrangements = [3,5,7]

        for controller in controllers:
            for num_barrels in barrel_arrangements:
                for a in range(3):
                    task = {'scenario': 'trashcans', 'num_barrels': num_barrels, 'controller': controller}
                    self.task_queue.put(task)

    def addTasks(self):
        controllers = ["eband", "dwa"]
        barrel_arrangements = [3]

        for controller in controllers:
            for num_barrels in barrel_arrangements:
                for a in range(10):
                    for repetition in range(3):
                        task = {'scenario': 'trashcans', 'num_barrels': num_barrels, 'controller': controller, 'seed': a, 'repetition': repetition}
                        self.task_queue.put(task)


class GazeboMaster(mp.Process):
    def __init__(self, task_queue, result_queue, kill_flag, ros_port, gazebo_port, **kwargs):
        super(GazeboMaster, self).__init__()
        self.daemon = False

        self.task_queue = task_queue
        self.result_queue = result_queue
        self.ros_port = ros_port
        self.gazebo_port = gazebo_port
        self.core = None
        self.gazebo_launch = None
        self.controller_launch = None
        self.gazebo_driver = None
        self.current_world = None
        self.kill_flag = kill_flag
        self.is_shutdown = False

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
        while not self.is_shutdown:
            self.process_tasks()
            if not self.is_shutdown:
                print >> sys.stderr, "Relaunching!"

    def process_tasks(self):
        self.roslaunch_core()
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


                    self.roslaunch_gazebo(scenario.getGazeboLaunchFile()) #pass in world info

                    if not self.gazebo_launch._shutting_down:


                        self.roslaunch_controller(task["controller"])

                        try:

                            scenario.setupScenario()

                            print "Running test..."

                            #master = rosgraph.Master('/mynode')

                            #TODO: make this a more informative type
                            result = test_driver.run_test(goal_pose=scenario.getGoal())

                        except rospy.ROSException as e:
                            result = "service_timeout: " + str(e)
                            task["error"]= True
                            self.had_error = True

                        self.controller_launch.shutdown()

                    else:
                        result = "gazebo_crash"
                        task["error"] = True
                        self.had_error = True

                else:
                    result = "bad_task"

                task["result"] = result
                task["pid"] = os.getpid()
                self.return_result(task)
            except Queue.Empty, e:
                time.sleep(1)

            with self.kill_flag.get_lock():
                if self.kill_flag.value:
                    self.shutdown()

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

    def roslaunch_controller(self, controller_name):

        #controller_path =

        rospack = rospkg.RosPack()
        path = rospack.get_path("nav_scripts")

        # We'll assume Gazebo is launched are ready to go

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
        #roslaunch.configure_logging(uuid)
        print path

        self.controller_launch = roslaunch.parent.ROSLaunchParent(
            run_id=uuid, roslaunch_files=[path + "/launch/" + controller_name + "_controller.launch"],
            is_core=False, port=self.ros_port
        )
        self.controller_launch.start()


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

        self.gazebo_launch = roslaunch.parent.ROSLaunchParent(
            run_id=uuid, roslaunch_files=[world],
            is_core=False, port=self.ros_port
        )
        self.gazebo_launch.start()

        try:
            msg = rospy.wait_for_message("/odom", Odometry, 2)
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
        self.result_queue.put(result)
        self.task_queue.task_done()




if __name__ == "__main__":

    start_time = time.time()
    master = MultiMasterCoordinator()
    master.start()
    master.waitToFinish()
    master.shutdown()
    end_time = time.time()
    print "Total time: " + str(end_time - start_time)
