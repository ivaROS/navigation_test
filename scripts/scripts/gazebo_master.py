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
from std_msgs.msg import Empty
import Queue
from ctypes import c_bool

import signal

import socket
import contextlib

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

        self.num_masters = 1
        self.task_queue_capacity = 20 #2*self.num_masters
        self.task_queue = mp.JoinableQueue(maxsize=self.task_queue_capacity)
        self.result_queue_capacity = 20 #*self.num_masters
        self.result_queue = mp.JoinableQueue(maxsize=self.result_queue_capacity)
        self.gazebo_masters = []


    def start(self):
        self.startResultsProcessing()
        self.startProcesses()
        self.addTasks()

    def startResultsProcessing(self):
        self.result_thread = mp.Process(target=self.processResults,args=[self.result_queue])
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


    def processResults(self,queue):
        while not self.is_shutdown.value:
            try:
                task = queue.get(block=False)
                print "Result of " + task["world"] + ":" + task["controller"] + "= " + str(task["result"])
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
            process.join()
        #sys.exit(0)

    def waitToFinish(self):
        print "Waiting until everything done!"
        self.task_queue.join()
        print "All tasks processed!"
        self.result_queue.join()
        print "All results processed!"
        pass



    #This list should be elsewhere, possibly in the configs package
    def addTasks(self):
        task1 = {'world': 'rectangular','controller':'dwa'}
        task2 = {'world': 'rectangular','controller':'dwa'}

        for a in range(1):
            self.task_queue.put(task1)
            self.task_queue.put(task2)




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


        print "New master"

        self.ros_master_uri = "http://localhost:" + str(self.ros_port)
        self.gazebo_master_uri = "http://localhost:" + str(self.gazebo_port)
        os.environ["ROS_MASTER_URI"] = self.ros_master_uri
        os.environ["GAZEBO_MASTER_URI"]= self.gazebo_master_uri

        if 'DISPLAY' in os.environ:
            del os.environ['DISPLAY']   #To ensure that no GUI elements of gazebo activated

    def run(self):

        self.start_core()
        rospy.init_node('test_driver', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.odom_pub = rospy.Publisher(
            '/mobile_base/commands/reset_odometry', Empty, queue_size=10)
        while not self.is_shutdown:
            # TODO: If fail to run task, put task back on task queue
            try:

                task = self.task_queue.get(block=False)

                self.roslaunch_gazebo(task["world"]) #pass in world info

                if self.gazebo_driver is None:
                    self.gazebo_driver = GazeboDriver(as_node=False)

                self.roslaunch_controller("dwa_controller.launch")

                #This is where things could be put in a different file: all the nodes have been
                #started, so just have to make service calls, etc to configure environment


                print "Resetting robot..."
                # TODO: Check if reset successful; if not, wait briefly and try again,
                # eventually fail and throw error
                self.gazebo_driver.resetRobot()
                self.odom_pub.publish()

                #print "Waiting for move base message..."

                #time.sleep(3)

                print "Running test..."


                #master = rosgraph.Master('/mynode')

                #TODO: make this a more informative type
                result = test_driver.run_test()

                self.controller_launch.shutdown()
                #self.gazebo_launch.shutdown() #if possible, should probably world instead

                task["result"] = result
                self.return_result(task)
            except Queue.Empty, e:
                time.sleep(1)

            with self.kill_flag.get_lock():
                if self.kill_flag.value:
                    self.shutdown()

        print "GazeboMaster shutdown: killing core..."
        self.core.kill()


    def start_core(self):

        #env_prefix = "ROS_MASTER_URI="+ros_master_uri + " GAZEBO_MASTER_URI=" + gazebo_master_uri + " "

        my_command = "roscore -p " + str(self.ros_port)

        my_env = os.environ.copy()
        my_env["ROS_MASTER_URI"] = self.ros_master_uri
        my_env["GAZEBO_MASTER_URI"] = self.gazebo_master_uri

        print "Starting core..."
        self.core = subprocess.Popen(my_command, env=my_env, shell=True)
        print "Core started!"



    def roslaunch_controller(self, controller_name):

        rospack = rospkg.RosPack()
        path = rospack.get_path("nav_scripts")

        # We'll assume Gazebo is launched are ready to go

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
        #roslaunch.configure_logging(uuid)
        print path

        self.controller_launch = roslaunch.parent.ROSLaunchParent(
            run_id=uuid, roslaunch_files=[path + "/launch/" + controller_name],
            is_core=False, port=self.ros_port
        )
        self.controller_launch.start()


    def roslaunch_gazebo(self, world ="rectangular"):
        if world == self.current_world:
            return

        if self.gazebo_launch is not None:
            self.gazebo_launch.shutdown()

        self.current_world = world

        #Really, the logic of what launch file to start should be elsewhere

        rospack = rospkg.RosPack()
        path = rospack.get_path("nav_configs")

        # This will wait for a roscore if necessary, so as long as we detect any failures
        # in start_roscore, we should be fine
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
        #roslaunch.configure_logging(uuid) #What does this do?
        #print path

        self.gazebo_launch = roslaunch.parent.ROSLaunchParent(
            run_id=uuid, roslaunch_files=[path + "/launch/gazebo_" + world + "_world.launch"],
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
