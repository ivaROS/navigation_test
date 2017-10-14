#!/usr/bin/env python

import subprocess
import multiprocessing as mp
import os
import rospkg
import roslaunch
import time
import test_driver
from gazebo_driver_v2 import GazeboDriver
import rospy
from actionlib_msgs.msg import GoalStatusArray
import rosgraph
import threading
from std_msgs.msg import Empty

class MultiMasterCoordinator:
  def __init__(self):
    self.num_masters = 4
    self.task_queue_capacity = 2*self.num_masters
    self.task_queue = mp.Queue(maxsize=self.task_queue_capacity)
    self.result_queue_capacity = 2*self.num_masters
    self.result_queue = mp.Queue(maxsize=self.result_queue_capacity)

  def start(self):
    self.startResultsProcessing()
    self.startProcesses()
    self.addTasks()

  def processResults(self,queue):
    while True:
      task = queue.get()
      print "Result of " + task["world"] + ":" + task["controller"] + "= " + str(task["result"])

  def startResultsProcessing(self):
    self.result_thread = mp.Process(target=self.processResults,args=[self.result_queue])
    self.result_thread.daemon=True
    self.result_thread.start()

  def startProcesses(self):
    self.gazebo_masters = []
    for ind in xrange(self.num_masters):
      ros_port = 11318 + ind
      gazebo_port = ros_port + 100
      gazebo_master = GazeboMaster(self.task_queue, self.result_queue, ros_port, gazebo_port)
      gazebo_master.start()
      self.gazebo_masters.append(gazebo_master)
  
  def shutdown(self):
    for process in self.gazebo_masters:
      process.shutdown()

  def addTasks(self):
    task1 = {'world': 'rectangular','controller':'pips_dwa'}
    task2 = {'world': 'rectangular','controller':'pips_dwa'}

    for a in range(4):
      self.task_queue.put(task1)
      self.task_queue.put(task2)


class GazeboMaster(mp.Process):
  def __init__(self, task_queue, result_queue, ros_port, gazebo_port, **kwargs):
    super(GazeboMaster, self).__init__()
    self.task_queue = task_queue
    self.result_queue = result_queue
    self.ros_port = ros_port
    self.gazebo_port = gazebo_port
    self.core = None
    self.gazebo_launch = None
    self.controller_launch = None
    self.gazebo_driver = None

    print "New master"

    self.ros_master_uri = "http://localhost:" + str(self.ros_port)    
    self.gazebo_master_uri = "http://localhost:" + str(self.gazebo_port)
    os.environ["ROS_MASTER_URI"] = self.ros_master_uri
    os.environ["GAZEBO_MASTER_URI"]= self.gazebo_master_uri

  def run(self):

    self.start_core()
    rospy.init_node('test_driver', anonymous=True)
    self.odom_pub = rospy.Publisher(
      '/mobile_base/commands/reset_odometry', Empty, queue_size=10)
    while True:
      task = self.task_queue.get()
      self.roslaunch_gazebo() #pass in world info
      if self.gazebo_driver is None:
        self.gazebo_driver = GazeboDriver(as_node=False)
      self.roslaunch_controller("dwa_controller.launch")

      #This is where things could be put in a different file: all the nodes have been
      #started, so just have to make service calls, etc to configure environment
      print "Resetting robot..."
      self.gazebo_driver.resetRobot()
      self.odom_pub.publish()
      
      print "Waiting for move base message..."
      #msg = rospy.wait_for_msg("/move_base/status", GoalStatusArray)

      time.sleep(3)

      print "Running test..."


      master = rosgraph.Master('/mynode')

      result = test_driver.run_test()

      self.controller_launch.shutdown()
      #self.gazebo_launch.shutdown() #if possible, should probably world instead

      task["result"] = result
      self.result_queue.put(task)


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


  def roslaunch_gazebo(self):
    if self.gazebo_launch is not None:
      return
    #if world == current_world
    controller_name = "gazebo_depth.launch"

    rospack = rospkg.RosPack()
    path = rospack.get_path("pips_dwa_implementation")

    # We'll assume Gazebo is launched are ready to go

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
    #roslaunch.configure_logging(uuid)
    print path

    self.gazebo_launch = roslaunch.parent.ROSLaunchParent(
      run_id=uuid, roslaunch_files=[path + "/launch/" + controller_name],
      is_core=False, port=self.ros_port
      )
    self.gazebo_launch.start()

  def shutdown(self):
    self.core.kill

  '''
  def start_gazebo(self, world_state):
    if self.launch is not None:
      return


    #Make sure that necessary nodes are running; restart them if needed; also restart if new world
    #if gazebo server running
    #if correct world
    #if robot running

    start_gazebo="roslaunch pips_dwa_implementation gazebo_depth.launch"

    ros_master_uri = "http://localhost:" + str(self.ros_port)
    gazebo_master_uri = "http://localhost:" + str(self.gazebo_port)

    env_prefix = "ROS_MASTER_URI="+ros_master_uri + " GAZEBO_MASTER_URI=" + gazebo_master_uri + " "

    my_command = env_prefix + start_gazebo + " -p " + str(self.ros_port)

    my_env = os.environ.copy()
    my_env["ROS_MASTER_URI"] = ros_master_uri
    p = subprocess.Popen(my_command, env=my_env, shell=True)

    #p = subprocess.Popen(['external_program', 'arg1', 'arg2'])
    # Process is now running in the background, do other stuff...
    
    # Check if process has completed
    #if p.poll() is not None:
    
    # Wait for process to complete
    #p.wait()
  '''


  def return_result(self,result):
    self.result_queue.put(result)
  




if __name__ == "__main__":
    #try:
        master = MultiMasterCoordinator()
        master.start()
    #except:
        print "Keyboard Interrupt"
