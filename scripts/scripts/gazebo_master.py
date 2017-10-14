#!/usr/bin/env python

import subprocess
import multiprocessing as mp
import os
import rospkg
import roslaunch
import time

class MultiMasterCoordinator:
  def __init__(self):
    self.num_masters = 1


  def startProcesses(self):
    self.gazebo_masters = []
    for ind in xrange(self.num_masters):
      ros_port = 11311 + ind
      gazebo_port = ros_port + 100
      gazebo_master = GazeboMaster(task_queue, result_queue, ros_port, gazebo_port)
      gazebo_master.start()
      self.gazebo_masters.append(gazebo_master)
  


class GazeboMaster(mp.Process):
  def __init__(self, task_queue, result_queue, ros_port, gazebo_port, **kwargs):
    super(GazeboMaster, self).__init__()
    self.task_queue = task_queue
    self.result_queue = result_queue
    self.ros_port = ros_port
    self.gazebo_port = gazebo_port
    self.launch = None
    self.core = None

    self.ros_master_uri = "http://localhost:" + str(self.ros_port)    
    self.gazebo_master_uri = "http://localhost:" + str(self.gazebo_port)
    os.environ["ROS_MASTER_URI"] = self.ros_master_uri
    os.environ["GAZEBO_MASTER_URI"]= self.gazebo_master_uri

  def run(self):
    self.use_roslaunch()
    #self.start_core()
    #self.start_gazebo(None)
    while True:
      #task = self.task_queue.get()
      #self.start_gazebo(task.world_state)
      #self.start_gazebo(None)
      time.sleep(100)


  def start_core(self):

    env_prefix = "ROS_MASTER_URI="+ros_master_uri + " GAZEBO_MASTER_URI=" + gazebo_master_uri + " "

    my_command = env_prefix + "roscore -p " + str(self.ros_port)

    my_env = os.environ.copy()
    my_env["ROS_MASTER_URI"] = ros_master_uri

    print "Starting core..."
    self.core = subprocess.Popen(my_command, env=my_env, shell=True)
    print "Core started!"



  def use_roslaunch(self):
    controller_name = "gazebo_depth.launch"

    rospack = rospkg.RosPack()
    path = rospack.get_path("pips_dwa_implementation")

    # We'll assume Gazebo is launched are ready to go

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    #roslaunch.configure_logging(uuid)
    print path

    self.launch = roslaunch.parent.ROSLaunchParent(
      run_id=uuid, roslaunch_files=[path + "/launch/" + controller_name],
      is_core=True, port=self.ros_port
      )
    self.launch.start()


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


  def return_result(self,result):
    self.result_queue.put(result)
  




if __name__ == "__main__":
    try:
        a = GazeboMaster(None, None, 11313, 11512)
        a.start()
    except e:
        print "Keyboard Interrupt"
