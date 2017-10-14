#!/usr/bin/env python

import rospy
import random
import sys, os, time
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, Point, Quaternion, Transform, TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from copy import deepcopy
from depth_learning.msg import GazeboState
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException, StaticTransformBroadcaster
import tf
#from pips_test import gazebo_driver

from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import GetModelState
import numpy as np
from std_srvs.srv import Empty

from gazebo_ros import gazebo_interface
import std_srvs.srv
  



#Copied from pips_test: gazebo_driver.py
# Load model xml from file
def load_model_xml(filename):
  if os.path.exists(filename):
      if os.path.isdir(filename):
          print "Error: file name is a path?", filename
          sys.exit(0)

      if not os.path.isfile(filename):
          print "Error: unable to open file", filename
          sys.exit(0)
  else:
      print "Error: file does not exist", filename
      sys.exit(0)

  f = open(filename,'r')
  model_xml = f.read()
  if model_xml == "":
      print "Error: file is empty", filename
      sys.exit(0)

  return model_xml

class GazeboDriver():
  # Copied from pips_test: gazebo_driver.py
  def barrel_points(self,xmin, ymin, xmax, ymax, grid_size, num_barrels):
    # Get a dense grid of points
    points = np.mgrid[xmin:xmax:grid_size, ymin:ymax:grid_size]
    points = points.swapaxes(0, 2)
    points = points.reshape(points.size / 2, 2)

    # Choose random indexes
    idx = self.random.sample(range(points.shape[0]), num_barrels)
    print idx

    # Generate offsets
    off = self.nprandom.rand(num_barrels, 2) * grid_size / 2.0

    # Compute barrel points
    barrels = points[idx] + off

    for barrel in barrels:
      yield barrel

  def statesCallback(self, data): #This comes in at ~100hz
    self.models = data
  
  def camInfoCallback(self, data):
    self.camInfo = data
    
  def depthCallback(self, data):
    if self.haveTransform():
      if self.models and self.camInfo:
        self.saveState(data,self.models,self.camInfo,self.transform)
        self.newScene()
  
  def haveTransform(self):
    if self.transform==None:
      try:
        self.transform = self.tfBuffer.lookup_transform('camera_depth_optical_frame', 'base_footprint', rospy.Time())
        return True
      except (LookupException, ConnectivityException, ExtrapolationException):
        return False
    else:
      return True
    
  def saveState(self, depthIm, modelStates, camInfo, transform):
    state = GazeboState(header=depthIm.header, image=depthIm, camera_info=camInfo, model_states=modelStates, transform=transform)
    self.statePub.publish(state)
  
  def newScene(self):
    self.pauseService()
    self.resetRobot()
    self.moveBarrels(self.num_barrels)
    self.unpauseService()

  def setPose(self, model_name, pose):
    ## Check if our model exists yet
    if(model_name in self.models.name):
    
      state = ModelState(model_name=model_name, pose=pose)
      
      response = self.modelStateService(state)

      if(response.success):
        rospy.loginfo("Successfully set model pose")
        return True
    
    rospy.loginfo("failed to set model pose")
    return False


      
  def resetRobotImpl(self, pose):
    self.pauseService()
    p = Pose()
    p.position.x = pose[0]
    p.position.y = pose[1]
    p.position.z = pose[2]
    quaternion = tf.transformations.quaternion_from_euler(pose[3], pose[4], pose[5])
    #print quaternion
    p.orientation.x = quaternion[0]
    p.orientation.y = quaternion[1]
    p.orientation.z = quaternion[2]
    p.orientation.w = quaternion[3]
    self.setPose('mobile_base', p)
    self.unpauseService()
    
    
  def resetRobot(self):
    self.setPose(self.robotName, self.robotPose)

  def resetBarrels(self, n):
      name = None
      for i in range(n):
          name = "barrel{}".format(i)
          pose = self.poses[i]
          self.setPose(name, pose)
      
  #Adapted from pips_test: gazebo_driver.py
  def spawn_barrel(self, model_name, initial_pose):
    # Must be unique in the gazebo world - failure otherwise
    # Spawning on top of something else leads to bizarre behavior
    model_path = os.path.expanduser("~/.gazebo/models/first_2015_trash_can/model.sdf")
    model_xml = load_model_xml(model_path)
    robot_namespace = rospy.get_namespace()
    gazebo_namespace = "/gazebo"
    reference_frame = ""
    
    success = gazebo_interface.spawn_sdf_model_client(model_name, model_xml,
        robot_namespace, initial_pose, reference_frame, gazebo_namespace)

  def moveBarrelsTest(self,n, x, y):
    self.poses = []
    for i in range(n):
      name = "barrel{}".format(i)
      pose = Pose()
      pose.position.x = x[i]
      pose.position.y = y[i]
      pose.orientation.w = 1
      self.poses.append(pose)
      if not self.setPose(name, pose):
	self.spawn_barrel(name, pose)
	
  def moveBarrels(self,n):
    self.poses = []
    for i, xy in enumerate(self.barrel_points(self.minx,self.miny,self.maxx,self.maxy,self.grid_spacing, n)):
      print i, xy
      name = "barrel{}".format(i)
      print name
      
      pose = Pose()
      pose.position.x = xy[0]
      pose.position.y = xy[1]

      pose.orientation.w = 1
      
      self.poses.append(pose)
      
      if not self.setPose(name,pose):
        self.spawn_barrel(name, pose)
      
  def shutdown(self):
    self.unpauseService()
    self.resetWorldService()

  def run(self):
    self.depthSub = rospy.Subscriber('image', Image, self.depthCallback, queue_size=self.queue_size)
    publish_static_transform()
    rospy.spin()

  def reset(self):
    self.random.seed(self.seed)
    self.nprandom = np.random.RandomState(self.seed)
  
  def getRandInt(self, lower, upper):
    a = range(lower, upper + 1)
    start = self.random.choice(a)
    a.remove(start)
    end = self.random.choice(a)
    output = [start, end]
    return output
  


  def __init__(self, as_node = True, seed=None):
    if as_node:
      rospy.init_node('gazebo_state_recorder')
    
      rospy.loginfo("gazebo_state_recorder node started")
    
    self.robotName = 'mobile_base'

    self.queue_size = 50
    self.num_barrels = 3
    self.minx = -3.5
    self.maxx = 0.5
    self.miny = 1.0
    self.maxy = 5.0
    self.grid_spacing = 1.0
    
    
    
    self.poses = []
    self.robotPose = Pose()
    self.robotPose.position.x = 2
    self.robotPose.position.y = 3
    self.robotPose.orientation.x = 0
    self.robotPose.orientation.y = 0
    self.robotPose.orientation.z = 1
    self.robotPose.orientation.w = 0

    self.random = random.Random()
    seed = 2
    self.seed = seed
    self.random.seed(seed)
    self.nprandom = np.random.RandomState(seed)

    self.models = None
    self.camInfo = None
    self.transform = None
    
    model_state_service_name = 'gazebo/set_model_state'
    pause_service_name = '/gazebo/pause_physics'
    unpause_service_name = '/gazebo/unpause_physics'
    get_model_state_service_name = 'gazebo/get_model_state'
    
    
    self.tfBuffer = Buffer()
    self.tfListener = TransformListener(self.tfBuffer)
    

    
    
    rospy.loginfo("Waiting for service...")
    rospy.wait_for_service(get_model_state_service_name)
    self.modelStateService = rospy.ServiceProxy(model_state_service_name, SetModelState)
    rospy.loginfo("Service found...")
    
    rospy.wait_for_service(pause_service_name)
    self.pauseService = rospy.ServiceProxy(pause_service_name, Empty)
    rospy.loginfo("Service found...")
    
    self.resetWorldService = rospy.ServiceProxy('/gazebo/reset_world', std_srvs.srv.Empty)

    rospy.wait_for_service(unpause_service_name)
    self.unpauseService = rospy.ServiceProxy(unpause_service_name, Empty)
    rospy.loginfo("Service found...")
    
    self.stateSub = rospy.Subscriber('gazebo/model_states', ModelStates, self.statesCallback, queue_size=self.queue_size)
    
    self.camInfoSub = rospy.Subscriber('camera_info', CameraInfo, self.camInfoCallback, queue_size=self.queue_size)
    self.statePub = rospy.Publisher('gazebo_data', GazeboState, queue_size=self.queue_size)
    
    self.resetWorldService()
    self.unpauseService()
    
    #rospy.on_shutdown(self.shutdown)




  


if __name__ == '__main__':
  try:
    a = Prototype()
    a.run()
  except rospy.ROSInterruptException:
    rospy.loginfo("exception")
