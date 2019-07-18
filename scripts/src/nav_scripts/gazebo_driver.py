#!/usr/bin/env python

import rospy
import random
import sys, os, time
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, Point, Quaternion, Transform, TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from copy import deepcopy
import rospkg

from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException, StaticTransformBroadcaster
import tf
import math
#from pips_test import gazebo_driver

from gazebo_msgs.msg import ModelStates, ModelState, LinkState
from gazebo_msgs.srv import SetModelState, SetLinkState
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import DeleteModel

import numpy as np

from gazebo_ros import gazebo_interface
import std_srvs.srv as std_srvs
  
import std_msgs.msg as std_msgs



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
  def barrel_points(self,xmins, ymins, xmaxs, ymaxs, min_dist, num_barrels, max_tries =500):
    '''
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
    '''

    xmaxs = np.array(xmaxs)
    xmins = np.array(xmins)
    ymaxs = np.array(ymaxs)
    ymins = np.array(ymins)

    region_weights = (xmaxs - xmins)*(ymaxs - ymins)
    region_weights = region_weights / (np.sum(region_weights))
    region_inds = range(len(xmins))

    sampled_regions = self.nprandom.choice(region_inds,replace=True,p=region_weights,size=max_tries)




    n = 0
    i = 0
    barrels = []
    while n < num_barrels and i < max_tries:
      a = self.random.random()
      b = self.random.random()

      region_ind = sampled_regions[i]

      xmax = xmaxs[region_ind]
      xmin = xmins[region_ind]
      ymax = ymaxs[region_ind]
      ymin = ymins[region_ind]

      depth = xmax - xmin
      width = ymax - ymin

      length = max(depth, width)

      if a * length < depth and b * length < width:
        x = xmin + a * length
        y = ymin + b * length
        point = np.array((x,y))

        barrel_valid = True
        for barrel in barrels:
          if np.linalg.norm(point-barrel) < min_dist:
            barrel_valid = False
            break

        if barrel_valid:
          barrels.append(point)
          n+=1
      i+=1


    for barrel in barrels:
      yield barrel

  def statesCallback(self, data): #This comes in at ~100hz
    self.models = data


  def newScene(self):
    self.pause()
    self.resetRobot()
    self.moveBarrels(self.num_barrels)
    self.unpause()

  def setPose(self, model_name, pose):
    retval = False

    ## Check if our model exists yet
    if( self.models is not None and model_name in self.models.name):

      try:
        state = ModelState(model_name=model_name, pose=pose)

        response = self.setModelState(state)

        if(response.success):
          rospy.loginfo("Successfully set pose of " + str(model_name) + " to " + str(pose))
          retval = True
        else:
          rospy.logwarn("Error setting model pose: " + str(response.status_message))
          retval = False
      except rospy.ServiceException as e:
        rospy.logwarn("Error setting pose: " + str(e))
        retval = False

    #time.sleep(.01)
    return retval
    
    #rospy.loginfo("failed to set model pose")
    #return False

  def setLink(self, model_name, pose):
    ## Check if our model exists yet
    if( self.models is not None and model_name in self.models.name):

      try:
        linkstate = LinkState(link_name=model_name + "::link", pose=pose)

        response = self.setLinkStateService(linkstate)

        if(response.success):
          rospy.loginfo("Successfully set link state of " + str(model_name) + " to " + str(pose))
          return True
        else:
          rospy.logwarn("Error setting link state: " + str(response.status_message))
      except rospy.ServiceException as e:
        pass
        rospy.logwarn("Error setting pose: " + str(e))
      return False

  def pause(self):
    rospy.wait_for_service(self.pause_service_name, timeout=self.service_timeout)
    return self.pauseService()

  def unpause(self):
    rospy.wait_for_service(self.unpause_service_name, timeout=self.service_timeout)
    return self.unpauseService()

  def resetWorld(self):
    rospy.wait_for_service(self.reset_world_service_name, timeout=self.service_timeout)
    return self.resetWorldService()

  def setModelState(self, state):
    rospy.wait_for_service(self.set_model_state_service_name, timeout=self.service_timeout)
    return self.setModelStateService(state)

  def deleteModel(self, name):
    rospy.wait_for_service(self.delete_model_service_name, timeout=self.service_timeout)
    return self.deleteModelService(model_name=name)

  def resetRobotImpl(self, pose):
    self.pause()
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
    self.unpause()

  def resetOdom(self):
    self.odom_pub.publish()
    
  def moveRobot(self, pose):
    self.setPose(self.robotName, pose)

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
    # model_path = os.path.expanduser("~/.gazebo/models/first_2015_trash_can/model.sdf")
    # Fot book chapter
    # model_path = os.path.expanduser("~/.gazebo/models/drc_practice_blue_cylinder/model.sdf")
    path = self.rospack.get_path("nav_configs")
    # model_path = os.path.expanduser(path + "/models/box_lus.sdf")
    model_path = os.path.expanduser(path + "/models/box_lus.sdf")
    model_xml = load_model_xml(model_path)
    robot_namespace = rospy.get_namespace()
    gazebo_namespace = "/gazebo"
    reference_frame = ""
    
    success = gazebo_interface.spawn_sdf_model_client(model_name, model_xml,
        robot_namespace, initial_pose, reference_frame, gazebo_namespace)

  # Adapted from pips_test: gazebo_driver.py
  def spawn_obstacle(self, model_name, model_type, initial_pose):
    # Must be unique in the gazebo world - failure otherwise
    # Spawning on top of something else leads to bizarre behavior

    # model_filenames = {'box':'box_lus.sdf', 'cylinder':'cylinder.sdf'}
    model_filenames = {'box':'box_lus.sdf', 'cylinder':'cylinder.sdf', 'pole':'pole_005_06.sdf', 'square_post':'box_02_02_05.sdf'}

    if model_type not in model_filenames:
      rospy.logerr("Model type [" + str(model_type) + "] is unknown! Known types are: ") #TODO: print list of types
      return False

    model_xml = load_model_xml(self.model_path + model_filenames[model_type])
    robot_namespace = rospy.get_namespace()
    gazebo_namespace = "/gazebo"
    reference_frame = ""

    success = gazebo_interface.spawn_sdf_model_client(model_name, model_xml,
                                                      robot_namespace, initial_pose, reference_frame, gazebo_namespace)

    #time.sleep(.1)
    return success

  def spawn_package_model(self, model_name, package_name, model_path, initial_pose):
    # Must be unique in the gazebo world - failure otherwise
    # Spawning on top of something else leads to bizarre behavior


    package_path = self.rospack.get_path(package_name)

    model_xml = load_model_xml(package_path + model_path)
    robot_namespace = rospy.get_namespace()
    gazebo_namespace = "/gazebo"
    reference_frame = ""

    success = gazebo_interface.spawn_sdf_model_client(model_name, model_xml,
                                                      robot_namespace, initial_pose, reference_frame, gazebo_namespace)

    return success

  def spawn_local_database_model(self, model_name, model_type, initial_pose):
    # Must be unique in the gazebo world - failure otherwise
    # Spawning on top of something else leads to bizarre behavior


    package_path = os.path.expanduser("~/.gazebo/models/")
    model_path = model_type + "/model.sdf"

    model_xml = load_model_xml(package_path + model_path)
    robot_namespace = rospy.get_namespace()
    gazebo_namespace = "/gazebo"
    reference_frame = ""

    success = gazebo_interface.spawn_sdf_model_client(model_name, model_xml,
                                                      robot_namespace, initial_pose, reference_frame, gazebo_namespace)

    return success

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
	
  def moveBarrels(self,n,minx=None,miny=None,maxx=None,maxy=None,grid_spacing=None):
    self.poses = []

    minx=self.minx if minx is None else minx
    maxx=self.maxx if maxx is None else maxx
    miny=self.miny if miny is None else miny
    maxy=self.maxy if maxy is None else maxy
    grid_spacing=self.grid_spacing if grid_spacing is None else grid_spacing



    barrel_names = [name for name in self.models.name if  "barrel" in name]

    for i, xy in enumerate(self.barrel_points(xmins=minx,ymins=miny,xmaxs=maxx,ymaxs=maxy,min_dist=grid_spacing, num_barrels=n)):
      #print i, xy
      name = "barrel{}".format(i)
      #print name

      if name in barrel_names : barrel_names.remove(name)
      
      pose = Pose()
      pose.position.x = xy[0]
      pose.position.y = xy[1]

      # random orientation
      angle = 2*math.pi*self.random.uniform(0, 1)
      quaternion = tf.transformations.quaternion_from_euler(0, 0, angle)
      pose.orientation.x = quaternion[0]
      pose.orientation.y = quaternion[1]
      pose.orientation.z = quaternion[2]
      pose.orientation.w = quaternion[3]
      
      self.poses.append(pose)

      #print str(pose)
      
      if not self.setPose(name,pose):
        self.spawn_barrel(name, pose)

    for name in barrel_names:
      res = self.deleteModel(name=name)
      if not res.success:
        print res.status_message

  def moveObstacles(self, n, minx=None, miny=None, maxx=None, maxy=None, grid_spacing=None, model_types = ['box','cylinder']):
    self.poses = []

    minx = self.minx if minx is None else minx
    maxx = self.maxx if maxx is None else maxx
    miny = self.miny if miny is None else miny
    maxy = self.maxy if maxy is None else maxy
    grid_spacing = self.grid_spacing if grid_spacing is None else grid_spacing

    barrel_names = [name for name in self.models.name if "obstacle" in name]


    num_types = {}
    for model_type in model_types:
      num_types[model_type] = 0

    current_obstacles = [(self.models.name[i], self.models.pose[i]) for i in range(len(self.models.name)) if "obstacle" in self.models.name[i]]


    for name, pose in current_obstacles:
      pose.position.z += 2

      self.setPose(name, pose)
      #self.setLink(name, pose)

    for i, xy in enumerate(
            self.barrel_points(xmins=minx, ymins=miny, xmaxs=maxx, ymaxs=maxy, min_dist=grid_spacing, num_barrels=n, max_tries=2000)):
      # print i, xy

      model_type = self.random.choice(model_types)
      num_type = num_types[model_type]
      num_types[model_type] +=1

      name = model_type + "_obstacle{}".format(num_type)
      # print name


      pose = Pose()
      pose.position.x = xy[0]
      pose.position.y = xy[1]
      pose.position.z = 0

      pose.orientation.w = 1

      self.poses.append(pose)

      # print str(pose)

      if name in barrel_names:
        barrel_names.remove(name)
        self.setPose(name, pose)
      else:
        self.spawn_obstacle(name, model_type, pose)

    for name in barrel_names:
      print "Deleting: " + str(name)
      res = self.deleteModel(name=name)
      if not res.success:
        print res.status_message
      
  def shutdown(self):
    self.unpause()
    self.resetWorld()

  def run(self):
    rospy.spin()


  def reset(self, seed=None):
    if seed is not None:
      self.seed = seed
    self.random.seed(self.seed)
    self.nprandom = np.random.RandomState(self.seed)
  
  def getRandInt(self, lower, upper):
    a = range(lower, upper + 1)
    start = self.random.choice(a)
    a.remove(start)
    end = self.random.choice(a)
    output = [start, end]
    return output

  def updateModels(self, timeout=2):
    self.models = rospy.wait_for_message(self.model_state_topic_name, ModelStates, timeout=timeout)

  #TODO: make return value depend on results of checks
  def checkServicesTopics(self, timeout=2):
    self.updateModels(timeout)
    rospy.wait_for_service(self.get_model_state_service_name, timeout=timeout)
    rospy.wait_for_service(self.pause_service_name, timeout=timeout)
    rospy.wait_for_service(self.reset_world_service_name, timeout=timeout)
    rospy.wait_for_service(self.unpause_service_name, timeout=timeout)
    rospy.wait_for_service(self.delete_model_service_name, timeout=timeout)

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
    
    self.service_timeout = 2.0
    
    self.poses = []
    self.robotPose = Pose()


    self.random = random.Random()
    self.seed = 0
    self.random.seed(seed)
    self.nprandom = np.random.RandomState(seed)

    self.odom_pub = rospy.Publisher(
      '/mobile_base/commands/reset_odometry', std_msgs.Empty, queue_size=1)

    self.rospack = rospkg.RosPack()

    self.models = None
    
    self.set_model_state_service_name = 'gazebo/set_model_state'
    self.set_link_state_service_name = 'gazebo/set_link_state'
    self.pause_service_name = 'gazebo/pause_physics'
    self.unpause_service_name = 'gazebo/unpause_physics'
    self.get_model_state_service_name = 'gazebo/get_model_state'
    self.reset_world_service_name = "gazebo/reset_world"
    self.delete_model_service_name = "gazebo/delete_model"

    self.model_state_topic_name = 'gazebo/model_states'

    #rospy.loginfo("Waiting for service...")
    #rospy.wait_for_service(self.get_model_state_service_name)
    self.setModelStateService = rospy.ServiceProxy(self.set_model_state_service_name, SetModelState)
    #rospy.loginfo("Service found...")

    self.setLinkStateService = rospy.ServiceProxy(self.set_link_state_service_name, SetLinkState)

    #rospy.wait_for_service(self.pause_service_name)
    self.pauseService = rospy.ServiceProxy(self.pause_service_name, std_srvs.Empty)
    #rospy.loginfo("Service found...")

    #rospy.wait_for_service(self.reset_world_service_name)
    self.resetWorldService = rospy.ServiceProxy(self.reset_world_service_name, std_srvs.Empty)
    #rospy.loginfo("Service found...")

    #rospy.wait_for_service(self.unpause_service_name)
    self.unpauseService = rospy.ServiceProxy(self.unpause_service_name, std_srvs.Empty)
    #rospy.loginfo("Service found...")

    #rospy.wait_for_service(self.delete_model_service_name)
    self.deleteModelService = rospy.ServiceProxy(self.delete_model_service_name, DeleteModel)
    #rospy.loginfo("Service found...")

    
    #self.stateSub = rospy.Subscriber(self.model_state_topic_name, ModelStates, self.statesCallback, queue_size=self.queue_size)

    #rospy.wait_for_message(self.model_state_topic_name, ModelStates)
        #self.statePub = rospy.Publisher('gazebo_data', GazeboState, queue_size=self.queue_size)
    
    #self.resetWorldService()
    #self.unpauseService()
    
    #rospy.on_shutdown(self.shutdown)

    path = self.rospack.get_path("nav_configs")
    self.model_path = path + "/models/"




  


if __name__ == '__main__':
  try:
    a = GazeboDriver()
    a.run()
  except rospy.ROSInterruptException:
    rospy.loginfo("exception")
