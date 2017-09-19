#!/usr/bin/env python
import rospy
import sys, os, time
import random

from gazebo_ros import gazebo_interface
from gazebo_msgs.msg import *
import std_srvs.srv

import tf.transformations as tft
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Wrench

import numpy as np

def exception_wrap(func):
    def wrapper (*args, **kwargs):
        try:
            func(*args, **kwargs)
        except Exception as e:
            print "Error:", e
    return wrapper

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

def spawn_barrel(xy, name):
    # Must be unique in the gazebo world - failure otherwise
    # Spawning on top of something else leads to bizarre behavior
    model_name = name
    model_path = os.path.expanduser("~/.gazebo/models/first_2015_trash_can/model.sdf")
    model_xml = load_model_xml(model_path)
    robot_namespace = rospy.get_namespace()
    gazebo_namespace = "/gazebo"
    reference_frame = ""

    # Initial pose
    initial_pose = Pose()
    initial_pose.position.x = xy[0]
    initial_pose.position.y = xy[1]
    initial_pose.position.z = 0.0

    # convert rpy to quaternion for Pose message
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    tmpq = tft.quaternion_from_euler(roll,pitch,yaw)
    initial_pose.orientation = Quaternion(tmpq[0],tmpq[1],tmpq[2],tmpq[3])

    success = gazebo_interface.spawn_sdf_model_client(model_name, model_xml,
        robot_namespace, initial_pose, reference_frame, gazebo_namespace)

def spawn_dumpster(xy, name):
    # Must be unique in the gazebo world - failure otherwise
    # Spawning on top of something else leads to bizarre behavior
    model_name = name
    model_path = os.path.expanduser("~/.gazebo/models/dumpster/model.sdf")
    model_xml = load_model_xml(model_path)
    robot_namespace = rospy.get_namespace()
    gazebo_namespace = "/gazebo"
    reference_frame = ""

    # Initial pose
    initial_pose = Pose()
    initial_pose.position.x = xy[0]
    initial_pose.position.y = xy[1]
    initial_pose.position.z = 0.0

    # convert rpy to quaternion for Pose message
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    tmpq = tft.quaternion_from_euler(roll,pitch,yaw)
    initial_pose.orientation = Quaternion(tmpq[0],tmpq[1],tmpq[2],tmpq[3])

    success = gazebo_interface.spawn_sdf_model_client(model_name, model_xml,
        robot_namespace, initial_pose, reference_frame, gazebo_namespace)

def spawn_model(model, xy, name):
    # Must be unique in the gazebo world - failure otherwise
    # Spawning on top of something else leads to bizarre behavior
    model_name = name
    model_path = os.path.expanduser("~/.gazebo/models/"+model+"/model.sdf")
    model_xml = load_model_xml(model_path)
    robot_namespace = rospy.get_namespace()
    gazebo_namespace = "/gazebo"
    reference_frame = ""

    # Initial pose
    initial_pose = Pose()
    initial_pose.position.x = xy[0]
    initial_pose.position.y = xy[1]
    initial_pose.position.z = 0.0

    # convert rpy to quaternion for Pose message
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    tmpq = tft.quaternion_from_euler(roll,pitch,yaw)
    initial_pose.orientation = Quaternion(tmpq[0],tmpq[1],tmpq[2],tmpq[3])

    success = gazebo_interface.spawn_sdf_model_client(model_name, model_xml,
        robot_namespace, initial_pose, reference_frame, gazebo_namespace)

def barrel_points(xmin, ymin, xmax, ymax, grid_size, num_barrels):
    # Get a dense grid of points
    points = np.mgrid[xmin:xmax:grid_size, ymin:ymax:grid_size]
    points = points.swapaxes(0, 2)
    points = points.reshape(points.size / 2, 2)

    # Choose random indexes
    idx = random.sample(range(points.shape[0]), num_barrels)
    print idx

    # Generate offsets
    off = np.random.rand(num_barrels, 2) * grid_size / 2.0

    # Compute barrel points
    barrels = points[idx] + off

    for barrel in barrels:
        yield barrel

def spawn_barrels(n):
    position = []
    for i, xy in enumerate(barrel_points(-4.0, 1.0, 0.0, 5.0, 1.0, n)):
        print i, xy
        position.append(xy)
        name = "barrel{}".format(i)
        print name
        spawn_barrel(xy, name)
    return position
delete_model = exception_wrap(rospy.ServiceProxy('/gazebo/delete_model', gazebo_interface.DeleteModel))

def delete_barrels(n):
    for i in range(n):
        name = "barrel{}".format(i)
        delete_model(name)

reset_world = rospy.ServiceProxy('/gazebo/reset_world', std_srvs.srv.Empty)
reset_simulation = rospy.ServiceProxy('/gazebo/reset_simulation', std_srvs.srv.Empty)


if __name__ == "__main__":
    try:
        rospy.init_node('spawner', anonymous=True)
        delete_barrels(5)
        spawn_barrels(5)
        time.sleep(5)
    except rospy.ROSInterruptException:
        print "Keyboard Interrupt"
