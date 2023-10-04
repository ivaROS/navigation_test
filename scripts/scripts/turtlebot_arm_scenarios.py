#!/usr/bin/env python

from __future__ import division
from past.utils import old_div
from gazebo_driver_v2 import GazeboDriver
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion


#rospy.init_node('test_driver', anonymous=True)

driver = GazeboDriver()
driver.checkServicesTopics()

def moveBlock(name, pose):

    if(not driver.setPose(model_name=name, pose=pose)):

        model_type = "cinder_block_wide"
        driver.spawn_local_database_model(name, model_type, initial_pose=pose)


rospy.Rate(1).sleep()

fw = 1.4

block_height = .14
block_width = .4

robot_radius = .22


rotated = Quaternion(x=.707, w=.707)
upright = Quaternion(w=1)


r = rospy.Rate(.2)

while not rospy.is_shutdown():

    driver.pause()
    driver.moveRobot(Pose(orientation=upright))

    moveBlock("cinder_block_h", Pose(position=Point(fw,-robot_radius - old_div(block_width,2),0), orientation=upright))
    moveBlock("cinder_block_v", Pose(position=Point(fw,robot_radius + block_height,old_div(block_width,2)), orientation=rotated))
    driver.unpause()

    driver.updateModels()
    r.sleep()


    driver.pause()
    driver.moveRobot(Pose(orientation=upright))

    moveBlock("cinder_block_h", Pose(position=Point(fw,robot_radius + old_div(block_width,2),0), orientation=upright))
    moveBlock("cinder_block_v", Pose(position=Point(fw,-robot_radius,old_div(block_width,2)), orientation=rotated))
    driver.unpause()

    r.sleep()

#result = test_driver.run_test(goal_pose=scenario.getGoal())
