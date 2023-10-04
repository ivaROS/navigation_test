#!/usr/bin/env python3

from builtins import object
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
import std_msgs.msg as std_msgs
import std_srvs.srv as std_srvs
import numpy as np
import cv2
import tf.transformations as transformations
import math
from geometry_msgs.msg import Pose, Quaternion, TransformStamped, Vector3
import tf2_ros
from gazebo_msgs.msg import ContactsState

import sys
from kobuki_msgs.msg import ButtonEvent
from sensor_msgs.msg import Imu
import numpy as np

import actionlib
import move_base_msgs.msg as move_base_msgs
from pprint import pprint
import tf
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent


class BumperCollisionDetector(object):
    def __init__(self):
        rospy.init_node('bumper_collision_detector')

        self.bumper_pub = rospy.Publisher("mobile_base/events/bumper", BumperEvent, queue_size=1, latch=False)
        self.bumper_sub = rospy.Subscriber("base_bumper", ContactsState, self.BumperContactsCallback)


    def BumperContactsCallback(self, data):
        for contact_state in data.states:
            for contact_position in contact_state.contact_positions:
                if contact_position.z > .01:
                    bumper_event = BumperEvent()
                    bumper_event.state=BumperEvent.PRESSED
                    self.bumper_pub.publish(bumper_event)
                    return


if __name__ == '__main__':
    try:
        BumperCollisionDetector()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("exception")
