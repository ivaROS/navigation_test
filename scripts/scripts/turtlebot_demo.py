#!/usr/bin/env python

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
from gazebo_msgs.srv import SetModelState

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
from kobuki_msgs.msg import BumperEvent, WheelDropEvent
import math

class DemoResetter():
    def __init__(self):
        rospy.init_node('Prototype')

        self.map_pub = rospy.Publisher("/map",OccupancyGrid, queue_size=1,latch=True)
        self.clear_costmap_srv = None

        self.publishMap()  

        self.setupGoals()
        self.pub = rospy.Publisher("/mobile_base/commands/reset_odometry",std_msgs.Empty, queue_size=1)

        self.client = actionlib.SimpleActionClient('move_base', move_base_msgs.MoveBaseAction)
        print "waiting for server"
        self.client.wait_for_server()
        print "Done!"
        
        self.stop = False

        rospy.Subscriber("/mobile_base/events/button",ButtonEvent,self.ButtonEventCallback)
        rospy.Subscriber("/mobile_base/events/wheel_drop",WheelDropEvent,self.WheelDropEventCallback)
        
        self.navigate()

        rospy.spin()

    def setupGoals(self):
        self.goals = []

        goalA = Pose()
        goalA.position.x = 5
        goalA.orientation.w = 1

        goalB = Pose()
        goalB.position.x = 0
        goalB.orientation.w = -1

        self.goals.append(goalA)
        self.goals.append(goalB)


    def navigate(self):
        while not rospy.is_shutdown():
            if not self.stop:
                try:
                    goal = next(self.goal_generator)
                    self.navigateToGoal(goal_pose=goal)
                    self.resetCostmaps()
                except Exception, e:
                    print e
                    pass
                    rospy.sleep(.1)
            else:
                rospy.sleep(.2)


    def getNextGoal(self):
        while True:
            for goal in self.goals:
                print goal
                yield goal

    def navigateToGoal(self, goal_pose):


        # Create the goal point
        goal = move_base_msgs.MoveBaseGoal()
        goal.target_pose.pose = goal_pose
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "odom"

        # Send the goal!
        print "sending goal"
        self.client.send_goal(goal)
        print "waiting for result"

        r = rospy.Rate(5)

        start_time = rospy.Time.now()

        keep_waiting = True
        while keep_waiting:
            state = self.client.get_state()
            # print "State: " + str(state)
            if state is not GoalStatus.ACTIVE and state is not GoalStatus.PENDING:
                keep_waiting = False
            else:
                r.sleep()

        state = self.client.get_state()
        if state == GoalStatus.SUCCEEDED:
            return True

        return True


    def resetCostmaps(self):
        if self.clear_costmap_srv is None:
            rospy.wait_for_service('/move_base/clear_costmaps')
            self.clear_costmap_srv = rospy.ServiceProxy('/move_base/clear_costmaps', std_srvs.Empty)
        self.clear_costmap_srv()
        print "reset costmaps"

    def resetOdom(self):
        print "reset Odom"
        self.pub.publish(std_msgs.Empty())

    def resetGoals(self):
        print "cancel goals"
        self.client.cancel_all_goals()
        self.goal_generator = self.getNextGoal()


    def ButtonEventCallback(self,data):
        if ( data.state == ButtonEvent.PRESSED and data.button==ButtonEvent.Button0) :
            rospy.loginfo("Reset request received")
            self.resetCostmaps()
            self.resetOdom()
            self.resetGoals()
            self.stop = False

    def WheelDropEventCallback(self,data):
        if ( data.state == WheelDropEvent.DROPPED) :
            rospy.loginfo("Stop request received")
            self.stop = True
            self.client.cancel_all_goals()

    def publishMap(self):

        self.OCC_OBSTACLE=100
        self.OCC_UNKNOWN=0

        self.resolution = .05
        world_width = 3
        world_height = 5.5
        
        map_origin = Pose()
        map_origin.position.x = -.25
        map_origin.position.y = -world_width/2.0#
        map_origin.orientation.w = 1# .505
        map_origin.orientation.z = 0#.505

        map_width = int((world_height) / self.resolution)
        map_height = int((world_width) / self.resolution)

        empty_map = self.OCC_UNKNOWN*np.ones(shape=(map_width,map_height), dtype=np.int8)

        map_image = empty_map

        map_image[0] = self.OCC_OBSTACLE
        map_image[-1] = self.OCC_OBSTACLE
        map_image[:,0] = self.OCC_OBSTACLE
        map_image[:,-1] = self.OCC_OBSTACLE


        map_msg = OccupancyGrid()
        map_msg.header.frame_id = "odom"
        map_msg.header.stamp = rospy.Time.now()
        map_msg.data = np.reshape(a=map_image,newshape=-1,order='F').tolist()    #First flatten the array, then convert it to a list

        metadata = MapMetaData()
        metadata.resolution = self.resolution
        metadata.map_load_time = rospy.Time.now()
        metadata.width = map_width
        metadata.height = map_height

        metadata.origin = map_origin

        map_msg.info = metadata

        self.map_pub.publish(map_msg)




 
if __name__ == '__main__':
    try:
        DemoResetter()
    except rospy.ROSInterruptException:
   	rospy.loginfo("exception")
