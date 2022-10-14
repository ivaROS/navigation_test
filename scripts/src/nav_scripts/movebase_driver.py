#!/usr/bin/env python
from __future__ import print_function
from builtins import str
from builtins import object
import rospy
#import actionlib
from move_base_msgs.msg import *
import tf
import tf.transformations
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
import math
import std_srvs.srv as std_srvs
import datetime
import os
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback
import threading
import time
import angles
from std_msgs.msg import Int16 as Int16msg
from nav_scripts.interruptible import InterruptedSleepException #, Rate, SimpleActionClient
from nav_scripts.monitor import SimpleActionClient, Rate
from nav_scripts.task_pipeline import TaskProcessingException, ExceptionLevels

class RobotImplException(TaskProcessingException):
    def __init__(self, msg="", **kwargs):
        super(TaskProcessingException, self).__init__(msg=msg, **kwargs)

class RobotImpl(object):
    name = "None"

    def __init__(self, task):
        self.task = task

    def get_terminal_conditions(self):
        raise NotImplementedError("You must implement 'get_terminal_conditions' for the robot impl!")

class RobotImpls(object):
    impls = {}

    @staticmethod
    def get(task):
        try:
            robot_impl_type = task["robot_impl"]
        except KeyError as e:
            rospy.logwarn("Warning! Task does not specify robot_impl type [" + str(task) + "]!")
            raise RobotImplException("No robot_impl type provided!", exc_level=ExceptionLevels.BAD_CONFIG, task=task) from e

        try:
            return RobotImpls.impls[robot_impl_type](task=task)
        except KeyError as e:
            rospy.logerr("Error! Unknown RobotImpl type [" + robot_impl_type + "]: " + str(e))
            raise RobotImplException("Unknown RobotImpl type [" + robot_impl_type + "]", exc_level=ExceptionLevels.BAD_CONFIG, task=task) from e


    @staticmethod
    def register(robot):
        if robot.name not in RobotImpls.impls:
            RobotImpls.impls[robot.name] = robot
        elif RobotImpls.impls[robot.name] == robot:
            rospy.logwarn("Ignoring repeated registration of RobotImpl [" + robot.name + "]")
        else:
            rospy.logerr("Error! A robot has already been registered with the given name! [" + robot.name + "]. Current RobotImpl not added")


class TerminalCondition(object):

    def __init__(self, name):
        self.name = name
        pass

    def eval(self):
        raise NotImplementedError("Must implement the 'eval' method for each TerminalCondition!")


class BaseBumperChecker(TerminalCondition):
    def __init__(self, name="bumper"):
        super().__init__(name=name)
        self.collided = False

    def eval(self):
        return "BUMPER_COLLISION" if self.collided else None


def quaternionToYaw(q):
    euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    return yaw

def poseToString(pose):
    if pose is None:
        return "NONE"

    yaw = quaternionToYaw(pose.orientation)

    return str(pose.position.x) + ":" + str(pose.position.y) + ":" + str(yaw)

class TrajTypeRecorder(object):
    def __init__(self):
        self.traj_results = {"axial":0, "radial":0, "none":0}
        self.traj_type_sub = rospy.Subscriber("/traj_gap_type", Int16msg, self.callback, queue_size=5)

    def callback(self, msg):
        gap_type = msg.data
        if gap_type == -1:
            self.traj_results["none"] +=1
        elif gap_type == 1:
            self.traj_results["axial"] +=1
        elif gap_type == 2:
            self.traj_results["radial"] +=1
        else:
            rospy.logerr("Unknown traj result: " + str(gap_type))

    def get_results(self):
        return {"num_traj_type_none": self.traj_results["none"], "num_traj_type_radial": self.traj_results["radial"], "num_traj_type_axial": self.traj_results["axial"]}


class OdomAccumulator(object):
    def __init__(self):
        #Note: it might be more efficient to use the simple action client's callback rather than this separate subscriber
        self.feedback_subscriber = rospy.Subscriber("move_base/feedback", MoveBaseActionFeedback, self.feedbackCB, queue_size=5)
        self.path_length = 0
        self.prev_msg = None
        self.accum_rotation = 0

    def feedbackCB(self, feedback):
        if self.prev_msg is not None:
            prev_pos = self.prev_msg.feedback.base_position.pose.position
            cur_pos = feedback.feedback.base_position.pose.position

            deltaX = cur_pos.x - prev_pos.x
            deltaY = cur_pos.y - prev_pos.y
            deltaZ = cur_pos.z - prev_pos.z

            displacement = math.sqrt(deltaX*deltaX + deltaY*deltaY +deltaZ*deltaZ)
            self.path_length += displacement

            prev_yaw = quaternionToYaw(self.prev_msg.feedback.base_position.pose.orientation)
            curr_yaw = quaternionToYaw(feedback.feedback.base_position.pose.orientation)

            delta_yaw = angles.shortest_angular_distance(curr_yaw, prev_yaw)
            self.accum_rotation += math.fabs(delta_yaw)

        self.prev_msg = feedback

    def getPathLength(self):
        return self.path_length

    def getTotalRotation(self):
        return self.accum_rotation



def reset_costmaps():
    service = rospy.ServiceProxy("move_base/clear_costmaps", std_srvs.Empty)
    service()



class MoveBaseTask:
    #TODO: Add recording capability

    def __init__(self, *, goal_pose, monitor, task=None, **kwargs):
        self.monitor = monitor
        self.goal_pose = goal_pose
        self.timeout = task["timeout"] if task is not None and "timeout" in task else 300
        #record = task["record"] if "record" in task else False
        if len(kwargs) > 0:
            rospy.logwarn("Unrecognized args passed to MoveBaseTask!: " + str(kwargs))
        rospy.loginfo("Beginning navigation test with timeout [" + str(self.timeout) + "]")

        #robot_impl_type = task["robot_impl_type"] if task is not None and "robot_impl_type" in task else "turtlebot"
        self.robot_impl = RobotImpls.get(task)
        self.terminal_conditions = []
        self.task = task

    def run(self):
        self.setup_checkers()   #TODO: give more descriptive name
        self.setup_terminal_conditions()
        self.initialize_action_client()
        self.send_goal()
        self.wait_for_finish()
        self.fill_result()
        return self.get_result()


    def setup_checkers(self):
        self.odom_accumulator = OdomAccumulator()

    def setup_terminal_conditions(self):
        self.terminal_conditions += self.robot_impl.get_terminal_conditions()

    def initialize_action_client(self):
        self.client = SimpleActionClient('move_base', MoveBaseAction)
        print("waiting for server")
        rospy.loginfo("Waiting for MoveBaseActionServer...")

        action_server_wait_time = 20
        try:
            action_server_wait_time = self.task["action_server_wait_time"]
        except (KeyError, TypeError) as e:
            rospy.logdebug("action_server_wait_time not specified, using default [" + str(action_server_wait_time) + "]")
        start_time = rospy.Time.now()
        end_time = start_time + rospy.Duration(action_server_wait_time)

        wall_timeout = 1
        # try:
        #     wall_timeout = min(wall_timeout, action_server_wait_time)
        # except TypeError as e:
        #     pass


        if self.client.wait_for_server(timeout=rospy.Duration(secs=action_server_wait_time), wall_poll_period=0.2):
            print("Done!")
            rospy.loginfo("Found MoveBaseActionServer!")
        else:
            rospy.logerr("MoveBaseActionServer not found!")
            raise TaskProcessingException("MoveBaseActionServer not found!")


        time.sleep(3)


    def send_goal(self):
        # Create the goal point
        goal = MoveBaseGoal()
        goal.target_pose = self.goal_pose
        goal.target_pose.header.stamp = rospy.Time.now()

        self.client.send_goal(goal)


    def check_if_finished(self):
        # TODO: Move this to a TerminalCondition
        state = self.client.get_state()
        # print "State: " + str(state)
        if state is not GoalStatus.ACTIVE and state is not GoalStatus.PENDING:
            return self.get_goal_status()
        else:
            for condition in self.terminal_conditions:
                res = condition.eval()
                if res:
                    return res

        #elif self.bumper_checker.collided:
        #    return "BUMPER_COLLISION"
        #elif self.odom_checker.collided:
        #    return "OTHER_COLLISION"
        #elif self.odom_checker.not_moving:
        #    return "STUCK"
        #TODO: Move this to a TerminalCondition
        if (rospy.Time.now() - self.start_time > rospy.Duration(self.timeout)):
            return "TIMED_OUT"

        return False


    def wait_for_finish(self):
        r = Rate(hz=5, wall_poll_period=0.4)

        self.result={}
        self.start_time = rospy.Time.now()

        #TODO: move failure conditions to classes and just iterate over a list of them
        while True:
            result = self.check_if_finished()
            if result:
                break
            else:
                r.sleep()

            rospy.loginfo_throttle(period=5, msg="Waiting for result...")

        self.result["time"]= str(rospy.Time.now() - self.start_time)
        self.result["result"] = result

    def get_goal_status(self):
        print("done!")

        # 3 means success, according to the documentation
        # http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatus.html
        print("getting goal status")
        print(self.client.get_goal_status_text())
        print("done!")
        print("returning state number")
        state = self.client.get_state()
        if state == GoalStatus.SUCCEEDED:
            result = "SUCCEEDED"
        elif state == GoalStatus.ABORTED:
            result = "ABORTED"
        elif state == GoalStatus.LOST:
            result = "LOST"
        elif state == GoalStatus.REJECTED:
            result = "REJECTED"
        elif state == GoalStatus.ACTIVE:    #TODO: pretty sure this condition will never happen
            result = "TIMED_OUT"
        else:
            result = "UNKNOWN"
        return result

    def fill_result(self):
        self.result['path_length'] = str(self.odom_accumulator.getPathLength())
        self.result['total_rotation'] = str(self.odom_accumulator.getTotalRotation())
        # 'end_pose': poseToString(self.end_pose),

    def get_result(self):
        rospy.loginfo("Got result [" + str(self.result) + "]")
        return self.result


def run_test(goal_pose, monitor=None, task=None, timeout=None, record=None):
    if timeout is not None or record is not None:
        rospy.logwarn("Passing 'timeout' and 'record' to this function is deprecated; the values will be used for now, but please specify them as entries in 'task'")
        if timeout is not None:
            task["timeout"] = timeout
        if record is not None:
            task["record"] = record

    mt = MoveBaseTask(goal_pose=goal_pose, monitor=monitor, task=task)
    return mt.run()

import nav_scripts.turtlebot_impl
