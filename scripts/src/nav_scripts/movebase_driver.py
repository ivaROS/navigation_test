#!/usr/bin/env python
from __future__ import print_function
from builtins import str
from builtins import object
import rospy
#import actionlib
from move_base_msgs.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseArray, PolygonStamped
from visualization_msgs.msg import MarkerArray
from pprint import pprint
import tf
import tf.transformations
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry, Path, OccupancyGrid, MapMetaData
from kobuki_msgs.msg import BumperEvent
import tf2_ros
import math
import std_srvs.srv as std_srvs
from sensor_msgs.msg import LaserScan, Image, PointCloud2
import rosbag
import datetime
import os
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback
import threading
import time
import angles
from std_msgs.msg import Int16 as Int16msg
from nav_scripts.interruptible import Rate, InterruptedSleepException, SimpleActionClient
from nav_scripts.task_pipeline import TaskProcessingException, ExceptionLevels

from tf2_msgs.msg import TFMessage

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
            rospy.logwarn("Warning! Task does not specify robot_impl type [" + str(task) + "], using 'turtlebot' as default")
            robot_impl_type = "turtlebot"
        except TypeError as e:
            rospy.logwarn("Warning! Task was not provided, using 'turtlebot' as default")
            robot_impl_type = "turtlebot"

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

class MPCStatsRecorder(object):
    def __init__(self, scenario, seed, controller_args=None):
        from potential_gap_mpc.msg import OptimalStats
        self.lock = threading.Lock()

        states_ind = "_"
        if controller_args is not None:
            if 'use_bezier' in controller_args.keys():
                states_ind = "_" + str(controller_args["use_bezier"]) + str(controller_args["use_pose_controller"]) \
                                + str(controller_args["use_keyhole"]) + str(controller_args["use_ni"]) \
                                + str(controller_args["use_po"]) + str(controller_args["has_feedforward"])

        bagpath = "~/simulation_data/sgap_benchmark/rosbag/" + scenario + "_" + str(seed) + states_ind + "_" + str(datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S_%f")) + ".bag"
        self.bagfilepath = os.path.expanduser(bagpath)
        print("bag file = " + self.bagfilepath + "\n")
        self.bagfile = rosbag.Bag(f=self.bagfilepath, mode='w', compression=rosbag.Compression.LZ4)

        self.stats_sub = rospy.Subscriber("/move_base/PGMpcHcLocalPlannerROS/mpc_opt_stats", OptimalStats, self.statsCB, queue_size=4)

        self.stats = None

    def bagfile_path(self):
        return self.bagfilepath

    def statsCB(self, data):
        rospy.logdebug("Stats received!")
        self.stats = data
        # self.record(self.stats)

    def record(self, stats):
        self.lock.acquire()
        self.bagfile.write("stats", stats, rospy.Time.now())
        self.lock.release()

    def done(self):
        rospy.logdebug("'Done' Commanded!")

        self.lock.acquire()
        self.bagfile.write("stats", self.stats, rospy.Time.now())
        self.stats_sub.unregister()
        self.bagfile.close()
        self.lock.release()
        rospy.logdebug("'Done' accomplished!")

class ResultRecorder(object):
    def __init__(self, scenario, seed, controller_args=None):
        import pips_msgs.msg

        self.lock = threading.Lock()

        states_ind = "_"
        if controller_args is not None:
            if 'use_bezier' in controller_args.keys():
                states_ind = "_" + str(controller_args["use_bezier"]) + str(controller_args["use_pose_controller"]) \
                                + str(controller_args["use_keyhole"]) + str(controller_args["use_ni"]) \
                                + str(controller_args["use_po"]) + str(controller_args["has_feedforward"])

        bagpath = "~/simulation_data/sgap_benchmark/rosbag/debug/" + scenario + "_" + str(seed) + states_ind + "_" + str(datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S_%f")) + ".bag"
        self.bagfilepath = os.path.expanduser(bagpath)
        print("bag file = " + self.bagfilepath + "\n")
        self.bagfile = rosbag.Bag(f=self.bagfilepath, mode='w', compression=rosbag.Compression.LZ4)
        self.start_bag = True

        self.map_record = False

        # self.stats = None
        self.odom = None
        self.scan = None
        self.vel = None
        self.pg_arcs = None
        self.ni_traj = None
        self.mpc_traj = None
        self.gap = None
        self.keyhole = None
        self.tf = None
        self.footprint = None

        self.tf_static = None
        self.map = None
        self.map_metadata = None

        # self.stats_sub = rospy.Subscriber("/move_base/PGMpcHcLocalPlannerROS/mpc_opt_stats", OptimalStats, self.statsCB, queue_size=4)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odomCB, queue_size=1)
        self.scan_sub = rospy.Subscriber("/point_scan", LaserScan, self.scanCB, queue_size=1)
        self.vel_sub = rospy.Subscriber("navigation_velocity_smoother/raw_cmd_vel", Twist, self.twistCB, queue_size=1)
        self.pg_arcs_sub = rospy.Subscriber("/pg_arcs", MarkerArray, self.pgArcCB, queue_size=1)
        self.ni_traj_sub = rospy.Subscriber("/move_base/PGMpcHcLocalPlannerROS/GenAndTest/tested_paths", pips_msgs.msg.PathArray, self.niTrajCB, queue_size=1)
        self.mpc_traj_sub = rospy.Subscriber("/move_base/PGMpcHcLocalPlannerROS/mpc_traj", Path, self.mpcTrajCB, queue_size=1)
        self.gap_sub = rospy.Subscriber("/move_base/PGMpcHcLocalPlannerROS/static_inf_gap", MarkerArray, self.gapCB, queue_size=1)
        self.keyhole_sub = rospy.Subscriber("/move_base/PGMpcHcLocalPlannerROS/keyhole_levelset", PointCloud2, self.keyholeCB, queue_size=1)
        self.tf_sub = rospy.Subscriber("/tf", TFMessage, self.tfCB, queue_size=1)
        self.footprint_sub = rospy.Subscriber("/move_base/global_costmap/footprint", PolygonStamped, self.footprintCB, queue_size=1)

        self.tf_static_sub = rospy.Subscriber("/tf_static", TFMessage, self.tfStaticCB, queue_size=1)
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.mapCB, queue_size=1)
        self.map_data_sub = rospy.Subscriber("/map_metadata", MapMetaData, self.mapMetaDataCB, queue_size=1)

    def tfStaticCB(self, data):
        self.lock.acquire()
        # if self.start_bag:
        #     self.bagfile.write("/tf_static", data, rospy.Time.now())
        self.tf_static = data
        self.lock.release()

    def mapCB(self, data):
        self.lock.acquire()
        # if self.start_bag:
        #     self.bagfile.write("/map", data, data.header.stamp)
        self.map = data
        self.lock.release()

    def mapMetaDataCB(self, data):
        self.lock.acquire()
        # if self.start_bag:
        #     self.bagfile.write("/map_metadata", data, rospy.Time.now())
        self.map_metadata = data
        self.lock.release()

    def record(self, odom, tf, footprint, scan, vel, pg_arcs, ni_traj, mpc_traj, gap, keyhole):
        self.lock.acquire()
        # if odom is None or scan is None or rgb_image is None or feedback is None or trajectory is None:
        #     return

        current_time = rospy.Time.now()

        if self.start_bag: #self.last_sample is None or current_time - self.last_sample > self.sample_period:

            start_t = time.time()
            if not self.map_record and self.map is not None and self.tf_static is not None and self.map_metadata is not None:
                self.bagfile.write("/tf_static", self.tf_static, current_time)
                self.bagfile.write("/map", self.map, current_time)
                self.bagfile.write("/map_metadata", self.map_metadata, current_time)

                self.map_record = True

            if tf is not None:
                self.bagfile.write("/tf", tf, current_time)
            if scan is not None:
                self.bagfile.write("/point_scan", scan, current_time)
            if odom is not None:
                self.bagfile.write("/odom", odom, current_time)
            if footprint is not None:
                self.bagfile.write("/move_base/global_costmap/footprint", footprint, current_time)
            if vel is not None:
                self.bagfile.write('navigation_velocity_smoother/raw_cmd_vel', vel, current_time)
            if pg_arcs is not None:
                self.bagfile.write("/pg_arcs", pg_arcs, current_time)
            if ni_traj is not None:
                self.bagfile.write("/move_base/PGMpcHcLocalPlannerROS/GenAndTest/tested_paths", ni_traj, current_time)
            if mpc_traj is not None:
                self.bagfile.write("/move_base/PGMpcHcLocalPlannerROS/mpc_traj", mpc_traj, current_time)
            if gap is not None:
                self.bagfile.write("/move_base/PGMpcHcLocalPlannerROS/static_inf_gap", gap, current_time)
            if keyhole is not None:
                self.bagfile.write("/move_base/PGMpcHcLocalPlannerROS/keyhole_levelset", keyhole, current_time)

            self.last_sample = current_time

            rospy.logdebug("Sample recorded! Took: " + str((time.time() - start_t)*1000) + "ms")

        self.lock.release()

    def bagfile_path(self):
        return self.bagfilepath

    def odomCB(self, data):
        rospy.logdebug("Odom received!")

        # self.lock.acquire()
        self.odom = data
        # self.lock.release()

        self.record(self.odom, self.tf, self.footprint, self.scan, self.vel, self.pg_arcs, self.ni_traj, self.mpc_traj, self.gap, self.keyhole)
        rospy.logdebug("Odom recorded!")

    def footprintCB(self, data):
        self.lock.acquire()
        self.footprint = data
        self.lock.release()

    def tfCB(self, data):
        self.lock.acquire()
        self.tf = data
        self.lock.release()

    def scanCB(self, data):
        rospy.logdebug("Scan received!")

        self.lock.acquire()
        self.scan = data
        self.lock.release()
        rospy.logdebug("Scan updated!")

    def twistCB(self, data):
        rospy.logdebug("Command received!")

        self.lock.acquire()
        self.vel = data
        self.lock.release()
        rospy.logdebug("Command updated!")

    def pgArcCB(self, data):
        rospy.logdebug("PGArcs received!")

        self.lock.acquire()
        self.pg_arcs = data
        self.lock.release()
        rospy.logdebug("PGArcs updated!")

    def niTrajCB(self, data):
        rospy.logdebug("NITraj received!")

        self.lock.acquire()
        self.ni_traj = data
        self.lock.release()
        rospy.logdebug("NITraj updated!")

    def mpcTrajCB(self, data):
        rospy.logdebug("MPCTraj received!")

        self.lock.acquire()
        self.mpc_traj = data
        self.lock.release()
        rospy.logdebug("MPCTraj updated!")

    def gapCB(self, data):
        rospy.logdebug("Gap received!")

        self.lock.acquire()
        self.gap = data
        self.lock.release()
        rospy.logdebug("Gap updated!")

    def keyholeCB(self, data):
        rospy.logdebug("Keyhole received!")

        self.lock.acquire()
        self.keyhole = data
        self.lock.release()
        rospy.logdebug("Keyhole updated!")

    def done(self):
        rospy.logdebug("'Done' Commanded!")

        self.lock.acquire()
        self.vel_sub.unregister()
        self.scan_sub.unregister()
        self.pg_arcs_sub.unregister()
        self.ni_traj_sub.unregister()
        self.odom_sub.unregister()
        self.mpc_traj_sub.unregister()
        self.gap_sub.unregister()
        self.keyhole_sub.unregister()
        self.tf_sub.unregister()
        self.footprint_sub.unregister()

        self.tf_static_sub.unregister()
        self.map_sub.unregister()
        self.map_data_sub.unregister()

        self.bagfile.close()
        self.start_bag = False
        self.lock.release()
        rospy.logdebug("'Done' accomplished!")



#Not currently in use
class OdomChecker(object):
    def __init__(self):
        #self.odom_timer = rospy.Timer(period = rospy.Duration(1), callback = self.checkOdom)
        self.not_moving = False
        self.collided = False

    def checkOdom(self, event=None):
        try:
            print("timer callback")
            now = rospy.Time.now()
            past = now - rospy.Duration(5.0)
            trans = self.tfBuffer.lookup_transform_full(
                target_frame='odom',
                target_time=rospy.Time.now(),
                source_frame='base_footprint',
                source_time=past,
                fixed_frame='odom',
                timeout=rospy.Duration(1.0)
            )
            print(str(trans))
            displacement = math.sqrt(trans.transform.translation.x*trans.transform.translation.x + trans.transform.translation.y*trans.transform.translation.y)
            print("Odom displacement: " + str(displacement))
            if(displacement < .05):
                self.not_moving = True

            past = now - rospy.Duration(1.0)
            trans = self.tfBuffer.lookup_transform_full(
                target_frame='map',
                target_time=now,
                source_frame='odom',
                source_time=past,
                fixed_frame='odom',
                timeout=rospy.Duration(1.0)
            )
            print(str(trans))
            displacement = math.sqrt(trans.transform.translation.x*trans.transform.translation.x + trans.transform.translation.y*trans.transform.translation.y)
            print("map displacement: " + str(displacement))
            if(displacement >.1):
                self.collided = True


        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            print(e)
            pass

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

        if self.task["stat_record"]:
            self.stats_recorder = MPCStatsRecorder(self.task["scenario"], self.task["seed"], self.task["controller_args"])
        
        if self.task["record"]:
            self.recorder = ResultRecorder(self.task["scenario"], self.task["seed"], self.task["controller_args"])

    def run(self):
        self.setup_checkers()   #TODO: give more descriptive name
        self.setup_terminal_conditions()
        self.initialize_action_client()
        self.send_goal()
        self.wait_for_finish()
        self.fill_result()
        if self.task["stat_record"]:
            self.stats_recorder.done()
            self.result["stat_bag_file_path"] = self.stats_recorder.bagfile_path()
        if self.task["record"]:
            self.recorder.done()
            self.result["bag_file_path"] = self.recorder.bagfile_path()
        return self.get_result()


    def setup_checkers(self):
        # self.bumper_checker = BumperChecker()
        #self.odom_checker = OdomChecker()
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
        except KeyError as e:
            rospy.logdebug("action_server_wait_time not specified, using default [" + str(action_server_wait_time) + "]")
        start_time = rospy.Time.now()
        end_time = start_time + rospy.Duration(action_server_wait_time)

        wall_timeout = 1
        try:
            wall_timeout = min(wall_timeout, action_server_wait_time)
        except TypeError as e:
            pass

        while rospy.Time.now() < end_time or not action_server_wait_time:
            try:
                if self.client.wait_for_server(timeout=rospy.Duration(secs=action_server_wait_time), wall_timeout=wall_timeout):
                    print("Done!")
                    rospy.loginfo("Found MoveBaseActionServer!")
                    break
                else:
                    rospy.logerr("MoveBaseActionServer not found!")
                    return TaskProcessingException("MoveBaseActionServer not found!")
            except InterruptedSleepException as e:
                if self.monitor is not None:
                    self.monitor.update()


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
        r = Rate(hz=5, timeout=1)

        self.result={}
        self.start_time = rospy.Time.now()

        #TODO: move failure conditions to classes and just iterate over a list of them
        while True:
            result = self.check_if_finished()
            if result:
                break
            else:
                try:
                    r.sleep()
                except InterruptedSleepException as e:
                    print("Interrupted sleep")
                    pass
                finally:
                    if self.monitor is not None:
                        self.monitor.update()

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
        elif state == GoalStatus.ACTIVE:     #TODO: pretty sure this condition will never happen
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