#!/usr/bin/env python
from __future__ import print_function
from builtins import str
from builtins import object
import rospy
#import actionlib
from move_base_msgs.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseArray
from pprint import pprint
import tf
import tf.transformations
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
import tf2_ros
import math
import std_srvs.srv as std_srvs
from sensor_msgs.msg import LaserScan, Image
import rosbag
import datetime
import os
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback
import threading
import time
import angles
from std_msgs.msg import Int16 as Int16msg
from nav_scripts.interruptible import Rate, InterruptedSleepException, SimpleActionClient
from nav_scripts.task_pipeline import TaskProcessingException


class BumperChecker(object):
    def __init__(self):
        self.sub = rospy.Subscriber("mobile_base/events/bumper", BumperEvent, self.bumperCB, queue_size=5)
        self.collided = False

    def bumperCB(self,data):
        if data.state == BumperEvent.PRESSED:
            self.collided = True

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

class ResultRecorder(object):
    def __init__(self):
        from laser_classifier_ros.msg import GlobalSample
        self.lock = threading.Lock()

        bagpath = "~/simulation_data/" + str(datetime.datetime.now()) + ".bag"
        self.bagfilepath = os.path.expanduser(bagpath)
        print("bag file = " + self.bagfilepath + "\n")
        self.bagfile = rosbag.Bag(f=self.bagfilepath, mode='w', compression=rosbag.Compression.LZ4)

        #self.vel_sub = rospy.Subscriber("navigation_velocity_smoother/raw_cmd_vel", Twist, self.twistCB, queue_size=1)
        self.sample_sub = rospy.Subscriber("/move_base/TebLocalPlannerROS/teb_poses", PoseArray, self.sampleCB, queue_size=4)
        self.scan_sub = rospy.Subscriber("point_scan", LaserScan, self.scanCB, queue_size=1)
        self.rgb_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.imageCB, queue_size=1)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odomCB, queue_size=1)


        self.scan = None
        self.feedback = None
        self.odom = None
        self.rgb_image = None
        self.last_sample = None
        self.sample_period = rospy.Duration(1)


    def record(self, odom, scan, rgb_image, feedback, trajectory):
        self.lock.acquire()
        if odom is None or scan is None or rgb_image is None or feedback is None or trajectory is None:
            return

        current_time = rospy.Time.now()

        if True: #self.last_sample is None or current_time - self.last_sample > self.sample_period:

            start_t = time.time()
            self.bagfile.write("scan", scan, scan.header.stamp)
            self.bagfile.write("odom", odom, scan.header.stamp)
            self.bagfile.write('rgb_image', rgb_image, scan.header.stamp)
            self.bagfile.write("feedback", feedback, scan.header.stamp)
            self.bagfile.write("trajectory", trajectory, scan.header.stamp)
            self.last_sample = current_time

            rospy.logdebug("Sample recorded! Took: " + str((time.time() - start_t)*1000) + "ms")

        self.lock.release()


    def twistCB(self, data):
        rospy.logdebug("Command received!")

        if(self.scan is not None and self.feedback is not None):
            self.record(data, self.scan, self.feedback)

    def sampleCB(self, data):
        rospy.logdebug("Trajectory received!")

        self.record(self.odom, self.scan, self.rgb_image, self.feedback, data)

    def scanCB(self, data):
        rospy.logdebug("Scan received!")

        self.lock.acquire()
        self.scan = data
        self.lock.release()
        rospy.logdebug("Scan updated!")

    def imageCB(self, data):
        rospy.logdebug("RGB image received!")

        self.lock.acquire()
        self.rgb_image = data
        self.lock.release()
        rospy.logdebug("RGB image updated!")

    def setGoal(self, data):
        rospy.logdebug("Goal received!")

        self.lock.acquire()
        self.bagfile.write("goal", data, data.target_pose.header.stamp)
        self.lock.release()
        rospy.logdebug("Goal recorded!")


    def feedback_cb(self, data):
        rospy.logdebug("Pose received!")

        self.lock.acquire()
        self.feedback = data
        self.lock.release()
        rospy.logdebug("Pose recorded!")

    def odomCB(self, data):
        rospy.logdebug("Odom received!")

        self.lock.acquire()
        self.odom = data
        self.lock.release()
        rospy.logdebug("Odom recorded!")

    def done(self):
        rospy.logdebug("'Done' Commanded!")

        self.lock.acquire()
        #self.vel_sub.unregister()
        self.scan_sub.unregister()
        self.sample_sub.unregister()
        self.rgb_sub.unregister()
        self.odom_sub.unregister()
        self.bagfile.close()
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

    def __init__(self, *, task, goal_pose, monitor, **kwargs):
        self.monitor = monitor
        self.goal_pose = goal_pose
        self.timeout = task["timeout"] if "timeout" in task else 300
        #record = task["record"] if "record" in task else False
        rospy.logwarn("Unrecognized args passed to MoveBaseTask!: " + str(kwargs))
        rospy.loginfo("Beginning navigation test with timeout [" + str(self.timeout) + "]")

    def run(self):
        self.setup_checkers()
        self.initialize_action_client()
        self.send_goal()
        self.wait_for_finish()
        self.fill_result()
        return self.get_result()


    def setup_checkers(self):
        self.bumper_checker = BumperChecker()
        #self.odom_checker = OdomChecker()
        self.odom_accumulator = OdomAccumulator()

    def initialize_action_client(self):
        self.client = SimpleActionClient('move_base', MoveBaseAction)
        print("waiting for server")
        rospy.loginfo("Waiting for MoveBaseActionServer...")
        while True:
            try:
                if self.client.wait_for_server(timeout=rospy.Duration(secs=20), wall_timeout=1):
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


    def stop_waiting(self):
        state = self.client.get_state()
        # print "State: " + str(state)
        if state is not GoalStatus.ACTIVE and state is not GoalStatus.PENDING:
            return self.get_goal_status()
        elif self.bumper_checker.collided:
            return "BUMPER_COLLISION"
        #elif self.odom_checker.collided:
        #    return "OTHER_COLLISION"
        #elif self.odom_checker.not_moving:
        #    return "STUCK"
        elif (rospy.Time.now() - self.start_time > rospy.Duration(self.timeout)):
            return "TIMED_OUT"

        return False


    def wait_for_finish(self):
        r = Rate(hz=5, timeout=1)

        self.result={}
        self.start_time = rospy.Time.now()

        #TODO: move failure conditions to classes and just iterate over a list of them
        while True:
            result = self.stop_waiting()
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
        elif state == GoalStatus.ACTIVE:
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


def run_test(goal_pose, record=False, timeout=None, monitor=None):
    mt = MoveBaseTask(goal_pose=goal_pose, monitor=monitor, timeout=timeout)
    return mt.run()
