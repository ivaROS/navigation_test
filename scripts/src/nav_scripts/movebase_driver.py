#!/usr/bin/env python
from __future__ import print_function
from builtins import str
from builtins import object
import rospy
import actionlib
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


def run_testImpl(pose):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    print("waiting for server")
    client.wait_for_server()
    print("Done!")

    # Create the goal point
    goal = MoveBaseGoal()
    goal.target_pose.pose.position.x = pose[0]
    goal.target_pose.pose.position.y = pose[1]
    goal.target_pose.pose.position.z = pose[2]
    quaternion = tf.transformations.quaternion_from_euler(pose[3], pose[4], pose[5])
    
    goal.target_pose.pose.orientation.x = quaternion[0]
    goal.target_pose.pose.orientation.y = quaternion[1]
    goal.target_pose.pose.orientation.z = quaternion[2]
    goal.target_pose.pose.orientation.w = quaternion[3]
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    # Send the goal!
    print("sending goal")
    client.send_goal(goal)
    print("waiting for result")
    client.wait_for_result(rospy.Duration(300))
    print("done!")

    # 3 means success, according to the documentation
    # http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatus.html
    print("getting goal status")
    print(client.get_goal_status_text())
    print("done!")
    print("returning state number")
    return client.get_state() == 3

def reset_costmaps():
    service = rospy.ServiceProxy("move_base/clear_costmaps", std_srvs.Empty)
    service()

def run_test(goal_pose, record=False, timeout=None, monitor=None):
    if timeout is None:
        timeout = 300

    rospy.loginfo("Beginning navigation test with timeout [" + str(timeout) + "]")

    # Get a node handle and start the move_base action server
    # init_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)

    # init_pose = PoseWithCovarianceStamped()
    # init_pose.header.frame_id = 'map'
    # init_pose.header.stamp = rospy.Time.now()
    # init_pose.pose.pose.position.x = 0.0
    # init_pose.pose.pose.position.y = 0.0
    # init_pose.pose.pose.position.z = 0.0
    # init_pose.pose.pose.orientation.x = 0.0
    # init_pose.pose.pose.orientation.y = 0.0
    # init_pose.pose.pose.orientation.z = 0.0
    # init_pose.pose.pose.orientation.w = 1.0
    # init_pose.pose.covariance[0] = 0.1; # pos.x
    # init_pose.pose.covariance[7] = 0.1; # pos.y
    # init_pose.pose.covariance[14] = 1000000.0;
    # init_pose.pose.covariance[21] = 1000000.0;
    # init_pose.pose.covariance[28] = 1000000.0;
    # init_pose.pose.covariance[35] = 0.05; # orientation.z

    #init_pub.publish(init_pose)

    # Action client for sending position commands
    bumper_checker = BumperChecker()
    odom_checker = OdomChecker()
    odom_accumulator = OdomAccumulator()
    #traj_recorder = TrajTypeRecorder()

    #record = False

    if record:
        result_recorder = ResultRecorder()

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    print("waiting for server")
    rospy.loginfo("Waiting for MoveBaseActionServer...")
    if client.wait_for_server(timeout=rospy.Duration(secs=20)):
        print("Done!")
        rospy.loginfo("Found MoveBaseActionServer!")
    else:
        rospy.logerr("MoveBaseActionServer not found!")
        return "MoveBaseActionServer not found!"


    # Create the goal point
    goal = MoveBaseGoal()
    goal.target_pose = goal_pose
    goal.target_pose.header.stamp = rospy.Time.now()

    end_pose = None
    def set_cur_pose(data):
        end_pose = data.feedback.base_position.pose

        if record:
            result_recorder.feedback_cb(data)



    if record:
        result_recorder.setGoal(goal)

    # Send the goal!
    print("sending goal")
    rospy.loginfo("Sending goal...")
    if record:
        client.send_goal(goal, feedback_cb=set_cur_pose)
    else:
        client.send_goal(goal)

    print("waiting for result")

    r = rospy.Rate(5)

    start_time = rospy.Time.now()

    result = None

    keep_waiting = True
    while keep_waiting:
        if monitor is not None:
            monitor.update()
        state = client.get_state()
        #print "State: " + str(state)
        if state is not GoalStatus.ACTIVE and state is not GoalStatus.PENDING:
            keep_waiting = False
        elif bumper_checker.collided:
            keep_waiting = False
            result = "BUMPER_COLLISION"
        elif odom_checker.collided:
            keep_waiting = False
            result = "OTHER_COLLISION"
        elif odom_checker.not_moving:
            keep_waiting = False
            result = "STUCK"
        elif (rospy.Time.now() - start_time > rospy.Duration(timeout)):
            keep_waiting = False
            result = "TIMED_OUT"
        else:
            r.sleep()
        rospy.loginfo_throttle(period=5, msg="Waiting for result...")

    task_time = str(rospy.Time.now() - start_time)

    if record:
        result_recorder.done()

    path_length = str(odom_accumulator.getPathLength())
    total_rotation = str(odom_accumulator.getTotalRotation())

    if result is None:
        #client.wait_for_result(rospy.Duration(45))
        print("done!")


        # 3 means success, according to the documentation
        # http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatus.html
        print("getting goal status")
        print(client.get_goal_status_text())
        print("done!")
        print("returning state number")
        #return client.get_state() == 3
        state = client.get_state()
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

    rospy.loginfo("Got result [" + str(result) + "]")
    res = {'result': result, 'time': task_time, 'path_length': path_length, 'end_pose': poseToString(end_pose), 'total_rotation': total_rotation}

    #res.update(traj_recorder.get_results())

    if record:
        res.update({'bag_file_path': result_recorder.bagfilepath})

    return res

if __name__ == "__main__":
    try:
        rospy.init_node('pips_test', anonymous=True)
        run_test()
    except rospy.ROSInterruptException:
        print("Keyboard Interrupt")
