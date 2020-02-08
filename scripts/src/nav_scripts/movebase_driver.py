#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from pprint import pprint
import tf
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
import tf2_ros
import math
import std_srvs.srv as std_srvs
from sensor_msgs.msg import LaserScan
import rosbag
import datetime
import os
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseActionFeedback
import threading
import time

class BumperChecker:
    def __init__(self):
        self.sub = rospy.Subscriber("mobile_base/events/bumper", BumperEvent, self.bumperCB, queue_size=5)
        self.collided = False

    def bumperCB(self,data):
        if data.state == BumperEvent.PRESSED:
            self.collided = True

class ResultRecorder:
    def __init__(self):
        from laser_classifier_ros.msg import GlobalSample
        self.lock = threading.Lock()

        bagpath = "~/simulation_data/" + str(datetime.datetime.now()) + ".bag"
        self.bagfilepath = os.path.expanduser(bagpath)
        print "bag file = " + self.bagfilepath + "\n"
        self.bagfile = rosbag.Bag(f=self.bagfilepath, mode='w', compression=rosbag.Compression.LZ4)

        #self.vel_sub = rospy.Subscriber("navigation_velocity_smoother/raw_cmd_vel", Twist, self.twistCB, queue_size=1)
        self.sample_sub = rospy.Subscriber("/move_base/DWAPlannerROS/global_sample", GlobalSample, self.sampleCB, queue_size=4)
        self.scan_sub = rospy.Subscriber("point_scan", LaserScan, self.scanCB, queue_size=1)

        self.scan = None
        self.feedback = None


    def record(self, twist, scan, feedback):
        self.lock.acquire()
        start_t = time.time()
        self.bagfile.write("scan", scan, scan.header.stamp)
        self.bagfile.write("cmd", twist, scan.header.stamp)
        self.bagfile.write("feedback", feedback, scan.header.stamp)
        self.lock.release()
        rospy.logdebug("Sample recorded! Took: " + str((time.time() - start_t)*1000) + "ms")


    def twistCB(self, data):
        rospy.logdebug("Command received!")

        if(self.scan is not None and self.feedback is not None):
            self.record(data, self.scan, self.feedback)

    def sampleCB(self, data):
        rospy.logdebug("Command received!")

        self.lock.acquire()
        if(self.scan is not None):
            data.scan = self.scan
            self.bagfile.write("global_sample", data, self.scan.header.stamp)
        self.lock.release()

    def scanCB(self, data):
        rospy.logdebug("Scan received!")

        self.lock.acquire()
        self.scan = data
        self.lock.release()
        rospy.logdebug("Scan updated!")


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


    def done(self):
        rospy.logdebug("'Done' Commanded!")

        self.lock.acquire()
        #self.vel_sub.unregister()
        self.scan_sub.unregister()
        self.sample_sub.unregister()
        self.bagfile.close()
        self.lock.release()
        rospy.logdebug("'Done' accomplished!")



#Not currently in use
class OdomChecker:
    def __init__(self):
        #self.odom_timer = rospy.Timer(period = rospy.Duration(1), callback = self.checkOdom)
        self.not_moving = False
        self.collided = False

    def checkOdom(self, event=None):
        try:
            print "timer callback"
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
            print str(trans)
            displacement = math.sqrt(trans.transform.translation.x*trans.transform.translation.x + trans.transform.translation.y*trans.transform.translation.y)
            print "Odom displacement: " + str(displacement)
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
            print str(trans)
            displacement = math.sqrt(trans.transform.translation.x*trans.transform.translation.x + trans.transform.translation.y*trans.transform.translation.y)
            print "map displacement: " + str(displacement)
            if(displacement >.1):
                self.collided = True


        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException),e:
            print e
            pass

class OdomAccumulator:
    def __init__(self):
        self.feedback_subscriber = rospy.Subscriber("move_base/feedback", MoveBaseActionFeedback, self.feedbackCB, queue_size=5)
        self.path_length = 0
        self.prev_msg = None

    def feedbackCB(self, feedback):
        if self.prev_msg is not None:
            prev_pos = self.prev_msg.feedback.base_position.pose.position
            cur_pos = feedback.feedback.base_position.pose.position

            deltaX = cur_pos.x - prev_pos.x
            deltaY = cur_pos.y - prev_pos.y

            displacement = math.sqrt(deltaX*deltaX + deltaY*deltaY)
            self.path_length += displacement


        self.prev_msg = feedback

    def getPathLength(self):
        return self.path_length


def run_testImpl(pose):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    print "waiting for server"
    client.wait_for_server()
    print "Done!"

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
    print "sending goal"
    client.send_goal(goal)
    print "waiting for result"
    client.wait_for_result(rospy.Duration(300))
    print "done!"

    # 3 means success, according to the documentation
    # http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatus.html
    print "getting goal status"
    print(client.get_goal_status_text())
    print "done!"
    print "returning state number"
    return client.get_state() == 3

def reset_costmaps():
    service = rospy.ServiceProxy("move_base/clear_costmaps", std_srvs.Empty)
    service()

def run_test(goal_pose, record=False):
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

    record = False

    if record:
        result_recorder = ResultRecorder()

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    print "waiting for server"
    client.wait_for_server()
    print "Done!"

    # Create the goal point
    goal = MoveBaseGoal()
    goal.target_pose = goal_pose
    goal.target_pose.header.stamp = rospy.Time.now()

    if record:
        result_recorder.setGoal(goal)

    # Send the goal!
    print "sending goal"
    if record:
        client.send_goal(goal, feedback_cb= result_recorder.feedback_cb)
    else:
        client.send_goal(goal)

    print "waiting for result"

    r = rospy.Rate(5)

    start_time = rospy.Time.now()

    result = None

    keep_waiting = True
    while keep_waiting:
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
        elif (rospy.Time.now() - start_time > rospy.Duration(300)):
            keep_waiting = False
            result = "TIMED_OUT"
        else:
            r.sleep()

    task_time = str(rospy.Time.now() - start_time)

    if record:
        result_recorder.done()

    path_length = str(odom_accumulator.getPathLength())

    if result is None:
        #client.wait_for_result(rospy.Duration(45))
        print "done!"


        # 3 means success, according to the documentation
        # http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatus.html
        print "getting goal status"
        print(client.get_goal_status_text())
        print "done!"
        print "returning state number"
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

    if record:
        return {'result': result, 'time': task_time, 'path_length': path_length, 'bag_file_path': result_recorder.bagfilepath}
    else:
        return {'result': result, 'time': task_time, 'path_length': path_length}

if __name__ == "__main__":
    try:
        rospy.init_node('pips_test', anonymous=True)
        run_test()
    except rospy.ROSInterruptException:
        print "Keyboard Interrupt"
