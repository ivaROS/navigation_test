#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped
from pprint import pprint
import tf
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
import tf2_ros
import math


class BumperChecker:
    def __init__(self):
        self.sub = rospy.Subscriber("mobile_base/events/bumper", BumperEvent, self.bumperCB, queue_size=5)
        self.collided = False

    def bumperCB(self,data):
        if data.state == BumperEvent.PRESSED:
            self.collided = True

class OdomChecker:
    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer)
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
  
def run_test(goal_pose):
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

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    print "waiting for server"
    client.wait_for_server()
    print "Done!"

    # Create the goal point
    goal = MoveBaseGoal()
    goal.target_pose = goal_pose
    goal.target_pose.header.stamp = rospy.Time.now()

    # Send the goal!
    print "sending goal"
    client.send_goal(goal)
    print "waiting for result"

    r = rospy.Rate(5)

    start_time = rospy.Time.now()

    keep_waiting = True
    while keep_waiting:
        state = client.get_state()
        #print "State: " + str(state)
        if state is not GoalStatus.ACTIVE and state is not GoalStatus.PENDING:
            keep_waiting = False
        elif bumper_checker.collided:
            keep_waiting = False
            return "BUMPER_COLLISION"
        elif odom_checker.collided:
            keep_waiting = False
            return "OTHER_COLLISION"
        elif odom_checker.not_moving:
            keep_waiting = False
            return "STUCK"
        elif (rospy.Time.now() - start_time > rospy.Duration(120)):
            keep_waiting = False
            return "TIMED_OUT"
        else:
            r.sleep()


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
        return {'result':"SUCCEEDED", 'time': str(rospy.Time.now() - start_time) }
    elif state == GoalStatus.ABORTED:
        return "ABORTED"
    elif state == GoalStatus.LOST:
        return "LOST"
    elif state == GoalStatus.REJECTED:
        return "REJECTED"
    elif state == GoalStatus.ACTIVE:
        return "TIMED_OUT"

    else:
        return "UNKNOWN"

if __name__ == "__main__":
    try:
        rospy.init_node('pips_test', anonymous=True)
        run_test()
    except rospy.ROSInterruptException:
        print "Keyboard Interrupt"
