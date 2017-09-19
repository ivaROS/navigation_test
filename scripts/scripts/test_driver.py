#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped
from pprint import pprint
import tf
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
    client.wait_for_result(rospy.Duration(120))
    print "done!"

    # 3 means success, according to the documentation
    # http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatus.html
    print "getting goal status"
    print(client.get_goal_status_text())
    print "done!"
    print "returning state number"
    return client.get_state() == 3
  
def run_test():
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
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    print "waiting for server"
    client.wait_for_server()
    print "Done!"

    # Create the goal point
    goal = MoveBaseGoal()
    goal.target_pose.pose.position.x = -5.0
    goal.target_pose.pose.position.y = 3.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 1.0
    goal.target_pose.pose.orientation.w = 0.0
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()

    # Send the goal!
    print "sending goal"
    client.send_goal(goal)
    print "waiting for result"
    client.wait_for_result(rospy.Duration(45))
    print "done!"

    # 3 means success, according to the documentation
    # http://docs.ros.org/api/actionlib_msgs/html/msg/GoalStatus.html
    print "getting goal status"
    print(client.get_goal_status_text())
    print "done!"
    print "returning state number"
    return client.get_state() == 3

if __name__ == "__main__":
    try:
        rospy.init_node('pips_test', anonymous=True)
        run_test()
    except rospy.ROSInterruptException:
        print "Keyboard Interrupt"
