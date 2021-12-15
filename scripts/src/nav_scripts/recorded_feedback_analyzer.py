#!/usr/bin/env python

import time
from nav_scripts.result_analyzer import ResultAnalyzer
from nav_scripts.testing_scenarios import TestingScenarios
import rosbag
import os
import rospy
from geometry_msgs.msg import PoseArray, PoseStamped, Pose, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from tf import transformations
import numpy as np

def preprocess_feedback(case):
    pose_array = None

    if "bag_file_path" in case:
        rosbag_path = case["bag_file_path"]
        if os.path.isfile(rosbag_path):
            with rosbag.Bag(f=rosbag_path, mode='r', skip_index=False) as inbag:
                for topic, msg, t in inbag.read_messages(topics=['feedback']):
                    pose_stamped = msg.base_position

                    if pose_array is None:
                        pose_array = PoseArray(header=pose_stamped.header)

                    pose_array.poses.append(pose_stamped.pose)
                    #print(msg)

    return pose_array



def get_transforming_pose(target_pose, source_pose):
    def pose_to_transform(pose):
        transform = TransformStamped()
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation = pose.orientation
        return transform

    def transform_to_pose(transform):
        pose = Pose()
        pose.orientation = transform.transform.rotation
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        return pose

    def transformToMat(transform_stamped):
        # numpy arrays to 4x4 transform matrix

        transform = transform_stamped.transform
        t_array = [transform.translation.x, transform.translation.y, transform.translation.z]
        quat_array = [transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w]
        trans_mat = transformations.translation_matrix(t_array)
        rot_mat = transformations.quaternion_matrix(quat_array)
        # create a 4x4 matrix
        transf_mat = np.dot(trans_mat, rot_mat)
        return transf_mat

    #Based on http://www.cg.info.hiroshima-cu.ac.jp/~miyazaki/knowledge/teche53.html
    def invertMatTransform(mat):
        inv = np.zeros_like(mat)
        R_t = np.transpose(mat[0:3,0:3])
        inv[0:3,0:3] = R_t
        inv[3,3] = 1
        Rt = -R_t.dot(mat[0:3,3])
        inv[0:3,3] = Rt
        return inv

    def matToTransform(mat):
        trans = TransformStamped()

        # go back to quaternion and 3x1 arrays
        rot_array = transformations.quaternion_from_matrix(mat)
        trans_array = transformations.translation_from_matrix(mat)

        trans.transform.translation.x = trans_array[0]
        trans.transform.translation.y = trans_array[1]
        trans.transform.translation.z = trans_array[2]

        trans.transform.rotation.x = rot_array[0]
        trans.transform.rotation.y = rot_array[1]
        trans.transform.rotation.z = rot_array[2]
        trans.transform.rotation.w = rot_array[3]

        return trans

    To_r = pose_to_transform(pose=source_pose)
    Tr_m = pose_to_transform(pose=target_pose)

    # Approach inspired by https://answers.ros.org/question/215656/how-to-transform-a-pose/?answer=215666#post-id-215666
    To_r_mat = transformToMat(To_r)
    Tr_m_mat = transformToMat(Tr_m)

    #Tm_o_mat = Tr_m_mat.dot(To_r_mat)
    #Tm_o_mat = Tr_m_mat.dot(invertMatTransform(To_r_mat))
    #Tm_o_mat = invertMatTransform(Tr_m_mat).dot(To_r_mat)
    #Tm_o_mat = invertMatTransform(Tr_m_mat).dot(invertMatTransform(To_r_mat))
    #Tm_o_mat = invertMatTransform(invertMatTransform(Tr_m_mat).dot(invertMatTransform(To_r_mat)))
    #Tm_o_mat = invertMatTransform(invertMatTransform(Tr_m_mat).dot(To_r_mat))
    #Tm_o_mat = invertMatTransform(Tr_m_mat.dot(To_r_mat))
    Tm_o_mat = invertMatTransform(Tr_m_mat)


    Tm_o = matToTransform(Tm_o_mat)
    transforming_pose = transform_to_pose(transform=Tm_o)
    return transforming_pose


class RecordedFeedbackAnalyzer(object):

    # cases should be a list returned by calling ResultAnalyzer.getCases()
    def set_cases(self, cases):
        self.cases = cases
        self.scenarios = TestingScenarios()

    def preprocess_feedback(self):
        test_case = self.cases[23]

        for case in [test_case]:
            poses = preprocess_feedback(case=case)
            case["poses"]=poses

    def generate_visualization(self, label="poses", color=ColorRGBA(g=1,a=1)):
        marker = Marker()
        marker.type = Marker.LINE_LIST
        marker.color = color
        marker.ns = label
        marker.scale.x=0.05

        for case in self.cases:
            pose_array = case["poses"] if "poses" in case else None
            if pose_array is not None:
                if marker.header.frame_id=="":
                    marker.header.frame_id = pose_array.header.frame_id
                poses = pose_array.poses
                num_poses = len(poses)
                if num_poses > 0:
                    for i in range(1,num_poses):
                        prev_pose = poses[i-1]
                        pose = poses[i]
                        marker.points.append(prev_pose.position)
                        marker.points.append(pose.position)

                    #Now, need to lookup the starting pose in the world frame in order to generate the appropriate transform
                    scenario = self.scenarios.getScenario(task=case)
                    global_start_pose = scenario.getStartingPoseMsg()
                    odom_start_pose = poses[0]
                    transforming_pose = get_transforming_pose(target_pose=global_start_pose, source_pose=odom_start_pose)
                    #marker.pose = transforming_pose
                    marker.pose = global_start_pose
                    #marker.pose = Pose()
                    #marker.pose.orientation.w = 1
                    #marker.pose.position.x = 1
                    marker.header.frame_id = scenario.nav_frame_id

        return marker



if __name__ == '__main__':
    filenames = ["/home/justin/simulation_data/results_2021-12-15 00:00:33.914481"]

    rospy.init_node("marker_feedback_analyzer")

    pub = rospy.Publisher("feedback_visuals_rm_dot_invor", data_class=MarkerArray, latch=True, queue_size=5)

    feedback_analyser = RecordedFeedbackAnalyzer()

    result_analyzer = ResultAnalyzer()
    result_analyzer.readFiles(filenames=filenames)

    feedback_analyser.set_cases(cases=result_analyzer.getCases())
    feedback_analyser.preprocess_feedback()
    collision_marker = feedback_analyser.generate_visualization(label="collisions", color=ColorRGBA(r=1,a=1))

    marker_array = MarkerArray()
    marker_array.markers.append(collision_marker)

    pub.publish(marker_array)
    rospy.spin()