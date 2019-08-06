import rospy
import random
import sys, os, time
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, Point, Quaternion, Transform, TransformStamped, PointStamped, PoseArray
from copy import deepcopy
import threading

import math

from nav_msgs.msg import OccupancyGrid

import numpy as np

import std_srvs.srv as std_srvs

import std_msgs.msg as std_msgs


class CostmapDriver(object):
    def __init__(self, seed=0):
        self.thresh = 99#253 #costmap_2d::INSCRIBED_INFLATED_OBSTACLE
        self.size_x = 0
        self.size_y = 0
        self.resolution = 1
        self.origin_x = 0
        self.origin_y = 0

        self.data = None
        self.lock = threading.Lock()

        #self.inflated_ground_truth_map_topic = "ground_truth/map/inflated"

        self.inflated_ground_truth_map_topic = "/groundtruth_costmap_inflator/costmap/costmap"

        #self.map_sub = rospy.Subscriber(self.inflated_ground_truth_map_topic, OccupancyGrid, self.mapCB, queue_size=1)

        rospy.loginfo("Waiting for message on " + self.inflated_ground_truth_map_topic)
        map = rospy.wait_for_message(self.inflated_ground_truth_map_topic, OccupancyGrid)
        self.mapCB(map)

        self.pose_sub = rospy.Subscriber("/clicked_point", PointStamped, self.pointCB, queue_size=1)

        self.random = random.Random()
        self.seed = seed
        self.random.seed(seed)
        self.nprandom = np.random.RandomState(seed)

    def mapCB(self, map):
        self.lock.acquire()

        self.size_x = map.info.width
        self.size_y = map.info.height
        self.resolution = map.info.resolution
        self.origin_x = map.info.origin.position.x
        self.origin_y = map.info.origin.position.y
        self.data = map.data
        self.preprocessMap()
        self.lock.release()

    def pointCB(self, point):
        self.lock.acquire()
        issafe = self.isSafe(point.point.x, point.point.y)
        self.lock.release()
        print issafe

    def preprocessMap(self):
        map = np.reshape(np.array(self.data), newshape=(self.size_y, self.size_x))
        safe_cells = np.argwhere(map < self.thresh)
        self.safe_poses = np.empty(shape=safe_cells.shape, dtype=np.float32)
        self.safe_poses[:,0] = self.origin_x + (safe_cells[:,1] + 0.5) * self.resolution
        self.safe_poses[:,1] = self.origin_y + (safe_cells[:,0] + 0.5) * self.resolution


    def getSafePose(self):
        self.lock.acquire()
        num_poses = self.safe_poses.shape[0]
        if num_poses == 0:
            return None

        rand_ind = self.nprandom.randint(low=0, high=num_poses)
        pos = self.safe_poses[rand_ind,:]
        self.lock.release()

        pos += self.nprandom.random_sample(size=2)*self.resolution/2

        return pos

    def isSafe(self, wx, wy):
        cost = self.getCost(wx,wy)
        print cost
        return cost < self.thresh

    def getCost(self, wx, wy):
        mx, my = self.worldToMap(wx,wy)
        index = self.cellsToIndex(mx,my)
        cost = self.data[index]
        return cost

    def worldToMap(self, wx, wy):
        mx = (int)((wx - self.origin_x) / self.resolution)
        my = (int)((wy - self.origin_y) / self.resolution)
        return mx, my

    def cellsToIndex(self, mx, my):
        return my * self.size_x + mx

    def indexToCells(self, index):
        my = index / self.size_x
        mx = index - (my * self.size_x)
        return mx, my
    
    def mapToWorld(self, mx, my):
        wx = self.origin_x + (mx + 0.5) * self.resolution
        wy = self.origin_y + (my + 0.5) * self.resolution
        return wx, wy




if __name__ == '__main__':
    try:
        print "Costmap Driver Main\n"
        rospy.init_node("costmap_driver_test")

        driver = CostmapDriver(seed=0)

        pub = rospy.Publisher("safe_poses", PoseStamped, queue_size=10)
        r = rospy.Rate(2)

        poseStamped = PoseStamped()
        poseStamped.header.frame_id="map"
        poseStamped.pose.orientation.w = 1

        while not rospy.is_shutdown():
            if driver.data is not None:
                pos = driver.getSafePose()
                if pos is not None:
                    poseStamped.pose.position.x = pos[0]
                    poseStamped.pose.position.y = pos[1]
                    poseStamped.header.stamp = rospy.Time.now()
                    pub.publish(poseStamped)

            r.sleep()


    except Exception as e:
        print(e)

