#!/usr/bin/env python

import rospy

import std_srvs.srv as std_srvs

from rtabmap_ros.msg import OdomInfo

class OdomResetter():
    def __init__(self):
        self.odom_info_topic = "rtabmap/odom_info"
        self.reset_odom_service_name = "rtabmap/reset_odom"

        rospy.init_node("rtabmap_odom_resetter")

        self.odom_sub = rospy.Subscriber(self.odom_info_topic, OdomInfo, self.OdomCB, queue_size=1)

        self.reset_odom = rospy.ServiceProxy(self.reset_odom_service_name, std_srvs.Empty)

    def OdomCB(self, odom_info):
        if odom_info.lost:
            self.reset_odom()

if __name__ == "__main__":
    OdomResetter()
    rospy.spin()
