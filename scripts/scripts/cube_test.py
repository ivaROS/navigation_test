#!/usr/bin/env python
from __future__ import print_function
from builtins import input
from builtins import range
import rospy
import rospkg
import roslaunch

import test_driver
import gazebo_driver
import std_msgs.msg
import sys
import time

launch = None

def gazebo_setup():
    print("Gazebo setup")
    gazebo_driver.reset_world()
    gazebo_driver.spawn_model("cube_20k", (-1, 3.3), "dumpster1")

def gazebo_teardown():
    print("Gazebo teardown")
    gazebo_driver.delete_model("dumpster1")
    # Pause physics?

def launch_controller(controller_name):
    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    path = rospack.get_path('pips_test')

    # Start launchfile
    # We'll assume Gazebo is launched are ready to go
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    global launch
    launch = roslaunch.parent.ROSLaunchParent(uuid, [path + "/launch/" + controller_name])
    launch.start()

    # Give it time to boot up
    time.sleep(10)

def kill_controller():
    # This kills the launch, not the script! Beautiful.
    global launch
    launch.shutdown()
    # print "I can still do stuff here"

def reset_odom(pub):
    pub.publish()

def main(controller_name):
    durations = list()
    for i in range(1):
        odom_pub = rospy.Publisher('/mobile_base/commands/reset_odometry', std_msgs.msg.Empty, queue_size=10)
        rospy.init_node("tester", anonymous=True)

        gazebo_setup()
        reset_odom(odom_pub)
        launch_controller(controller_name)

        start = rospy.get_rostime()
        test_driver.run_test()
        end = rospy.get_rostime()

        durations.append(end-start)
        kill_controller()
        gazebo_teardown()

    for duration in durations:
        print(duration)

if __name__=="__main__":
    print("1: Base Local Planner, 2: Teb local Planner, 3: DWA local planner, 4: Eband local planner, 5: PIPS DWA")
    #Expand This
    controller_name = None
    temp = eval(input("Select your controller type:  "))
    if (temp == 1):
        controller_name = "baseLocalPlanner_controller.launch"
    elif (temp == 2):
        controller_name = "teb_controller.launch"
    elif (temp == 3):
        controller_name = "dwa_controller.launch"
    elif (temp == 4):
        controller_name = "eband_controller.launch"
    elif (temp == 5):
    	controller_name = "pips_dwa_controller.launch"
    main(controller_name)
