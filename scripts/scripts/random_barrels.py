#!/usr/bin/env python
from __future__ import print_function
import rospy
import rospkg
import roslaunch

import test_driver
from gazebo_driver_v2 import Prototype
import std_msgs.msg
import sys
import time

launch = None

def gazebo_setup():
    print("Gazebo setup")
    gazebo_driver.shutdown()
    gazebo_driver.moveBarrels(3)
    # Unpause physics?

#def gazebo_teardown():
    #print "Gazebo teardown"
    #gazebo_driver.delete_barrels(5)
    ## Pause physics?

def launch_controller(controller_name):
    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    path = None
    if (controller_name == "pips_dwa_launcher.launch"):
      path = rospack.get_path("pips_dwa_implementation")
    else:
      path = rospack.get_path('pips_test')

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
    for i in range(3):
        odom_pub = rospy.Publisher('/mobile_base/commands/reset_odometry', std_msgs.msg.Empty, queue_size=10)
        #rospy.init_node("tester", anonymous=True)
        reset_odom(odom_pub)
        gazebo_setup()
        launch_controller(controller_name)

        start = rospy.get_rostime()
        test_driver.run_test()
        end = rospy.get_rostime()

        durations.append(end-start)
        kill_controller()
        reset_odom(odom_pub)
        #gazebo_teardown()

    for duration in durations:
        print(duration)

if __name__=="__main__":
    print("1: Base Local Planner, 2: Teb local Planner, 3: DWA local planner, 4: Eband local planner, 5: PIPS DWA")
    #Expand This
    controller_name = None
    gazebo_driver = Prototype()
    temp = input("Select your controller type:  ")
    if (temp == 1):
        controller_name = "baseLocalPlanner_controller.launch"
    elif (temp == 2):
        controller_name = "teb_controller.launch"
    elif (temp == 3):
        controller_name = "dwa_controller.launch"
    elif (temp == 4):
        controller_name = "eband_controller.launch"
    elif (temp == 5):
    	controller_name = "pips_dwa_launcher.launch"
    print("controller name is " + controller_name)
    main(controller_name)
