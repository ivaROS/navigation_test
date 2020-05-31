#!/usr/bin/env python
import roslaunch
import rospy
import rospkg
import rosbag
import math
import os
from shutil import move
import datetime
import dynamic_reconfigure.client


def get_rosbag_name_fromlaunch(launch_file):
    with open(launch_file) as f:
        lines = f.readlines()
    
    for line in lines:
        if line.find("rosbag") != -1:
            rosbag_name = (line.split("args=\"")[-1])[:-4]
            return rosbag_name
    
    print("failed to parse target rosbag name!")
    return None

def get_rosbag_duration(bag_file_dir):
    if bag_file_dir is None:
        return 5 # arbitrary wait
    try:
        bag = rosbag.Bag(bag_file_dir)
    except:
        print("No such bag file {}".format(bag_file_dir))
        return 5
    duration = bag.get_end_time() - bag.get_start_time()
    bag.close()
    return duration

def run_test(rosbag_launch_dir = 'launch/rosbag', tests = ["scalability"], logging_location = "log_data", test_host_pkg = 'nav_scripts'):

    # Getting log export location
    pkg_dir = rospkg.RosPack().get_path(test_host_pkg)
    data_location = os.path.join(os.path.dirname(pkg_dir), logging_location)
    if not os.path.exists(data_location):
        os.makedirs(data_location)

    # egocircle_launchfile = os.path.join(pkg_dir, 'launch', second_tests[0])
    
    for testtype in tests:
        print("--------Starting {} test--------".format(testtype))

        controller_launchfile_dir = os.path.join(pkg_dir, 'launch', testtype)
        # rosbag_file_dir = os.path.join(pkg_dir, 'launch', "rosbag")
        rosbag_file_dir = os.path.join(pkg_dir, rosbag_launch_dir)
        list_of_log_directorys = list()

        for rosbag_file_name in os.listdir(rosbag_file_dir):
            for launch_file_name in os.listdir(controller_launchfile_dir):
                # Launch the planner
                launch_full_path = os.path.join(controller_launchfile_dir, launch_file_name)
                # play the bag file
                bag_launch_path = os.path.join(rosbag_file_dir, rosbag_file_name)
                rosbag_name = get_rosbag_name_fromlaunch(bag_launch_path)
                rosbag_time = get_rosbag_duration(rosbag_name)
                
                # Get ROS logging location
                uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
                list_of_log_directorys.append(uuid)
                roslaunch.configure_logging(uuid)
                launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_full_path, bag_launch_path])
                rospy.loginfo(launch_full_path)
                rospy.loginfo(bag_launch_path)

                # Run the experiment
                launch.start()
                rospy.loginfo("started")
                rospy.sleep(math.ceil(rosbag_time) + 10)

                rospy.sleep(1)
                launch.shutdown()
                rospy.sleep(1)

                # Finish up
                print("All {} run for {} ended, moving files".format(testtype, launch_file_name))
                timestamp = str(datetime.datetime.now()).replace(' ', '-').replace(':', '-').replace('.', '-').replace('/', '-')

                outputpath = os.path.join(data_location)

                rospy.sleep(10)

                if not os.path.exists(outputpath):
                    os.makedirs(outputpath)
                for x in list_of_log_directorys:
                    file_name = str(x)
                    move(os.path.join(os.path.expanduser('~'), '.ros/log', x), outputpath)
                list_of_log_directorys = list()

def parse_results(result_path = 'demo', test_host_pkg = 'nav_scripts'):
    log_path = os.path.join(os.path.dirname(rospkg.RosPack().get_path(test_host_pkg)), result_path)
    files = os.listdir(log_path)
    files = [os.path.join(log_path, x) for x in files]
    # Latest file
    result = max(files, key = os.path.getctime)

    target_file = None
    for log_file in os.listdir(result):
        file_check = log_file.split('-')
        if (file_check[0] == 'move_base'):
            target_file = log_file
            break
    
    with open(os.path.join(result, target_file)) as file:
        lines = file.readlines()

    result_record = {    
        "updateAllTEBs" : [0, 0],
        "exploreEquivalence" : [0, 0],
        "buildGraph" : [0, 0],
        "optimizeGraph" : [0, 0],
        "buildGraph" : [0, 0],
        "optimizeGraph" : [0, 0],
        "buildGraph" : [0, 0],
        "optimizeGraph" : [0, 0],
        "buildGraph" : [0, 0],
        "optimizeGraph" : [0, 0],
        "computeCurrentCost" : [0, 0],
        "optimizeAllTEBs" : [0, 0],
        "deleteTebDetours" : [0, 0]
    }

    for line in lines:
        ros_log_msg = line.split(":")[-1]
        check1 = ros_log_msg.split('took')
        if len(check1) == 1:
            continue
        
        record_key = check1[0].split("]")
        if len(record_key) == 1:
            continue
        record_key = record_key[0][2:20]
        record_value = check1[-1][1:-7]
        result_record[record_key][0] += 1
        result_record[record_key][1] += float(record_value)
    print(" Category \t\t # of Calls \t\t ave time")
    for k in result_record:
        v = result_record[k]
        print("\033[1m {:<20} {:^20} {:>10}ms \033[0m".format(k, v[0], v[1] / v[0]))
        