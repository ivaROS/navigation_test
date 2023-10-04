#!/usr/bin/env python3

from nav_scripts.ros_launcher_helper import EnvironmentSourcer, RoslaunchEnvironmentSourcer
import os


def env_only(bash_source_file):
    with EnvironmentSourcer(bash_source_file=bash_source_file):
        print(os.environ.values())
        pass

def env_and_roslaunch(bash_source_file):
    with RoslaunchEnvironmentSourcer(bash_source_file=bash_source_file) as rospack:
        print(os.environ.values())
        print(rospack.ros_paths)
        pass

def try_source(bash_source_file):
    print("\n" + str(bash_source_file))
    try:
        #env_only(bash_source_file=bash_source_file)
        env_and_roslaunch(bash_source_file=bash_source_file)
    except FileNotFoundError as e:
        print("Unable to source: " + str(e))

if __name__ == "__main__":
    default_source = "/home/justin/catkin_ws/rwddevel/setup.bash"
    bad_bash_source = "/home/justin/workspaces/ros_catkin_ws/rddevel/setup.bash"
    good_bash_source = "/home/justin/workspaces/ros_catkin_ws/rwddevel/setup.bash"

    source_list = [None, default_source, bad_bash_source, good_bash_source, None]

    for source in source_list:
        try_source(source)
