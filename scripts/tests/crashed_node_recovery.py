import time

import rospy
from nav_scripts.ros_launcher_helper import GazeboLauncher
from nav_scripts.testing_scenarios import TestingScenarios


def run_test(task):
    rospy.set_param('/use_sim_time', 'True')
    rospy.init_node('test_driver', anonymous=True)
    scenarios = TestingScenarios()

    scenario = scenarios.getScenario(task)

    launcher = GazeboLauncher(ros_port=None, gazebo_port=None)
    with launcher:
        launcher.launch(world=scenario.getGazeboLaunchFile(), world_args={'gazebo_gui':True})

        while True:
            launcher.roslaunch_object.spin_once()
            print("Shutting down? Wrapper: " + str(launcher.shutting_down()) + ", pm.done: " + str(launcher.roslaunch_object.pm.done) + ", pm.is_shutdown: " + str(launcher.roslaunch_object.pm.is_shutdown) + ", pm.has_jobs: " + str(launcher.roslaunch_object.pm.has_main_thread_jobs()))
            time.sleep(0.1)

    print("Exiting")


if __name__ == "__main__":
    task = {'scenario': 'campus'}
    run_test(task)