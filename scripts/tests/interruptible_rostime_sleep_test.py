import rospy
from nav_scripts.interruptible import Rate, InterruptedSleepException


def interruptible_sleep_test(rate, timeout):
    r = Rate(hz=rate, wall_timeout=timeout)

    while True:
        try:
            r.sleep()
        except InterruptedSleepException as e:
            print("Do some other chore")
        else:
            print("Do main chore")



if __name__ == "__main__":
    rospy.init_node(name="interruptible_rostime_sleep_test")
    interruptible_sleep_test(rate=1, timeout=5)