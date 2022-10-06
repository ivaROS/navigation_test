from nav_scripts.monitor import TaskProcessingMonitor
from nav_scripts.interruptible import sleep
import rospy

def monitored_sleep(self):
    """
    Attempt sleep at the specified rate. sleep() takes into
    account the time elapsed since the last successful
    sleep().

    @raise ROSInterruptException: if ROS shutdown occurs before
    sleep completes
    @raise ROSTimeMovedBackwardsException: if ROS time is set
    backwards
    """

    def poll_func():
        TaskProcessingMonitor.check_monitor()

    curr_time = rospy.rostime.get_rostime()
    try:
        sleep(duration=self._remaining(curr_time), wall_poll_period=0.2, poll_func=poll_func)
    except rospy.exceptions.ROSTimeMovedBackwardsException:
        if not self._reset:
            raise
        self.last_time = rospy.rostime.get_rostime()
        return

    self.last_time = self.last_time + self.sleep_dur

rospy.Rate.sleep = monitored_sleep