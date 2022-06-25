import rospy
import genpy
import time

class InterruptedSleepException(BaseException): pass

class Rate(rospy.Rate):

    def __init__(self, hz, timeout, reset=False):
        super(Rate,self).__init__(hz=hz, reset=reset)
        self.timeout = timeout

    def sleep(self):
        """
        Attempt sleep at the specified rate. sleep() takes into
        account the time elapsed since the last successful
        sleep().

        @raise ROSInterruptException: if ROS shutdown occurs before
        sleep completes
        @raise ROSTimeMovedBackwardsException: if ROS time is set
        backwards
        """
        curr_time = rospy.rostime.get_rostime()
        try:
            sleep(duration=self._remaining(curr_time), timeout=self.timeout)
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            if not self._reset:
                raise
            self.last_time = rospy.rostime.get_rostime()
            return
        except InterruptedSleepException as e:
            print("Returning from sleep early!")
            raise

        self.last_time = self.last_time + self.sleep_dur

        # detect time jumping forwards, as well as loops that are
        # inherently too slow
        if curr_time - self.last_time > self.sleep_dur * 2:
            self.last_time = curr_time


def sleep(duration, timeout=None):
    """
    sleep for the specified duration in ROS time. If duration
    is negative, sleep immediately returns.

    @param duration: seconds (or rospy.Duration) to sleep
    @type  duration: float or Duration
    @raise ROSInterruptException: if ROS shutdown occurs before sleep
    completes
    @raise ROSTimeMovedBackwardsException: if ROS time is set
    backwards
    """
    if rospy.rostime.is_wallclock():
        return rospy.sleep(duration=duration)
    else:
        initial_rostime = rospy.rostime.get_rostime()
        if not isinstance(duration, genpy.Duration):
            duration = genpy.Duration.from_sec(duration)

        rostime_cond = rospy.rostime.get_rostime_cond()

        def walltime():
            return time.time()

        initial_walltime = walltime()
        max_walltime = initial_walltime + timeout

        def get_wait_time(max_time):
            if timeout is None:
                wait_time = max_time
            else:
                rem_time = max_walltime - walltime()
                if rem_time < 0:
                    raise InterruptedSleepException
                wait_time = min(max_time, rem_time)
            #print("Wait time: " + str(wait_time))
            return wait_time

        # #3123
        if initial_rostime == genpy.Time(0):
            # break loop if time is initialized or node is shutdown
            while initial_rostime == genpy.Time(0) and \
                    not rospy.core.is_shutdown():
                with rostime_cond:
                    rostime_cond.wait(get_wait_time(0.3))
                initial_rostime = rospy.rostime.get_rostime()

        sleep_t = initial_rostime + duration

        # break loop if sleep_t is reached, time moves backwards, or
        # node is shutdown
        while rospy.rostime.get_rostime() < sleep_t and \
                rospy.rostime.get_rostime() >= initial_rostime and \
                not rospy.core.is_shutdown():
            with rostime_cond:
                rostime_cond.wait(get_wait_time(0.5))

        if rospy.rostime.get_rostime() < initial_rostime:
            time_jump = (initial_rostime - rospy.rostime.get_rostime()).to_sec()
            raise rospy.exceptions.ROSTimeMovedBackwardsException(time_jump)
        if rospy.core.is_shutdown():
            raise rospy.exceptions.ROSInterruptException("ROS shutdown request")
