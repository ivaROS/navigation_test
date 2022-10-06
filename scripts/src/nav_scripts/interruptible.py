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
        def poll_func():
            raise InterruptedSleepException

        curr_time = rospy.rostime.get_rostime()
        try:
            sleep(duration=self._remaining(curr_time), wall_poll_period=self.timeout, poll_func=poll_func)
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


class HybridTimeout(object):

    def __init__(self, ros_timeout, wall_timeout=None):

        self.ros_timeout_time = rospy.get_rostime() + ros_timeout
        self.wall_timeout_time = time.time() + wall_timeout if wall_timeout is not None else None

    def is_time(self):
        rem_ros, rem_wall = self.time_remaining()
        if rem_ros <= 0:
            return True
        if rem_wall <= 0:
            raise InterruptedSleepException
        return False

    def time_remaining(self):
        rem_ros = self.ros_timeout_time - rospy.get_rostime()
        rem_wall = self.wall_timeout_time - time.time() if self.wall_timeout_time is not None else .001
        return (rem_ros, rem_wall)

    #Sleep for the shorter of the specified ros and wall timeouts
    def sleep(self):
        pass


import actionlib.action_client
class ActionClient(actionlib.action_client.ActionClient):
    ## @brief Waits for the ActionServer to connect to this client
    ##
    ## Often, it can take a second for the action server & client to negotiate
    ## a connection, thus, risking the first few goals to be dropped. This call lets
    ## the user wait until the network connection to the server is negotiated
    def wait_for_server(self, timeout=rospy.Duration(0.0), wall_timeout=None):
        # return super(SimpleActionClient,self).wait_for_server(timeout=timeout)
        #if not isinstance(timeout, genpy.Duration):
        #    timeout = genpy.Duration.from_sec(timeout)

        started = False
        ros_timeout_time = rospy.get_rostime() + timeout
        #print("ros_timeout_time:" + str(ros_timeout_time))
        wall_timeout_time = time.time() + wall_timeout if wall_timeout is not None else None
        while not rospy.is_shutdown():
            if self.last_status_msg:
                server_id = self.last_status_msg._connection_header['callerid']

                if self.pub_goal.impl.has_connection(server_id) and \
                        self.pub_cancel.impl.has_connection(server_id):
                    # We'll also check that all of the subscribers have at least
                    # one publisher, this isn't a perfect check, but without
                    # publisher callbacks... it'll have to do
                    status_num_pubs = 0
                    for stat in self.status_sub.impl.get_stats()[1]:
                        if stat[4]:
                            status_num_pubs += 1

                    result_num_pubs = 0
                    for stat in self.result_sub.impl.get_stats()[1]:
                        if stat[4]:
                            result_num_pubs += 1

                    feedback_num_pubs = 0
                    for stat in self.feedback_sub.impl.get_stats()[1]:
                        if stat[4]:
                            feedback_num_pubs += 1

                    if status_num_pubs > 0 and result_num_pubs > 0 and feedback_num_pubs > 0:
                        started = True
                        break
            #print("ros_time:" + str(rospy.get_rostime()))

            if timeout != rospy.Duration(0.0) and rospy.get_rostime() >= ros_timeout_time:
                break
            elif wall_timeout_time is not None and time.time() >= wall_timeout_time:
                raise InterruptedSleepException

            time.sleep(0.01)

        return started


import threading

import actionlib.simple_action_client
class SimpleActionClient(actionlib.simple_action_client.SimpleActionClient):

    def __init__(self, ns, ActionSpec):
        self.action_client = ActionClient(ns, ActionSpec)
        self.simple_state = actionlib.simple_action_client.SimpleGoalState.DONE
        self.gh = None
        self.done_condition = threading.Condition()

    def wait_for_server(self, timeout=rospy.Duration(0.0), wall_timeout=None):
        return self.action_client.wait_for_server(timeout, wall_timeout)


def sleep(duration, wall_poll_period=None, poll_func=None):
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
    #TODO: support wallclock as well
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
        max_walltime = initial_walltime + wall_poll_period

        def get_wait_time(max_time):
            if wall_poll_period is None:
                wait_time = max_time
            else:
                rem_time = max_walltime - walltime()
                if rem_time < 0:
                    if callable(poll_func):
                        poll_func()
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

