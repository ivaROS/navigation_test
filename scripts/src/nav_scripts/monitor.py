import threading

class TaskProcessingMonitor:
    stop_event = threading.Event()
    monitors = {}
    current_exceptions = []
    target_pid = None
    update_period = 1

    @classmethod
    def set_target_thread(cls):
        cls.target_pid = threading.get_native_id()

    @classmethod
    def enable_monitor(cls, monitor):
        print(f"Enabling [{monitor}]")
        cls.monitors[monitor] = True

    @classmethod
    def disable_monitor(cls, monitor):
        print(f"Disabling [{monitor}]")
        cls.monitors[monitor] = False

    @classmethod
    def set_exception(cls, exception):
        cls.current_exceptions.append(exception)
        cls.exc_event.set()

    @classmethod
    def check_monitor(cls):
        if cls.target_pid == threading.get_native_id():
            if cls.exc_event.is_set():
                #TODO: merge multiple exceptions
                exc = cls.current_exceptions[0]
                cls.current_exceptions.clear()
                cls.exc_event.clear()
                raise exc

    @classmethod
    def update(cls):
        for k, v in cls.monitors.items():
            if v:
                try:
                    k.update()
                except Exception as e:
                    cls.set_exception(exception=e)

    @classmethod
    def run(cls):
        while True:
            cls.update()
            time.sleep(cls.update_period)

    @classmethod
    def start(cls):
        cls.thread = threading.Thread(target=cls.run, daemon=True)
        cls.thread.start()

    @classmethod
    def init(cls):
        cls.exc_event = threading.Event()
        cls.start()
        cls.set_target_thread()

#NOTE: the interrupt_monitor could really be used in a more event-driven manner here, by grabbing the Event itself and having a thread block on it
from dataclasses import dataclass

def main():
    import multiprocessing as mp

    def run_instance(name, delay, monitors):
        TaskProcessingMonitor.init()
        print(f"Started {name}")

        def delayed_raise():
            while True:
                time.sleep(delay/2)
                print(f"{name}: {TaskProcessingMonitor.monitors}")

        t = threading.Thread(target=delayed_raise, daemon=True)
        t.start()

        @dataclass(frozen=True)
        class MyM:
            name: str

            def update(self):
                pass

        monitors = [MyM(name=m) for m in monitors]

        for m in monitors:
            TaskProcessingMonitor.enable_monitor(m)
            time.sleep(delay)

    m1 = mp.Process(target=run_instance, args=("monitor1", 2, range(10)))
    m2 = mp.Process(target=run_instance, args=("monitor2", 3, range(10,20)))
    m1.start()
    m2.start()

    m1.join()
    m2.join()

class RosLauncherMonitor(object):
    def __init__(self, launchers):
        self.launchers = launchers

    def append(self, monitor):
        if isinstance(monitor, list):
            self.launchers.extend(monitor)
        else:
            self.launchers.append(monitor)

    def update(self):
        for l in self.launchers:
            l.update()


#RosLauncherHelpers stay the same, just need the context manager to enable/disable checking
    def update(self):
        try:
            if self.roslaunch_object.pm.is_shutdown:
                #self.shutdown()
                raise self.exc_type_runtime(msg="Monitor update failed")
        except AttributeError as e: #If any of those don't exist, something is wrong
            print(str(e))
            raise self.exc_type_runtime(msg="Monitor update failed") from e


import rospy
import genpy
import time

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



import actionlib.action_client
class ActionClient(actionlib.action_client.ActionClient):
    ## @brief Waits for the ActionServer to connect to this client
    ##
    ## Often, it can take a second for the action server & client to negotiate
    ## a connection, thus, risking the first few goals to be dropped. This call lets
    ## the user wait until the network connection to the server is negotiated
    def wait_for_server(self, timeout=rospy.Duration(0.0), wall_poll_period=1):
        # return super(SimpleActionClient,self).wait_for_server(timeout=timeout)
        #if not isinstance(timeout, genpy.Duration):
        #    timeout = genpy.Duration.from_sec(timeout)

        started = False
        ros_timeout_time = rospy.get_rostime() + timeout
        #print("ros_timeout_time:" + str(ros_timeout_time))
        wall_timeout_time = time.time() + wall_poll_period if wall_poll_period is not None else None
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
                TaskProcessingMonitor.check_monitor()
                wall_timeout_time = time.time() + wall_poll_period

            time.sleep(0.01)

        return started


import actionlib.simple_action_client
class SimpleActionClient(actionlib.simple_action_client.SimpleActionClient):

    def __init__(self, ns, ActionSpec):
        self.action_client = ActionClient(ns, ActionSpec)
        self.simple_state = actionlib.simple_action_client.SimpleGoalState.DONE
        self.gh = None
        self.done_condition = threading.Condition()

    def wait_for_server(self, timeout=rospy.Duration(0.0), wall_poll_period=None):
        return self.action_client.wait_for_server(timeout=timeout, wall_poll_period=wall_poll_period)


from nav_scripts.interruptible import InterruptedSleepException

class Rate(rospy.Rate):

    def __init__(self, hz, wall_poll_period=None, reset=False):
        super(Rate,self).__init__(hz=hz, reset=reset)
        self.wall_poll_period = wall_poll_period
        self.target_wall_time = None

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

        def reset_wall_time():
            if self.wall_poll_period is not None:
                self.target_wall_time = time.time() + self.wall_poll_period

        if self.target_wall_time is None:
            reset_wall_time()


        def get_remaining_wall_time():
            if self.wall_poll_period is None:
                return None
            wall_time = time.time()
            time_diff = self.target_wall_time - wall_time
            period = max(0, time_diff)
            print(f"Remaining wall time: {time_diff}, period: {period}")
            return period

        def poll_func():
            raise InterruptedSleepException

        curr_time = rospy.rostime.get_rostime()
        try:
            sleep(duration=self._remaining(curr_time), wall_poll_period=get_remaining_wall_time(), poll_func=poll_func)
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            if not self._reset:
                raise
            self.last_time = rospy.rostime.get_rostime()
            return
        except InterruptedSleepException as e:
            print("Returning from sleep early!")
            if get_remaining_wall_time() == 0:
                TaskProcessingMonitor.check_monitor()
                reset_wall_time()

        self.last_time = self.last_time + self.sleep_dur

        # detect time jumping forwards, as well as loops that are
        # inherently too slow
        if curr_time - self.last_time > self.sleep_dur * 2:
            self.last_time = curr_time



import nav_scripts.patch_sleep



if __name__ == "__main__":
    main()