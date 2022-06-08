from __future__ import print_function
from __future__ import division

import threading
import time

import rosgraph_msgs.msg
from future import standard_library
from builtins import str
from builtins import range
from past.utils import old_div
from builtins import object

import multiprocessing as mp
import roslaunch
import rospkg
import rospy
from nav_msgs.msg import Odometry
import os
import sys
import cProfile

#TODO: Move the actual launching part into separate process and use queus to send commands and retrieve results

class PipeHelper(object):

    def __init__(self, enabled=True):
        self.enabled = enabled
        if enabled:
            self.pipe_input, self.pipe_output = mp.Pipe(duplex=True)

    def processing_thread(self):
        while True:
            args = self.pipe_output.recv()
            res = self.func(args)
            self.pipe_output.send(res)

    def set_target_function(self, func):
        self.func = func
        if self.enabled:
            self.thread = threading.Thread(target=self.processing_thread)
            self.thread.daemon=True
            self.thread.start()

    def call(self, args=None):
        if self.enabled:
            self.pipe_input.send(args)
            return self.pipe_input.recv()
        else:
            return self.func(args)


class LaunchInfo(object):
    def __init__(self, info=None, args=None):
        self.info = info
        self.args = {} if args is None else args

class LaunchConfig(object):

    def __init__(self):
        self.launch_files = []
        self.bash_source_file = None
        self.args = {}



class RoslaunchShutdownWrapper(roslaunch.parent.ROSLaunchParent):

    def shutdown(self):
        server = self.server.server if self.server is not None and self.server.server is not None else None
        super(RoslaunchShutdownWrapper, self).shutdown()
        if server:
            server.shutdown()

    def __del__(self):
        print("Shutting down now!!!!!!")
        self.shutdown()



# From https://stackoverflow.com/a/7198338
def source_workspace(bash_file):
    import os, subprocess as sp, json

    source = 'source ' + bash_file
    dump = sys.executable + ' -c "from __future__ import print_function;import os, json;print(json.dumps(dict(os.environ)))"'
    pipe = sp.Popen(['/bin/bash', '-c', '%s && %s' % (source, dump)], stdout=sp.PIPE)
    env = json.loads(pipe.stdout.read())
    return env

import errno

class EnvironmentSourcer(object):
    def __init__(self, bash_source_file=None):
        self.bash_source_file = None

        if bash_source_file is not None:
            if os.path.isfile(bash_source_file):
                self.bash_source_file = bash_source_file
            else:
                raise FileNotFoundError(errno.ENOENT, os.strerror(errno.ENOENT), bash_source_file) #https://stackoverflow.com/a/36077407

    def __enter__(self):
        if self.bash_source_file is not None:
            self.old_environ = os.environ
            os.environ = source_workspace(bash_file=self.bash_source_file)


    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.bash_source_file is not None:
            os.environ = self.old_environ


import roslaunch.node_args

class RoslaunchEnvironmentSourcer(EnvironmentSourcer):
    rospack_cache = {}
    roslaunch_rospack_mutex = threading.Lock()

    def __init__(self, bash_source_file=None):
        super(RoslaunchEnvironmentSourcer, self).__init__(bash_source_file=bash_source_file)
        self.roslaunch_rospack_bak = None

    def __enter__(self):
        super(RoslaunchEnvironmentSourcer, self).__enter__()
        try:
            rospack = RoslaunchEnvironmentSourcer.rospack_cache[self.bash_source_file]
        except KeyError as e:
            rospack = rospkg.RosPack()
            RoslaunchEnvironmentSourcer.rospack_cache[self.bash_source_file] = rospack

        #Ensure that no other instance of RoslaunchEnvironmentSourcer attempts to modify roslaunch.node_args._rospack while we are using it
        RoslaunchEnvironmentSourcer.roslaunch_rospack_mutex.__enter__()

        #Backup roslaunch's current rospack instance and replace with desired instance
        self.roslaunch_rospack_bak = roslaunch.node_args._rospack
        roslaunch.node_args._rospack = rospack

        return rospack


    def __exit__(self, exc_type, exc_val, exc_tb):
        super(RoslaunchEnvironmentSourcer, self).__exit__(exc_type=exc_type, exc_val=exc_val, exc_tb=exc_tb)

        #Restore roslaunch's rospack instance
        roslaunch.node_args._rospack = self.roslaunch_rospack_bak

        RoslaunchEnvironmentSourcer.roslaunch_rospack_mutex.__exit__(exc_type, exc_val, exc_tb)


class StdOutputHider(object):

    def __init__(self, enabled):
        self.hide_stdout = enabled

    def __enter__(self):
        if self.hide_stdout:
            # Remapping stdout to /dev/null
            sys.stdout = open(os.devnull, "w")

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.hide_stdout:
            sys.stdout = sys.__stdout__



import socket
import contextlib

def port_in_use(port):
    with contextlib.closing(socket.socket(socket.AF_INET, socket.SOCK_STREAM)) as sock:
        if sock.connect_ex(('127.0.0.1', port)) == 0:
            print("Port " + str(port) + " is in use")
            return True
        else:
            print("Port " + str(port) + " is not in use")
            return False



class PortSelector(object):

    @classmethod
    def port(cls):
        port_object = cls.current_port
        with port_object.get_lock():
            while port_in_use(port_object.value):
                port_object.value += 1
            val = port_object.value
            port_object.value += 1

        return val


class RosPort(PortSelector):
    current_port = mp.Value('i', 11311)



import os
class RosEnv(object):
    port = None

    @staticmethod
    def init(use_existing_roscore=False):
        ros_port = RosEnv.get_ros_port() if use_existing_roscore else RosPort.port()
        RosEnv.set_ros_port(ros_port=ros_port)
        RosEnv.port = ros_port

    @staticmethod
    def set_ros_port(ros_port):
        ros_master_uri = "http://localhost:" + str(ros_port)
        os.environ["ROS_MASTER_URI"] = ros_master_uri

    @staticmethod
    def get_ros_port():
        try:
            ros_master_uri = os.environ["ROS_MASTER_URI"]
        except KeyError as e:
            print("No ROS_MASTER_URI specified!")
            raise e
        else:
            port_ind = ros_master_uri.rindex(":")
            port_str = ros_master_uri[port_ind+1:]
            ros_port = int(port_str)
            return ros_port






#If a launcher raises an error, this class catches it and shuts the launcher down
class LauncherErrorCatcher(object):

    def __init__(self, launcher):
        self.launcher = launcher

    def __enter__(self):
        pass

    def __exit__(self, exc_type, exc_val, exc_tb):
        if exc_type is not None and issubclass(exc_type, type(self.launcher).exc_type):
            print("Caught error from " + str(self.launcher.name) + ", shutting it down")
            self.launcher.shutdown()
        pass


class NavBenchException(BaseException):
    pass

#Raising this should cause everything to shutdown ASAP
class FatalNavBenchException(NavBenchException):
    pass

#Raising this should cause the worker to move on to the next task
class NonFatalNavBenchException(NavBenchException):
    pass

class RosLauncherException(NonFatalNavBenchException):
    pass



class RosLauncherHelper(object):

    @classmethod
    def init(cls):
        class RosLauncherTypeException(RosLauncherException): pass
        class RosLauncherRuntimeException(RosLauncherTypeException): pass
        class RosLauncherLaunchException(RosLauncherTypeException): pass

        cls.exc_type = RosLauncherTypeException
        cls.exc_type_launch = RosLauncherLaunchException
        cls.exc_type_runtime = RosLauncherRuntimeException


    def __init__(self, name, hide_stdout=False, use_mp=False, profile=False, is_core=False):
        #self.ros_port=ros_port
        self.roslaunch_object = None
        self.hide_stdout=hide_stdout
        self.use_mp = use_mp
        self.name = name
        self.profile = profile
        self.is_core = is_core
        self.current_value = None
        self.current_args = None

        #self.ros_master_uri = "http://localhost:" + str(self.ros_port)
        #os.environ["ROS_MASTER_URI"] = self.ros_master_uri

        self.monitor_event = mp.Event()

        if self.use_mp:
            self.startup_event = mp.Event()
            self.launcher_status = PipeHelper(enabled=True)
            self.pipe_input, self.pipe_output = mp.Pipe(duplex=True)
            self.launcher_process = mp.Process(target=self.launch_process)
            self.launcher_process.daemon=True
            print("Starting roslaunch process")
            self.launcher_process.start()
            self.pipe_output.close()
            print("Waiting for roslaunch process to finish initializing")
            self.startup_event.wait()
            print("Roslaunch process event satisfied!")
        else:
            self.launcher_status = PipeHelper(enabled=False)
            self.launcher_status.set_target_function(func=self.status_func)
            self.profiling = cProfile.Profile()
            self.profiling.disable()

    def launch(self, launch_info):
        if self.use_mp:
            print("Process " + str(os.getpid()) + " Requested launch of " + str(launch_info.info) + " with args: " + str(launch_info.args))
            self.pipe_input.send(launch_info)
            res=self.pipe_input.recv()
        else:
            res = self.launch_impl(launch_info=launch_info)

        return res

    def launch_impl(self, launch_info):
        if self.roslaunch_object is not None:
            self.roslaunch_object.shutdown()

        #Perform any necessary processing that doesn't depend on rospack
        bash_source_file = self.get_bash_source(launch_info=launch_info)
        args = launch_info.args

        with RoslaunchEnvironmentSourcer(bash_source_file=bash_source_file) as rospack:
            launch_files = self.get_launch_files(launch_info=launch_info, rospack=rospack)
            #TODO: verify that all files exist?

            print("Launching Process [" + self.name + "] with pid: " + str(os.getpid()))
            print("Launch files=" + str(launch_files) + "\nargs=" + str(args) + "\nbash_source_file=" + str(bash_source_file))

            uuid = roslaunch.rlutil.get_or_generate_uuid(None, not self.is_core)

            for key, value in list(args.items()):
                var_name = "GM_PARAM_" + key.upper()
                value = str(value)
                os.environ[var_name] = value
                print("Setting environment variable [" + var_name + "] to '" + value + "'")

            with open(os.devnull, "w") if self.hide_stdout else contextlib.nullcontext() as error_out:
                with contextlib.redirect_stderr(error_out) if self.hide_stdout else contextlib.nullcontext():
                    with StdOutputHider(enabled=self.hide_stdout):
                        try:
                            self.roslaunch_object = RoslaunchShutdownWrapper(
                                run_id=uuid, roslaunch_files=launch_files,
                                is_core=self.is_core, port=RosEnv.port
                            )
                            self.roslaunch_object.start()
                        except roslaunch.core.RLException as e:
                            raise type(self).exc_type_launch from e


        return True


    def start_monitor_thread(self):
        def monitor_thread_func():
            pass


        self.monitor_thread = threading.Thread(target=monitor_thread_func)


    #Check if everything is running correctly; raise appropriate exception if not
    def update(self):
        try:
            if self.roslaunch_object.pm.is_shutdown:
                raise self.exc_type_runtime
        except AttributeError as e:
            print(str(e))
            raise self.exc_type_runtime from e



    #Override for additional launcher-specific processing
    def get_bash_source(self, launch_info):

        if 'bash_source_file' in launch_info.args:
            arg_bash_file = launch_info.args['bash_source_file']
            del launch_info.args['bash_source_file']
            return arg_bash_file
        return None


    #Override this in derived classes
    def get_launch_files(self, launch_info, rospack):
        print("Error! 'get_launch_file' has not been implemented for this launcher!")
        raise NotImplementedError()
    

    def status_func(self, cmd):
        if cmd is None:
            return self.roslaunch_object._shutting_down if self.roslaunch_object is not None else False
        elif cmd =="profile":
            self.profiling.enable()
        elif cmd=="noprofile":
            self.profiling.disable()
            self.profiling.print_stats('cumulative')
        elif cmd is True:
            if self.roslaunch_object is not None:
                res = self.roslaunch_object.shutdown()
                self.roslaunch_object = None
                return res
            else:
                return True


    def launch_process(self):
        self.pipe_input.close()

        #status = lambda c : self.roslaunch_object._shutting_down
        self.launcher_status.set_target_function(func=self.status_func)
        self.startup_event.set()

        self.profiling = cProfile.Profile()
        self.profiling.disable()

        print("Finished setting up roslaunch process!")

        keep_going = True
        while keep_going:
            launch_info = self.pipe_output.recv()
            res=self.launch_impl(launch_info=launch_info)
            self.pipe_output.send(res)

    #TODO: shutdown 'dependent' launchers as well?
    def shutdown(self):
        self.launcher_status.call(True)
        print("Shut down [" +str(self.name) + ']')
        self.current_value=None
        self.current_args=None

    def shutting_down(self):
        return self.launcher_status.call(None)

    def enable_profiling(self):
        return self.launcher_status.call("profile")

    def disable_profiling(self):
        return self.launcher_status.call("noprofile")

    def __enter__(self):
        pass

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.shutdown()

class RosLauncherMonitor(object):
    def __init__(self, launchers):
        self.launchers = launchers

    def update(self):
        for l in self.launchers:
            l.update()

class RoscoreLauncher(RosLauncherHelper):
    def __init__(self, use_existing_roscore):
        super(RoscoreLauncher, self).__init__(name="core", hide_stdout=False, use_mp=False, profile=False, is_core=True)
        RosEnv.init(use_existing_roscore=use_existing_roscore)
        self.use_existing_roscore = use_existing_roscore

    def __enter__(self):
        if not self.use_existing_roscore:
            self.launch()
        else:
            print("Not starting a new ROS Core")
            #TODO: Verify that the port number specified by environment variables matches that given by RosPort

    def __exit__(self, exc_type, exc_val, exc_tb):
        print("GazeboMaster shutdown: killing core...")
        super(RoscoreLauncher, self).__exit__(exc_type, exc_val, exc_tb)

    def launch(self):
        RosLauncherHelper.launch(self=self, launch_info=LaunchInfo())

    def get_launch_files(self, launch_info, rospack):
        return []

    def update(self):
        return self.use_existing_roscore or RosLauncherHelper.update(self)

RoscoreLauncher.init()



class GazeboPort(PortSelector):
    current_port = mp.Value('i', 11411)

from rosgraph_msgs.msg import Clock as ClockMsg

class GazeboLauncher(RosLauncherHelper):
    gazebo_launch_mutex = mp.Lock()

    def __init__(self, robot_launcher=None):
        super(GazeboLauncher, self).__init__(name="gazebo", hide_stdout=False, use_mp=False, profile=False, is_core=False)

        gazebo_port = GazeboPort.port()
        gazebo_master_uri = "http://localhost:" + str(gazebo_port)
        os.environ["GAZEBO_MASTER_URI"] = gazebo_master_uri

        self.robot_launcher=robot_launcher

    def launch(self, world, world_args=None):
        if world == self.current_value and world_args == self.current_args:
            if not self.shutting_down():
                print("No need to launch gazebo world, already running!")
                return True
            else:
                print("Gazebo crashed, restarting")

        print("Launching..." + str(world))

        #New Gazebo world, so will need new robot
        if self.robot_launcher is not None:
            self.robot_launcher.shutdown()

        self.current_value = world
        self.current_args = world_args

        if world_args is None:
            world_args = {}

        if not 'gazebo_gui' in world_args:
            world_args = {'gazebo_gui':'false'}

        info = LaunchInfo(info=[world], args=world_args)

        #with self.gazebo_launch_mutex:
        with GazeboLauncher.gazebo_launch_mutex:
            res = RosLauncherHelper.launch(self=self, launch_info=info)

        try:
            msg = rospy.wait_for_message("/clock", ClockMsg, 30)
        except rospy.exceptions.ROSException as e:
            print("Error! clock not received!")
            raise self.exc_type_launch("No clock message received!") from e

        return res

    def get_launch_files(self, launch_info, rospack):
        return launch_info.info

GazeboLauncher.init()


class RobotLauncher(RosLauncherHelper):
    def __init__(self):
        super(RobotLauncher, self).__init__(name="gazebo", hide_stdout=False, use_mp=False,
                                             profile=False, is_core=False)
        self.current_value = None
        self.current_args = None

    def launch(self, robot, robot_args=None):
        rospy.loginfo("Launching Gazebo Robot [" + robot + "] with args [" + str(robot_args) + "]")

        if robot == self.current_value and robot_args == self.current_args:
            if not self.shutting_down():
                return
            else:
                print("Error with robot, restarting")

        self.current_value = robot
        self.current_args = robot_args

        info = LaunchInfo(info=robot, args=robot_args)
        res = RosLauncherHelper.launch(self=self, launch_info=info)

        odom_topic = "/odom"
        if 'odom_topic' in robot_args:
            odom_topic = robot_args['odom_topic']
        # Wait for gazebo simulation to be running
        try:
            msg = rospy.wait_for_message(odom_topic, Odometry, 30)
        except rospy.exceptions.ROSException as e:
            print("Error! /odom not received!")
            raise self.exc_type_launch("No message received on odom_topic [" + odom_topic + "]") from e

        return res

    def get_launch_files(self, launch_info, rospack):
        robot = launch_info.info
        if os.path.isfile(robot):
            robot_path = robot
        else:
            robot_path = rospack.get_path("nav_configs") + "/launch/spawn_" + robot + ".launch"

            if not os.path.isfile(robot_path):
                rospy.logerr(msg="Error! Cannot find robot!")
                raise RuntimeError("Cannot find robot " + str(robot))

        return [robot_path]

RobotLauncher.init()