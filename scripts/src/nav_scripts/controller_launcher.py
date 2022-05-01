from builtins import str
from builtins import object

import rospkg, rospy
import math
import os


from nav_scripts.ros_launcher_helper import RosLauncherHelper, LaunchInfo

class ControllerConfig(object):

    def __init__(self, name, filepath, environment=None):
        self.name=name
        self.filepath = filepath
        self.environment = environment




#TODO: Move all controller launch logic as well as controller-specific data into this class

class ControllerLauncher(RosLauncherHelper):
    impls = {}
    rospack = rospkg.RosPack()

    def __init__(self, ros_port, use_mp):
        super(ControllerLauncher, self).__init__(name="controller", ros_port=ros_port, hide_stdout=True,
                                                 use_mp=use_mp, profile=True)

    # TODO: use kwargs or something to pass everything to the other process?
    def launch(self, robot, controller_name, controller_args=None):
        info = LaunchInfo(info=controller_name, args=controller_args)
        info.robot = robot
        res = RosLauncherHelper.launch(self=self, launch_info=info)
        return res

    def get_bash_file(self, launch_info):
        arg_bash_file = super().get_bash_file(launch_info=launch_info)

        try:
            bash_source_file = ControllerLauncher.getEnvironment(launch_info.info)
        except KeyError as e:
            bash_source_file = None

        if arg_bash_file is not None and arg_bash_file != bash_source_file:
            print(
                "Warning, environment provided in args will overwrite the environment provided by config")
            bash_source_file = arg_bash_file

        return bash_source_file

    def get_launch_files(self, launch_info, rospack):
        controller_name = launch_info.info
        robot = launch_info.robot

        controller_path = None
        if os.path.isfile(controller_name):
            controller_path = controller_name
        elif ControllerLauncher.contains(controller_name):
            controller_path = ControllerLauncher.getPath(name=controller_name)

        if controller_path is None:
            path = rospack.get_path("nav_scripts")
            controller_path = path + "/launch/" + robot + "_" + controller_name + "_controller.launch"

        if controller_path is None:
            raise RuntimeError("Can't find controller [" + str(controller_name) + "] for robot [" + str(robot) + "]")

        if not os.path.isfile(controller_path):
            print("Error! Attempting to load controller from path [" + str(
                controller_path) + "], but the file does not exist!")
            raise FileNotFoundError("Can't find controller at " + str(controller_path))
        else:
            return [controller_path]


    '''
    def launch(self, task):
        if 'controller' not in task:
            return False
        
        controller_name = task['controller']
        controller_args = task["controller_args"] if "controller_args" in task else {}
        task.update(controller_args)  # Adding controller arguments to main task dict for easy logging

        if os.path.isfile(controller_name):
            controller_path = controller_name
        elif ControllerLauncher.contains(controller_name):
            controller_path = ControllerLauncher.getPath(name=controller_name)
        else:
            if 'robot' not in task:
                print "Error! Robot name is required when using default controller launch files"
                return False
            
            path = self.rospack.get_path("nav_scripts")
            controller_path = path + "/launch/" + task['robot'] + "_" + controller_name + "_controller.launch"

        # We'll assume Gazebo is launched are ready to go

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
        #roslaunch.configure_logging(uuid)
        #print path

        #Remapping stdout to /dev/null
        sys.stdout = open(os.devnull, "w")

        # Set environment variables to specify controller arguments
        for key,value in controller_args.items():
            var_name = "GM_PARAM_"+ key.upper()
            value = str(value)
            os.environ[var_name] = value
            print("Setting environment variable [" + var_name + "] to '" + value + "'")

        self.controller_launch = roslaunch.parent.ROSLaunchParent(
            run_id=uuid, roslaunch_files=[controller_path],
            is_core=False, port=self.ros_port #, roslaunch_strs=controller_args
        )
        self.controller_launch.start()

        sys.stdout = sys.__stdout__
    '''


    def getController(self, task):
        try:
            controller_type = task["controller"]
            try:
                return ControllerLauncher.impls[controller_type](task=task)
            except KeyError as e:
                rospy.logerr("Error! Unknown controller type [" + controller_type + "]: " + str(e))
                return None
                #TODO: maybe throw some kind of exception to make it clear to the caller that this case failed?

        except KeyError:
            rospy.logerr("Error! Task does not specify controller type [" + str(task) + "]: " + str(e))

    #TODO: add robot type as optional parameter
    #TODO: allow passing in package name and relative path of controller launch file, then get full path using rospack for the specified environment
    @staticmethod
    def registerController(name, filepath, environment=None):
        if name not in ControllerLauncher.impls:
            ControllerLauncher.impls[name] = ControllerConfig(name=name, filepath=filepath, environment=environment)
        elif ControllerLauncher.impls[name].filepath == filepath:
            rospy.loginfo("Ignoring repeated registration of controller [" + name + "]")
        else:
            rospy.logwarn("Warning! A controller has already been registered with the given name! [" + name + "] but a different path. Old path was [" + str(ControllerLauncher.impls[name]) + "], while new path is [" + str(filepath) + "] The new path will be ignored")

    @staticmethod
    def getControllerTypes():
        return ControllerLauncher.impls

    @staticmethod
    def contains(name):
        return name in ControllerLauncher.impls

    @staticmethod
    def getPath(name):
        return ControllerLauncher.impls[name].filepath

    @staticmethod
    def getEnvironment(name):
        return ControllerLauncher.impls[name].environment

    @staticmethod
    def getFieldNames():
        fieldnames = ["controller"]
        for controller in list(ControllerLauncher.getControllerTypes().values()):
            fieldnames.extend(controller.getUniqueFieldNames())
        return fieldnames

