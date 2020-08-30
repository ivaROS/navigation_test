import rospkg, rospy
import math

#TODO: Move all controller launch logic as well as controller-specific data into this class
class ControllerLauncher:
    impls = {}
    rospack = rospkg.RosPack()

    def __init__(self):
        pass


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
    @staticmethod
    def registerController(name, filepath):
        if name not in ControllerLauncher.impls:
            ControllerLauncher.impls[name] = filepath
        elif ControllerLauncher.impls[name] == filepath:
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
        return ControllerLauncher.impls[name]

    @staticmethod
    def getFieldNames():
        fieldnames = ["controller"]
        for controller in ControllerLauncher.getScenarioTypes().itervalues():
            fieldnames.extend(controller.getUniqueFieldNames())
        return fieldnames