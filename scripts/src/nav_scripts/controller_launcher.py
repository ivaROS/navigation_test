import rospkg, rospy
import math

#TODO: Move all controller launch logic as well as controller-specific data into this class
class ControllerLauncher:
    impls = {}
    rospack = rospkg.RosPack()

    def __init__(self):
        pass

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