import rospy
from nav_scripts.task_pipeline import TaskProcessingException, ExceptionLevels


class RobotImplException(TaskProcessingException):
    def __init__(self, msg="", **kwargs):
        super(TaskProcessingException, self).__init__(msg=msg, **kwargs)

class RobotImpl(object):
    name = "None"

    def __init__(self, task):
        self.task = task

    def get_terminal_conditions(self):
        raise NotImplementedError("You must implement 'get_terminal_conditions' for the robot impl!")

class RobotImpls(object):
    impls = {}

    @staticmethod
    def get(task):
        try:
            robot_impl_type = task["robot_impl"]
        except KeyError as e:
            rospy.logwarn("Warning! Task does not specify robot_impl type [" + str(task) + "], using 'turtlebot' as default")
            robot_impl_type = "turtlebot"
        except TypeError as e:
            rospy.logwarn("Warning! Task was not provided, using 'turtlebot' as default")
            robot_impl_type = "turtlebot"

        try:
            return RobotImpls.impls[robot_impl_type](task=task)
        except KeyError as e:
            rospy.logerr("Error! Unknown RobotImpl type [" + robot_impl_type + "]: " + str(e))
            raise RobotImplException("Unknown RobotImpl type [" + robot_impl_type + "]", exc_level=ExceptionLevels.BAD_CONFIG, task=task) from e


    @staticmethod
    def register(robot):
        if robot.name not in RobotImpls.impls:
            RobotImpls.impls[robot.name] = robot
        elif RobotImpls.impls[robot.name] == robot:
            rospy.logwarn("Ignoring repeated registration of RobotImpl [" + robot.name + "]")
        else:
            rospy.logerr("Error! A robot has already been registered with the given name! [" + robot.name + "]. Current RobotImpl not added")
