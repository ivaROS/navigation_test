from __future__ import division

import time
from builtins import str
from builtins import range
from past.utils import old_div
from builtins import object
import rospkg, rospy
from nav_scripts.gazebo_driver import GazeboDriver
from nav_scripts.costmap_driver import CostmapDriver
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
import numpy as np
import random
import tf
import math
import os
from nav_scripts.task_pipeline import TaskProcessingException, ExceptionLevels


class TestingScenarioError(TaskProcessingException):
    def __init__(self, msg="", **kwargs):
        super(TestingScenarioError, self).__init__(msg=msg, **kwargs)



class TestingScenarios(object):
    impls = {}

    def __init__(self):
        TestingScenario.gazebo_driver = GazeboDriver(as_node=False)

    def getScenario(self, task):
        try:
            scenario_type = task["scenario"]
        except KeyError as e:
            rospy.logerr("Error! Task does not specify scenario type [" + str(task) + "]: " + str(e))
            raise TestingScenarioError("Task does not specify scenario type", exc_level=ExceptionLevels.BAD_CONFIG, task=task) from e
        else:
            try:
                return TestingScenarios.impls[scenario_type](task=task)
            except KeyError as e:
                rospy.logerr("Error! Unknown scenario type [" + scenario_type + "]: " + str(e))
                raise TestingScenarioError("Unknown scenario type [" + scenario_type + "]", exc_level=ExceptionLevels.BAD_CONFIG, task=task) from e


    @staticmethod
    def registerScenario(scenario):
        if scenario.name not in TestingScenarios.impls:
            TestingScenarios.impls[scenario.name] = scenario
        elif TestingScenarios.impls[scenario.name] == scenario:
            rospy.logwarn("Ignoring repeated registration of scenario [" + scenario.name + "]")
        else:
            rospy.logerr("Error! A scenario has already been registered with the given name! [" + scenario.name + "]. Current scenario not added")

    @staticmethod
    def getScenarioTypes():
        return TestingScenarios.impls

    @staticmethod
    def getFieldNames():
        fieldnames = ["scenario"]
        for scenario in list(TestingScenarios.getScenarioTypes().values()):
            fieldnames.extend(scenario.getUniqueFieldNames())
        return fieldnames

def getPoseMsg(pose):
    pose_msg = Pose()
    pose_msg.position.x = pose[0]
    pose_msg.position.y = pose[1]
    if len(pose) == 4:
        pose_msg.position.z = pose[2]

    yaw_ind = 3 if len(pose)==4 else 2
    q = tf.transformations.quaternion_from_euler(0, 0, pose[yaw_ind])
    pose_msg.orientation = Quaternion(*q)

    return pose_msg



#Abstract top level interface. Holds things that are static and common to all scenarios
class TestingScenario(object):
    gazebo_driver = None
    rospack = rospkg.RosPack()
    nav_frame_id = "map"

    def __init__(self, task):
        pass


#Things that are taken care of at this level: seed, random
#TODO: possibly move most of this class to the TestingScenario class, and have this class only be for general cases that specify world, start, and end pose
class GeneralScenario(TestingScenario):
    name = "general"

    def __init__(self, task):
        super(GeneralScenario, self).__init__(task=task)

        self.seed= task['seed'] if 'seed' in task else 0
        self.random = random.Random()
        self.random.seed(self.seed)

        self.init_pose_msg = None
        self.target_pose_msg = None

        self.task = task

        if not hasattr(self, 'world'):
            if "world" not in task:
                rospy.logerr("["+self.name + "] scenario requires 'world' to be specified in task!")
            else:
                self.world = task["world"]

        #TODO: these really are only specific to this class, putting them here breaks inheritance model
        if type(self) == GeneralScenario:
            #TODO: throw exceptions if task does not contain necessary information for scenario


            if "init_pose" not in task:
                rospy.logerr("[general] scenario requires 'init_pose'!")
            else:
                self.init_pose = task["init_pose"]

            if "target_pose" not in task:
                rospy.logerr("[general] scenario requires 'target_pose'!")
            else:
                self.target_pose = task["target_pose"]

    #Override this in scenarios using worlds not included within navigation_test
    def getGazeboLaunchFile(self):
        path = self.rospack.get_path("nav_configs")
        return path + "/launch/gazebo_" + self.world + "_world.launch"

    def getWorldArgs(self):
        args = {}
        #NOTE: gazebo_recording_path only has an effect if gazebo_recording enabled elsewhere, so technically the path
        #could always be added to the world_args. However, with the current system that would unnecessarily clutter
        #the generated results file
        if 'world_args' in self.task:
            world_args = self.task['world_args']

            if 'gazebo_recording' in world_args and world_args['gazebo_recording']:
                controller = self.task["controller"] if 'controller' in self.task else "none"
                repeat = self.task["repeat"] if 'repeat' in self.task else "none"
                task_str = str(self.name) + str(self.seed) + str(controller) + str(repeat) + str('%010x' % random.randrange(16**10))

                gazebo_recording_path = "~/.gazebo/log/nav_bench"
                gazebo_recording_path = os.path.join(gazebo_recording_path, task_str)
                gazebo_recording_path = os.path.expanduser(gazebo_recording_path)
                args['gazebo_recording_path'] = gazebo_recording_path

        return args

    def cleanup(self):
        import subprocess
        if 'gazebo_recording' in self.task and self.task['gazebo_recording']:
            #If logging enabled, need to wait for it to flush to file
            #stop logging
            print("Send command to stop logging...")
            result = subprocess.run(["gz", "log", '--record', "0"], env=os.environ)
            print("Wait 2 seconds...")
            time.sleep(2)
            print("Wait for Gazebo to resume updates...")
            self.gazebo_driver.updateModels(timeout=30)
            print("Hopefully finished writing log files...")
            pass

    #Override this to set starting pose
    def getStartingPose(self):
        return self.init_pose

    #TODO: if start pose specified, use that instead, ex: 'init_pose'
    def getStartingPoseMsg(self):
        if self.init_pose_msg is None:
            start_pose = self.getStartingPose()
        #TODO: possibly add to task?
            self.init_pose_msg = getPoseMsg(start_pose)
        return self.init_pose_msg

    #Override this to set goal
    def getGoal(self):
        return self.target_pose

    #TODO: if goal pose specified, use that instead, ex: 'target_pose'
    def getGoalMsg(self):
        if self.target_pose_msg is None:
            goal = PoseStamped()
            target_pose = self.getGoal()
            goal.pose = getPoseMsg(target_pose)
            goal.header.frame_id = self.nav_frame_id
            self.target_pose_msg = goal
        return self.target_pose_msg

    # Override this to place obstacles, etc
    def setupEnvironment(self):
        pass

    #Override this to for full control over scenario setup
    def setupScenario(self):
        rospy.loginfo("Setting up scenario...")
        # TODO: Check if reset successful; if not, wait briefly and try again,
        # eventually fail and throw error
        rospy.loginfo("Waiting for services, etc...")
        self.gazebo_driver.checkServicesTopics(10)

        self.gazebo_driver.pause()
        rospy.loginfo("Moving robot...")
        self.gazebo_driver.moveRobot(self.getStartingPoseMsg()) #TODO: Use RobotImpl's model_name to specify robot's name
        self.gazebo_driver.resetOdom() #NOTE: This only affects the real-world turtlebot
        self.gazebo_driver.reset(self.seed)
        rospy.loginfo("Configuring environment...")
        self.setupEnvironment()
        self.gazebo_driver.unpause()
        rospy.loginfo("Done!")

    @classmethod
    def getUniqueFieldNames(cls):
        return ["world", "seed", 'init_pose', 'target_pose']

TestingScenarios.registerScenario(GeneralScenario)
