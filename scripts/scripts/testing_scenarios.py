import rospkg
from gazebo_driver_v2 import GazeboDriver
from geometry_msgs.msg import Pose, PoseStamped

class TestingScenarios:
    def __init__(self):
        self.gazebo_driver = GazeboDriver(as_node=False)

    def getScenario(self, task):
        if "scenario" in task:
            scenario_type = task["scenario"]
            if scenario_type == "trashcans":
                return TrashCanScenario(task=task, gazebo_driver=self.gazebo_driver)
        elif "init_pose" in task and "goal" in task and "world" in task:
            return TestingScenario(task["world"],task["init_pose"],task["goal"],self.gazebo_driver)
        else:
            return None



class TestingScenario:
    def __init__(self, world, init_pose, target_pose, gazebo_driver):
        self.gazebo_driver = gazebo_driver
        self.world = world
        self.init_pose = init_pose
        self.target_pose = target_pose

    def getGazeboLaunchFile(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path("nav_configs")
        return path + "/launch/gazebo_" + self.world + "_world_shallow.launch"

    def getStartingPose(self):
        return self.init_pose

    def getGoal(self):
        return self.target_pose

    def setupScenario(self):
        print "Resetting robot..."
        # TODO: Check if reset successful; if not, wait briefly and try again,
        # eventually fail and throw error
        self.gazebo_driver.pause()
        self.gazebo_driver.moveRobot(self.init_pose)
        self.gazebo_driver.resetOdom()
        self.gazebo_driver.unpause()




class TrashCanScenario(TestingScenario):
    def __init__(self, task, gazebo_driver):
        self.gazebo_driver = gazebo_driver

        self.world = "rectangular"

        self.seed = task["seed"] if "seed" in task else 0
        self.num_barrels = task["num_barrels"] if "num_barrels" in task else 0

        self.init_pose = Pose()
        self.init_pose.position.x = 2
        self.init_pose.position.y = 3
        self.init_pose.orientation.x = 0
        self.init_pose.orientation.y = 0
        self.init_pose.orientation.z = 1
        self.init_pose.orientation.w = 0

        self.target_pose = PoseStamped()
        self.target_pose.pose.position.x = -5.0
        self.target_pose.pose.position.y = 3.0
        self.target_pose.pose.orientation.x = 0.0
        self.target_pose.pose.orientation.y = 0.0
        self.target_pose.pose.orientation.z = 1.0
        self.target_pose.pose.orientation.w = 0.0
        self.target_pose.header.frame_id = 'map'



    def setupScenario(self):
        self.gazebo_driver.checkServicesTopics(10)
        self.gazebo_driver.pause()
        self.gazebo_driver.moveRobot(self.init_pose)
        self.gazebo_driver.resetOdom()
        self.gazebo_driver.reset(self.seed)
        self.gazebo_driver.moveBarrels(self.num_barrels)
        self.gazebo_driver.unpause()
