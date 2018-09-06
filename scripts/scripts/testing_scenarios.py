import rospkg
from gazebo_driver_v2 import GazeboDriver
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
import numpy as np
import random
import tf

class TestingScenarios:
    def __init__(self):
        self.gazebo_driver = GazeboDriver(as_node=False)

    def getScenario(self, task):
        if "scenario" in task:
            scenario_type = task["scenario"]
            if scenario_type == "trashcans":
                return TrashCanScenario(task=task, gazebo_driver=self.gazebo_driver)
            elif scenario_type == "fourth_floor":
                return FourthFloorScenario(task=task, gazebo_driver=self.gazebo_driver)
            elif scenario_type == "campus":
                return CampusScenario(task=task, gazebo_driver=self.gazebo_driver)
            elif scenario_type == "sector":
                return SectorScenario(task=task, gazebo_driver=self.gazebo_driver)
            elif scenario_type == "sparse":
                return SparseScenario(task=task, gazebo_driver=self.gazebo_driver)
            else:
                print "Error! Unknown scenario type [" + scenario_type + "]"
                return None
        elif "init_pose" in task and "goal" in task and "world" in task:
            return TestingScenario(task["world"],task["init_pose"],task["goal"],self.gazebo_driver)
        else:
            return None

    @staticmethod
    def getScenarioTypes():
        scenarios = [TrashCanScenario, SectorScenario, CampusScenario]
        return scenarios


    @staticmethod
    def getFieldNames():
        fieldnames = ["scenario"]
        for scenario in TestingScenarios.getScenarioTypes():
            fieldnames.extend(scenario.getUniqueFieldNames())
        return fieldnames

class TestingScenario:
    def __init__(self, world, init_pose, target_pose, gazebo_driver):
        self.gazebo_driver = gazebo_driver
        self.world = world
        self.init_pose = init_pose
        self.target_pose = target_pose

    def getGazeboLaunchFile(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path("nav_configs")
        return path + "/launch/gazebo_" + self.world + "_world.launch"

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

    @staticmethod
    def getUniqueFieldNames():
        return [""]


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
        self.target_pose.pose.orientation.z = 0
        self.target_pose.pose.orientation.w = 1
        self.target_pose.header.frame_id = 'map'

    @staticmethod
    def getUniqueFieldNames():
        return ["num_barrels", "seed"]

    def setupScenario(self):
        self.gazebo_driver.checkServicesTopics(10)
        self.gazebo_driver.pause()
        self.gazebo_driver.moveRobot(self.init_pose)
        self.gazebo_driver.resetOdom()
        self.gazebo_driver.reset(self.seed)
        self.gazebo_driver.moveBarrels(self.num_barrels)
        self.gazebo_driver.unpause()



class CampusScenario(TestingScenario):
    def __init__(self, task, gazebo_driver):
        self.gazebo_driver = gazebo_driver

        self.world = "campus"

        self.seed = task["seed"] if "seed" in task else 0
        self.num_barrels = task["num_barrels"] if "num_barrels" in task else 0

        self.init_id = task["init_id"] if "init_id" in task else 0

        self.target_id = task["target_id"] if "target_id" in task else 0


        self.random = random.Random()
        self.random.seed(self.seed)

        self.init_pose = Pose()
        self.init_pose.position.x = -12
        self.init_pose.position.y = 4
        self.init_pose.orientation.x = 0
        self.init_pose.orientation.y = 0
        self.init_pose.orientation.z = 0
        self.init_pose.orientation.w = 1

        self.target_poses = [[-12,4,1.57], [13,-10,1.57], [13,9.4,1.57], [-5.15,-9.25,1.57], [-13.5,15.25,0], [5.5,6.5,-1.57], [1.5,2,-1.57]]
        self.init_poses = [[-13,.5,0]]

        self.target_pose = PoseStamped()
        self.target_pose.pose.position.x = 4
        self.target_pose.pose.position.y = 16
        self.target_pose.pose.orientation.x = 0.0
        self.target_pose.pose.orientation.y = 0.0
        self.target_pose.pose.orientation.z = 0.0
        self.target_pose.pose.orientation.w = 1.0
        self.target_pose.header.frame_id = 'map'

        Zone1= [[-14.21, 9.98],[-5, 6]]
        Zone2= [[-6.1, 15.5],[-0.2, 12.96]]
        Zone3= [[-4.61, 7.75],[-0.97, 5.08]]
        Zone4= [[-3.64, 3.53],[-0.96, 1.707]]
        Zone5= [[-3.96, 0.68],[-1, -1]]
        Zone6= [[-5, -2],[0, -5]]
        Zone7= [[-9.77, -5.18],[-5.46, -7.01]]
        Zone8= [[-10.95, -0.85],[-8.59, -1.54]]
        Zone9= [[9.49, 4],[13.45, -6.83]]
        Zone10= [[0.64, -5.94],[6, -11.27]]
        Zone11= [[3.61, 2.76],[7, 0]]
        Zone12= [[0.61, -0.25],[3.94, -2.07]]
        Zone13= [[0.34, 11.7],[11.65, 10.65]]

        zones = [Zone1,Zone2,Zone3,Zone4,Zone5,Zone6,Zone7,Zone8,Zone9,Zone10,Zone11,Zone12,Zone13]

        zones = np.swapaxes(zones,0,2)
        self.minx = zones[0][0]
        self.maxx = zones[0][1]
        self.maxy = zones[1][0]
        self.miny = zones[1][1]
        pass

    @staticmethod
    def getUniqueFieldNames():
        return ["num_barrels", "seed", "target_id", "init_id"]

    def getPoseMsg(self, pose):
        pose_msg = Pose()
        pose_msg.position.x = pose[0]
        pose_msg.position.y = pose[1]

        q = tf.transformations.quaternion_from_euler(0, 0, pose[2])
        # msg = Quaternion(*q)

        pose_msg.orientation = Quaternion(*q)

        return pose_msg

    def getStartingPose(self):
        pose = self.init_poses[self.init_id]
        init_pose = self.getPoseMsg(pose=pose)

        return init_pose

    def getGoal(self):
        pose = self.target_poses[self.target_id]
        init_pose = self.getPoseMsg(pose=pose)
        pose_stamped = PoseStamped()
        pose_stamped.pose = init_pose
        pose_stamped.header.frame_id="map"
        return pose_stamped

    def setupScenario(self):
        self.gazebo_driver.checkServicesTopics(10)
        self.gazebo_driver.pause()
        self.gazebo_driver.moveRobot(self.getStartingPose())
        self.gazebo_driver.resetOdom()
        self.gazebo_driver.reset(self.seed)
        self.gazebo_driver.moveBarrels(self.num_barrels, minx=self.minx, maxx=self.maxx, miny=self.miny, maxy=self.maxy)
        self.gazebo_driver.unpause()




class SectorScenario(TestingScenario):
    def __init__(self, task, gazebo_driver):
        self.gazebo_driver = gazebo_driver

        self.world = "sector_laser"

        self.seed = task["seed"] if "seed" in task else 0

        self.poses = [[-9,9,-.78], [-9,0,0], [-9,-9,.78], [9,-9,2.36], [9,0,3.14], [9,9,-2.36]   ] # [0,-9,1.57]


        self.init_id = task["init_id"] if "init_id" in task else 0

        self.target_id = task["target_id"] if "target_id" in task else (self.init_id + len(self.poses)/2) % len(self.poses)


        self.random = random.Random()
        self.random.seed(self.seed)



    @staticmethod
    def getUniqueFieldNames():
        return ["num_barrels", "seed", "target_id", "init_id"]

    def getPoseMsg(self, pose):
        pose_msg = Pose()
        pose_msg.position.x = pose[0]
        pose_msg.position.y = pose[1]

        q = tf.transformations.quaternion_from_euler(0, 0, pose[2])
        # msg = Quaternion(*q)

        pose_msg.orientation = Quaternion(*q)

        return pose_msg

    def getStartingPose(self):
        y = self.random.random()*(18) - 9
        pose = [-9, y, 0]
        init_pose = self.getPoseMsg(pose=pose)

        return init_pose

    def getGoal(self):
        y = self.random.random()*(18) - 9

        pose = [9,y,3.14]
        init_pose = self.getPoseMsg(pose=pose)
        pose_stamped = PoseStamped()
        pose_stamped.pose = init_pose
        pose_stamped.header.frame_id="map"
        return pose_stamped

    def setupScenario(self):
        self.gazebo_driver.checkServicesTopics(10)
        self.gazebo_driver.pause()
        self.gazebo_driver.moveRobot(self.getStartingPose())
        self.gazebo_driver.resetOdom()
        self.gazebo_driver.reset(self.seed)
        self.gazebo_driver.unpause()



class FourthFloorScenario(TestingScenario):
    def __init__(self, task, gazebo_driver):
        self.gazebo_driver = gazebo_driver

        self.world = "fourth_floor"

        self.seed = task["seed"] if "seed" in task else 0

        self.init_pose = Pose()
        self.init_pose.position.x = -48
        self.init_pose.position.y = 17
        self.init_pose.orientation.x = 0
        self.init_pose.orientation.y = 0
        self.init_pose.orientation.z = 0
        self.init_pose.orientation.w = 1

        self.target_pose = PoseStamped()
        self.target_pose.pose.position.x = 4
        self.target_pose.pose.position.y = 16
        self.target_pose.pose.orientation.x = 0.0
        self.target_pose.pose.orientation.y = 0.0
        self.target_pose.pose.orientation.z = 0.0
        self.target_pose.pose.orientation.w = 1.0
        self.target_pose.header.frame_id = 'map'

    @staticmethod
    def getUniqueFieldNames():
        return ["num_barrels", "seed"]

    def setupScenario(self):
        self.gazebo_driver.checkServicesTopics(10)
        self.gazebo_driver.pause()
        self.gazebo_driver.moveRobot(self.init_pose)
        self.gazebo_driver.resetOdom()
        self.gazebo_driver.reset(self.seed)
        self.gazebo_driver.unpause()



class SparseScenario(TestingScenario):
    def __init__(self, task, gazebo_driver):
        self.gazebo_driver = gazebo_driver

        self.world = "empty_room_20x20"

        self.seed = task["seed"] if "seed" in task else 0
        self.min_obstacle_spacing = task["min_obstacle_spacing"] if "min_obstacle_spacing" in task else 3
        self.num_obstacles = task["num_obstacles"] if "num_obstacles" in task else 500


        self.random = random.Random()
        self.random.seed(self.seed)

        self.minx = [-7]
        self.miny = [-9.5]
        self.maxx = [6.5]
        self.maxy = [9.5]


    @staticmethod
    def getUniqueFieldNames():
        return ["num_obstacles", "seed", "min_obstacle_spacing"]

    def getPoseMsg(self, pose):
        pose_msg = Pose()
        pose_msg.position.x = pose[0]
        pose_msg.position.y = pose[1]

        q = tf.transformations.quaternion_from_euler(0, 0, pose[2])
        # msg = Quaternion(*q)

        pose_msg.orientation = Quaternion(*q)

        return pose_msg

    def getStartingPose(self):
        y = self.random.random()*(18) - 9
        pose = [-9, y, 0]
        init_pose = self.getPoseMsg(pose=pose)

        return init_pose

    def getGoal(self):
        y = self.random.random()*(18) - 9

        pose = [8,y,0]
        init_pose = self.getPoseMsg(pose=pose)
        pose_stamped = PoseStamped()
        pose_stamped.pose = init_pose
        pose_stamped.header.frame_id="map"
        return pose_stamped

    def setupScenario(self):
        self.gazebo_driver.checkServicesTopics(10)
        self.gazebo_driver.pause()
        self.gazebo_driver.moveRobot(self.getStartingPose())
        self.gazebo_driver.resetOdom()
        self.gazebo_driver.reset(self.seed)
        self.gazebo_driver.unpause()
        self.gazebo_driver.moveObstacles(self.num_obstacles, minx=self.minx, maxx=self.maxx, miny=self.miny, maxy=self.maxy, grid_spacing=self.min_obstacle_spacing)
