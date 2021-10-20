import rospkg, rospy
from nav_scripts.gazebo_driver import GazeboDriver
from nav_scripts.costmap_driver import CostmapDriver
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
import numpy as np
import random
import tf
import math


class TestingScenarios:
    impls = {}

    def __init__(self):
        TestingScenario.gazebo_driver = GazeboDriver(as_node=False)

    def getScenario(self, task):
        try:
            scenario_type = task["scenario"]
            try:
                return TestingScenarios.impls[scenario_type](task=task)
            except KeyError as e:
                rospy.logerr("Error! Unknown scenario type [" + scenario_type + "]: " + str(e))
                return None
                #TODO: maybe throw some kind of exception to make it clear to the caller that this case failed?

        except KeyError:
            rospy.logerr("Error! Task does not specify scenario type [" + str(task) + "]: " + str(e))

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
        for scenario in TestingScenarios.getScenarioTypes().itervalues():
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
        rospy.loginfo("Resetting robot...")
        # TODO: Check if reset successful; if not, wait briefly and try again,
        # eventually fail and throw error
        self.gazebo_driver.checkServicesTopics(10)

        self.gazebo_driver.pause()
        self.gazebo_driver.moveRobot(self.getStartingPoseMsg())
        self.gazebo_driver.resetOdom()
        self.gazebo_driver.reset(self.seed)
        self.setupEnvironment()
        self.gazebo_driver.unpause()

    @staticmethod
    def getUniqueFieldNames():
        return ["world", "seed", 'init_pose', 'target_pose']


TestingScenarios.registerScenario(GeneralScenario)

'''
class TrashCanScenario(GeneralScenario):
    def __init__(self, task, gazebo_driver):
        self.gazebo_driver = gazebo_driver

        self.world = "rectangular"

        self.seed = task["seed"] if "seed" in task else 0
        self.num_barrels = task["num_obstacles"] if "num_obstacles" in task else 0

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
        return ["num_obstacles", "seed"]

    def setupScenario(self):
        self.gazebo_driver.checkServicesTopics(10)
        self.gazebo_driver.pause()
        self.gazebo_driver.moveRobot(self.init_pose)
        self.gazebo_driver.resetOdom()
        self.gazebo_driver.reset(self.seed)
        self.gazebo_driver.moveBarrels(self.num_barrels)
        self.gazebo_driver.unpause()
'''


class CampusScenario(GeneralScenario):
    name='campus'
    world='campus'
    #TODO: a more generic approach for checking parameters
    #defaults={'seed': 0, 'num_obstacles':0, 'init_id': 0, 'min_obstacle_spacing': 0, 'target_id': None}

    def __init__(self, task):
        super(CampusScenario, self).__init__(task=task)

        self.num_obstacles = task["num_obstacles"] if "num_obstacles" in task else 0
        self.init_id = task["init_id"] if "init_id" in task else 0
        self.min_spacing = task["min_obstacle_spacing"] if "min_obstacle_spacing" in task else 0    #TODO: set this to a saner value, or maybe just require it?
        self.target_id = task["target_id"] if "target_id" in task else None


        self.init_pose = Pose()
        self.init_pose.position.x = -12
        self.init_pose.position.y = 4
        self.init_pose.orientation.x = 0
        self.init_pose.orientation.y = 0
        self.init_pose.orientation.z = 0
        self.init_pose.orientation.w = 1

        self.target_poses = [[-12,4,1.57], [13,-10,1.57], [13,9.4,1.57], [-5.15,-9.25,1.57], [-13.5,15.25,0], [5.5,6.5,-1.57], [1.5,2,-1.57]]
        self.init_poses = [[-13,.5,0]]

        if self.target_id is None:
            self.target_id = self.random.randint(0,len(self.target_poses) - 1)
            task["target_id"] = self.target_id

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
        return ["num_obstacles", "target_id", "init_id", "min_obstacle_spacing"]

    def getStartingPose(self):
        pose = self.init_poses[self.init_id]
        return pose

    def getGoal(self):
        pose = self.target_poses[self.target_id]
        return pose

    def setupEnvironment(self):
        #TODO: make 'moveBarrels' do what is says it does, and possibly add new functions to allow for specifying the types of obstacles to place
        self.gazebo_driver.moveBarrels(self.num_obstacles, minx=self.minx, maxx=self.maxx, miny=self.miny, maxy=self.maxy, grid_spacing=self.min_spacing)

TestingScenarios.registerScenario(CampusScenario)


class CampusObstacleScenario(CampusScenario):
    name="campus_obstacle"
    world="campus"

    def __init__(self, task):
        super(CampusObstacleScenario, self).__init__(task=task)

        self.num_obstacles = task["num_obstacles"] if "num_obstacles" in task else 500 #TODO: set this to a saner value, or maybe just require it?

    def setupEnvironment(self):
        self.gazebo_driver.moveObstacles(self.num_obstacles, minx=self.minx, maxx=self.maxx, miny=self.miny, maxy=self.maxy, grid_spacing=self.min_spacing)

TestingScenarios.registerScenario(CampusObstacleScenario)


class FullCampusObstacleScenario(CampusObstacleScenario):
    name="full_campus_obstacle"
    world = "campus"

    def __init__(self, task):
        super(FullCampusObstacleScenario, self).__init__(task=task)

TestingScenarios.registerScenario(FullCampusObstacleScenario)


class StereoCampusObstacleScenario(CampusObstacleScenario):
    name = "stereo_campus_obstacle"
    world = "stereo_campus_obstacle"

    def __init__(self, task, gazebo_driver):
        super(StereoCampusObstacleScenario, self).__init__(task=task)

TestingScenarios.registerScenario(StereoCampusObstacleScenario)


class SectorScenario(GeneralScenario):
    name = "sector"
    world = "sector"
    poses = [[-9, 9, -.78], [-9, 0, 0], [-9, -9, .78], [9, -9, 2.36], [9, 0, 3.14], [9, 9, -2.36]]# [0,-9,1.57]

    def __init__(self, task):
        super(SectorScenario, self).__init__(task=task)
        # TODO: create class dictionary of default values, do all checking/setting of values at general level. Support lambdas
        self.init_id = task["init_id"] if "init_id" in task else 0
        self.target_id = task["target_id"] if "target_id" in task else (self.init_id + len(self.poses)/2) % len(self.poses)


    @staticmethod
    def getUniqueFieldNames():
        return ["num_obstacles", "target_id", "init_id"]

    def getStartingPose(self):
        y = self.random.random()*(18) - 9
        pose = [-9, y, 0]
        return pose

    def getGoal(self):
        y = self.random.random()*(18) - 9

        pose = [9,y,3.14]
        return pose

TestingScenarios.registerScenario(SectorScenario)


class SectorLaserScenario(SectorScenario):
    name="sector_laser"
    world="sector"

    def __init__(self, task):
        super(SectorLaserScenario, self).__init__(task=task)

TestingScenarios.registerScenario(SectorLaserScenario)


class FullSectorLaserScenario(SectorLaserScenario):
    name="full_sector_laser"
    world="sector"

    def __init__(self, task):
        super(FullSectorLaserScenario, self).__init__(task=task)

TestingScenarios.registerScenario(FullSectorLaserScenario)


class StereoSectorLaserScenario(SectorLaserScenario):
    name = "stereo_sector_laser"
    world = "stereo_sector_laser"

    def __init__(self, task):
        super(StereoSectorLaserScenario, self).__init__(task=task)

TestingScenarios.registerScenario(StereoSectorLaserScenario)


class SectorExtraScenario(SectorScenario):
    name = "sector_extra"
    world = "sector"

    def __init__(self, task):
        super(SectorExtraScenario, self).__init__(task=task)

        self.num_obstacles = task["num_obstacles"] if "num_obstacles" in task else 0
        self.min_spacing = task["min_obstacle_spacing"] if "min_obstacle_spacing" in task else None

        # Zone1 = [[35.5, 14.5], [30, 8.2]]
        # Zone2 = [[25, -10], [19.6, -13.9]]
        # Zone3 = [[-14.4, -13.8], [-9.93, -18.9]]
        # Zone4 = [[-30.5, 10.8], [-24, 7.8]]
        # Zone5 = [[-37.3, 14.8], [-34, 11.1]]
        # Zone6 = [[-33.3, -22.3], [-28.7, -26.5]]
        # Zone7 = [[2.2, 8.4], [8.2, 7]]
        # Zone8 = [[19.5, 24.1], [25.3, 19.2]]
        #
        # zones = [Zone1, Zone2, Zone3, Zone4, Zone5, Zone6, Zone7, Zone8]

        Zone1 = [[-8.5,9.5],[8.5,-9.5]]
        zones = [Zone1]
        zones = np.swapaxes(zones, 0, 2)

        x1 = zones[0][0]
        x2 = zones[0][1]
        y1 = zones[1][0]
        y2 = zones[1][1]

        self.minx = np.minimum(x1, x2)
        self.maxx = np.maximum(x1, x2)
        self.maxy = np.maximum(y1, y2)
        self.miny = np.minimum(y1, y2)

    @staticmethod
    def getUniqueFieldNames():
        return ["num_obstacles", "target_id", "init_id", "min_obstacle_spacing"]

    def setupEnvironment(self):
        self.gazebo_driver.moveBarrels(self.num_obstacles, minx=self.minx, maxx=self.maxx, miny=self.miny, maxy=self.maxy, grid_spacing=self.min_spacing)

TestingScenarios.registerScenario(SectorExtraScenario)


class FullSectorExtraScenario(SectorExtraScenario):
    name = "full_sector_extra"
    world = "sector_extra"

    def __init__(self, task):
        super(FullSectorExtraScenario, self).__init__(task=task)

TestingScenarios.registerScenario(FullSectorExtraScenario)


class StereoSectorExtraScenario(SectorExtraScenario):
    name = "stereo_sector_extra"
    world = "stereo_sector_extra"

    def __init__(self, task):
        super(StereoSectorExtraScenario, self).__init__(task=task)

TestingScenarios.registerScenario(StereoSectorExtraScenario)


class FourthFloorScenario(GeneralScenario):
    name= "fourth_floor"
    world = "fourth_floor"

    def __init__(self, task):
        super(FourthFloorScenario, self).__init__(task=task)

        self.init_id = task["init_id"] if "init_id" in task else None
        self.target_id = task["target_id"] if "target_id" in task else None

        #TODO: use multiple inheritance to move common things like these to common classes
        self.num_obstacles = task["num_obstacles"] if "num_obstacles" in task else 0
        self.min_spacing = task["min_obstacle_spacing"] if "min_obstacle_spacing" in task else None

        self.target_poses = [[38.87,11.19,3.14],[16.05,-15.5,-1.57],[-7.72,-12.5,-1.57],[-17.38,12.87,-1.57],[-40.77,14.2,0],[-33.83,-28.41,0.785],[-2.34,13.34,-0.785],[17.44,25.05,-0.785]]

        if self.init_id is None:
            self.init_id = self.random.randint(0, len(self.target_poses) - 1)
            task["init_id"] = self.init_id

        if self.target_id is None:
            init = self.target_poses[self.init_id]
            #To reduce the average run time, only consider nearest few goal locations
            dis = []
            for pose in self.target_poses:
                dis.append(math.sqrt((init[0]-pose[0])**2+(init[1]-pose[1])**2))
            dis_idx = sorted(range(len(dis)), key=dis.__getitem__)
            init_rand = self.random.randint(1,3)
            self.target_id = dis_idx[init_rand]

            task["target_id"] = self.target_id


            # dis = math.sqrt((self.target_poses[self]))

        Zone1 = [[35.5, 14.5], [30,8.2]]
        Zone2 = [[25,-10],[19.6,-13.9]]
        Zone3 = [[-14.4,-13.8],[-9.93,-18.9]]
        Zone4 = [[-30.5,10.8],[-24,7.8]]
        Zone5 = [[-37.3,14.8],[-34,11.1]]
        Zone6 = [[-33.3,-22.3],[-28.7,-26.5]]
        Zone7 = [[2.2,8.4],[8.2,7]]
        Zone8 = [[19.5,24.1],[25.3,19.2]]


        zones = [Zone1, Zone2, Zone3, Zone4, Zone5, Zone6, Zone7, Zone8]

        zones = np.swapaxes(zones, 0, 2)

        x1 = zones[0][0]
        x2 = zones[0][1]
        y1 = zones[1][0]
        y2 = zones[1][1]

        self.minx = np.minimum(x1,x2)
        self.maxx = np.maximum(x1,x2)
        self.maxy = np.maximum(y1,y2)
        self.miny = np.minimum(y1,y2)


    @staticmethod
    def getUniqueFieldNames():
        return ["num_obstacles","min_obstacle_spacing","target_id","init_id"]

    def getStartingPose(self):
        pose = self.target_poses[self.init_id]
        return pose

    def getGoal(self):
        pose = self.target_poses[self.target_id]
        return pose

    def setupEnvironment(self):
        self.gazebo_driver.moveBarrels(self.num_obstacles, minx=self.minx, maxx=self.maxx, miny=self.miny, maxy=self.maxy, grid_spacing=self.min_spacing)

TestingScenarios.registerScenario(FourthFloorScenario)


class FourthFloorObstacleScenario(FourthFloorScenario):
    name="fourth_floor_obstacle"
    world="fourth_floor"

    def __init__(self, task):
        super(FourthFloorObstacleScenario, self).__init__(task=task)

        self.num_obstacles = task["num_obstacles"] if "num_obstacles" in task else 500  #TODO: replace with saner value or simply require it?

    def setupEnvironment(self):
        self.gazebo_driver.moveObstacles(self.num_obstacles, minx=self.minx, maxx=self.maxx, miny=self.miny, maxy=self.maxy, grid_spacing=self.min_spacing)

TestingScenarios.registerScenario(FourthFloorObstacleScenario)


class FullFourthFloorObstacleScenario(FourthFloorObstacleScenario):
    name = "full_fourth_floor_obstacle"
    world = "fourth_floor"

    def __init__(self, task):
        super(FullFourthFloorObstacleScenario, self).__init__(task=task)

TestingScenarios.registerScenario(FullFourthFloorObstacleScenario)


class StereoFourthFloorObstacleScenario(FourthFloorObstacleScenario):
    name = "stereo_fourth_floor_obstacle"
    world = "stereo_fourth_floor_obstacle"

    def __init__(self, task):
        super(StereoFourthFloorObstacleScenario, self).__init__(task=task)

TestingScenarios.registerScenario(StereoFourthFloorObstacleScenario)


class SparseScenario(GeneralScenario):  #TODO: maybe just have one of these and just specify 'min_obstacle_spacing'
    name="sparse"
    world = "empty_room_20x20"
    obstacle_types = ['pole', 'square_post']

    def __init__(self, task):
        super(SparseScenario, self).__init__(task=task)

        self.min_obstacle_spacing = task["min_obstacle_spacing"] if "min_obstacle_spacing" in task else 5
        self.num_obstacles = task["num_obstacles"] if "num_obstacles" in task else 500

        self.minx = [-7]
        self.miny = [-9.5]
        self.maxx = [6.5]
        self.maxy = [9.5]


    @staticmethod
    def getUniqueFieldNames():
        return ["num_obstacles", "min_obstacle_spacing"]

    def getStartingPose(self):
        y = self.random.random()*(18) - 9
        pose = [-9, y, 0]
        return pose

    def getGoal(self):
        y = self.random.random()*(18) - 9
        pose = [8,y,0]
        return pose

    def setupEnvironment(self):
        self.gazebo_driver.moveObstacles(self.num_obstacles, minx=self.minx, maxx=self.maxx, miny=self.miny,
                                         maxy=self.maxy, grid_spacing=self.min_obstacle_spacing, model_types=self.obstacle_types)

TestingScenarios.registerScenario(SparseScenario)


class DenseScenario(SparseScenario):
    name="dense"
    def __init__(self, task):
        super(DenseScenario, self).__init__(task=task)

        self.min_obstacle_spacing = task["min_obstacle_spacing"] if "min_obstacle_spacing" in task else 1

TestingScenarios.registerScenario(DenseScenario)


class MediumScenario(SparseScenario):
    name="medium"
    def __init__(self, task):
        super(MediumScenario, self).__init__(task=task)

        self.min_obstacle_spacing = task["min_obstacle_spacing"] if "min_obstacle_spacing" in task else 2

TestingScenarios.registerScenario(MediumScenario)

'''
class CorridorZigzagScenario(TestingScenario):
    def __init__(self, task, gazebo_driver):
        ##TODO: move common elements (gazebo driver, seed, etc) to super.
        ##TODO: Also make the 'names' of scenarios properties of the scenarios themselves
        #super(Corridor1Scenario, self).__init__(task=task, gazebo_driver=gazebo_driver)
        self.gazebo_driver = gazebo_driver

        self.world = "corridor_zigzag"

        seed = task["seed"] if "seed" in task else 0
        self.min_obstacle_spacing = task["min_obstacle_spacing"]
        self.num_obstacles = task["num_obstacles"]
        self.seed = seed + 1000 * self.num_obstacles    #ensures that changing number of obstacles produces completely different scenes, not just incrementally different

        self.init_pose = Pose()
        self.init_pose.position.x = -6
        self.init_pose.orientation.w = 1

        self.target_pose = PoseStamped()
        self.target_pose.pose.position.x = 6
        self.target_pose.pose.position.y = 3
        self.target_pose.pose.orientation.w = 1.0
        self.target_pose.header.frame_id = 'map'

        Zone1 = [[-4,-1.1], [0,1.25]]
        Zone2 = [[1.25,-1.1], [3.25,3.5]]

        zones = [Zone1, Zone2]

        #zones = np.swapaxes(zones, 0, 2)
        mins = np.min(zones,axis=1)
        maxs = np.max(zones, axis=1)
        self.minx = mins[:,0]
        self.maxx = maxs[:,0]
        self.maxy = maxs[:,1]
        self.miny = mins[:,1]


    def setupScenario(self):
        self.gazebo_driver.checkServicesTopics(10)
        self.gazebo_driver.pause()
        self.gazebo_driver.moveRobot(self.getStartingPose())
        self.gazebo_driver.resetOdom()
        self.gazebo_driver.reset(self.seed)
        self.gazebo_driver.moveObstacles(self.num_obstacles, minx=self.minx, maxx=self.maxx, miny=self.miny,
                                         maxy=self.maxy, grid_spacing=self.min_obstacle_spacing)
        self.gazebo_driver.unpause()

class CorridorZigzagDoorScenario(CorridorZigzagScenario):
    def __init__(self, task, gazebo_driver):
        super(CorridorZigzagDoorScenario, self).__init__(task=task, gazebo_driver=gazebo_driver)
        self.world = "corridor_zigzag_door"

'''

class TrainingRoomScenario(GeneralScenario):
    name="training_room"
    world = "training_room"

    def __init__(self, task):
        super(TrainingRoomScenario, self).__init__(task=task)

    @staticmethod
    def getUniqueFieldNames():
        return []

    def getStartingPose(self):
        pose = self.costmap_driver.getSafePose()
        pose = [pose[0], pose[1], self.random.uniform(0, 2 * math.pi)]
        return pose

    def getGoal(self):
        pose = self.costmap_driver.getSafePose()
        pose = [pose[0], pose[1], 0]
        return pose

    def setupScenario(self):
        self.gazebo_driver.checkServicesTopics(10)
        self.costmap_driver = CostmapDriver(self.seed)

        self.gazebo_driver.pause()
        self.gazebo_driver.moveRobot(self.getStartingPoseMsg())
        self.gazebo_driver.resetOdom()
        self.gazebo_driver.reset(self.seed)
        self.gazebo_driver.unpause()

TestingScenarios.registerScenario(TrainingRoomScenario)
