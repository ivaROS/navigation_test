import rospkg
from nav_scripts.costmap_driver import CostmapDriver
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
import numpy as np
import random
import tf
import math

class TestingScenarios:
    def __init__(self):
        self.initialized = True

    def getScenario(self, task):
        if "scenario" in task:
            scenario_type = task["scenario"]
            if scenario_type == "fourth_floor":
                return FourthFloorScenario(task=task) 
            elif scenario_type == "fourth_floor_obstacle":
                return FourthFloorObstacleScenario(task=task) 
            elif scenario_type == "full_fourth_floor_obstacle":
                return FullFourthFloorObstacleScenario(task=task) 
            elif scenario_type == "stereo_fourth_floor_obstacle":
                return StereoFourthFloorObstacleScenario(task=task) 
            elif scenario_type == "campus":
                return CampusScenario(task=task) 
            elif scenario_type == "campus_obstacle":
                return CampusObstacleScenario(task=task) 
            elif scenario_type == "full_campus_obstacle":
                return FullCampusObstacleScenario(task=task) 
            elif scenario_type == "stereo_campus_obstacle":
                return StereoCampusObstacleScenario(task=task) 
            elif scenario_type == "sector":
                return SectorScenario(task=task) 
            elif scenario_type == "sector_laser":
                return SectorLaserScenario(task=task) 
            elif scenario_type == "full_sector_laser":
                return FullSectorLaserScenario(task=task) 
            elif scenario_type == "stereo_sector_laser":
                return StereoSectorLaserScenario(task=task) 
            elif scenario_type == "sector_extra":
                return SectorExtraScenario(task=task) 
            elif scenario_type == "full_sector_extra":
                return FullSectorExtraScenario(task=task) 
            elif scenario_type == "stereo_sector_extra":
                return StereoSectorExtraScenario(task=task) 
            elif scenario_type == "sparse":
                return SparseScenario(task=task) 
            elif scenario_type == "dense":
                return DenseScenario(task=task) 
            elif scenario_type == "medium":
                return MediumScenario(task=task)
            else:
                print "Error! Unknown scenario type [" + scenario_type + "]"
                return None
        elif "init_pose" in task and "goal" in task and "world" in task:
            return TestingScenario(task["world"],task["init_pose"],task["goal"])
        else:
            return None

    @staticmethod
    def getScenarioTypes():
        scenarios = [SectorScenario, CampusScenario, FourthFloorObstacleScenario, CampusObstacleScenario, SectorExtraScenario, SparseScenario, DenseScenario, MediumScenario]
        return scenarios


    @staticmethod
    def getFieldNames():
        fieldnames = ["scenario"]
        for scenario in TestingScenarios.getScenarioTypes():
            fieldnames.extend(scenario.getUniqueFieldNames())
        return fieldnames

def getPoseMsg(pose):
    pose_msg = Pose()
    pose_msg.position.x = pose[0]
    pose_msg.position.y = pose[1]

    q = tf.transformations.quaternion_from_euler(0, 0, pose[2])

    pose_msg.orientation = Quaternion(*q)

    return pose_msg

class TestingScenario(object):
    def __init__(self, world, init_pose, target_pose, goal_frame):
        self.world = world
        self.init_pose = init_pose
        self.target_pose = target_pose
        self.goal_frame = goal_frame

    def getStartingPose(self):
        return self.getPoseMsg(self.init_pose)

    def getGoal(self):
        goal = PoseStamped()
        goal.pose = self.getPoseMsg(self.target_pose)
        goal.header.frame_id = self.goal_frame
        return goal

    @staticmethod
    def getUniqueFieldNames():
        return [""]

    def getPoseMsg(self, pose):
        pose_msg = Pose()
        pose_msg.position.x = pose[0]
        pose_msg.position.y = pose[1]

        q = tf.transformations.quaternion_from_euler(0, 0, pose[2])
        # msg = Quaternion(*q)
        pose_msg.orientation = Quaternion(*q)

        return pose_msg

class CampusScenario(TestingScenario):
    def __init__(self, task, goal_frame):
        self.world = "campus"
        self.goal_frame = goal_frame

        self.seed = task["seed"] if "seed" in task else 0
        self.num_barrels = task["num_obstacles"] if "num_obstacles" in task else 0

        self.init_id = task["init_id"] if "init_id" in task else 0

        self.min_spacing = task["min_obstacle_spacing"] if "min_obstacle_spacing" in task else 0

        self.target_id = task["target_id"] if "target_id" in task else None


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
        self.target_pose.header.frame_id = self.goal_frame

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
        return ["num_obstacles", "seed", "target_id", "init_id", "min_obstacle_spacing"]

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
        pose_msg = self.getPoseMsg(pose=pose)
        pose_stamped = PoseStamped()
        pose_stamped.pose = pose_msg
        pose_stamped.header.frame_id=self.goal_frame
        return pose_stamped

class CampusObstacleScenario(CampusScenario):
    def __init__(self, task):
        super(CampusObstacleScenario, self).__init__(task=task)

        self.world = "campus_obstacle"

        self.num_obstacles = task["num_obstacles"] if "num_obstacles" in task else 500

class FullCampusObstacleScenario(CampusObstacleScenario):
    def __init__(self, task):
        super(FullCampusObstacleScenario, self).__init__(task=task)

        self.world = "full_campus_obstacle"

class StereoCampusObstacleScenario(CampusObstacleScenario):
    def __init__(self, task, gazebo_driver):
        super(StereoCampusObstacleScenario, self).__init__(task=task)

        self.world = "stereo_campus_obstacle"

class SectorScenario(TestingScenario):
    def __init__(self, task, goal_frame):
        self.goal_frame = goal_frame
        self.world = "sector"
        self.seed = task["seed"] if "seed" in task else 0
        self.poses = [[-9,9,-.78], [-9,0,0], [-9,-9,.78], [9,-9,2.36], [9,0,3.14], [9,9,-2.36]   ] # [0,-9,1.57]
        self.init_id = task["init_id"] if "init_id" in task else 0
        self.target_id = task["target_id"] if "target_id" in task else (self.init_id + len(self.poses)/2) % len(self.poses)
        self.random = random.Random()
        self.random.seed(self.seed)

    @staticmethod
    def getUniqueFieldNames():
        return ["num_obstacles", "seed", "target_id", "init_id"]

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
        pose_stamped.header.frame_id=self.goal_frame
        return pose_stamped

class SectorLaserScenario(SectorScenario):
    def __init__(self, task):
        super(SectorLaserScenario, self).__init__(task=task)

        self.world = "sector_laser"

class FullSectorLaserScenario(SectorLaserScenario):
    def __init__(self, task):
        super(FullSectorLaserScenario, self).__init__(task=task)

        self.world = "full_sector_laser"

class StereoSectorLaserScenario(SectorLaserScenario):
    def __init__(self, task):
        super(StereoSectorLaserScenario, self).__init__(task=task)

        self.world = "stereo_sector_laser"

class SectorExtraScenario(SectorScenario):
    def __init__(self, task):
        super(SectorExtraScenario, self).__init__(task=task)

        self.world = "sector_extra"

        self.num_barrels = task["num_obstacles"] if "num_obstacles" in task else 0

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
        return ["num_obstacles", "seed", "target_id", "init_id", "min_obstacle_spacing"]

class FullSectorExtraScenario(SectorExtraScenario):
    def __init__(self, task):
        super(FullSectorExtraScenario, self).__init__(task=task)

        self.world = "full_sector_extra"

class StereoSectorExtraScenario(SectorExtraScenario):
    def __init__(self, task):
        super(StereoSectorExtraScenario, self).__init__(task=task)

        self.world = "stereo_sector_extra"

class FourthFloorScenario(TestingScenario):
    def __init__(self, task, goal_frame):

        self.world = "fourth_floor"

        self.seed = task["seed"] if "seed" in task else 0

        self.init_id = task["init_id"] if "init_id" in task else None
        self.target_id = task["target_id"] if "target_id" in task else None    ##TODO: Replace these with randomly chosen ones
        self.num_barrels = task["num_obstacles"] if "num_obstacles" in task else 0

        self.min_spacing = task["min_obstacle_spacing"] if "min_obstacle_spacing" in task else None

        self.random = random.Random()
        self.random.seed(self.seed)

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


        self.target_poses = [[38.87,11.19,3.14],[16.05,-15.5,-1.57],[-7.72,-12.5,-1.57],[-17.38,12.87,-1.57],[-40.77,14.2,0],[-33.83,-28.41,0.785],[-2.34,13.34,-0.785],[17.44,25.05,-0.785]]

        if self.init_id is None:
            self.init_id = self.random.randint(0, len(self.target_poses) - 1)
            task["init_id"] = self.init_id

        if self.target_id is None:
            init = self.target_poses[self.init_id]
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
        return ["num_obstacles", "seed","min_obstacle_spacing"]

    def getPoseMsg(self, pose):
        pose_msg = Pose()
        pose_msg.position.x = pose[0]
        pose_msg.position.y = pose[1]

        q = tf.transformations.quaternion_from_euler(0, 0, pose[2])

        pose_msg.orientation = Quaternion(*q)

        return pose_msg

    def getStartingPose(self):
        pose = self.target_poses[self.init_id]
        init_pose = self.getPoseMsg(pose=pose)

        return init_pose

    def getGoal(self):
        pose = self.target_poses[self.target_id]
        init_pose = self.getPoseMsg(pose=pose)
        pose_stamped = PoseStamped()
        pose_stamped.pose = init_pose
        pose_stamped.header.frame_id="map"
        return pose_stamped


class FourthFloorObstacleScenario(FourthFloorScenario):
    def __init__(self, task, gazebo_driver):
        super(FourthFloorObstacleScenario, self).__init__(task=task, gazebo_driver=gazebo_driver)

        self.world = "fourth_floor_obstacle"

        self.num_obstacles = task["num_obstacles"] if "num_obstacles" in task else 500

    # def getUniqueFieldNames():
    #     return ["num_obstacles", "seed", "min_obstacle_spacing"]

    def setupScenario(self):
        self.gazebo_driver.checkServicesTopics(10)
        self.gazebo_driver.pause()
        self.gazebo_driver.moveRobot(self.getStartingPose())
        self.gazebo_driver.resetOdom()
        self.gazebo_driver.reset(self.seed)
        self.gazebo_driver.moveObstacles(self.num_obstacles, minx=self.minx, maxx=self.maxx, miny=self.miny, maxy=self.maxy, grid_spacing=self.min_spacing)
        self.gazebo_driver.unpause()

class FullFourthFloorObstacleScenario(FourthFloorObstacleScenario):
    def __init__(self, task, gazebo_driver):
        super(FullFourthFloorObstacleScenario, self).__init__(task=task, gazebo_driver=gazebo_driver)

        self.world = "full_fourth_floor_obstacle"

class StereoFourthFloorObstacleScenario(FourthFloorObstacleScenario):
    def __init__(self, task, gazebo_driver):
        super(StereoFourthFloorObstacleScenario, self).__init__(task=task, gazebo_driver=gazebo_driver)

        self.world = "stereo_fourth_floor_obstacle"

class SparseScenario(TestingScenario):
    def __init__(self, task, goal_frame):
        self.world = "empty_room_20x20"
        self.seed = task["seed"] if "seed" in task else 0
        self.min_obstacle_spacing = task["min_obstacle_spacing"] if "min_obstacle_spacing" in task else 5
        self.num_obstacles = task["num_obstacles"] if "num_obstacles" in task else 50

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

class DenseScenario(SparseScenario):
    def __init__(self, task):
        super(DenseScenario, self).__init__(self=self, task=task)

        self.min_obstacle_spacing = task["min_obstacle_spacing"] if "min_obstacle_spacing" in task else 1

class MediumScenario(SparseScenario):
    def __init__(self, task):
        super(MediumScenario, self).__init__(task=task)
        self.min_obstacle_spacing = task["min_obstacle_spacing"] if "min_obstacle_spacing" in task else 2
