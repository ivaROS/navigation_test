#!/usr/bin/env python

from __future__ import print_function
from builtins import str
from builtins import range
from builtins import object
from nav_scripts.testing_scenarios import TestingScenarios
#from nav_scripts.gazebo_driver import load_model_xml
import rospy
import time
from lxml import etree
from copy import deepcopy
import tf
import os

def format_pose_msg(pose_msg):
    quat = pose_msg.orientation
    pos = pose_msg.position
    (r,p,y) = tf.transformations.euler_from_quaternion([quat.x,quat.y,quat.z,quat.w])
    vals = [pos.x, pos.y, pos.z, r, p, y]
    vals = [str(v) for v in vals]
    s = ' '
    formatted = s.join(vals)
    return formatted


class ScenarioExporter(object):
    scenarios=TestingScenarios()
    model_filenames = {'box': 'box_lus.sdf', 'cylinder': 'cylinder.sdf', 'pole': 'pole_005_06.sdf', 'square_post': 'box_02_02_05.sdf'}

    def __init__(self, task):
        self.task = task
        self.scenario = self.scenarios.getScenario(task)
        self.process()


    def process(self):
        rospy.loginfo("Processing task " + str(self.task))
        self.load_environment_xml()
        self.setup_scenario()
        #self.set_start_location()
        self.add_obstacles()
        self.export()

    def set_start_location(self):
        models = self.world.findall("model")
        for model in models:
            if model.attrib['name'] == "NamedLocations":
                pose = model.find("pose")
                #TODO: add desired start location as new DefaultStart

    def export(self):
        base_path = self.scenario.gazebo_driver.rospack.get_path("nav_configs") + "/world/pregenerated_worlds/" + self.scenario.name + "_" + str(self.task["seed"])
        self.export_xml(base_path=base_path)
        self.export_config(base_path=base_path)

    def export_config(self, base_path):
        output_path = base_path + ".txt"
        rospy.loginfo("Writing start/goal poses to " + str(output_path) + "...")

        start_pose = self.scenario.getStartingPoseMsg()
        goal_pose = self.scenario.getGoalMsg()

        with open(output_path, "w") as config_file:
            string = format_pose_msg(pose_msg=start_pose) + "\n" + format_pose_msg(pose_msg=goal_pose.pose) + "\n"
            config_file.write(string)

        #start_str = StringIO()
        #start_pose.serialize(buff=start_str)
        pass


    def export_xml(self, base_path):
        world_path = base_path +  ".world"
        rospy.loginfo("Writing modified world xml to " + str(world_path) + "...")
        self.tree.write(world_path, pretty_print=True)

    def setup_scenario(self):
        rospy.loginfo("Setting up the scenario...")
        self.scenario.setupScenario()
        self.scenario.getGoalMsg()

    def load_environment_xml(self):
        rospy.loginfo("Parsing world xml...")
        world_name = self.scenario.world
        base_world = self.scenario.gazebo_driver.rospack.get_path("nav_configs") + "/world/" + world_name + ".world"
        self.tree = etree.parse(source=base_world)
        root = self.tree.getroot()
        self.world = root.find("world")

    def add_obstacles(self):
        rospy.Rate(10).sleep()
        rospy.loginfo("Adding obstacles to xml...")
        self.scenario.gazebo_driver.updateModels()
        models = self.scenario.gazebo_driver.models
        current_obstacles = [(models.name[i], models.pose[i]) for i in range(len(models.name)) if "obstacle" in models.name[i]]
        for obstacle in current_obstacles:
            self.add_model(model_name=obstacle[0], pose_msg=obstacle[1])

    def add_model(self, model_name, pose_msg):
        components = model_name.split('_obstacle')
        if len(components) != 2:
            rospy.logerr("Model doesn't match somehow")
            return

        model_type = components[0]

        if model_type not in self.model_filenames:
            rospy.logerr("Model type [" + str(model_type) + "] is unknown! Known types are: ")  # TODO: print list of types; maybe model_filenames.iter()?
            return False

        model_tree = etree.parse(source=self.scenario.gazebo_driver.model_path + self.model_filenames[model_type])
        model = model_tree.getroot()
        if model is not None:
            model = model.find("model")
            if model is not None:
                model.attrib['name'] = model_name
                static = model.find("static")
                static.text = 'true'
                pose = model.find("pose")
                pose.text = format_pose_msg(pose_msg=pose_msg)
                self.world.append(model)


class ObstacleExtractor(object):
    def __init__(self, world_file):
        self.process(world_file=world_file)

    def process(self, world_file):
        print("Processing task " + world_file)

        dirname, basename  = os.path.split(world_file)
        name=os.path.splitext(basename)[0]
        model_path = os.path.join(dirname,name) + ".sdf"

        self.load_environment_xml(world_file=world_file)
        self.create_new_model(name=name)
        self.add_obstacles()
        self.export_model(model_path=model_path)

    def export_model(self, model_path):
        print("Writing model xml to " + str(model_path) + "...")
        self.model_tree.write(model_path, pretty_print=True)

    def load_environment_xml(self, world_file):
        print("Parsing world xml...")
        self.tree = etree.parse(source=world_file)
        self.root = self.tree.getroot()
        self.world = self.root.find("world")


    def create_new_model(self, name):
        world_sdf = self.root
        version = world_sdf.get("version")
        sdf = etree.Element("sdf", version=version)
        self.super_model = etree.Element("model", name=name)
        sdf.append(self.super_model)
        self.model_tree = etree.ElementTree(element=sdf)


    def add_obstacles(self):
        models = self.world.findall("model")
        for model in models:
            if "_obstacle" in model.attrib['name']:
                self.super_model.append(deepcopy(model))


def generate_dense_worlds():
    rospy.init_node('test_driver', anonymous=True)

    task = {'scenario': 'dense', 'robot': 'turtlebot', 'min_obstacle_spacing': 0.5}

    for seed in range(61, 100):
        task['seed'] = seed
        scenario = ScenarioExporter(task=task)

def extract_models_from_worlds(path):
    world_files = [f for f in os.listdir(path) if os.path.isfile(os.path.join(path, f)) and os.path.splitext(f)[1] == ".world"]
    for file in world_files:
        world_file = os.path.join(path, file)
        extractor = ObstacleExtractor(world_file=world_file)
        pass

if __name__ == "__main__":
    #generate_dense_worlds()
    extract_models_from_worlds(path="/home/justin/catkin_ws/src/navigation_test/configs/world/pregenerated_worlds/")

    rospy.loginfo("Done!!!")
