#!/usr/bin/env python

from nav_scripts.testing_scenarios import TestingScenarios
#from nav_scripts.gazebo_driver import load_model_xml
import rospy
import time
from lxml import etree
import copy
import tf
from geometry_msgs.msg import Pose
from io import StringIO


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
            pass

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

        #model_xml = load_model_xml(self.model_path + model_filenames[model_type])
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


    def add_model2(self, name_in, pose_in, scale_in, mesh_num_in, models_type):
        link_el = etree.Element('link', name=name_in)

        pose_el = etree.Element('pose')
        pose_el.text = ''.join(str(e) + ' ' for e in pose_in)
        link_el.append(pose_el)

        visual_el = etree.Element('visual', name='visual')
        geometry_el = etree.Element('geometry')
        mesh_el = etree.Element('mesh')
        uri_el = etree.Element('uri')
        uri_el.text = 'file://' + models_type + '/Tree' + str(mesh_num_in) + '.dae'
        mesh_el.append(uri_el)
        scale_el = etree.Element('scale')
        scale_el.text = str(scale_in) + ' ' + str(scale_in) + ' ' + str(scale_in)
        mesh_el.append(scale_el)
        geometry_el.append(mesh_el)
        visual_el.append(geometry_el)
        link_el.append(visual_el)

        collision_el = etree.Element('collision', name='collision')
        collision_el.append(copy.deepcopy(geometry_el))
        contacts_el = etree.Element('max_contacts')
        contacts_el.text = '0'
        collision_el.append(contacts_el)
        link_el.append(collision_el)

        self.root.find('model').append(link_el)

#def getXML(base_world, obstacles, start_pose, goal_pose):


rospy.init_node('test_driver', anonymous=True)

#scenarios = TestingScenarios()
task= {'scenario': 'dense', 'robot':'turtlebot', 'min_obstacle_spacing':0.5}


for seed in range(61,100):
    task['seed'] = seed
    scenario = ScenarioExporter(task=task)

rospy.loginfo("Done!!!")
