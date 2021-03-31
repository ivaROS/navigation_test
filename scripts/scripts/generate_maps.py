#!/usr/bin/env python

#import time
import rospy
import subprocess
from nav_scripts.testing_scenarios import TestingScenarios
from std_srvs.srv import Empty as EmptyService
import os
import shlex

class MapGeneratorInterfaceInstance(object):

    def __init__(self, scenarios, map_gen_service, task, base_path):
        self.task = task
        self.map_saver = None
        self.filepath = str(base_path + '/' + str(task['seed']))
        self.poll_period = rospy.Duration(0.1)
        self.max_wait_time = rospy.Duration(2)

        self.scenarios = scenarios
        self.map_gen_service = map_gen_service
        pass

    def run(self):
        self.setupScenario()
        if not self.generateMap():
            return False

        try:
            self.startMapSaver()
            return self.waitForMapSaver()
        except Exception as e:
            rospy.logerr("Something went wrong when getting map: " + str(ex))
            if self.map_saver is not None and self.map_saver.returncode is None:
                self.map_saver.kill()
        #finally:



    def setupScenario(self):
        scenario = self.scenarios.getScenario(task=self.task)

        if scenario is not None:
            # scenario.gazebo_driver.checkServicesTopics(10)
            # scenario.gazebo_driver.pause()
            # scenario.gazebo_driver.reset(scenario.seed)
            # scenario.setupEnvironment()
            # scenario.gazebo_driver.unpause()
            scenario.setupScenario() #This requires a robot

    def generateMap(self):
        try:
            resp = self.map_gen_service()
            if resp:
                rospy.loginfo("Map service successfully returned")
            return resp
        except rospy.ServiceException as exc:
            rospy.logerr("Map gen service did not process request: " + str(exc))
            return False

    def startMapSaver(self):
        command_str = str('rosrun map_server map_saver map:=/groundtruth/map -f ' + str(self.filepath))
        args = shlex.split(command_str)
        self.map_saver = subprocess.Popen(args=args)
        #subprocess.check_output("ls non_existent_file; exit 0", stderr = subprocess.STDOUT, shell = True)
        pass

    def waitForMapSaver(self):
        def getTime():
            return rospy.Time.now() #rospy.Time.from_sec(time.time())

        start_time = getTime()
        while self.map_saver.returncode is None and getTime() - start_time < self.max_wait_time:
            #time.sleep(secs=self.poll_period)
            rospy.sleep(self.poll_period)
            self.map_saver.poll()

        if self.map_saver.returncode is None:
            self.map_saver.kill()
            rospy.logerr("Map saver failed to close within " + str(self.max_wait_time.to_sec()) + 's')
            return False

        return True


class MapGeneratorInterface(object):

    def __init__(self):
        self.scenarios = TestingScenarios()
        rospy.wait_for_service('/gazebo_2dmap_plugin/generate_map')
        self.map_gen_service = rospy.ServiceProxy('/gazebo_2dmap_plugin/generate_map', EmptyService)
        self.base_path = '/tmp/maps/dense'
        if not os.path.isdir(self.base_path):
            os.makedirs(name=self.base_path) # exist_ok=True
        pass


    def processScenarios(self, tasks):
        #self.scenarios.gazebo_driver.pause()
        for task in tasks:
            gen = MapGeneratorInterfaceInstance(scenarios=self.scenarios, map_gen_service=self.map_gen_service, task=task, base_path = self.base_path)
            if not gen.run():
                pass
                rospy.logerr("Stopped generating maps!")
                break
            else:
                rospy.loginfo("Successfully generated map!")



if __name__ == '__main__':
    rospy.init_node('map_generator')
    rospy.loginfo("map_generator node started")
    gen = MapGeneratorInterface()

    def getTasks():
        for [scenario, min_obstacle_spacing] in [['dense', 0.5]]:
            for seed in range(0,100):
                task = {'seed': seed, 'scenario': scenario, 'min_obstacle_spacing': min_obstacle_spacing}
                yield task

    gen.processScenarios(tasks = getTasks())