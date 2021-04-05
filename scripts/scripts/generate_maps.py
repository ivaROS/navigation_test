#!/usr/bin/env python

#import time
import rospy
import subprocess
from nav_scripts.testing_scenarios import TestingScenarios
from std_srvs.srv import Empty as EmptyService
from nav_msgs.srv import GetPlan as PlanningService
import os
import shlex

class ServiceInterface(object):

    def __init__(self, service_topic, service_class, timeout=None):
        self.service_topic = service_topic
        self.service_proxy = rospy.ServiceProxy(service_topic, service_class)

        def wait_for_service():
            try:
                rospy.wait_for_service(service=service_topic, timeout=timeout)
                return True
            except Exception as exc: #ROSException is more specific to failure to connect
                print("Service unavailable! (Timeout=" + str(timeout) + ")" + str(exc))
                return False

        self.wait = wait_for_service
        self.call = self.service_proxy


class MapGeneratorInterfaceInstance(object):
    scenarios=None
    map_gen_service=None
    planning_service=None

    poll_period=None
    map_gen_max_wait_time=None
    map_gen_to_planning_wait_time=None
    generation_to_planning_wait_time=None

    def __init__(self,task, base_path=None):
        self.task = task
        self.map_saver = None
        #self.filepath = str(base_path + '/' + str(task['seed']))
        #self.poll_period = rospy.Duration(0.1)
        #self.map_gen_max_wait_time = rospy.Duration(20)
        #self.map_gen_to_planning_wait_time = rospy.Duration(0.5)

        #self.scenarios = scenarios
        #self.map_gen_service = map_gen_service
        pass

    def run(self):
        self.setupScenario()

        #if not self.generateMap() or not self.planGlobalPath():
        #    return False

        if not self.generateMap():
            return False
        if not self.planGlobalPath():
            return False

        return True

        #return self.saveMap()

        # try:
        #     return self.saveMap()
        # except Exception as ex:
        #     rospy.logerr("Something went wrong when getting map: " + str(ex))
        #     if self.map_saver is not None and self.map_saver.returncode is None:
        #         self.map_saver.kill()
        #     return False

        # try:
        #     self.startMapSaver()
        #     return self.waitForMapSaver()
        # except Exception as ex:
        #     rospy.logerr("Something went wrong when getting map: " + str(ex))
        #     if self.map_saver is not None and self.map_saver.returncode is None:
        #         self.map_saver.kill()
        # finally:



    def setupScenario(self):
        self.scenario = self.scenarios.getScenario(task=self.task)

        if self.scenario is not None:
            rospy.loginfo("Setting up scenario...")
            self.scenario.setupScenario() #This requires a robot
            return True
        else:
            rospy.logwarn("Task does not specify valid scenario! Task=" + str(self.task))
            return False

    def generateMap(self):
        try:
            if not self.map_gen_service.wait():
                return False

            rospy.loginfo("Calling map generation service...")
            resp = self.map_gen_service.call()
            if resp:
                rospy.loginfo("Map generation service was successful")
            else:
                rospy.logerr("Map generation service failed!")
            return resp
        except rospy.ServiceException as exc:
            rospy.logerr("Map generation service encountered an exception: " + str(exc))
            return False

    def planGlobalPath(self):
        rospy.sleep(self.generation_to_planning_wait_time)

        if not self.planning_service.wait():
            return False

        goal = self.scenario.getGoalMsg()
        try:
            resp = self.planning_service.call(goal=goal)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
            return False
        else:
            if resp:
                rospy.loginfo("Task feasible! [" + str(self.task) + "]")
            else:
                rospy.logwarn("Task infeasible! [" + str(self.task) + "]")
            return True

    def saveMap(self):
        map_saver = None
        try:
            command_str = str('rosrun map_server map_saver map:=/groundtruth/map -f ' + str(self.filepath))
            #args = shlex.split(command_str)

            map_saver = subprocess.Popen(args=command_str, shell=True, stdout=subprocess.PIPE)

            def getTime():
                return rospy.Time.now()  # rospy.Time.from_sec(time.time())

            start_time = getTime()
            # for line in iter(map_saver.stdout.readline, b''):
            #     print line,
            # p.stdout.close()

            while map_saver.returncode is None and getTime() - start_time < self.map_gen_max_wait_time:
                # time.sleep(secs=self.poll_period)
                rospy.sleep(self.poll_period)
                map_saver.poll()

            if map_saver.returncode is None:
                map_saver.kill()
                rospy.logerr("Map saver failed to finish within " + str(self.map_gen_max_wait_time.to_sec()) + 's')
                return False
            elif map_saver.returncode is not 0:
                rospy.logerr("Map saver failed with return code " + str(map_saver.returncode))
                return False
            else:
                return True
        except Exception as ex:
            rospy.logerr("Something went wrong running map saver: " + str(ex))

            if map_saver is not None and map_saver.returncode is None:
                rospy.logwarn("Attempting to kill map saver...")
                map_saver.kill()

            return False

    # def startMapSaver(self):
    #     command_str = str('rosrun map_server map_saver map:=/groundtruth/map -f ' + str(self.filepath))
    #     args = shlex.split(command_str)
    #     self.map_saver = subprocess.Popen(args=args)
    #     #subprocess.check_output("ls non_existent_file; exit 0", stderr = subprocess.STDOUT, shell = True)
    #     pass
    #
    # def waitForMapSaver(self):
    #     def getTime():
    #         return rospy.Time.now() #rospy.Time.from_sec(time.time())
    #
    #     start_time = getTime()
    #     while self.map_saver.returncode is None and getTime() - start_time < self.max_wait_time:
    #         #time.sleep(secs=self.poll_period)
    #         rospy.sleep(self.poll_period)
    #         self.map_saver.poll()
    #
    #     if self.map_saver.returncode is None:
    #         self.map_saver.kill()
    #         rospy.logerr("Map saver failed to close within " + str(self.max_wait_time.to_sec()) + 's')
    #         return False
    #
    #     return True


class MapGeneratorInterface(object):

    def __init__(self):
        poll_period = rospy.Duration(0.1)
        map_gen_max_wait_time = rospy.Duration(20)
        map_gen_to_planning_wait_time = rospy.Duration(0.5)
        generation_to_planning_wait_time = rospy.Duration(1)

        self.scenarios = TestingScenarios()
        #rospy.wait_for_service('/gazebo_2dmap_plugin/generate_map')
        self.map_gen_service = ServiceInterface(service_topic='/gazebo_2dmap_plugin/generate_map', service_class=EmptyService, timeout=1)
        self.path_gen_service = ServiceInterface(service_topic='/global_planner_node/planner/make_plan', service_class=PlanningService, timeout=1)
        self.base_path = '/tmp/maps/dense'
        if False:
            self.base_path = '/tmp/maps/dense'
            if not os.path.isdir(self.base_path):
                os.makedirs(name=self.base_path) # exist_ok=True
            pass

        planparams = MapGeneratorInterfaceInstance
        planparams.scenarios = self.scenarios
        planparams.map_gen_service = self.map_gen_service
        planparams.planning_service = self.path_gen_service

        planparams.poll_period = poll_period
        planparams.map_gen_max_wait_time = map_gen_max_wait_time
        planparams.map_gen_to_planning_wait_time = map_gen_to_planning_wait_time
        planparams.generation_to_planning_wait_time = generation_to_planning_wait_time


    def processScenarios(self, tasks):
        #self.scenarios.gazebo_driver.pause()
        for task in tasks:
            #gen = MapGeneratorInterfaceInstance(scenarios=self.scenarios, map_gen_service=self.map_gen_service, task=task, base_path = self.base_path)
            gen = MapGeneratorInterfaceInstance(task=task, base_path = self.base_path)

            if not gen.run():
                pass
                rospy.logerr("Stopped testing scenarios!")
                break
            else:
                rospy.loginfo("Successfully tested scenario!")



if __name__ == '__main__':
    rospy.init_node('map_generator')
    rospy.loginfo("map_generator node started")
    gen = MapGeneratorInterface()

    def getTasks():
        for [scenario, min_obstacle_spacing] in [['dense', 5]]:
            for seed in range(0,100):
                task = {'seed': seed, 'scenario': scenario, 'min_obstacle_spacing': min_obstacle_spacing}
                yield task

    gen.processScenarios(tasks = getTasks())