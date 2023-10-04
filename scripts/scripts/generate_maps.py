#!/usr/bin/env python

#import time
from builtins import str
from builtins import range
from builtins import object
import rospy
import subprocess
from nav_scripts.testing_scenarios import TestingScenarios
from std_srvs.srv import Empty as EmptyService
from nav_msgs.srv import GetPlan as PlanningService
from geometry_msgs.msg import PoseStamped

import csv
import os
import shlex

class ServiceInterface(object):

    def __init__(self, service_topic, service_class, timeout=None):
        self.service_topic = service_topic
        self.service_proxy = rospy.ServiceProxy(service_topic, service_class)

        def wait_for_service():
            try:
                rospy.wait_for_service(service=service_topic, timeout=timeout)
            except Exception as exc: #ROSException is more specific to failure to connect
                exc.message = "Error waiting for service [" + str(service_topic) + "] with Timeout=" + str(timeout) + ")" + exc.message
                raise

        self.wait = lambda : rospy.wait_for_service(service=service_topic, timeout=timeout)
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
        self.generateMap()
        return self.planGlobalPath()

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
        else:
            rospy.logwarn("Task does not specify valid scenario! Task=" + str(self.task))
            raise  ValueError("Task does not specify valid scenario! Task=" + str(self.task))

    def generateMap(self):
        self.map_gen_service.wait()

        try:
            rospy.loginfo("Calling map generation service...")
            resp = self.map_gen_service.call()
            if resp:
                rospy.loginfo("Map generation service was successful")
            else:
                rospy.logerr("Map generation service failed!")
                raise Exception("Map generation service failed!")

        except rospy.ServiceException as exc:
            #rospy.logerr("Map generation service encountered an exception: " + str(exc))
            exc.message = "Error generating map: " + str(exc)
            raise

    def planGlobalPath(self):
        goal = self.scenario.getGoalMsg()

        self.goal_pub.publish(goal)
        rospy.loginfo("Pausing briefly to ensure costmap updates...")
        rospy.sleep(self.map_gen_to_planning_wait_time)

        self.planning_service.wait()

        start = PoseStamped()
        start.pose = self.scenario.getStartingPoseMsg()
        start.header = goal.header

        rospy.loginfo("Attempting to plan path...")
        try:
            resp = self.planning_service.call(start=start, goal=goal)
            rospy.loginfo("Planning completed")
        except rospy.ServiceException as exc:
            exc.message = "Service did not process request! " + exc.message
            raise
        else:
            if len(resp.plan.poses) > 0:
                rospy.logwarn("Task feasible! [" + str(self.task) + "]")
                rospy.sleep(1)
                return True
            else:
                rospy.logerr("Task infeasible! [" + str(self.task) + "]")
                return False



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

class FakeWriter(object):
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        return False

    def write(self, topic, msg, t):
        pass


class MapGeneratorInterface(object):

    def __init__(self):
        poll_period = rospy.Duration(0.1)
        map_gen_max_wait_time = rospy.Duration(20)
        map_gen_to_planning_wait_time = rospy.Duration(0.5)

        self.scenarios = TestingScenarios()
        #rospy.wait_for_service('/gazebo_2dmap_plugin/generate_map')
        self.map_gen_service = ServiceInterface(service_topic='/gazebo_2dmap_plugin/generate_map', service_class=EmptyService, timeout=1)
        self.path_gen_service = ServiceInterface(service_topic='/global_planner_node/planner/make_plan', service_class=PlanningService, timeout=1)

        if False:
            self.base_path = base_path
            if not os.path.isdir(self.base_path):
                os.makedirs(name=self.base_path) # exist_ok=True
            pass

        goal_pub = rospy.Publisher("current_goal", PoseStamped, queue_size=4)

        planparams = MapGeneratorInterfaceInstance
        planparams.scenarios = self.scenarios
        planparams.map_gen_service = self.map_gen_service
        planparams.planning_service = self.path_gen_service

        planparams.poll_period = poll_period
        planparams.map_gen_max_wait_time = map_gen_max_wait_time
        planparams.map_gen_to_planning_wait_time = map_gen_to_planning_wait_time
        planparams.goal_pub = goal_pub

    def processScenarios(self, tasks, outputfile_name=None):
        #self.scenarios.gazebo_driver.pause()

        with open(outputfile_name, 'wb') if outputfile_name is not None else FakeWriter() as csvfile:
            fieldnames = ['seed', 'scenario', 'min_obstacle_spacing', 'feasible']

            if outputfile_name is not None:
                datawriter = csv.DictWriter(csvfile, fieldnames=fieldnames, restval='', extrasaction='ignore')
                datawriter.writeheader()


            for task in tasks:
                #gen = MapGeneratorInterfaceInstance(scenarios=self.scenarios, map_gen_service=self.map_gen_service, task=task, base_path = self.base_path)
                gen = MapGeneratorInterfaceInstance(task=task, base_path = "")

                res = gen.run()
                rospy.loginfo("Successfully tested scenario!")

                task['feasible'] = str(res)

                if outputfile_name is not None:
                    datawriter.writerow(task)
                    csvfile.flush()



if __name__ == '__main__':
    rospy.init_node('map_generator')
    rospy.loginfo("map_generator node started")
    gen = MapGeneratorInterface()

    def getTasks():
        for [scenario, min_obstacle_spacing] in [['dense', 0.5]]:
            for seed in range(68,100):
                task = {'seed': seed, 'scenario': scenario, 'min_obstacle_spacing': min_obstacle_spacing}
                yield task

    gen.processScenarios(tasks = getTasks(), outputfile_name=None)