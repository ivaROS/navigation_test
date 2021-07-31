#!/usr/bin/env python

import subprocess
import multiprocessing as mp
import os
import sys
import rospkg
import roslaunch
import time
# import nav_scripts.movebase_driver as test_driver
import rosgraph
import threading
import Queue
from ctypes import c_bool
import rospkg
rospack = rospkg.RosPack()

import math
from move_base_msgs.msg import *
from sensor_msgs.msg import Range
import signal
import actionlib
from actionlib_msgs.msg import GoalStatus

import socket
import contextlib

from stdr_testing_scenarios import TestingScenarios

from actionlib_msgs.msg import GoalStatusArray
from nav_msgs.msg import Odometry
import rospy
import csv
import datetime
import tf2_ros
from stdr_testing_scenarios import SectorScenario, CampusScenario, FourthFloorScenario, SparseScenario
import rosbag
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan

from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseArray

class BumperChecker:
    def __init__(self):
        self.sub = rospy.Subscriber("robot0/bumpers", Range, self.bumperCB, queue_size=5)
        self.collided = False

    def bumperCB(self, data):
        if data.range < 0:
            self.collided = True

class OdomAccumulator:
    def __init__(self):
        self.feedback_subscriber = rospy.Subscriber("/robot0/odom", Odometry, self.odomCB, queue_size=5)
        self.path_length = 0
        self.prev_msg = None

    def odomCB(self, odom):
        if self.prev_msg is not None:
            prev_pos = self.prev_msg.pose.pose.position
            cur_pos = odom.pose.pose.position

            deltaX = cur_pos.x - prev_pos.x
            deltaY = cur_pos.y - prev_pos.y

            displacement = math.sqrt(deltaX*deltaX + deltaY*deltaY)
            self.path_length += displacement
        self.prev_msg = odom

    def getPathLength(self):
        return self.path_length

class ResultRecorder:
    def __init__(self, taskid):
        self.lock = threading.Lock()
        self.tf_sub = rospy.Subscriber("tf", TFMessage, self.tf_cb, queue_size = 1000)
        self.scan_sub = rospy.Subscriber("point_scan", LaserScan, self.scan_cb, queue_size = 5)
        self.score_sub = rospy.Subscriber("traj_score", MarkerArray, self.score_cb, queue_size = 5)
        self.traj_sub = rospy.Subscriber("all_traj_vis", MarkerArray, self.traj_cb, queue_size = 5)
        self.exe_traj_sub = rospy.Subscriber("pg_traj", PoseArray, self.exe_traj_cb, queue_size = 5)

        bagpath = "~/simulation_data/bagfile/" + str(datetime.datetime.now()) + "_" + str(taskid) + ".bag"
        self.bagfilepath = os.path.expanduser(bagpath)
        print "bag file = " + self.bagfilepath + "\n"
        self.bagfile = rosbag.Bag(f=self.bagfilepath, mode='w', compression=rosbag.Compression.LZ4)
        self.scan_data = None
        self.tf_data = None
        self.bag_closed = False

    def exe_traj_cb(self, data):
        if self.tf_data is None or self.bag_closed:
            return
        self.lock.acquire()
        self.bagfile.write("pg_traj", data, self.tf_data.transforms[0].header.stamp)
        self.lock.release()
        rospy.logdebug("Exe Traj written")

    def score_cb(self, data):
        if self.tf_data is None or self.bag_closed:
            return
        self.lock.acquire()
        self.bagfile.write("traj_score", data, self.tf_data.transforms[0].header.stamp)
        self.lock.release()
        rospy.logdebug("Score written")

    def traj_cb(self, data):
        if self.tf_data is None or self.bag_closed:
            return
        self.lock.acquire()
        self.bagfile.write("all_traj_vis", data, self.tf_data.transforms[0].header.stamp)
        self.lock.release()
        rospy.logdebug("Traj written")

    def scan_cb(self, data):
        if self.bag_closed:
            return
        self.lock.acquire()
        self.scan_data = data
        self.bagfile.write("point_scan", data, data.header.stamp)
        self.lock.release()
        rospy.logdebug("Laserscan written")
    
    def tf_cb(self, data):
        if self.bag_closed:
            return
        self.lock.acquire()
        self.tf_data = data
        self.bagfile.write("tf", data, self.tf_data.transforms[0].header.stamp)
        self.lock.release()
        rospy.logdebug("tf written")

    def done(self):
        self.bag_closed = True
        self.lock.acquire()
        self.scan_sub.unregister()
        self.tf_sub.unregister()
        self.score_sub.unregister()
        self.traj_sub.unregister()
        self.bagfile.close()
        self.lock.release()
        rospy.logdebug("Result finished")

# Send goal to Move base and receive result
def run_test(goal_pose, record = False, taskid=0):
    bumper_checker = BumperChecker()
    odom_accumulator = OdomAccumulator()

    if record:
        result_recorder = ResultRecorder(taskid)

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) # potential_gap_egocircle_path_follower
    print "waiting for server"
    client.wait_for_server()
    print "Done!"

    goal = MoveBaseGoal()
    goal.target_pose = goal_pose
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = 'map_static'

    print "sending goal"
    client.send_goal(goal)
    print "waiting for result"

    r = rospy.Rate(5)
    start_time = rospy.Time.now()
    result = None

    keep_waiting = True
    counter = 0
    result = None
    while keep_waiting:
        try:
            state = client.get_state()
            if state is not GoalStatus.ACTIVE and state is not GoalStatus.PENDING:
                keep_waiting = False
            elif bumper_checker.collided:
                keep_waiting = False
                result = "BUMPER_COLLISION"
            elif (rospy.Time.now() - start_time > rospy.Duration(600)):
                keep_waiting = False
                result = "TIMED_OUT"
            else:
                counter += 1
                # if blocks, sim time cannot be 0
                r.sleep()
        except:
            keep_waiting = "False"
            

    print result
    task_time = str(rospy.Time.now() - start_time)
    path_length = str(odom_accumulator.getPathLength())

    if record:
        print "Acquire Record Done"
        result_recorder.done()
        print "Acquired"

    if result is None:
        print "done!"
        print "getting goal status"
        print(client.get_goal_status_text())
        print "done!"
        print "returning state number"
        state = client.get_state()
        if state == GoalStatus.SUCCEEDED:
            result = "SUCCEEDED"
        elif state == GoalStatus.ABORTED:
            result = "ABORTED"
        elif state == GoalStatus.LOST:
            result = "LOST"
        elif state == GoalStatus.REJECTED:
            result = "REJECTED"
        elif state == GoalStatus.ACTIVE:
            result = "TIMED_OUT"
        else:
            result = "UNKNOWN"
    print result
    
    return {'result': result, 'time': task_time, 'path_length': path_length}

def port_in_use(port):
    with contextlib.closing(socket.socket(socket.AF_INET, socket.SOCK_STREAM)) as sock:
        if sock.connect_ex(('127.0.0.1', port)) == 0:
            print "Port " + str(port) + " is in use"
            return True
        else:
            print "Port " + str(port) + " is not in use"
            return False


class MultiMasterCoordinator:
    def __init__(self, num_masters=4, record=False):
        signal.signal(signal.SIGINT, self.signal_shutdown)
        signal.signal(signal.SIGTERM, self.signal_shutdown)
        self.children_shutdown = mp.Value(c_bool, False)
        self.soft_shutdown = mp.Value(c_bool, False)

        self.should_shutdown = False
        self.num_masters = num_masters

        self.save_results = True
        self.task_queue_capacity = 2000 #2*self.num_masters
        self.task_queue = mp.JoinableQueue(maxsize=self.task_queue_capacity)
        self.result_queue_capacity = 2000 #*self.num_masters
        self.result_queue = mp.JoinableQueue(maxsize=self.result_queue_capacity)
        self.stdr_masters = []
        self.result_list = []
        self.stdr_launch_mutex = mp.Lock()
        self.record = record

        self.fieldnames = ["controller"]
        self.fieldnames.extend(TestingScenarios.getFieldNames())
        self.fieldnames.extend(["taskid","world", "pid", "result","time","path_length","robot"])
        self.fieldnames.extend(["fov", "radial_extend", "projection", "r_min", 'r_norm', 'k_po', 'reduction_threshold', 'reduction_target'])
        self.fieldnames.extend(["sim_time", "obstacle_cost_mode", "sum_scores"])
        self.fieldnames.extend(["bag_file_path",'converter', 'costmap_converter_plugin', 'global_planning_freq', 'feasibility_check_no_poses', 'simple_exploration', 'weight_gap', 'gap_boundary_exponent', 'egocircle_early_pruning', 'gap_boundary_threshold', 'gap_boundary_ratio', 'feasibility_check_no_tebs', 'gap_exploration', 'gap_h_signature', ])

    def start(self):
        self.startResultsProcessing()
        self.startProcesses()

    def startResultsProcessing(self):
        self.result_thread = threading.Thread(target=self.processResults,args=[self.result_queue])
        self.result_thread.daemon=True
        self.result_thread.start()

    def startProcesses(self):
        self.ros_port = 11411
        self.gazebo_port = self.ros_port + 100
        for ind in xrange(self.num_masters):
            self.addProcess(ind)

    def addProcess(self, ind):
        while port_in_use(self.ros_port):
            self.ros_port += 1

        while port_in_use(self.gazebo_port):
            self.gazebo_port += 1

        stdr_master = STDRMaster(
            self.task_queue,
            self.result_queue,
            self.children_shutdown,
            self.soft_shutdown,
            self.ros_port,
            self.gazebo_port,
            stdr_launch_mutex=self.stdr_launch_mutex,
            record=self.record)
        stdr_master.start()
        self.stdr_masters.append(stdr_master)

        self.ros_port += 1
        self.gazebo_port += 1

        time.sleep(1)


    def processResults(self,queue):

        outputfile_name = "~/simulation_data/results_" + str(datetime.datetime.now())
        #outputfile_name = "/data/fall2018/chapter_experiments/chapter_experiments_" + str(datetime.datetime.now())
        outputfile_name = os.path.expanduser(outputfile_name)

        with open(outputfile_name, 'wb') as csvfile:
            seen = set()
            fieldnames = [x for x in self.fieldnames if not (x in seen or seen.add(x))] #http://www.martinbroadhurst.com/removing-duplicates-from-a-list-while-preserving-order-in-python.html

            datawriter = csv.DictWriter(csvfile, fieldnames=fieldnames, restval='', extrasaction='ignore')
            datawriter.writeheader()

            while not self.should_shutdown: #This means that results stop getting saved to file as soon as I try to kill it
                try:
                    task = queue.get(block=False)

                    result_string = "Result of ["
                    for k,v in task.iteritems():
                        #if "result" not in k:
                            result_string+= str(k) + ":" + str(v) + ","
                    result_string += "]"

                    print result_string

                    if "error" not in task:
                        self.result_list.append(result_string)
                        if self.save_results:
                            datawriter.writerow(task)
                            csvfile.flush()
                    else:
                        del task["error"]
                        self.task_queue.put(task)
                        self.addProcess()

                    #print "Result of " + task["world"] + ":" + task["controller"] + "= " + str(task["result"])
                    queue.task_done()
                except Queue.Empty, e:
                    #print "No results!"
                    time.sleep(1)

    def signal_shutdown(self,signum,frame):
        self.shutdown()

    def shutdown(self):
        with self.children_shutdown.get_lock():
            self.children_shutdown.value = True

        for process in mp.active_children():
            process.join()

        self.should_shutdown = True

    def waitToFinish(self):
        print "Waiting until everything done!"
        self.task_queue.join()
        print "All tasks processed!"
        with self.soft_shutdown.get_lock():
            self.soft_shutdown.value = True

        #The problem is that this won't happen if I end prematurely...
        self.result_queue.join()
        print "All results processed!"

    # This list should be elsewhere, possibly in the configs package
    def addTasks(self):
        worlds = ["dense_laser", "campus_laser", "sector_laser", "office_laser"] # "dense_laser", "campus_laser", "sector_laser", "office_laser"
        fovs = ['90', '120', '180', '240', '300', '360']
        seeds = list(range(1))
        controllers = ['potential_gap']
        pi_selection = ['3.14159', '1.57079']
        taskid = 0

        # Nonholonomic Potential Gap Experiments
        # for world in ["office_laser"]:
        for world in worlds:
            for robot in ['holonomic']:
                for controller in controllers:
                    for fov in fovs:
                        for seed in seeds:
                            task = {
                                    'controller' : controller, 
                                    'robot' : robot,
                                    'world' : world,
                                    'fov' : fov,
                                    'seed' : seed,
                                    "taskid" : taskid,
                                    'controller_args': {
                                        'far_feasible' : str(world != 'office_laser'),
                                        'holonomic' : str(robot == 'holonomic')
                                        }
                                    }
                            taskid += 1
                            self.task_queue.put(task)


class STDRMaster(mp.Process):
    def __init__(self,
    task_queue,
    result_queue,
    kill_flag,
    soft_kill_flag,
    ros_port,
    stdr_port,
    stdr_launch_mutex,
    record, **kwargs):
        super(STDRMaster, self).__init__()
        self.daemon = False

        self.task_queue = task_queue
        self.result_queue = result_queue
        self.ros_port = ros_port
        self.stdr_port = stdr_port
        self.stdr_launch_mutex = stdr_launch_mutex
        self.core = None
        self.stdr_launch = None
        self.controller_launch = None
        self.stdr_driver = None
        self.current_world = None
        self.kill_flag = kill_flag
        self.soft_kill_flag = soft_kill_flag
        self.is_shutdown = False
        self.had_error = False
        self.record = record

        self.tfBuffer = None        


        self.gui = False

        print "New master"

        self.ros_master_uri = "http://localhost:" + str(self.ros_port)
        self.stdr_master_uri = "http://localhost:" + str(self.stdr_port)
        os.environ["ROS_MASTER_URI"] = self.ros_master_uri
        os.environ["STDR_MASTER_URI"]= self.stdr_master_uri

        #if 'SIMULATION_RESULTS_DIR' in os.environ:

        if self.gui==False:
            if 'DISPLAY' in os.environ:
                del os.environ['DISPLAY']   #To ensure that no GUI elements of gazebo activated
        else:
            if 'DISPLAY' not in os.environ:
                os.environ['DISPLAY']=':0'

    def run(self):
        while not self.is_shutdown and not self.had_error:
            self.process_tasks()
            time.sleep(5)
            if not self.is_shutdown:
                print >> sys.stderr, "(Not) Relaunching on " + str(os.getpid()) + ", ROS_MASTER_URI=" + self.ros_master_uri
        print "Run totally done"

    def process_tasks(self):
        # if self.tfBuffer is None:
        #     self.tfBuffer = tf2_ros.Buffer()
        #     self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.roslaunch_core()
        # rospy.set_param('/use_sim_time', 'True')
        rospy.init_node('test_driver', anonymous=True)
        rospy.on_shutdown(self.shutdown)

        # scenarios = TestingScenarios()

        self.had_error = False

        while not self.is_shutdown and not self.had_error:
            # TODO: If fail to run task, put task back on task queue
            try:

                task = self.task_queue.get(block=False)

                # scenario = scenarios.getScenario(task)

                if True:

                    (start, goal) = self.generate_start_goal(task)
                    self.move_robot(start)
                    self.roslaunch_stdr(task) #pass in world info
                    time.sleep(5)

                    # rospy.sleep(5)
                    # try:
                    # except:
                    #     print "move robot failed"
                    #     self.roslaunch_stdr(task) #pass in world info
                    #     self.move_robot(start)


                    if task["controller"] is None:
                        result = "nothing"
                    elif not self.stdr_launch._shutting_down:
                        # controller_args = task["controller_args"] if "controller_args" in task else {}

                        try:
                            controller_args = task["controller_args"] if "controller_args" in task else {}
                            # scenario.setupScenario()
                            print "Upper level"
                            print controller_args
                            self.roslaunch_controller(task["robot"], task["controller"], controller_args)
                            task.update(controller_args)    #Adding controller arguments to main task dict for easy logging

                            print "Running test..."

                            #master = rosgraph.Master('/mynode')
                            #TODO: make this a more informative type
                            time.sleep(5)
                            result = run_test(goal_pose=goal, record=self.record, taskid=task["taskid"])
                            self.controller_launch.shutdown()


                        except rospy.ROSException as e:
                            result = "ROSException: " + str(e)
                            task["error"]= True
                            self.had_error = True

                        self.controller_launch.shutdown()

                    else:
                        result = "gazebo_crash"
                        task["error"] = True
                        self.had_error = True

                else:
                    result = "bad_task"

                if isinstance(result, dict):
                    task.update(result)
                else:
                    task["result"] = result
                task["pid"] = os.getpid()
                self.return_result(task)

                if self.had_error:
                    print >> sys.stderr, result


            except Queue.Empty, e:
                with self.soft_kill_flag.get_lock():
                    if self.soft_kill_flag.value:
                        self.shutdown()
                        print "Soft shutdown requested"
                time.sleep(1)


            with self.kill_flag.get_lock():
                if self.kill_flag.value:
                    self.shutdown()

        print "Done with processing, killing launch files..."
        # It seems like killing the core should kill all of the nodes,
        # but it doesn't
        if self.stdr_launch is not None:
            self.stdr_launch.shutdown()

        if self.controller_launch is not None:
            self.controller_launch.shutdown()

        print "STDRMaster shutdown: killing core..."
        self.core.shutdown()
        #self.core.kill()
        #os.killpg(os.getpgid(self.core.pid), signal.SIGTERM)
        print "All cleaned up"

    
    def generate_start_goal(self, task):
        # try:
        #     trans = self.tfBuffer.lookup_transform("map_static", 'world', rospy.Time())
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     pass
        # print trans

        print task["world"]
        trans = [0, 0]

        if task["world"] == "sector_laser":
            scenario = SectorScenario(task, "world")
            trans[0] = 10.217199
            trans[1] = 10.1176
        elif task["world"] == "office_laser":
            scenario = FourthFloorScenario(task, "world")
            trans[0] = 43.173023
            trans[1] = 30.696842
        elif task["world"] == "dense_laser":
            scenario = SparseScenario(task, "world")
            trans[0] = 9.955517
            trans[1] = 9.823917
        elif task["world"] == "campus_laser":
            scenario = CampusScenario(task, "world")
            trans[0] = 14.990204
            trans[1] = 13.294787

        start = scenario.getStartingPose()
        goal = scenario.getGoal()
        start.position.x += trans[0]
        start.position.y += trans[1]
        goal.pose.position.x += trans[0]
        goal.pose.position.y += trans[1]
            
        return (start, goal)

    def start_core(self):

        #env_prefix = "ROS_MASTER_URI="+ros_master_uri + " GAZEBO_MASTER_URI=" + gazebo_master_uri + " "

        my_command = "roscore -p " + str(self.ros_port)

        #my_env = os.environ.copy()
        #my_env["ROS_MASTER_URI"] = self.ros_master_uri
        #my_env["GAZEBO_MASTER_URI"] = self.gazebo_master_uri

        print "Starting core..."
        self.core = subprocess.Popen(my_command.split()) # preexec_fn=os.setsid
        print "Core started! [" + str(self.core.pid) + "]"

    def roslaunch_core(self):

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        #roslaunch.configure_logging(uuid)

        self.core = roslaunch.parent.ROSLaunchParent(
            run_id=uuid, roslaunch_files=[],
            is_core=True, port=self.ros_port
        )
        self.core.start()

    def roslaunch_controller(self, robot, controller_name, controller_args={}):

        #controller_path =
        print "RosLaunch controller"
        print controller_args.items()

        rospack = rospkg.RosPack()
        path = rospack.get_path("potential_gap")

        # We'll assume Gazebo is launched are ready to go

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
        #roslaunch.configure_logging(uuid)
        #print path

        #Remapping stdout to /dev/null
        # sys.stdout = open(os.devnull, "w")

        for key,value in controller_args.items():
            var_name = "GM_PARAM_"+ key.upper()
            value = str(value)
            os.environ[var_name] = value
            print "Setting environment variable [" + var_name + "] to '" + value + "'"

        self.controller_launch = roslaunch.parent.ROSLaunchParent(
            run_id=uuid, roslaunch_files=[path + "/launch/" + controller_name + "_" + robot + "_controller.launch"],
            is_core=False, port=self.ros_port #, roslaunch_strs=controller_args
        )
        self.controller_launch.start()

        # sys.stdout = sys.__stdout__

    def roslaunch_stdr(self, task):
        if self.stdr_launch is not None:
            self.stdr_launch.shutdown()
        
        world = task["world"]
        fov = task["fov"]
        robot = task["robot"]

        # self.stdr_launch = world
        self.current_robot = robot

        map_num = "GM_PARAM_MAP_NUM"
        seed_num = str(task['seed'])
        os.environ[map_num] = seed_num
        print("Setting environment variable [" + map_num + "] to '" + seed_num + "'")

        fov = "GM_PARAM_RBT_FOV"
        seed_fov = str(task['fov'])
        os.environ[fov] = seed_fov

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, True)
        print world
        # launch_file_name = "stdr_" + robot + "_" + world + fov + "_world.launch"
        launch_file_name = "stdr_" + robot + "_" + world + "_world.launch"
        path = rospack.get_path("potential_gap")

        with self.stdr_launch_mutex:
            self.stdr_launch = roslaunch.parent.ROSLaunchParent(
                run_id=uuid, roslaunch_files=[path + "/launch/" + launch_file_name],
                is_core=False #, roslaunch_strs=controller_args
            )
            self.stdr_launch.start()

    def shutdown(self):
        self.is_shutdown = True

    # TODO: add conditional logic to trigger this
    def task_error(self, task):
        self.task_queue.put(task)
        self.task_queue.task_done()
        self.shutdown()

    def return_result(self,result):
        print "Returning completed task: " + str(result)
        self.result_queue.put(result)
        self.task_queue.task_done()

    
    def move_robot(self, location):
        print "Moving robot to"
        # print location

        # position
        os.environ["GM_PARAM_RBT_X"] = str(location.position.x)
        os.environ["GM_PARAM_RBT_Y"] = str(location.position.y)
        return


if __name__ == "__main__":
    master = MultiMasterCoordinator(25, record=False)
    start_time = time.time()
    master.start()
    master.addTasks()
    
    #master.singletask()
    master.waitToFinish()
    #rospy.spin()
    master.shutdown()
    end_time = time.time()
    print "Total time: " + str(end_time - start_time)
