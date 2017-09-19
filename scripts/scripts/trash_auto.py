#!/usr/bin/env python
import rospy
import rospkg
import roslaunch

import test_driver
from gazebo_driver_v2 import Prototype
import std_msgs.msg
import sys
import time
from nav_msgs.msg import Odometry
from kobuki_msgs.msg import BumperEvent
from gazebo_msgs.srv import GetModelState
import csv
import math

num_controller = 8
launch = None
controller_name = []


baseResult = []
baseTime = []
ebandResult = []
ebandTime = []
tebResult = []
tebTime = []
LPipsToGoResult = []
LPipsToGoTime = []
LPipsGoalBiasedResult = []
LPipsGoalBiasedTime = []
LDWAToGoResult = []
LDWAToGoTime = []
LDWAGoalBiasedResult = []
LDWAGoalBiasedTime = []
LNaiveResult = []
LNaiveTime = []

returnedPosition = []
masterResult = [baseResult, ebandResult, tebResult, LPipsToGoResult, LPipsGoalBiasedResult, LDWAToGoResult, LDWAGoalBiasedResult, LNaiveResult]
masterTime = [baseTime, ebandTime, tebTime, LPipsToGoTime, LPipsGoalBiasedTime, LDWAToGoTime, LDWAGoalBiasedTime, LNaiveTime]
currentPos = [0, 0, 0]
crapStatus = 0

goal_x = -5
goal_y = 3.0
filenum = 33
numTest = 50
numbarrels = 7


def gazebo_setup():
    print "Gazebo setup"
    gazebo_driver.shutdown()
    global returnedPosition
    gazebo_driver.moveBarrels(numbarrels)

    # Unpause physics?


def gazebo_bridge():
    print "Gazebo Bridge"
    gazebo_driver.resetRobot()
    gazebo_driver.resetBarrels(numbarrels)


def gazebo_teardown():
    print "Gazebo teardown"
    gazebo_driver.delete_barrels(numbarrels)
    # Pause physics?


def launch_controller(controller_name):
    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    path = None
    if (controller_name == "LDwa_togo.launch"):
        path = rospack.get_path("pips_dwa_implementation")
    elif (controller_name == "LDwa_goalBiased.launch"):
        path = rospack.get_path("pips_dwa_implementation")
    elif (controller_name == "Lpips_togo.launch"):
        path = rospack.get_path("pips_dwa_implementation")
    elif (controller_name == "Lpips_goalBiased.launch"):
	path = rospack.get_path("pips_dwa_implementation")
    elif (controller_name == "naive_follower.launch"):
	path = rospack.get_path("pips_dwa_implementation")
    else:
        path = rospack.get_path('pips_test')
    # We'll assume Gazebo is launched are ready to go
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    print path
    global launch
    launch = roslaunch.parent.ROSLaunchParent(
        uuid, [path + "/launch/" + controller_name])
    launch.start()

    # Give it time to boot up
    time.sleep(15)


def kill_controller():
    # This kills the launch, not the script! Beautiful.
    global launch
    launch.shutdown()
    # print "I can still do stuff here"


def reset_odom(pub):
    pub.publish()


def writeCSVfile():
    global filenum
    outputfile_name = 'recorded_data_jin_machine_attempt1/data_' + \
        str(filenum) + "_numbarrels_" + str(numbarrels) + '.csv'
    with open(outputfile_name, 'wb') as csvfile:
        datawriter = csv.writer(csvfile, delimiter=' ',
                                quoting=csv.QUOTE_MINIMAL)
        testNumber = ['test ' + str(filenum)]
        datawriter.writerow(testNumber)
        positions = []
        for j in range(numbarrels):
            global returnedPosition
            # print(str(returnedPosition[j][0]))
        datawriter.writerow(['DWA Result', masterResult[0]])
        datawriter.writerow(['DWA Time', masterTime[0]])
        datawriter.writerow(['Eband Result', masterResult[1]])
        datawriter.writerow(['Eband Time', masterTime[1]])
        datawriter.writerow(['TEB Result', masterResult[2]])
        datawriter.writerow(['TEB Time', masterTime[2]])
        datawriter.writerow(['LPIPS ToGo Result', masterResult[3]])
        datawriter.writerow(['LPIPS ToGo Time', masterTime[3]])
        datawriter.writerow(['LPIPS Gaussian Result', masterResult[4]])
        datawriter.writerow(['LPIPS Gaussian Time', masterTime[4]])
        datawriter.writerow(['LDWA ToGo Result', masterResult[5]])
        datawriter.writerow(['LDWA ToGo Time', masterTime[5]])
        datawriter.writerow(['LDWA Gaussian Result', masterResult[6]])
        datawriter.writerow(['LDWA Gaussian Time', masterTime[6]])
        datawriter.writerow(['NAIVE Gaussian Result', masterResult[7]])
        datawriter.writerow(['NAIVE Gaussian Time', masterTime[7]])



def odomData(data):
    global currentPos
    currentPos[0] = data.pose.pose.position.x
    currentPos[1] = data.pose.pose.position.y
    currentPos[2] = data.pose.pose.position.z


def callService():
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp1 = gms(model_name="mobile_base", relative_entity_name="world")
        return resp1
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def getState():
    data = callService()
    current_x = data.pose.position.x
    print("My Position is ", current_x)
    current_y = data.pose.position.y
    print("My Position is ", current_y)
    diff_x = (goal_x - current_x) * (goal_x - current_x)
    diff_y = (goal_y - current_y) * (goal_y - current_y)
    diff_total = math.sqrt(diff_x + diff_y)
    if (diff_total < 1.0):
        return True
    else:
        return False


def clearLists():
    global baseResult
    global baseTime
    global ebandResult
    global ebandTime
    global tebResult
    global tebTime
    global LPipsToGoResult
    global LPipsToGoTime
    global LPipsGoalBiasedResult
    global LPipsGoalBiasedTime
    global LDWAToGoResult
    global LDWAToGoTime
    global LDWAGoalBiasedResult
    global LDWAGoalBiasedTime
    global LNaiveResult
    global LNaiveTime
    global returnedPosition
    global masterResult
    global masterTime
    baseResult = []
    baseTime = []
    ebandResult = []
    ebandTime = []
    tebResult = []
    tebTime = []
    LPipsToGoResult = []
    LPipsToGoTime = []
    LPipsGoalBiasedResult = []
    LPipsGoalBiasedTime = []
    LDWAToGoResult = []
    LDWAToGoTime = []
    LDWAGoalBiasedResult = []
    LDWAGoalBiasedTime = []
    LNaiveResult = []
    LNaiveTime = []
    returnedPosition = []
    masterResult = [baseResult, ebandResult, tebResult, LPipsToGoResult, LPipsGoalBiasedResult, LDWAToGoResult, LDWAGoalBiasedResult, LNaiveResult]
    masterTime = [baseTime, ebandTime, tebTime, LPipsToGoTime, LPipsGoalBiasedTime, LDWAToGoTime, LDWAGoalBiasedTime, LNaiveTime]


def main(controller_name):
    odom_pub = rospy.Publisher(
        '/mobile_base/commands/reset_odometry', std_msgs.msg.Empty, queue_size=10)
    getState()
    for segment in range(numTest * 3):
        global filenum

        gazebo_setup()
        reset_odom(odom_pub)
        for i in range(num_controller):
            global crapStatus
            reset_odom(odom_pub)

            launch_controller(controller_name[i])

            start = time.time()
            output = test_driver.run_test()
            print("Result was " + str(output))

            if (crapStatus == 1):
                masterResult[i].append('fail_bumped')
            else:
                if (output == True):
                    output2 = getState()
                    if (output2):
                        masterResult[i].append('success')
                    else:
                        masterResult[i].append('fail_bumped')
                else:
                    masterResult[i].append('fail_noFinding')
            crapStatus = 0
            masterTime[i].append(time.time() - start)
            kill_controller()
            time.sleep(5)
            gazebo_bridge()

        reset_odom(odom_pub)
        writeCSVfile()
        filenum = filenum + 1
        clearLists()
        if(filenum > numTest):
            global numbarrels
            filenum = 1
            numbarrels = numbarrels + 2
            clearLists()


if __name__ == "__main__":
    gazebo_driver = Prototype()
    controller_name.append("baseLocalPlanner_controller.launch")
    controller_name.append("eband_controller.launch")
    controller_name.append("teb_controller.launch")
    controller_name.append("Lpips_togo.launch")
    controller_name.append("Lpips_goalBiased.launch")
    controller_name.append("LDwa_togo.launch")
    controller_name.append("LDwa_goalBiased.launch")
    controller_name.append("naive_follower.launch")
    main(controller_name)
