#!/usr/bin/env python
from __future__ import print_function
from builtins import str
from builtins import range
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

num_controller = 1
launch = None
controller_name = []


numTR2 = []
baseTime = []
numTR3 = []
ebandTime = []
numTR7 = []
tebTime = []


returnedPosition = []
masterResult = [numTR2, numTR3, numTR7]
masterTime = [baseTime, ebandTime, tebTime]
currentPos = [0, 0, 0]
crapStatus = 0

goal_x = -5
goal_y = 3.0
filenum = 1
numTest = 50
numbarrels = 5


def gazebo_setup():
    print("Gazebo setup")
    gazebo_driver.shutdown()
    global returnedPosition
    gazebo_driver.moveBarrels(numbarrels)

    # Unpause physics?


def gazebo_bridge():
    print("Gazebo Bridge")
    gazebo_driver.resetRobot()
    gazebo_driver.resetBarrels(numbarrels)


def gazebo_teardown():
    print("Gazebo teardown")
    gazebo_driver.delete_barrels(numbarrels)
    # Pause physics?


def launch_controller(controller_name):
    # get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()
    path = None
    path = rospack.get_path('pips_dwa_implementation')
    # We'll assume Gazebo is launched are ready to go
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    print(path)
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
    outputfile_name = 'recorded_data_jin_machine_attempt2/data_' + \
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
        datawriter.writerow(['TR2 Result', masterResult[0]])
        datawriter.writerow(['TR2 Time', masterTime[0]])
        datawriter.writerow(['TR3 Result', masterResult[1]])
        datawriter.writerow(['TR3 Time', masterTime[1]])
        datawriter.writerow(['TR7 Result', masterResult[2]])
        datawriter.writerow(['TR7 Time', masterTime[2]])



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
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def getState():
    data = callService()
    current_x = data.pose.position.x
    print(("My Position is ", current_x))
    current_y = data.pose.position.y
    print(("My Position is ", current_y))
    diff_x = (goal_x - current_x) * (goal_x - current_x)
    diff_y = (goal_y - current_y) * (goal_y - current_y)
    diff_total = math.sqrt(diff_x + diff_y)
    if (diff_total < 1.0):
        return True
    else:
        return False


def clearLists():
    global numTR2
    global baseTime
    global numTR3
    global ebandTime
    global numTR7
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
    numTR2 = []
    baseTime = []
    numTR3 = []
    ebandTime = []
    numTR7 = []
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
    masterResult = [numTR2, numTR3, numTR7]
    masterTime = [baseTime, ebandTime, tebTime]


def main(controller_name):
    odom_pub = rospy.Publisher(
        '/mobile_base/commands/reset_odometry', std_msgs.msg.Empty, queue_size=10)
    getState()
    for segment in range(1, 51):
        global filenum

        gazebo_setup()
        reset_odom(odom_pub)
        if (segment not in [3, 10, 12, 16, 19, 23, 26, 27, 29, 30, 38]):
	  time.sleep(1)
	  continue     
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
    controller_name.append("LDwa_togo.launch")
    main(controller_name)
