import test_driver
from testing_scenarios import TestingScenarios
import rospy


seed = 51

num_barrels = 3

task = {'seed':seed, 'scenario':'sector'}

rospy.init_node('test_driver', anonymous=True)

scenarios = TestingScenarios()
scenario = scenarios.getScenario(task)

rospy.Rate(1).sleep()

scenario.setupScenario()
result = test_driver.run_test(goal_pose=scenario.getGoal())
