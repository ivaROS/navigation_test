import test_driver
from testing_scenarios import TestingScenarios
import rospy


seed = 1
num_barrels = 3

task = {'seed':seed, 'num_barrels':num_barrels, 'scenario':'trashcans'}

rospy.init_node('test_driver', anonymous=True)

scenarios = TestingScenarios()
scenario = scenarios.getScenario(task)
scenario.setupScenario()
result = test_driver.run_test(goal_pose=scenario.getGoal())
