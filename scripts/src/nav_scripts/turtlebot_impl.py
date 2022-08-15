
import rospy
from nav_scripts.movebase_driver import BaseBumperChecker, RobotImpls, RobotImpl as BaseImpl
from kobuki_msgs.msg import BumperEvent




class BumperChecker(BaseBumperChecker):
    def __init__(self):
        super().__init__(name="turtlebot_bumper")
        self.sub = rospy.Subscriber("mobile_base/events/bumper", BumperEvent, self.bumperCB, queue_size=5)

    def bumperCB(self,data):
        if data.state == BumperEvent.PRESSED:
            self.collided = True




class RobotImpl(BaseImpl):
    name = "turtlebot"

    def get_terminal_conditions(self):
        return [BumperChecker()]



RobotImpls.register(RobotImpl)

