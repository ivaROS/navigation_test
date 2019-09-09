import rospy
from nav_scripts.gazebo_driver import GazeboDriver
from geometry_msgs.msg import Twist

#https://stackoverflow.com/a/6599441/2906021
def read_single_keypress():
    """Waits for a single keypress on stdin.

    This is a silly function to call if you need to do it a lot because it has
    to store stdin's current setup, setup stdin for reading single keystrokes
    then read the single keystroke then revert stdin back after reading the
    keystroke.

    Returns a tuple of characters of the key that was pressed - on Linux,
    pressing keys like up arrow results in a sequence of characters. Returns
    ('\x03',) on KeyboardInterrupt which can happen when a signal gets
    handled.

    """
    import termios, fcntl, sys, os
    fd = sys.stdin.fileno()
    # save old state
    flags_save = fcntl.fcntl(fd, fcntl.F_GETFL)
    attrs_save = termios.tcgetattr(fd)
    # make raw - the way to do this comes from the termios(3) man page.
    attrs = list(attrs_save) # copy the stored version to update
    # iflag
    attrs[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK
                  | termios.ISTRIP | termios.INLCR | termios. IGNCR
                  | termios.ICRNL | termios.IXON )
    # oflag
    attrs[1] &= ~termios.OPOST
    # cflag
    attrs[2] &= ~(termios.CSIZE | termios. PARENB)
    attrs[2] |= termios.CS8
    # lflag
    attrs[3] &= ~(termios.ECHONL | termios.ECHO | termios.ICANON
                  | termios.ISIG | termios.IEXTEN)
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    # turn off non-blocking
    #fcntl.fcntl(fd, fcntl.F_SETFL, flags_save & ~os.O_NONBLOCK)
    # read a single keystroke
    ret = []
    try:
        ret.append(sys.stdin.read(1)) # returns a single character
        fcntl.fcntl(fd, fcntl.F_SETFL, flags_save | os.O_NONBLOCK)
        c = sys.stdin.read(1) # returns a single character
        while len(c) > 0:
            ret.append(c)
            c = sys.stdin.read(1)
    except KeyboardInterrupt:
        ret.append('\x03')
    finally:
        # restore old state
        termios.tcsetattr(fd, termios.TCSAFLUSH, attrs_save)
        fcntl.fcntl(fd, fcntl.F_SETFL, flags_save)
    return tuple(ret)

class SimulationStepper(object):
    def __init__(self):
        self.mode = 0
        self.driver = GazeboDriver(as_node=True)
        self.sub = rospy.Subscriber("navigation_velocity_smoother/raw_cmd_vel", Twist, self.pause, queue_size=1)

    def pause(self, cmd):
        #print(str(cmd))
        if self.mode == 2 or self.mode == 1:
            self.driver.pause()
            key = raw_input("Paused! Press 's' to step, or 'space' to resume, followed by 'Enter'")
            #key = read_single_keypress()
            if key == 's':
                self.driver.unpause()
            elif key == ' ':
                self.mode = 0
                self.driver.unpause()
        elif self.mode == 0:
            key = raw_input("Press 's' to step, or 'space' to pause, followed by 'Enter'")
            #key = read_single_keypress()
            if key == 's':
                self.mode = 2
            elif key == ' ':
                self.mode = 1




if __name__ == "__main__":
    sim = SimulationStepper()
    rospy.spin()



