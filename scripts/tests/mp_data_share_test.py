import multiprocessing as mp
import socket
import contextlib

def port_in_use(port):
    with contextlib.closing(socket.socket(socket.AF_INET, socket.SOCK_STREAM)) as sock:
        if sock.connect_ex(('127.0.0.1', port)) == 0:
            print("Port " + str(port) + " is in use")
            return True
        else:
            print("Port " + str(port) + " is not in use")
            return False

class PortSelector(object):

    @classmethod
    def port(cls):
        port_object = cls.current_port
        with port_object.get_lock():
            while port_in_use(port_object.value):
                port_object.value += 1
            val = port_object.value
            port_object.value += 1

        return val



class RosPort(PortSelector):
    current_port = mp.Value('i', 11311)


class GazeboPort(PortSelector):
    current_port = mp.Value('i', 11411)


import os
class RosEnv(object):
    port = None

    @staticmethod
    def init(use_existing_roscore=False):
        ros_port = RosEnv.get_ros_port() if use_existing_roscore else RosPort.port()
        RosEnv.set_ros_port(ros_port=ros_port)
        RosEnv.port = ros_port

    @staticmethod
    def set_ros_port(ros_port):
        ros_master_uri = "http://localhost:" + str(ros_port)
        os.environ["ROS_MASTER_URI"] = ros_master_uri

    @staticmethod
    def get_ros_port():
        try:
            ros_master_uri = os.environ["ROS_MASTER_URI"]
        except KeyError as e:
            print("No ROS_MASTER_URI specified!")
            raise e
        else:
            port_ind = ros_master_uri.rindex(":")
            port_str = ros_master_uri[port_ind+1:]
            ros_port = int(port_str)
            return ros_port



class PortTester(mp.Process):
    ros_port = RosPort()
    gazebo_port = GazeboPort()

    def __init__(self):
        super(PortTester, self).__init__()
        self.ros = PortTester.ros_port
        self.gazebo = PortTester.gazebo_port


    def run(self):
        print("Ros port = " + str(PortTester.ros_port.port()))
        print("Gazebo port = " + str(PortTester.gazebo_port.port()))


def run_test(num):
    processes = [PortTester() for _ in range(num)]
    print("Created PortTesters")

    for p in processes:
        p.start()

    print("Started PortTesters, waiting to join")

    for p in processes:
        p.join()

    print("All Done")

    man = RosEnv()
    man.set_ros_port(1000)
    print(man.get_ros_port())


if __name__ == "__main__":
    run_test(5)