#!/usr/bin/env python

from gazebo_master import MultiMasterCoordinator
import time

if __name__ == "__main__":
    start_time = time.time()
    master = MultiMasterCoordinator(1)
    master.start()
    master.addDemoTask()
    master.waitToFinish()
    master.shutdown()
    end_time = time.time()
    print "Total time: " + str(end_time - start_time)
