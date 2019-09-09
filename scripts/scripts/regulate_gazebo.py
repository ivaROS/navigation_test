import psutil
import rospy
import gazebo_driver_v2 as driver
import time

driver = driver.GazeboDriver(as_node=True)

def getProc():
    for proc in psutil.process_iter():
        if "callgrind" in proc.name():
            return proc
    return None

def getUsage(process):
    return process.cpu_percent(interval=0)

current_rate=1
paused=False

p = getProc()
if p is not None:
    while True:
        cur_usage = getUsage(p)
        print cur_usage
        if not paused and cur_usage > 90:
            driver.pause()
            paused=True
            print "Pausing!"
        elif paused and cur_usage < 80:
            driver.unpause()
            paused=False
            print "Resuming!"
        time.sleep(.2)
