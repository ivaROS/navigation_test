# navigation_test

| NavBench currently only supports ROS Kinetic |
| --- |


NavBench is the local planner navigation testbench used in IVALab. This testbench is capable of testing controllers of different configurations in different world of randomly populated obstacles and randomly assigned start and goal position. 

There are two packages under this repository, ```nav_scripts``` and ```nav_configs```. ```nav_scripts``` are the package that initiates the testing. ```nav_configs``` contains the supporting files for ```nav_scripts```, including robot and sensor models, configuration files for controllers, Gazebo worlds, and launch files for starting simulations. More details will be covered in subsequent sections. 

## Test Worlds
Provided worlds include:
- dense: empty square world with only side walls
- full_campus_obstacle (Campus World): outdoor free space consists of large free spaces connected by narrower corridors.
- fourth_floor_world (Office World): simplified model of the fourth floor of the building containing our lab. 

To visualize the worlds, edit the launch files in ```nav_configs/launch/``` so that default value of the ```gui``` argument is true. Run one of the following to visualize the world.

```bash
$ roslaunch nav_configs gazebo_turtlebot_empty_room_20x20_world.launch
$ roslaunch nav_configs gazebo_turtlebot_campus_obstacle_world.launch
$ roslaunch nav_configs gazebo_turtlebot_fourth_floor_obstacle_world.launch
```

## Timing Demo
__this section should probably be moved elsewhere__ timing demo can be called with `python timing\_demo.py` under nav_scripts package. RosBag Path must be given to `nav\_scripts/scripts/launch/rosbag/rosbag.launch`

