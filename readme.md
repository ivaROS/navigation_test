# navigation_test

| WARNING: This is still WiP as we try to organize the packages and dependencies to bring the repo to public domain, please check back frequently! |
| --- |

| NavBench only supports ROS Kinetic and is intended for controllers as ```nav_core:BaseLocalPlanner``` plugin to the ```move_base``` stack. |
| --- |


NavBench is the local planner navigation testbench used in IVALab. This testbench is capable of testing controllers of different configurations in different world of randomly populated obstacles and randomly assigned start and goal position. 

There are two packages under this repository, ```nav_scripts``` and ```nav_configs```. ```nav_scripts``` are the package that initiates the testing. ```nav_configs``` contains the supporting files for ```nav_scripts```, including robot and sensor models, configuration files for controllers, Gazebo worlds, and launch files for launching Gazebos. More details will be covered in subsequent sections. 

## Test Worlds
Provided worlds are:
- dense: empty square world with only side walls
- full_campus_obstacle (Campus World): outdoor free space consists of large free spaces connected by narrower corridors.
- fourth_floor_world (Office World): simplified model of the fourth floor of the building containing our lab. 

To visualize the worlds, edit the launch files in ```nav_configs/launch/``` so that default value of ```gui``` is true. Run one of the following to visualize the world.

```bash
$ roslaunch nav_configs gazebo_turtlebot_empty_room_20x20_world.launch
$ roslaunch nav_configs gazebo_turtlebot_campus_obstacle_world.launch
$ roslaunch nav_configs gazebo_turtlebot_fourth_floor_obstacle_world.launch
```

To further populate the world with random obstacles:
```sh
$ python2 scripts/scripts/run_single_task.py
```
Specify minimum obstacle spacing in the same [file](scripts/scripts/run_single_task.py#L4)


## Running the Tests
### Visualized Demo Case
A demo case for benchmarking is provided. run
```bash
python2 gazebo_master_demo.py
```
To visualize the process, edit ```nav_configs/launch/gazebo_turtlebot_empty_room_20x20_world.launch``` that `gui` is true. 

### Dependencies and Machine Requirements
NavBench provides the option of launching multiple Gazebo simulations to help execute testing concurrently. This parameter can be specified in **XX location**. This will cause additional CPU usage and may impact simulation performance. We observed on average about 4 cores per individual Gazebo is needed on our Intel Xeon E5-2640 @ 2.50GHz (Single Core Passmark 1468 and Multi-Threaded score of 9512). 

### Run comprehensive test on Planners
| WARNING: Please read the previous section before proceeding |
| --- |
1. If you change the ``gui`` argument in launch file to true, revert it back to ``false`` otherwise the CPU load will 
1. Navigate to ```scripts/scripts```
2. Specify test output logging directory in function ```processResults``` in the [gazebo_master.py](scripts/scripts/gazebo_master.py#L104)
3. Specify the number of Multimaster in the [constructor](scripts/scripts/gazebo_master.py#L1544) of master object. 
4. Execute the following command
```bash
$ python2 gazebo_master.py
```
6. This set of test performs 100 runs of _TEB_ and _egoTEB_ individually on empty world with minimum space of 0.5m. The test should take ~40 minutes with 4 master running concurrently.
5. Upon completion, pass in the test log directory to ``analyze_results.py`` and execute the file to get a readme-style table containing the results. 


### Add your own controllers
| Note: Remove this maybe |
| --- |
1. Clone this repository into your catkin workspace and ```catkin_make```. The latest branch to clone is ```chapter``` branch. **change this to the latest branch** You would also need to clone ```gazebo_utils``` to make this work. 
2. Prepare the launch file for your local planner. The launch files should launch move_base, include necessary files and launch any necessary node required for your local planner other than everything provided by move_base.
Naming format should follow ```{robotmodel}_{local_planner_name}_controller.launch```, for example, ```turtlebot_teb_controller.launch```. Put your launch file in ```navigation_test/scripts/launch```
3. Find ```navigation_test/scripts/scripts/gazebo_master_test.py``` Scroll all the way to bottom. Fill out test_scenarios and test_controllers with the scenarios and controllers you want to test with. Run the file using python2.
4. There is default to be no Gazebo GUI, if you want to see exact testing details from Gazebo GUI, go to ```nav_configs/launch``` and find the launch file that launches the specific world. In the format of ```gazebo_{robot_name}_{world_name}_world.launch```
