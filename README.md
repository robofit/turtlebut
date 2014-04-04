turtlebut
=========

ROS packages modified for our TurtleBut robot that is based on the TurtleBot platform

## Simulation

Clone following repos to your catkin workspace:

* https://github.com/ros-simulation/gazebo_ros_pkgs
* https://github.com/ros-controls/control_toolbox
* https://github.com/ros-controls/ros_controllers
* https://github.com/ros-controls/ros_control

Install dependencies and compile.

Launch simulation:

```bash
roslaunch tb_gazebo robot_empty_world.launch
```

## Terminal examples

Control of Kinect position (look down):

```bash
rostopic pub /dynamixel/kinect_controller/command std_msgs/Float64 '{data: -0.2}'
```
