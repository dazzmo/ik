# Cassie Inverse Kinematics (IK) ROS Node

This project provides a ROS2 node for demonstrating the use of the inverse kinematics (IK) library developed in this repository. After IK computations are performed the robot's movements are visualized in RViz.

## Prerequisites
Ensure that the following packages are installed:
- ROS2
- [pinocchio](https://stack-of-tasks.github.io/pinocchio)
- RViz

For testing purposes we have:
* [googletest](https://github.com/google/googletest)
* [glog](https://github.com/google/glog)


## Installation
If you would like to run this ROS2 example, you can either clone this entire repository and run it as is
```sh
git clone https://github.com/dazzmo/ik
cd ik
colcon build
source ./install/local_setup.sh
ros2 launch ik_ros cassie_ik.launch.py
```
You can also install the ik library natively to your machine and then use the ik library whereever you need.