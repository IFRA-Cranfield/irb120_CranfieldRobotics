# IFRA-Cranfield: irb120_CranfieldRobotics

## Installation Steps

The steps below must be followed in order to properly set-up a ROS 2 Humble machine which is needed for the usage of the ROS 2 Packages in the irb120_CranfieldRobotics repository. It is recommended to install Ubuntu 22.04 Desktop on your PC for an optimal performance, but a VM could be used for simple simulations and executions.

__REQUIRED: Install the ros2_SimRealRobotControl GitHub Repository__

The ROS 2 packages developed in IRB120-CranfieldRobotics are based on IFRA-Cranfield's [ros2_SimRealRobotControl](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl) GitHub Repository. Therefore, ros2_SimRealRobotControl must be installed in order to set-up IRB120-CR in any Ubuntu 22.04 + ROS 2 Humble machine.

Installation steps can be found at: https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/blob/humble/instructions/Installation.md

__Download and install irb120_CranfieldRobotics__

```sh
cd ~/dev_ws/src
git clone https://github.com/IFRA-Cranfield/irb120_CranfieldRobotics
cd ~/dev_ws
colcon build
```   