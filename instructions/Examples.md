# IFRA-Cranfield: irb120_CranfieldRobotics

## ABB IRB-120 Robot Simulation and Control using ROS 2: Practical Examples

### Gazebo Simulation

This environment does not have any particular use/application, but simply visualizing the IRB120 robot and it's end-effector and enclosure in the Simulation Environment. Execute the following command to launch a ROS 2-Gazebo Simulation Environment of the IRB120-Cranfield Robot:

```sh
# ABB IRB-120 inside Cranfield Uni IA-Lab enclosure:
ros2 launch ros2srrc_launch simulation.launch.py package:=irb120cranfield config:=irb120cranfield_1
# ABB IRB-120 + Schunk EGP-64 Gripper inside CU IA-Lab enclosure: Cube Pick-and-Place use-case:
ros2 launch ros2srrc_launch simulation.launch.py package:=irb120cranfield config:=irb120cranfield_2
```

### Gazebo Simulation + MoveIt!2-based Robot Control

Execute the following command to launch the ROS 2-Gazebo Simulation Environment along with the MoveIt!2 Framework, enabling the robot to be controlled, monitored, and operated through MoveIt!2. It also loads RVIZ for visualization and gives access to the custom ROS 2 tools (/Move, /RobMove, /RobPose) for robot manipulation and monitoring.

```sh
# ABB IRB-120 inside Cranfield Uni IA-Lab enclosure:
ros2 launch ros2srrc_launch moveit2.launch.py package:=irb120cranfield config:=irb120cranfield_1
# ABB IRB-120 + Schunk EGP-64 Gripper inside CU IA-Lab enclosure: Cube Pick-and-Place use-case:
ros2 launch ros2srrc_launch moveit2.launch.py package:=irb120cranfield config:=irb120cranfield_2
```

Once the environment has been launched, there are few operations that can be done to interact with the robot. For more information, please have a look at this [link](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/blob/humble/instructions/RobotOperation.md).

- Robot Movement: 

    ```sh
    # MoveJ:
    ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveJ', movej: {joint1: 0.00, joint2: 0.00, joint3: 0.00, joint4: 0.00, joint5: 0.00, joint6: 0.00}, speed: 1.0}"
    # MoveL:
    ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveL', movel: {x: 0.00, y: 0.00, z: 0.00}, speed: 1.0}"
    # MOveR:
    ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveR', mover: {joint: '--', value: 0.00}, speed: 1.0}"
    # MoveROT:
    ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveROT', moverot: {yaw: 0.00, pitch: 0.00, roll: 0.00}, speed: 1.0}"
    # MoveRP:
    ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveRP', moverp: {x: 0.00, y: 0.00, z: 0.00, yaw: 0.00, pitch: 0.00, roll: 0.00}, speed: 1.0}"
    # MoveG (for the gripper):
    ros2 action send_goal -f /Move ros2srrc_data/action/Move "{action: 'MoveG', moveg: 0.0, speed: 1.0}"

    # RobMove:
    ros2 action send_goal -f /Robmove ros2srrc_data/action/Robmove "{type: '---', speed: 1.0, x: 0.0, y: 0.0, z: 0.0, qx: 0.0, qy: 0.0, qz: 0.0, qw: 0.0}"
    ```

- Monitor the state of the robot:

    ```sh
    # To check the state of the joints:
    ros2 run ros2srrc_execution RobotState.py
    ros 2 topic echo /joint_states

    # To check the end-effector pose:
    ros2 topic echo /Robpose
    ```

- Execute a Robot Program: The programs for the IRB120-Cranfield Robot are stored inside the irb120cranfield_execution ROS 2 Package. The following command is used to execute the programs (for more information, access this [link](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/blob/humble/instructions/ProgramExecution.md)):

    ```sh
    # Example for the irb120_demo.yaml program:
    ros2 run ros2srrc_execution ExecuteProgram.py package:=irb120cranfield_execution program:=irb120_demo
    ```

- Spawn objects into the GzSim Environment: The CAD and URDF files of the objects that are manipulated in our IRB120-Cranfield Robot's use-cases are stored in the irb120cranfield_gazebo package. The objects can be spawned to the Simulation Environment using this command (more info [here](https://github.com/IFRA-Cranfield/ros2_SimRealRobotControl/blob/humble/instructions/RobotOperation.md#extra-spawn-object-to-a-gazebo-environment)):

    ```sh
    # Generic command:
    ros2 run ros2srrc_execution SpawnObject.py --package "{}" --urdf "{}.urdf" --name "{}" --x {} --y {} --z {}

    # Command to spawn the RED CUBE on top of the IRB120 enclosure:
    ros2 run ros2srrc_execution SpawnObject.py --package "irb120cranfield_gazebo" --urdf "RedCube.urdf" --name "RedCube" --x 0.6 --y 0.55 --z 0.95
    ```

    Once the object has been spawned to the simulation environment, its pose can be checked with the following command (for more information, please visit [IFRA-Cranfield/IFRA_ObjectPose](https://github.com/IFRA-Cranfield/IFRA_ObjectPose)):
    ```sh
    ros2 topic echo /ObjectName/ObjectPose
    ros2 topic echo /WhiteCube/ObjectPose # For the white cube.
    ```

### MoveIt!2-based Control of the Real Robot

For more detailed instructions on how properly set-up any ABB Robot for ROS 2 and to connect to the ABB IRB-120 robot, please visit this [link-TBD]. Once that is ready, you can execute the following command to launch the ABB's ROS 2 driver along with MoveIt!2, and our custom ROS 2 tools for robot operation:

```sh
# In this set-up, we consider:
#   - Ubuntu PC's IP Address is -> 192.168.125.2, manually set in the PC.
#   - IRB120's IP Address is -> 192.168.125.1 (usually as default in the TEACH PENDANT).

# ABB IRB-120 inside Cranfield Uni IA-Lab enclosure:
ros2 launch ros2srrc_launch bringup_abb.launch.py package:=irb120cranfield config:=irb120cranfield_1 robot_ip:=192.168.125.1
# ABB IRB-120 + Schunk EGP-64 Gripper inside CU IA-Lab enclosure: Cube Pick-and-Place use-case:
ros2 launch ros2srrc_launch bringup_abb.launch.py package:=irb120cranfield config:=irb120cranfield_2 robot_ip:=192.168.125.1
```

Once the _Robot Bringup_ environment has been launched, the variety of tasks that can be done with the real robot are exactly the same as in simulation, with a few exceptions:

- Robot movements are exactly the same, but MoveG won't be available (this is only for Gazebo Simulation). In order to operate the Schunk EGP-64 gripper in the real ABB IRB-120 robot, the '/rws_client/set_io_signal' ROS 2 Service (ABB-RWS) is used. This implementation assumes that the Schunk gripper is connected to the Robot Controller's I/O board, with 2 signals labelled as {"OpenGripper","CloseGripper"}:

    ```sh
    # The ROS 2 server that operates the gripper is automatically launched within the bringup_abb.launch.py file.

    # To CLOSE the gripper:
    ros2 service call /rws_client/set_io_signal abb_robot_msgs/srv/SetIOSignal "{signal: 'OpenGripper', value: 0}"
    ros2 service call /rws_client/set_io_signal abb_robot_msgs/srv/SetIOSignal "{signal: 'CloseGripper', value: 1}"

    # To OPEN the gripper:
    ros2 service call /rws_client/set_io_signal abb_robot_msgs/srv/SetIOSignal "{signal: 'CloseGripper', value: 0}"
    ros2 service call /rws_client/set_io_signal abb_robot_msgs/srv/SetIOSignal "{signal: 'OpenGripper', value: 1}"
    ```

- Robot State Monitoring ROS 2 Nodes are available as for simulation.

- Robot Programs can be executed as for simulation:

    ```sh
    # Example for the irb120_demo.yaml program:
    ros2 run ros2srrc_execution ExecuteProgram.py package:=irb120cranfield_execution program:=irb120_demo
    ```

- Object Spawn feature is not available (this feature is only for Gazebo Simulation).

### Use-Case Application: Cube Pick and Place Task

__Gazebo Simulation Environment__

```sh
# 1. Launch the Sim Environment for the P&P Task:
ros2 launch ros2srrc_launch moveit2.launch.py package:=irb120cranfield config:=irb120cranfield_2

# 2. Spawn the cube (WhiteCube, RedCube, GreenCube, BlackCube or BlueCube) on top of the IRB120 enclosure:
ros2 run ros2srrc_execution SpawnObject.py --package "irb120cranfield_gazebo" --urdf "RedCube.urdf" --name "RedCube" --x 0.60 --y 0.70 --z 0.95

# 3. Execute the Cube Pick&Place Robot Program:
ros2 run ros2srrc_execution ExecuteProgram.py package:=irb120cranfield_execution program:=CubePP_irb120_sim
```

__Real ABB IRB-120 Robot__

```sh
# 1. Launch the IRB120 Robot's Robot Bringup ROS 2 Node, for the Cube P&P Task:
ros2 launch ros2srrc_launch bringup_abb.launch.py package:=irb120cranfield config:=irb120cranfield_2 robot_ip:=192.168.125.1

# 2. Place any cube in the (x: 0.60, y: 0.70) position on top of the IRB120 enclosure.

# 3. Execute the Cube Pick&Place Robot Program:
ros2 run ros2srrc_execution ExecuteProgram.py package:=irb120cranfield_execution program:=CubePP_irb120
```

### Object Pose Estimation using YOLO and OpenCV

The irb120cranfield_ope ROS 2 package performs real-time object pose estimation within the robot's workspace using a combination of YOLO and OpenCV. 

- A trained YOLO model detects colored cubes placed in the workspace, providing their pixel coordinates from the camera feed. 

- OpenCV techniques are then applied to convert these pixel coordinates into position coordinates relative to the camera. To align these coordinates with the robot's frame of reference, a camera-to-robot transformation is performed using an ArUco tag grid. 

- Finally, the estimated object poses are live published to a dedicated ROS 2 Topic, allowing seamless communication and integration with the robot's control system.

__Coloured Cube Pose Estimation: Gazebo Simulation__

Follow these steps to replicate the coloured cube pose estimation and pick & place task in simulation:

1. Launch the Gazebo Simulation Environment + MoveIt!2 Framework for the task:

    ```sh
    ros2 launch ros2srrc_launch moveit2.launch.py package:=irb120cranfield config:=irb120cranfield_3
    ```

2. Run the Cube Pose Estimation ROS 2 node:

    ```sh
    # Execute the PositionEstimation.py script:
    ros2 run irb120cranfield_ope PositionEstimation.py environment:=gazebo model:=ColouredCubes_irb120_sim visualize:=true

    # This script has the following input parameters:
    #   - environment: gazebo/robot -> To select between the simulation or real camera.
    #   - model -> To select the trained YOLO model, located in irb120cranfield_ope/yolo/models folder.
    #   - visualize: True/False -> To show the YOLO prediction output in the screen.

    # Please note that, for the PositionEstimation node to work properly, the ArUco grid must be completely visible!
    ```

3. Spawn any cube to the robot workspace:

    ```sh
    ros2 run ros2srrc_execution SpawnObject.py --package "irb120cranfield_gazebo" --urdf "BlueCube.urdf" --name "BlueCube" --x 0.6 --y 0.55 --z 0.95
    ros2 run ros2srrc_execution SpawnObject.py --package "irb120cranfield_gazebo" --urdf "GreenCube.urdf" --name "GreenCube" --x 0.6 --y 0.55 --z 0.95
    ros2 run ros2srrc_execution SpawnObject.py --package "irb120cranfield_gazebo" --urdf "RedCube.urdf" --name "RedCube" --x 0.6 --y 0.55 --z 0.95
    ros2 run ros2srrc_execution SpawnObject.py --package "irb120cranfield_gazebo" --urdf "WhiteCube.urdf" --name "WhiteCube" --x 0.6 --y 0.55 --z 0.95

    # The ColouredCubes.pt detection models have been trained to detect blue, green, red and white cubes.
    # Feel free to manually move the cubes around in the simulation environment, the PositionEstimation node will detect them!

    # Once the cubes have been spawned, you will be able to monitor their estimated position using:
    ros2 topic list
    ros2 topic echo /BlueCube/ObjectPoseEstimation
    ros2 topic echo /GreenCube/ObjectPoseEstimation
    ros2 topic echo /RedCube/ObjectPoseEstimation
    ros2 topic echo /WhiteCube/ObjectPoseEstimation
    ```

4. Run the Cube Pick & Place program:

    ```sh
    # Execute the PositionEstimation.py script:
    ros2 run irb120cranfield_ope CubePP.py environment:=gazebo cube:=BlueCube

    # This script has the following input parameters:
    #   - environment: gazebo/robot -> To select between the simulation or real camera.
    #   - cube: RedCube/WhiteCube/GreenCube/BlueCube -> To select the cube to be picked.
    ```

__Coloured Cube Pose Estimation: ABB IRB-120 Real Robot__

Follow these steps to replicate the coloured cube pose estimation and pick & place task in the real ABB IRB-120 robot:

1. Launch the Robot Bringup Environment + MoveIt!2 Framework for the task:

    ```sh
    ros2 launch ros2srrc_launch bringup_abb.launch.py package:=irb120cranfield config:=irb120cranfield_3 robot_ip:=192.168.125.1
    ```

2. Run the Cube Pose Estimation ROS 2 node:

    ```sh
    # Execute the PositionEstimation.py script:
    ros2 run irb120cranfield_ope PositionEstimation.py environment:=robot model:=ColouredCubes_irb120 visualize:=true

    # This script has the following input parameters:
    #   - environment: gazebo/robot -> To select between the simulation or real camera.
    #   - model -> To select the trained YOLO model, located in irb120cranfield_ope/yolo/models folder.
    #   - visualize: True/False -> To show the YOLO prediction output in the screen.

    # Please note that, for the PositionEstimation node to work properly, the ArUco grid must be completely visible!
    ```

3. Spawn any cube to the robot workspace:

    ```sh
    # Manually locate the coloured cubes on top of the robot workspace.

    # The ColouredCubes.pt detection models have been trained to detect blue, green, red and white cubes.
    # Feel free to manually move the cubes around in the simulation environment, the PositionEstimation node will detect them!

    # Once the cubes have been spawned, you will be able to monitor their estimated position using:
    ros2 topic list
    ros2 topic echo /BlueCube/ObjectPoseEstimation
    ros2 topic echo /GreenCube/ObjectPoseEstimation
    ros2 topic echo /RedCube/ObjectPoseEstimation
    ros2 topic echo /WhiteCube/ObjectPoseEstimation
    ```

4. Run the Cube Pick & Place program:

    ```sh
    # Execute the PositionEstimation.py script:
    ros2 run irb120cranfield_ope CubePP.py environment:=robot cube:=BlueCube

    # This script has the following input parameters:
    #   - environment: gazebo/robot -> To select between the simulation or real camera.
    #   - cube: RedCube/WhiteCube/GreenCube/BlueCube -> To select the cube to be picked.
    ```