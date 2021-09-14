# modrob_workstation

This is a Robot Control Interface for Modular Robots. The Interface provides a link from the "Workstation Computer", running the ROS Middleware System, to the Modular Robot Realtime Controller. The Robot Controller and the Workstation Computer shoud be connected via Ethernet.The Interface will send **down-stream Messages** to the Robot and receive **up-stream Messages** from the Robot. For real-time critical control commands, real-time message transfer is guaranteed. For non-realtime critical state commands, real-time message transfer is not guaranteed.

## How to build the Control Interface

To build the interface clone the repository into your catkin src directory.

### Compile the Workstation Library

To compile the Workstation Library, execute the following steps:
1. create a Build directory in the workstation folder and cd into it
```
mkdir workstation/build
cd workstation/build
```
2. generate a make file with **cmake**
```
cmake ..
```
3. compile the library with the generated **make file**
```
make
```

### Compile the ROS Interface

To complie the ROS Interface run the `catkin_make` command from the workspace root 
```
catkin_make
```
Note: The ROS Interface depends on the workstation library, the compilation will not work if the workstation isn't compiled as shown above.

## How to use the Control Interface

To run the ROS node use:
```
rosrun modrob_workstation robot_control_interface _robotIP:="YOUR_IP"
```

**YOUR_IP** needs to be the IP Adress of the Robot Computer
if you dont specify the `_robotIP` the Interface will use the default IP adress: 192.168.8.1

```
rosrun modrob_workstation robot_control_interface 
```

## Robot Description

The modrob_workstation package also contains a robot_description_publisher node. This node subscribes to the '/ns/robot_module_order' topic (ModuleOrder.msg) and when received, publishes a robot description to 'ns/robot_description' (RobotDescription.msg). The Node creates a private ROS parameter "generation", which controls whether the old prototype or the gen2 prototypes are used:

Value '1': Old generation. Requires stl_files folder in modrob_resources.

Value '2': Gen2. Requires ModuleLibrary folder in modrob_resources.

To start only the robot_description node, use:

```
rosrun modrob_workstation robot_description_publisher 
```
## URDF Generator

The modrob_workstation package also contains a urdf_generator node. This node subscribes to the 'ns/robot_description' topic (RobotDescription.msg) and when received sets a ROS string parameter '/urdf_file' with the urdf file.

To start only the urdf_generator node, use:

```
rosrun modrob_workstation urdf_generator 
```

### Launch files
The robot_description_publisher and the urdf_generator can be launched using:
```
roslaunch modrob_workstation robot_description_and_urdf.launch
```
_When working with the real robot, the robot_control_interface has to be started additionally_

### Dummy mode

To use the workstation in dummy mode (emulate a robot), use ... To be determined.

License: BSD

Authors: Robin Geissler, Lukas Weiss, Kasra Asadi, Vincent Jeltsch

Changes for WS20 by Authors: Maximilian Körsten
