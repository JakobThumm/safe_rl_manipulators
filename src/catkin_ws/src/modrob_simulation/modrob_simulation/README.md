# Modrob Gazebo Simulation
## Overview

_Note: This package was called modrob_visualization_gazebo before WS20_

This package allows for simulations of all feasible modular robot configurations. To guarantee a realistic simulation, physics, including **joint-/link-friction** and **-damping** as well as external forces(e.g. gravitation), are taken into account. The visualization is carried out by gazebo and can be interacted with through the gzclient.
We generate the robot-description in the form of **modrob[x].urdf** which is then translated into a **.sdf** by gazebo. For the following instructions to be valid, the **modrob_simulation-repository** needs to be cloned into **~/catkin_ws/src**. The ultimate goal of our project is to simulate robots of different configurations **grabbing and moving a GU10-Lightbulb**.

---

Previous knowledge of **ROS**, **Gazebo**, **C++**, and **XML** is highly beneficial. However, the most important functions used in our package are explained in the [ROS](http://wiki.ros.org/ROS/Tutorials)- and [Gazebo](http://gazebosim.org/tutorials)-Tutorials. Additionally, previous experience in **robot-manipulator-mechanics** and **control-theory** are expected during explanations in sections like **PID Parameter Tuning**.

## Setup

This package requires modrob_resources, if a gen2 robot is used.

Additionally, ros-melodic-ros-control and ros-melodic-effort-controllers packages are needed:

```
sudo apt-get install ros-melodic-ros-control ros-melodic-effort-controllers
```

## Simulation Demo

For a quick demonstration of **2 robots** in an empty-world environment we suggest the following commands:

```
roslaunch modrob_simulation urdf_and_gazebo.launch gripper:="true" control:="true" manual:="true"
```

The **module-orders** are published in a separate terminal:

```
rostopic pub -1 /ns/group_module_order modrob_simulation/RobotGroupModuleOrder "[{modules:[59,3,55,63,57,5], base_pos: {x: 0.2, y: 0.0, z: 0.0}}, {modules:[59,3,55,63,57,5], base_pos: {x: 1.2, y: 0.0, z: 0.0}}]"
```

The robots have been built and are simulated in gazebo. To move the robots, type either `#0` or `#1` in the terminal that was used for the first command. This informs our system that you have either chosen modrob1 or modrob0 to be controlled.
The following command moves the **first and second joint** of the chosen modrob.


```
[-0.5, 1.3]
```

For more elaborate descriptions of all processes involved during simulations and further features please refer to our **[wiki](https://gitlab.lrz.de/modularrobot/20ss/rv/modrob_visualization_gazebo/-/wikis/home)**.

> ***Should the last command result in no movement of any joint, PID-tuning needs to be carried out to make the system functional.<br>Since the default autogenerated PID-values might not suit all systems!!!***

---

## Package Content and Hierarchy

The main package is named **`modrob_simulation`** and can be found within the cloned **modrob_simulation** folder. This package contains the following folders:
<br>

- `/config`: Files needed for launch parameters: `general_config.yaml`, and configured control parameters: `modrob[x]_control.yaml`
- `/gazebo`: ROS_Bag-files: `Demo_Motion_6DOF.bag` and world-files: `single_modrob.world`
- `/launch`: Contains the launch-architecture; Described in: **"Launchfiles"**
- `/models`: The **gazebo-models** necessary for the reproduction of the Bulbs and Slides experiment
- `/msg`: **Message definitions** used for communication; Described below
- `/rviz`: The **RViz configuration file** for running RViz and Gazebo in parallel
- `/src`: The code from which all **nodes** are built
- `/stl_files`: The **meshes** used in our models and robots; Definitions of all usable modules `module_db/Modules.csv`
- `/urdf`: The generated urdf **definitions of all robots**
<br>

For more details please refer to the section: **"[Architecture](https://gitlab.lrz.de/modularrobot/20ss/rv/modrob_visualization_gazebo/-/wikis/1.-Simulation/1.-Architecture)"** within our [wiki](https://gitlab.lrz.de/modularrobot/20ss/rv/modrob_visualization_gazebo/-/wikis/home).


## Software and Hardware

The system was primarily tested on native **Ubuntu 18.04(Bionic)** running **ROS Melodic 1.14.7**, **Gazebo 9.0.0**, and **Qt 5.9.5**.
<br>All main nodes are written in C++ and adhere to the C++11 standard which is defined as the default C++ version in ROS.
<br>

The Simulation was carried out on an **ASUS Zephyrus GX502** containing an **i7-9750H(2.6GHz-4.5GHz)** and a **NVidia RTX 2070(8Gib VRAM)**.
This allows for smooth simulations at **60 frames per second**. While much worse specifications should still lead to the same result,
it has to be metioned that stutters and crashes were frequent when running on the provided image in Oracles **Virtual Box**.
However this should be attributed to the often less than optimal performance of Virtual Box when run on Windows based laptops and PCs.