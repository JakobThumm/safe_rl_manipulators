cmake_minimum_required(VERSION 2.8.3)
project(modrob_simulation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  tf
  gazebo_ros
)

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
  ${Boost_INCLUDE_DIR} 
  ${GAZEBO_INCLUDE_DIRS}
)
################################################
## Declare ROS messages, services and actions ##
################################################


# Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Collision.msg
   Collisions.msg
   JointConfigCommanded.msg
   JointConfigMeasured.msg
   JointDescription.msg
   LinkCollision.msg
   LinkDescription.msg
   LinkVisual.msg
   LinkSensor.msg
   Modes.msg
   RobotModuleOrder.msg
   RobotConfigCommanded.msg
   RobotConfigMeasured.msg
   RobotDescription.msg
   RobotGroupDescription.msg
   RobotGroupModuleOrder.msg
 )


# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   modrob_simulation
   geometry_msgs
 )



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS roscpp std_msgs message_runtime
  DEPENDS gazebo_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS})
include_directories(includes)
include_directories(/usr/include/eigen3)


######## real-time interface for the conversion to sensor_msgs::JointState
add_executable(motion_terminal src/motion_terminal.cpp)
target_link_libraries(motion_terminal ${catkin_LIBRARIES})
add_dependencies(motion_terminal modrob_simulation_generate_messages_cpp)

######## publishing of motions onto position_controller_topics
add_executable(motion_publisher src/motion_publisher.cpp)
target_link_libraries(motion_publisher ${catkin_LIBRARIES})
add_dependencies(motion_publisher modrob_simulation_generate_messages_cpp)

######## republishing of JointStates
add_executable(republishing src/republishing.cpp)
target_link_libraries(republishing ${catkin_LIBRARIES})
add_dependencies(republishing modrob_simulation_generate_messages_cpp)

######## Robot config measured publisher
add_executable(robot_config_measured_publisher src/Test_Nodes/robot_config_measured_publisher.cpp)
target_link_libraries(robot_config_measured_publisher ${catkin_LIBRARIES})
add_dependencies(robot_config_measured_publisher modrob_simulation_generate_messages_cpp)

######## Robot config command publisher
add_executable(robot_config_commanded_publisher src/Test_Nodes/robot_config_commanded_publisher.cpp)
target_link_libraries(robot_config_commanded_publisher ${catkin_LIBRARIES})
add_dependencies(robot_config_commanded_publisher modrob_simulation_generate_messages_cpp)

######## Robot Description publisher
add_executable(robot_description_publisher_gazebo src/Robot_Description_Publisher/robot_description_publisher.cpp)
target_link_libraries(robot_description_publisher_gazebo ${catkin_LIBRARIES})
add_dependencies(robot_description_publisher_gazebo modrob_simulation_generate_messages_cpp)

######## create_urdf_and_launch_gazebo
add_executable(create_urdf_and_launch_gazebo src/create_urdf_and_launch_gazebo.cpp)
target_link_libraries(create_urdf_and_launch_gazebo ${catkin_LIBRARIES})
add_dependencies(create_urdf_and_launch_gazebo modrob_simulation_generate_messages_cpp)

######## Joint State Publisher
add_executable(joint_state_publisher src/Test_Nodes/joint_state_publisher.cpp)
target_link_libraries(joint_state_publisher ${catkin_LIBRARIES})
add_dependencies(joint_state_publisher modrob_simulation_generate_messages_cpp)

######## Contact plugin
add_library(modrob_contact_plugin SHARED src/modrob_contact_plugin.cc)
target_link_libraries(modrob_contact_plugin ${GAZEBO_LIBRARIES})
add_dependencies(modrob_contact_plugin modrob_simulation_generate_messages_cpp)

# ######## RobotConfigCommanded To JointState Translator
# add_executable(robotconfigcommanded_to_jointstate_translator src/robotconfigcommanded_to_jointstate_translator.cpp)
# target_link_libraries(robotconfigcommanded_to_jointstate_translator ${catkin_LIBRARIES})
# add_dependencies(robotconfigcommanded_to_jointstate_translator modrob_simulation_generate_messages_cpp)

# ######## RobotConfigMeasured To JointState Translator
# add_executable(robotconfigmeasured_to_jointstate_translator src/robotconfigmeasured_to_jointstate_translator.cpp)
# target_link_libraries(robotconfigmeasured_to_jointstate_translator ${catkin_LIBRARIES})
# add_dependencies(robotconfigmeasured_to_jointstate_translator modrob_simulation_generate_messages_cpp)

######## Two Models Moving Test1
add_executable(two_models_moving_test1 src/Test_Nodes/two_models_moving_test1.cpp)
target_link_libraries(two_models_moving_test1 ${catkin_LIBRARIES})
add_dependencies(two_models_moving_test1 modrob_simulation_generate_messages_cpp)


