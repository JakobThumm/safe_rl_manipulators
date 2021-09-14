cmake_minimum_required(VERSION 3.0.2)
project(obstacles)

## Add support for C++17, supported in ROS Kinetic and newer
add_definitions(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp  
  std_msgs
  geometry_msgs
  custom_robot_msgs
  gazebo_ros
)

find_package(LibXml2 REQUIRED)
include_directories(${LIBXML2_INCLUDE_DIR})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(gazebo REQUIRED)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
  includes
  ${Boost_INCLUDE_DIR} 
  ${catkin_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS}
)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
  #INCLUDE_DIRS ${GAZEBO_INCLUDE_DIRS}
  #LIBRARIES obstacles ${GAZEBO_LIBRARY_DIRS}
  CATKIN_DEPENDS geometry_msgs roscpp std_msgs custom_robot_msgs
  DEPENDS gazebo_ros system_lib
)

add_library(cv_sphere SHARED src/cv_sphere.cc)
add_library(kalman_filter includes/obstacles/cv_model.h src/cv_model.cc includes/obstacles/kf_model.h src/kf_model.cc)
target_link_libraries(cv_sphere ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} kalman_filter)

add_library(stick SHARED includes/obstacles/stick.h src/stick.cc)
target_link_libraries(stick ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(motion_capture SHARED includes/obstacles/motion_capture.h includes/obstacles/skeleton_node.h
    src/motion_capture.cc)
add_dependencies(motion_capture ${catkin_EXPORTED_TARGETS})
target_link_libraries(motion_capture ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${LIBXML2_LIBRARIES})