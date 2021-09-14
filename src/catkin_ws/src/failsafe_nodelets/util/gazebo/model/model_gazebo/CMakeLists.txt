cmake_minimum_required(VERSION 3.0.2)
project(model_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

