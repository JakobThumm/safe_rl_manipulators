cmake_minimum_required(VERSION 3.0.2)
project(initialisation)

## Compile as C++17, supported in ROS noetic
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  modrob_workstation
  roscpp
  std_msgs
)

catkin_package(
  LIBRARIES initialisation
  CATKIN_DEPENDS roscpp std_msgs modrob_workstation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/initialisation.cc)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
