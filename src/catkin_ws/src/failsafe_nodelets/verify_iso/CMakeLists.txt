cmake_minimum_required(VERSION 3.0.2)
project(verify_iso)

## Compile as C++17, supported in ROS Noetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  nodelet
  geometry_msgs
  roscpp
  std_msgs
  custom_robot_msgs
  message_filters
  global_library
)

catkin_package(
  INCLUDE_DIRS include tests
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS nodelet geometry_msgs roscpp std_msgs custom_robot_msgs global_library
#  DEPENDS system_lib
)

include_directories(
  include
  tests
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME} src/nodelet.cc src/verify_iso.cc src/advanced_verify_iso.cc)
add_dependencies(${PROJECT_NAME}  ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} 
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES nodelet_plugins.xml launch/test.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_add_gtest(verify_iso_test tests/capsule_tests.cc tests/verify_iso_fixture.h)
target_link_libraries(verify_iso_test ${catkin_LIBRARIES} ${PROJECT_NAME})