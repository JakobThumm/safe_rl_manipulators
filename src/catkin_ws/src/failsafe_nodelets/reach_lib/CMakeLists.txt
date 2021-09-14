cmake_minimum_required(VERSION 3.0.2)
project(reach_lib)

## Compile as C++17
add_compile_options(-std=c++17)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  custom_robot_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp geometry_msgs custom_robot_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
set(REACH_LIB_SOURCES
    src/Articulated.cpp
    src/Articulated_A.cpp
    src/Articulated_P.cpp
    src/Articulated_V.cpp
    src/BodyPart.cpp
    src/BodyPart_A.cpp
    src/BodyPart_V.cpp
    src/Capsule.cpp
    src/Cylinder.cpp
    src/Extremity.cpp
    src/Point.cpp
    src/System.cpp)
add_library(${PROJECT_NAME}  ${REACH_LIB_SOURCES})
add_dependencies(${PROJECT_NAME}  ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


install(TARGETS ${PROJECT_NAME} 
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})


## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

