cmake_minimum_required(VERSION 3.1)
project(workstaion)


# set build type to Release
set(CMAKE_BUILD_TYPE Release)

# set c++ standard
set(CMAKE_CXX_STANDARD 14)

# include the headers into the project
include_directories(include)

# include all src files into the project
#file(GLOB SOURCES "src/*.cpp")
#file(GLOB HEADERS "include/*.h" "include/*.hpp")

#target_link_libraries(workstation pthread)

# generate shared library
add_library(
	workstation
	SHARED

	include/workstation.hpp
	src/workstation.cpp

	include/networking/UDPSocket.h
	src/networking/UDPSocket.cpp
	include/networking/TCPSocket.h
	src/networking/TCPSocket.cpp
	include/networking/UDPServerSocket.h
	src/networking/UDPServerSocket.cpp

	include/CommandMessage.hpp
	include/JointAngleCommanded.hpp
	include/JointTorqueCommanded.hpp
	include/RobotCommanded.hpp
	include/RobotStateCommanded.hpp
	include/RobotAngleCommanded.hpp
	include/RobotTorqueCommanded.hpp
	include/JointConfigMeasured.hpp
        include/RobotModuleOrder.hpp
	include/RobotConfigMeasured.hpp

	src/CommandMessage.cpp
	src/JointAngleCommanded.cpp
	src/JointTorqueCommanded.cpp
	src/RobotAngleCommanded.cpp
	src/RobotTorqueCommanded.cpp
	src/RobotStateCommanded.cpp
	src/RobotCommanded.cpp
        src/RobotModuleOrder.cpp
	src/RobotConfigMeasured.cpp
	src/JointConfigMeasured.cpp


)

# not working with self includes in library
#target_include_directories(workstation PUBLIC ${CMAKE_CURRENT_SOURCE_DIR/include})

# set location for lib installation
install(TARGETS workstation DESTINATION /usr/lib)
