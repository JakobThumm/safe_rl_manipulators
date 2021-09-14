#include "ros/ros.h"
#include "modrob_workstation/JointConfigCommanded.h"
#include "modrob_workstation/RobotConfigCommanded.h"
#include "modrob_workstation/RobotConfigMeasured.h"
#include "modrob_workstation/JointConfigMeasured.h"
#include "modrob_workstation/RobotStateCommanded.h"
#include "modrob_workstation/ModuleOrder.h"
#include "workstation_adapter.hpp"

#define DEFAULT_IP "192.168.8.1"

int main(int argc, char **argv){

	// init ROS with Node "robot_control_interface"
	ros::init(argc, argv, "robot_control_interface");
	
	// Create Handle for this Node
	ros::NodeHandle n("~");
	
	// Tell Master that the Node publishes to /ns/robot_config_measured
	ros::Publisher config_publisher = n.advertise<modrob_workstation::RobotConfigMeasured>("/ns/robot_config_measured", 1000);
	
	// Tell Master that the Node publishes to /ns/robot_module_order
	ros::Publisher module_publisher = n.advertise<modrob_workstation::ModuleOrder>("/ns/robot_module_order", 1000);

	std::cout << "Subscribers & Publishers setup successfully.." << std::endl;


	// Receiver method for workstation - receiveConfigMeasured
	std::function<void(RobotConfigMeasured)> receiveConfigMeasured([&config_publisher](RobotConfigMeasured config){
		config_publisher.publish(WorkstationAdapter::parseToConfigMeasured(config));
	});

	// Receiver method for workstation - receiveModuleOrder
	std::function<void(RobotModuleOrder)> receiveModuleOrder([&module_publisher](RobotModuleOrder moduleOrder){
		module_publisher.publish(WorkstationAdapter::parseToModuleOrder(moduleOrder));
	});

	// Set robotIP: default = "192.168.8.1"
	std::string robot_ip;
	n.param<std::string>("robotIP", robot_ip, DEFAULT_IP);
	ROS_INFO("Connecting to IP address: %s", robot_ip.c_str());
	// Delete ROS parameter robotIP - that needs to be done to set robotIP to DEFAULT_IP in the next run, if robotIP not set
	n.deleteParam("robotIP");

	// Init Workstation with the publisher methods receiveConfigMeasured and receiveModuleOrder that will be called if a received message comes in
	WorkstationAdapter workstation_adapter(robot_ip.c_str(), receiveConfigMeasured, receiveModuleOrder);

	// Tell Master that the Node subscribed to /ns/robot_config_commanded
	ros::Subscriber config_subscriber = n.subscribe("/ns/robot_config_commanded", 1000, &WorkstationAdapter::sendConfig, &workstation_adapter);

	// Tell Master that the Node subscribed to /ns/robot_state_commanded
	ros::Subscriber state_subscriber = n.subscribe("/ns/robot_state_commanded", 1000, &WorkstationAdapter::sendState, &workstation_adapter);

	// Enter a loop where config_subscriber and state_subscriber wait for messages
	ros::spin();


	return 0;

}


