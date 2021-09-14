#include "ros/ros.h"
#include "modrob_workstation/JointConfigCommanded.h"
#include "modrob_workstation/RobotConfigCommanded.h"
#include "modrob_workstation/RobotConfigMeasured.h"
#include "modrob_workstation/JointConfigMeasured.h"

void configCallback(const modrob_workstation::RobotConfigMeasured& msg){
	// print the received message
	std::cout<< "Joint_0 ---- angle: " << msg.joint_config_measured[0].joint_angle << " -- velocity: " << msg.joint_config_measured[0].joint_velocity << " -- acceleration: " << msg.joint_config_measured[0].joint_acceleration << " -- torque: "<< msg.joint_config_measured[0].joint_torque <<" -- temperature: " << msg.joint_config_measured[0].joint_temperature << std::endl;
	std::cout<< "Joint_1 ---- angle: " << msg.joint_config_measured[1].joint_angle << " -- velocity: " << msg.joint_config_measured[1].joint_velocity << " -- acceleration: " << msg.joint_config_measured[1].joint_acceleration << " -- torque: "<< msg.joint_config_measured[1].joint_torque <<" -- temperature: " << msg.joint_config_measured[1].joint_temperature << std::endl;
	std::cout<< "Joint_2 ---- angle: " << msg.joint_config_measured[2].joint_angle << " -- velocity: " << msg.joint_config_measured[2].joint_velocity << " -- acceleration: " << msg.joint_config_measured[2].joint_acceleration << " -- torque: "<< msg.joint_config_measured[2].joint_torque <<" -- temperature: " << msg.joint_config_measured[2].joint_temperature << std::endl;

}

int main(int argc, char **argv){

	// init ROS with Node "robot_control_interface"
	ros::init(argc, argv, "listener_dummy");

	// Create Handle for this Node
	ros::NodeHandle n;

	// Tell Master that the Node subscribed to /ns/robot_config_commanded
	ros::Subscriber config_subscriber = n.subscribe("/ns/robot_config_measured", 1000, configCallback);

	// Enters a loop to call callbacks
	ros::spin();

	return 0;

}
