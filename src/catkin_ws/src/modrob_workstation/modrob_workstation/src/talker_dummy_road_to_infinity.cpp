#include "ros/ros.h"
#include "modrob_workstation/JointConfigCommanded.h"
#include "modrob_workstation/RobotConfigCommanded.h"
#include <math.h>


int main(int argc, char **argv){
	// Initialize ROS with Node "talker_dummy"
	ros::init(argc, argv, "talker_dummy");
	// Create Handle for this Node
	ros::NodeHandle n;
	// Tell Master that the Node publishes to /ns/robot_config_commanded
	ros::Publisher config_publisher = n.advertise<modrob_workstation::RobotConfigCommanded>("/ns/robot_config_commanded", 1000);
	// Specify loop Rate
	ros::Rate loop_rate(1000);

	double counter = 0;
	while(ros::ok()){
		// init msg_joint_1
		modrob_workstation::JointConfigCommanded msg_joint_1;
		msg_joint_1.joint_angle = (double)(counter);
		msg_joint_1.joint_velocity = (double)(counter);
		msg_joint_1.joint_acceleration = (double)(counter);

		// init msg_joint_2
		modrob_workstation::JointConfigCommanded msg_joint_2;
		msg_joint_2.joint_angle = (double)(counter+1);
		msg_joint_2.joint_velocity = (double)(counter+1);
		msg_joint_2.joint_acceleration = (double)(counter+1);

		modrob_workstation::JointConfigCommanded msg_joint_3;
		msg_joint_3.joint_angle = (double)(counter+2);
		msg_joint_3.joint_velocity = (double)(counter+2);
		msg_joint_3.joint_acceleration = (double)(counter+2);

		// init msg
		modrob_workstation::RobotConfigCommanded msg;
		msg.joint_moves = {msg_joint_1,msg_joint_2,msg_joint_3};

		// print the message to terminal
		std::cout<< "Joint_0 ---- angle: " << msg.joint_moves[0].joint_angle << " -- velocity: " << msg.joint_moves[0].joint_velocity << " -- acceleration: " << msg.joint_moves[0].joint_acceleration << " --" << std::endl;
		//std::cout<< "Joint_1 ---- angle: " << msg.joint_moves[1].joint_angle << " -- velocity: " << msg.joint_moves[1].joint_velocity << " -- acceleration: " << msg.joint_moves[1].joint_acceleration << " --" << std::endl;

		// publish the message
		config_publisher.publish(msg);

		// not nessicary, just in case of callbacks
		ros::spinOnce();

		loop_rate.sleep();

		counter += 0.001;

	}
	return 0;
}

