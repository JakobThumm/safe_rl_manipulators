#include "ros/ros.h"
#include "modrob_workstation/JointConfigCommanded.h"
#include "modrob_workstation/RobotConfigCommanded.h"
#include "modrob_workstation/RobotStateCommanded.h"
#include <math.h>


int main(int argc, char **argv){
	// Initialize ROS with Node "talker_dummy_sinus"
	ros::init(argc, argv, "talker_dummy_sinus");
	// Create Handle for this Node
	ros::NodeHandle n;
	// Tell Master that the Node publishes to /ns/robot_config_commanded
	ros::Publisher config_publisher = n.advertise<modrob_workstation::RobotConfigCommanded>("/ns/robot_config_commanded", 1000);

	ros::Publisher state_publisher = n.advertise<modrob_workstation::RobotStateCommanded>("/ns/robot_state_commanded", 1000);

	sleep(1);

	// Specify loop Rate
	ros::Rate loop_rate(1000);

	modrob_workstation::RobotStateCommanded state_msg;
	state_msg.state = 4;
	state_publisher.publish(state_msg);

	ros::spinOnce();

	double counter = 0;
	while(ros::ok()){
		double start_pos = -0.168659;
		double amplitude = 0.5;
		double vorzeichen = -1;
		double offset = vorzeichen*(amplitude + start_pos);
		// init msg_joint_1  -0.168659
		modrob_workstation::JointConfigCommanded msg_joint_1;
		msg_joint_1.joint_angle = (double)(cos(counter)*vorzeichen*amplitude - offset);
		msg_joint_1.joint_velocity = (double)(-sin(counter)*vorzeichen*amplitude);
		msg_joint_1.joint_acceleration = (double)(-cos(counter)*vorzeichen*amplitude);

		// init msg_joint_2
		modrob_workstation::JointConfigCommanded msg_joint_2;
		msg_joint_2.joint_angle = (double)0/*(double)sin(counter+1)*/;
		msg_joint_2.joint_velocity = (double)0/*(double)cos(counter+1)*/;
		msg_joint_2.joint_acceleration = (double)0/*(double)-sin(counter+1)*/;

		modrob_workstation::JointConfigCommanded msg_joint_3;
		msg_joint_3.joint_angle = (double)0/*(double)sin(counter+2) * 0.2*/;
		msg_joint_3.joint_velocity = (double)0/*(double)cos(counter+2) * 0.2*/;
		msg_joint_3.joint_acceleration = (double)0/*(double)-sin(counter+2) * 0.2*/;

		// init msg
		modrob_workstation::RobotConfigCommanded msg;
		msg.joint_moves = {msg_joint_1,msg_joint_2,msg_joint_3};

		// print the message to terminal
		std::cout<< "Joint_0 ---- angle: " << msg.joint_moves[0].joint_angle << " -- velocity: " << msg.joint_moves[0].joint_velocity << " -- acceleration: " << msg.joint_moves[0].joint_acceleration << " --" << std::endl;
		std::cout<< "Joint_1 ---- angle: " << msg.joint_moves[1].joint_angle << " -- velocity: " << msg.joint_moves[1].joint_velocity << " -- acceleration: " << msg.joint_moves[1].joint_acceleration << " --" << std::endl;

		// publish the message
		config_publisher.publish(msg);
		
		// not nessicary, just in case of callbacks
		ros::spinOnce();
		
		//std::cout<< "Joint_0 ---- angle: " << msg.joint_moves[0].joint_angle << " -- velocity: " << msg.joint_moves[0].joint_velocity << " -- acceleration: " << msg.joint_moves[0].joint_acceleration << " --" << std::endl;
		
		counter += 0.00177;
		
		loop_rate.sleep();

	}
	return 0;
}

