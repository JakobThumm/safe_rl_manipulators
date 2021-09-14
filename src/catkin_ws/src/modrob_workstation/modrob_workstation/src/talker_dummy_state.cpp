#include "ros/ros.h"
#include "modrob_workstation/RobotStateCommanded.h"
#include <math.h>


int main(int argc, char **argv){
	// Initialize ROS with Node "talker_dummy_state"
	ros::init(argc, argv, "talker_dummy_state");
	// Create Handle for this Node
	ros::NodeHandle n;
	// Tell Master that the Node publishes to /ns/robot_state_commanded
	ros::Publisher state_publisher = n.advertise<modrob_workstation::RobotStateCommanded>("/ns/robot_state_commanded", 1000);
	// Specify loop Rate
	ros::Rate loop_rate(1);

	int counter = 0;
	while(ros::ok()){
		
		// init msg
		modrob_workstation::RobotStateCommanded msg;
		msg.state = counter;

		// print the message to terminal
		std::cout<< "State: " << (int)msg.state << std::endl;

		// publish the message
		state_publisher.publish(msg);

		// not nessicary, just in case of callbacks
		ros::spinOnce();

		loop_rate.sleep();

		counter++;

	}
	return 0;
}

