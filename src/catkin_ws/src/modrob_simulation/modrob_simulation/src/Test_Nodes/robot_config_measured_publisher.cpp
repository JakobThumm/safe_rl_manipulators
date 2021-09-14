

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <modrob_workstation/RobotConfigMeasured.h>
#include <modrob_workstation/JointConfigMeasured.h>

#include <sstream>

const double degree = M_PI/180;

modrob_workstation::RobotConfigMeasured pubConfig(){
	modrob_workstation::RobotConfigMeasured robotConfigMeasured;
	modrob_workstation::JointConfigMeasured revolute0;

	revolute0.joint_angle += revolute0.joint_velocity;
	revolute0.joint_velocity  = 3.0 * degree;
    	revolute0.joint_acceleration = 0.0;
    	revolute0.joint_torque  = 2.0;
    	revolute0.joint_temperature = 50.0;
    	robotConfigMeasured.joint_config_measured.push_back(revolute0);

	modrob_workstation::JointConfigMeasured revolute1;
    	revolute1.joint_angle += revolute1.joint_velocity;
	revolute1.joint_velocity  = 10.0 * degree;
    	revolute1.joint_acceleration = 0.0;
    	revolute1.joint_torque  = 9.0;
    	revolute1.joint_temperature = 110.0;
    	robotConfigMeasured.joint_config_measured.push_back(revolute1);

    	return robotConfigMeasured;

}

int main(int argc, char **argv)
{
    	ros::init(argc, argv, "robot_config_measured_publisher");
    	ros::NodeHandle n;
    	ros::Publisher chatter_pub = n.advertise<modrob_workstation::RobotConfigMeasured>("/ns/robot_config_measured", 1000);
    	ros::Rate loop_rate(10);

  	while (ros::ok())
  	{	
    		chatter_pub.publish(pubConfig());
		
    		ros::spinOnce();

    		loop_rate.sleep();
  	}

  return 0;
}
