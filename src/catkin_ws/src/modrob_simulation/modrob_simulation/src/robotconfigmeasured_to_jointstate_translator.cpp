#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <modrob_workstation/JointConfigMeasured.h>
#include <modrob_workstation/RobotConfigMeasured.h>

ros::Publisher joint_config_measured_publisher;
const double degree = M_PI/180;

void translate_robotconfigmeasured_to_jointstate(const modrob_workstation::RobotConfigMeasured::ConstPtr& msg)
{
	ROS_INFO("Got measured msg");
  	int number_of_joints = msg->joint_config_measured.size();
	ROS_INFO("Number of joints: %d", number_of_joints);

    sensor_msgs::JointState joint_states;
    joint_states.header.stamp = ros::Time::now();


	for (int i = 0; i < number_of_joints; i++){
		modrob_workstation::JointConfigMeasured jointConfig = msg->joint_config_measured[i];
		joint_states.name.push_back("joint" + std::to_string(i));
        joint_states.position.push_back(jointConfig.joint_angle);
		joint_states.velocity.push_back(jointConfig.joint_acceleration);
		joint_states.effort.push_back(jointConfig.joint_torque);

	}

	joint_config_measured_publisher.publish(joint_states);
}


int main(int argc, char** argv) 
{  
    ros::init(argc, argv, "robotconfigmeasured_to_jointstate_translator");
    ros::NodeHandle n;
    joint_config_measured_publisher = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
    ros::Rate loop_rate(10);   
    ros::Subscriber subcriberJointConfigMeasured = n.subscribe("/ns/robot_config_measured", 1000, translate_robotconfigmeasured_to_jointstate);  
    ros::spin();
    return 0;
}
