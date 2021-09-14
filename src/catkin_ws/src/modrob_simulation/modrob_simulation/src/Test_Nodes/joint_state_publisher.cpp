#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <modrob_workstation/JointConfigMeasured.h>
#include <modrob_workstation/RobotConfigMeasured.h>

ros::Publisher joint_pub;
sensor_msgs::JointState joint_state;
const double degree = M_PI/180;
double state = 0.0;

void chatterCallback(const modrob_workstation::RobotConfigMeasured::ConstPtr& msg)
{
  	int size_joints = msg->joint_config_measured.size();

	joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(size_joints); 
        joint_state.position.resize(size_joints);
	joint_state.velocity.resize(size_joints);
	joint_state.effort.resize(size_joints);

	for (int i = 0; i < size_joints; i++){
		modrob_workstation::JointConfigMeasured jc = msg->joint_config_measured[i];
		double jc_angle = jc.joint_angle;
		double jc_velocity = jc.joint_velocity;
		double jc_acceleration = jc.joint_acceleration;
		double jc_torque = jc.joint_torque;
		double jc_temperature = jc.joint_temperature;

		joint_state.name[i] ="joint" + std::to_string(i);
		// state += degree;
        joint_state.position[i] = jc_angle; 
		joint_state.velocity[i] = jc_velocity;
		joint_state.effort[i] = jc_torque;
	}

	//joint_state.name[0] ="joint0";
	//joint_state.name[1] ="joint1";

	joint_pub.publish(joint_state);
}


int main(int argc, char** argv) 
{
    ros::init(argc, argv, "joint_state_publisher");
    ros::NodeHandle n;
    joint_pub = n.advertise<sensor_msgs::JointState>("my_joint_states", 100);
    ros::Rate loop_rate(10);    

    ros::Subscriber sub = n.subscribe("/ns/robot_config_measured", 1000, chatterCallback);

    ros::spin();

    return 0;
}
