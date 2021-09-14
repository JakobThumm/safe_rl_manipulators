#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <modrob_workstation/JointConfigCommanded.h>
#include <modrob_workstation/RobotConfigCommanded.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

ros::Publisher joint_config_commanded_publisher;
ros::Publisher tool_activation_publisher;
const double degree = M_PI/180;
bool tool_activation = false;

void translate_robotconfigcommanded_to_jointstate(const modrob_workstation::RobotConfigCommanded::ConstPtr& msg)
{
	ROS_INFO("Got commanded msg");
  	int number_of_joints = msg->joint_moves.size();
	ROS_INFO("Number of joints: %d", number_of_joints);

    sensor_msgs::JointState joint_states;
    joint_states.header.stamp = ros::Time::now();


	for (int i = 0; i < number_of_joints; i++){
		modrob_workstation::JointConfigCommanded jointConfig = msg->joint_moves[i];
		joint_states.name.push_back("joint" + std::to_string(i));
        joint_states.position.push_back(jointConfig.joint_angle);
		joint_states.velocity.push_back(jointConfig.joint_acceleration);
		joint_states.effort.push_back(jointConfig.joint_torque);

	}

    joint_config_commanded_publisher.publish(joint_states);

    // Visualize tool activation
    if(msg->tool_activation != tool_activation){
        tool_activation = msg->tool_activation;
        visualization_msgs::MarkerArray markerArray;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "rc/tool_dummy";
        marker.ns = "joint_selection";
        marker.id = -1;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.06;
        marker.scale.y = 0.06;
        marker.scale.z = 0.06;
        marker.color.a = msg->tool_activation ? 0.8 : 0.01; 
        marker.color.r = 0.8;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.frame_locked = true;
        markerArray.markers.push_back(marker);
        tool_activation_publisher.publish(markerArray);
    }
	
}


int main(int argc, char** argv) 
{  
    ros::init(argc, argv, "robotconfigcommanded_to_jointstate_translator");
    ros::NodeHandle n;
    joint_config_commanded_publisher = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
    tool_activation_publisher = n.advertise<visualization_msgs::MarkerArray>( "/joint_selection/markers", 0 );
    ros::Rate loop_rate(10);   
    ros::Subscriber subcriberJointConfigMeasured = n.subscribe("/ns/robot_config_commanded", 1000, translate_robotconfigcommanded_to_jointstate);  
    ros::spin();
    return 0;
}
