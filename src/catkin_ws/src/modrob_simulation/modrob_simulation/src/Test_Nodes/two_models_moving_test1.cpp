#include "ros/ros.h"
#include "std_msgs/String.h"
#include <modrob_workstation/RobotConfigCommanded.h>
#include <modrob_workstation/JointConfigCommanded.h>
#include <modrob_workstation/RobotConfigMeasured.h>
#include <modrob_workstation/JointConfigMeasured.h>
#include <queue>          // std::queue

#include <sstream>

const double degree = M_PI/180;

const double angle_step = 0.01;
const int rate = 10;
const int maxDelay = 10;

std::queue<modrob_workstation::RobotConfigCommanded> commandedMsgQueue;
int delay = 0;
bool delayCountUp = true;

bool sendMeasured(){
    delay = delay>maxDelay?maxDelay:(delay<0?0:delay);
    bool res = commandedMsgQueue.size() > delay;
    if(res){
		delayCountUp = ((delay!=maxDelay)&&(delayCountUp))||(delay==0);
		delay = delay+(delayCountUp?1:-1);
		ROS_INFO("delay: %d; delayCountUp: %s", delay, delayCountUp ? "true" : "false");
	}
    return res;
}

modrob_workstation::RobotConfigMeasured measuredMsg(){
	//ROS_INFO("Build measured msg");
    modrob_workstation::RobotConfigCommanded currentCommandedMsg = commandedMsgQueue.front();
    //ROS_INFO("Current commanded msgs joint configs: %d", currentCommandedMsg.joint_moves.size());
	commandedMsgQueue.pop();
    modrob_workstation::RobotConfigMeasured measuredMsg;
	
    for (std::vector<modrob_workstation::JointConfigCommanded>::iterator it = currentCommandedMsg.joint_moves.begin() ; it != currentCommandedMsg.joint_moves.end(); ++it){
		//ROS_INFO("Create joint config measured msg");
        modrob_workstation::JointConfigCommanded jointNCommanded=*it;
        modrob_workstation::JointConfigMeasured jointNMeasured;
        jointNMeasured.joint_angle = jointNCommanded.joint_angle;
	    jointNMeasured.joint_velocity = jointNCommanded.joint_velocity;
        jointNMeasured.joint_acceleration = jointNCommanded.joint_acceleration;
        jointNMeasured.joint_torque = jointNCommanded.joint_torque;
        jointNMeasured.joint_temperature = 50;

        measuredMsg.joint_config_measured.push_back(jointNMeasured);
    }
	
	//ROS_INFO("Finished building measured msg");
	return measuredMsg;
}


modrob_workstation::RobotConfigCommanded robotConfigCommanded;
modrob_workstation::JointConfigCommanded revolute0;
modrob_workstation::JointConfigCommanded revolute1;


modrob_workstation::RobotConfigCommanded commandedMsg(){

	robotConfigCommanded.joint_moves.clear();
	
    robotConfigCommanded.tool_activation = false;
	
	revolute0.joint_angle += angle_step;
	revolute0.joint_velocity  = 3.0 * degree;
    revolute0.joint_acceleration = 0.0;
    revolute0.joint_torque  = 2.0;

    robotConfigCommanded.joint_moves.push_back(revolute0);
    
    revolute1.joint_angle += angle_step;
	revolute1.joint_velocity  = 10.0 * degree;
    revolute1.joint_acceleration = 0.0;
    revolute1.joint_torque  = 9.0;

    robotConfigCommanded.joint_moves.push_back(revolute1);

    commandedMsgQueue.push(robotConfigCommanded);

    return robotConfigCommanded;

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "two_models_moving_test1");
	ros::NodeHandle n("~");
	ROS_INFO("Advertising publisher");
	ros::Publisher commanded_pub = n.advertise<modrob_workstation::RobotConfigCommanded>("/ns/robot_config_commanded", 2000);
	ros::Publisher measured_pub = n.advertise<modrob_workstation::RobotConfigMeasured>("/ns/robot_config_measured", 2000);
	ros::Rate loop_rate(rate);

	ROS_INFO("Starting main loop");
  	while (ros::ok()) {	
        if(sendMeasured()) {
			ROS_INFO("Sending measured msg");
            measured_pub.publish(measuredMsg());
        } else {
			ROS_INFO("Sending commanded msg");
            commanded_pub.publish(commandedMsg());
        }

        ros::spinOnce();

        loop_rate.sleep();
  	}

  return 0;
}
