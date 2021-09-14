#include <string>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>


// Identifier for the robot that is controlled
uint robot_id = 0;

// Gripper parameters
const double finger1_zero_pos = 0.0;
const double finger2_zero_pos = 0.03;
const double finger1_max = 0.01;
const double finger2_max = 0.04;

// Flag for when a gripper is attached
bool gripper = false;

// Transmission start indicator
bool transmission_start = false;

// Current joint_states; updated by inverse_kin
sensor_msgs::JointState joint_states;

// callback for inverse kinematics of recieved cartesian trajectories
void input_callback(sensor_msgs::JointState state){
    transmission_start = true;
    for(int i = 0; i < state.position.size(); i++){
        joint_states.position[i] = state.position[i];
    }
        
    if(gripper){
        double f1 = joint_states.position[joint_states.position.size()-2];
        double f2 = joint_states.position[joint_states.position.size()-1];
        if(f1 > finger1_max){
            joint_states.position[joint_states.position.size()-2] = finger1_max;
        }
        if(f1 < finger1_zero_pos){
            joint_states.position[joint_states.position.size()-2] = finger1_zero_pos;
        }
        if(f2 > finger2_max){
            joint_states.position[joint_states.position.size()-1] = finger2_max;
        }
        if(f2 < finger2_zero_pos){
            joint_states.position[joint_states.position.size()-1] = finger2_zero_pos;
        }
    }
}



int main(int argc, char **argv){
    // Check assigned robot-id
    robot_id = std::stoi(std::string(argv[1]));
    std::cout << "The ID is:  " << robot_id << "\n";

    // inits
    ros::init(argc, argv, "motion_publisher", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::Rate r(100);

    // Publishers and Subscribers
    std::vector<ros::Publisher> publishers;
    ros::Subscriber sub;
    sub = n.subscribe("/modrob" + std::to_string(robot_id) + "/motion", 10, input_callback);

    int joint_num;
    if(ros::param::has("/modrob" + std::to_string(robot_id) + "_joint_num")){
        ros::param::get("/modrob" + std::to_string(robot_id) + "_joint_num", joint_num);
    }else{
        joint_num = 0;
    }

    for(int i = 0; i < joint_num; i++){
        ros::Publisher pub;
        publishers.push_back(pub);
        joint_states.position.push_back(0.0);
    }

    // Advertise publishers of the rotational joints
    int joint_count = 0;
    for(auto & it: publishers){
        it = n.advertise<std_msgs::Float64>("/modrob" + std::to_string(robot_id) + "/joint" + std::to_string(joint_count) +  "_position_controller/command", 10);
        joint_count++;
    }

    // Check whether a gripper is attached
    if(ros::param::has("/modrob" + std::to_string(robot_id) + "_has_gripper")){
        ros::param::get("/modrob" + std::to_string(robot_id) + "_has_gripper", gripper);
    }else{
        gripper = false;
    }

    // Generate and advertise gripper-position publishers
    ros::Publisher finger1;
    ros::Publisher finger2;
    if(gripper){
        finger1 = n.advertise<std_msgs::Float64>("/modrob" + std::to_string(robot_id) + "/hand_to_finger1_position_controller/command", 10);
        finger2 = n.advertise<std_msgs::Float64>("/modrob" + std::to_string(robot_id) + "/hand_to_finger2_position_controller/command", 10);
        joint_states.position.push_back(finger1_zero_pos);
        joint_states.position.push_back(finger2_zero_pos);
    }

    while(ros::ok()){

        // Start publishing after receiving an initial transmission
        if(transmission_start){
            int gripper_addon = 0;
            if(gripper){
                gripper_addon = 2;
            }

            // If we don't have enough joint values -> error state; revert to zero position
            if(joint_states.position.size() < joint_num+gripper_addon){
                for(int i = joint_states.position.size(); i < joint_num+gripper_addon; i++){
                    joint_states.position.push_back(0.0);
                }
            }

            // Publish onto the rotational joint-topics
            for(int i = 0; i < joint_num; i++){
                std_msgs::Float64 f;
                f.data = joint_states.position[i];

                publishers[i].publish(f);
            }

            // Publish onto the translational gripper-joint-topics
            if(gripper){
                std_msgs::Float64 f1;
                f1.data = joint_states.position[joint_states.position.size()-2];
                finger1.publish(f1);
                std_msgs::Float64 f2;
                f2.data = joint_states.position[joint_states.position.size()-1];
                finger2.publish(f2);
            }
        }


        ros::spinOnce();
    }
    return 0;
}