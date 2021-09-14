#include <string>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <modrob_workstation/JointConfigMeasured.h>
#include <modrob_workstation/RobotConfigMeasured.h>

int num_robots;
std::vector<ros::Publisher> publishers;
std::vector<ros::Subscriber> subscribers;

// This class enables the republishing of the joint-states of all robots to /ns/modrob[x]/robot_config_measured
class Republisher{

private:
    modrob_workstation::RobotConfigMeasured joint_states;

public:
    // Initializes a basic Republisher-Object with a given robot id
    Republisher(){}

    ~Republisher(){};

    modrob_workstation::RobotConfigMeasured get_joint_states(){
        return this->joint_states;
    }

    // Callback-fn: Convert the joint_State Msg into robot_config_measured per robot
    void translate_jointstate_to_robotconfigmeasured(const sensor_msgs::JointState::ConstPtr& msg)
    {

        ROS_INFO("Received joint_state msg.");
        int joint_size = msg->name.size();
        modrob_workstation::RobotConfigMeasured measured_joint_config;

        for(int i = 0; i < joint_size; i++){
            modrob_workstation::JointConfigMeasured jointConfig;
            jointConfig.joint_angle = msg->position[i];

            if(msg->velocity.size() <= i){
                jointConfig.joint_velocity = 0.0;
            }else{
                jointConfig.joint_velocity = msg->velocity[i];
            }

            if(msg->velocity.size() <= i){
                jointConfig.joint_torque = 0.0;
            }else{
                jointConfig.joint_torque = msg->effort[i];
            }

            jointConfig.joint_acceleration = 0.0;
            jointConfig.joint_temperature = 0.0;
            
            measured_joint_config.joint_config_measured.push_back(jointConfig);
        }
        this->joint_states = measured_joint_config;
        ROS_INFO("Published joint_states onto robot_config_measured.");
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "republishing");
    ros::NodeHandle n;
    ros::Rate r(100);
    
    int num_robots;
    if(ros::param::has("/num_robots")){
        ros::param::get("/num_robots", num_robots);
    } 
    ROS_INFO("There are %d robots.", num_robots);
    
    std::vector<Republisher> Rep;
    for(int j = 0; j < num_robots; j++){
        ros::Publisher pub;
        pub = n.advertise<modrob_workstation::RobotConfigMeasured>("/ns/modrob" + std::to_string(j) + "/robot_config_measured", 1000);
        publishers.push_back(pub);
        Republisher r = Republisher();
        Rep.push_back(r);
        ros::Subscriber sub;
        subscribers.push_back(sub);
        
    }

    int count = 0;
    for(auto & it: subscribers){
        it = n.subscribe("/modrob" + std::to_string(count) + "/joint_states", 1000, &Republisher::translate_jointstate_to_robotconfigmeasured, &Rep[count]);
        count++;
    }

    // Subscribe to each modrobX_joint_state and re-publish onto new topic(/ns/modrobX/robot_config_measured) 
    while(ros::ok()){
        //ROS_INFO("has rep %d",int(Rep.size()));
        for(int k = 0; k < num_robots; k++){
            publishers[k].publish(Rep[k].get_joint_states());
        }
        ros::spinOnce();
    }
    return 0;
}

