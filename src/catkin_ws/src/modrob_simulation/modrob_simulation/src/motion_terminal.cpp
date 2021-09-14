#include <string>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>


// Desired joint values
sensor_msgs::JointState joint_states;

// Publisher Array
std::vector<ros::Publisher> publishers;

// Number of robots
int robot_num = 0;

// Current robot-ID
int ID = -1;

// Transmission start
bool transmission = false;

//deciphers the user-input
void input_decider(std::string input){
    // The help command was chosen
    if(input == "-h"){
        std::cout << "\n-h: command list\n\n#_: substitute _ with the robots ID to change to that robot\n\n[,...,]: input joint-values; provide values for all joints of the robot\n\n";
    }

    // The controlled robot is being changed
    if(input.front() == '#' && input.size() > 1){
        int temp_ID = std::stoi(input.substr(1, input.size()-1));
        if(temp_ID >= 0 && temp_ID <= robot_num){
            ID = temp_ID;
            std::cout << "Robot " << ID << " selected!\n";
        }else{
            std::cout << "The input was out of bounds. Please select a robot number ranging from " << std::to_string(0) << " to " << std::to_string(robot_num) << "!\nYou have currently selected is robot #" << std::to_string(ID) << "\n";
        }
        joint_states.position = {};
    }

    // Transform the string array into sensor_msgs::JointState
    if(input.front() == '[' && input.at(input.size()-1) == ']'){
        input = input.substr(1, input.size()-2);
        std::vector<std::string> angles_str;
        std::vector<double> angles;
        std::stringstream ss(input);
        while(ss.good()){
            std::string substr;
            getline( ss, substr, ',');
            angles_str.push_back( substr );
        }
        for(auto const &it: angles_str){
            angles.push_back(std::stod(it));
        }
        joint_states.position = {};
        for(auto const &it : angles){
            joint_states.position.push_back(it);
        }
        std::cout << "Processing angles\t";
        transmission = true;
    }else{

        // Unrecognized input
        if(input != "-h" && input.front() != '#'){
            std::cout << "\nThis is not a valid format for joint angles!\n";
        }
    }    
}

int main(int argc, char **argv){

    // inits
    ros::init(argc, argv, "motion_terminal", ros::init_options::AnonymousName);
    ros::NodeHandle n;
    ros::Rate r(100);

    // determine the number of robots
    if(ros::param::has("/num_robots")){
        ros::param::get("/num_robots", robot_num);
    }else{
        robot_num = 10;
    }

    // Create as many publishers as there are joints
    for(int i = 0; i < robot_num; i++){
        ros::Publisher pub;
        publishers.push_back(pub);
    }

    // Advertise publishers of the rotational joints
    int robot_count = 0;
    for(auto & it: publishers){
        it = n.advertise<sensor_msgs::JointState>("/modrob" + std::to_string(robot_count) + "/motion", 10);
       robot_count++;
    }

    // First choose a robot that should be controled
    std::string input;
    std::cout << "This node facilitates real-time manipulation of joint values.\n\nFor a list of available comands use: -h\n\nPlease input the ID \nof the robot you want to control by typing #ID.\n";
    while(ID < 0 || ID > robot_num){
        std::string rob_ID;
        std::cin.clear();
        std::getline(std::cin, rob_ID);
        input_decider(rob_ID);
        if(ID < 0 || ID > robot_num){
            std::cout << "\nThis input is not valid! \nPlease type: #ID where ID is a positive integer indicating the ID of a robot.\n";
        }
    }

    // A Loop that reads inputs and sends them to the evaluatioin function
    while(ros::ok()){
        std::string input;
        std::cout << "Please input a command!\n\n";
        std::cin.clear();
        std::getline(std::cin, input);

        // Evaluate the input
        input_decider(input);
        
        if(transmission){
            publishers[ID].publish(joint_states);
        }

        ros::spinOnce();
    }
    return 0;
}