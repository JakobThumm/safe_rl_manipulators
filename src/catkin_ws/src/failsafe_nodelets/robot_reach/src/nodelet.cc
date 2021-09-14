#include "robot_reach/nodelet.h"


namespace robot_reach {
  /* ros init */
  void RobotReachNodelet::onInit() {
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    /// get the robot's parameters
    double nb_joints = 0, init_roll=0, init_pitch=0, init_yaw=0, init_x=0, init_y=0, init_z=0;
    std::vector<double> transformation_matrices, enclosures;
    std::string robot_name = "modrob0";
    if (ros::param::has("/robot_name")) {
        ros::param::get("/robot_name", robot_name);
    }
    if (ros::param::has("/" + robot_name + "/transformation_matrices")) {
        ros::param::get("/" + robot_name + "/transformation_matrices", transformation_matrices);
    }
    if (ros::param::has("/" + robot_name + "/enclosures")) {
        ros::param::get("/" + robot_name + "/enclosures", enclosures);
    }
    if (ros::param::has("/" + robot_name + "/nb_joints")) {
        ros::param::get("/" + robot_name + "/nb_joints", nb_joints);
    }
    if (ros::param::has("/" + robot_name + "/init_roll")) {
      ros::param::get("/" + robot_name + "/init_roll", init_roll);
    }
    if (ros::param::has("/" + robot_name + "/init_pitch")) {
      ros::param::get("/" + robot_name + "/init_pitch", init_pitch);
    }
    if (ros::param::has("/" + robot_name + "/init_yaw")) {
      ros::param::get("/" + robot_name + "/init_yaw", init_yaw);
    }
    if (ros::param::has("/" + robot_name + "/init_x")) {
      ros::param::get("/" + robot_name + "/init_x", init_x);
    }
    if (ros::param::has("/" + robot_name + "/init_y")) {
      ros::param::get("/" + robot_name + "/init_y", init_y);
    }
    
    if (ros::param::has("/" + robot_name + "/init_z")) {
      ros::param::get("/" + robot_name + "/init_z", init_z);
    }

    bool advanced_verify = false;
    if (ros::param::has("advanced_verify_iso")){
      ros::param::get("advanced_verify_iso", advanced_verify);
    }
    ros::Publisher pub_capsules;
    if (advanced_verify){
      pub_capsules = private_nh.advertise<custom_robot_msgs::StartGoalCapsuleArray>("/advanced_robot_capsules", 1000);
    } else {
      pub_capsules = private_nh.advertise<custom_robot_msgs::CapsuleArray>("/robot_capsules", 1000);
    }
    
    ROS_INFO("Creating robot object...");
    /// initialize the robot
    robot_reach_ = RobotReach(transformation_matrices, nb_joints, enclosures, pub_capsules, advanced_verify,
                  init_x, init_y, init_z, init_roll, init_pitch, init_yaw);

    /// subscribe to the path_edges topic
    sub_start_goal_ = private_nh.subscribe("/start_goal_motion", 1000, &RobotReach::reach, &robot_reach_);
  }
} // namespace robot_reach

PLUGINLIB_EXPORT_CLASS(robot_reach::RobotReachNodelet, nodelet::Nodelet);
    
