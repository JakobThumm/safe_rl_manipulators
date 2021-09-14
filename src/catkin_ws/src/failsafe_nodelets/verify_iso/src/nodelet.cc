#include "verify_iso/nodelet.h"

namespace verify_iso {
void VerifyISONodelet::onInit() {
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  // Initialize ROS publishers
  ros::Publisher pub_safe = private_nh.advertise<custom_robot_msgs::BoolHeadered>("/path_safety", 1000);
  // Read relevant ROS parameters
  bool advanced_verify = false;
  if (ros::param::has("advanced_verify_iso")) {
    ros::param::get("advanced_verify_iso", advanced_verify);
  }
  // Initialize ROS subscribers
  human_sub_.subscribe(private_nh, "/human_polycapsules", 1);
  human_reach_sub_P_.subscribe(private_nh, "/human_reach_capsules/P", 1);
  human_reach_sub_V_.subscribe(private_nh, "/human_reach_capsules/V", 1);
  human_reach_sub_A_.subscribe(private_nh, "/human_reach_capsules/A", 1);
  // Create verify ISO object and message synchronizer
  if (advanced_verify) {
    advanced_robot_sub_.subscribe(private_nh, "/advanced_robot_capsules", 1);
    advanced_verify_iso_ = AdvancedVerifyISO(pub_safe);
    advanced_reach_sync_ = new message_filters::Synchronizer<advanced_reach_policy>(advanced_reach_policy(10), advanced_robot_sub_, human_reach_sub_P_, human_reach_sub_V_, human_reach_sub_A_);
    advanced_reach_sync_->registerCallback(boost::bind(&AdvancedVerifyISO::verify_human_reach, &advanced_verify_iso_,  _1, _2, _3, _4));  
  } else {
    robot_sub_.subscribe(private_nh, "/robot_capsules", 1);
    verify_iso_ = VerifyISO(pub_safe);
    reach_sync_ = new message_filters::Synchronizer<reach_policy>(reach_policy(10), robot_sub_, human_reach_sub_P_, human_reach_sub_V_, human_reach_sub_A_);
    reach_sync_->registerCallback(boost::bind(&VerifyISO::verify_human_reach, &verify_iso_,  _1, _2, _3, _4)); 
    sync_ = new message_filters::Synchronizer<policy>(policy(10), robot_sub_, human_sub_);
    sync_->registerCallback(boost::bind(&VerifyISO::verify, &verify_iso_, _1, _2));
  }
}
} // namespace verify_iso

PLUGINLIB_EXPORT_CLASS(verify_iso::VerifyISONodelet, nodelet::Nodelet);