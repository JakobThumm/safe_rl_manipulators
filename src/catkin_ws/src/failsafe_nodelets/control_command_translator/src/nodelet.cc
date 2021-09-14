#include "control_command_translator/nodelet.h"

namespace control_command_translator {
void ControlCommandTranslatorNodelet::onInit() {
  ros::NodeHandle& private_nh = getPrivateNodeHandle();

  // Publisher for modrob arm controller
  std::string robot_name = "modrob0";
  if (ros::param::has("/robot_name")) {
      ros::param::get("/robot_name", robot_name);
  }
  std::string publish_topic = "/" + robot_name + "/arm_position_controller/command";
  ros::Publisher pos_command_pub = private_nh.advertise<std_msgs::Float64MultiArray>(publish_topic, 1000);
  int n_joints_robot = 3;
  if (ros::param::has("/nb_joints")) {
      ros::param::get("/nb_joints", n_joints_robot);
  }
  translator = new ControlCommandTranslator(pos_command_pub, n_joints_robot);

  // subscribes to the topic
  robot_config_commanded_sub = private_nh.subscribe("/ns/robot_config_commanded", 1000, 
      &ControlCommandTranslator::convertMsg, translator);
}
} // namespace control_command_translator

PLUGINLIB_EXPORT_CLASS(control_command_translator::ControlCommandTranslatorNodelet, nodelet::Nodelet);