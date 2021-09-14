#include <nodelet.hpp>

namespace fake_safety
{
  void FakeSafetyNodelet::onInit()
  {
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    // Publisher for modrob arm controller
    ros::Publisher safety_pub = private_nh.advertise<custom_robot_msgs::BoolHeadered>("/path_safety", 1000);
    safety = new FakeSafety(safety_pub);

    // subscribes to the topic
    safety_change_sub = private_nh.subscribe("/change_safety", 1000, &FakeSafety::change_safety, safety);
    t_brake_sub = private_nh.subscribe("/tbrake", 1000, &FakeSafety::publish_safety, safety);
  }
}

PLUGINLIB_EXPORT_CLASS(fake_safety::FakeSafetyNodelet, nodelet::Nodelet);
    
