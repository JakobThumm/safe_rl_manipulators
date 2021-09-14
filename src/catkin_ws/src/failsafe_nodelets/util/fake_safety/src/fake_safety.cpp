#include <fake_safety.hpp>

///////////////////////////FakeSafety///////////////////////////////////

FakeSafety::FakeSafety(const ros::Publisher &safety_publisher):
  safety_publisher(safety_publisher)
{
  is_safe=true;
}

FakeSafety::~FakeSafety(){};
////////////////////////////////////////////////////////////////////////////////////

void FakeSafety::change_safety(const std_msgs::BoolConstPtr& new_safe){
  is_safe = new_safe->data;
}

void FakeSafety::publish_safety(const custom_robot_msgs::DoubleHeaderedConstPtr& t_brake){
  /*
  Header header
  bool data
  */  
  custom_robot_msgs::BoolHeaderedPtr safety_msg(new custom_robot_msgs::BoolHeadered());
  safety_msg->data = is_safe;
  safety_msg->header.stamp = t_brake->header.stamp;
  safety_publisher.publish(safety_msg);
}