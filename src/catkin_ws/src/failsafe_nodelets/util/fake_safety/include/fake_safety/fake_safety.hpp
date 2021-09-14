#ifndef Fake_Safety_H
#define Fake_Safety_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <custom_robot_msgs/BoolHeadered.h>
#include <custom_robot_msgs/DoubleHeadered.h>
#include <std_msgs/Bool.h>

class FakeSafety{
private:
  ros::Publisher safety_publisher;
  bool is_safe;

public:
  /**
   * An empty FakeSafety destructor
   */
  ~FakeSafety();
  /**
   * A basic FakeSafety constructor
   */
   FakeSafety(const ros::Publisher &safety_publisher);

  /**
   * Change the continouosly published safety info.
   */
  void change_safety(const std_msgs::BoolConstPtr& new_safe);

  /**
   * Change the continouosly published safety info.
   */
  void publish_safety(const custom_robot_msgs::DoubleHeaderedConstPtr& t_brake);
};
#endif