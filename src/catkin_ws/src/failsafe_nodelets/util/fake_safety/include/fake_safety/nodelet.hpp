#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Bool.h>

#include <fake_safety.hpp>

namespace fake_safety
{
  class FakeSafetyNodelet : public nodelet::Nodelet
  {
    private:
      FakeSafety* safety;
      ros::Subscriber safety_change_sub;
      // Provides header stamp.
      ros::Subscriber t_brake_sub;
      
      virtual void onInit();
  };
}
