// -*- lsst-c++ -*/
/**
 * @file nodelet.h
 * @brief Defines the nodelet class for online verification
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <gazebo_msgs/GetPhysicsProperties.h>

#include "custom_robot_msgs/DoubleHeadered.h"
#include "custom_robot_msgs/Buffer.h"
#include "failsafe_consistent_planner/online_verification.h"
#include "failsafe_consistent_planner/motion.h"

#ifndef FAILSAFE_CONSISTENT_NODELET_H
#define FAILSAFE_CONSISTENT_NODELET_H

namespace online_verification
{

/**
 * @brief The nodelet for faster communication between ROS components and the online verification.
 */
class FailsafeConsistentNodelet : public nodelet::Nodelet
{
 public:
  /**
   * @brief Subscriber to initialization command
   */
  ros::Subscriber sub_init;

  /**
   * @brief Subscriber to safety topic
   */
  ros::Subscriber sub_path_safety;

  /**
   * @brief Subscriber to next cycle begins topic
   */
  ros::Subscriber sub_next_cycle;

  /**
   * @brief Subscriber to new desired goal
   */
  ros::Subscriber sub_new_goal;

  /**
   * @brief The Failsafe consistent planner
   */
  FailsafePathConsistent* fspc;

 private:
  /**
   * @brief Initializes the nodelet
   */
  virtual void onInit();
};
} // namespace online_verification

#endif // FAILSAFE_CONSISTENT_NODELET_H