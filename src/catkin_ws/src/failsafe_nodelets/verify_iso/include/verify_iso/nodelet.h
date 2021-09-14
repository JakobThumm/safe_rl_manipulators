// -*- lsst-c++ -*/
/**
 * @file nodelet.h
 * @brief Defines the verify ISO nodelet class
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "custom_robot_msgs/BoolHeadered.h"
#include "custom_robot_msgs/Visualization.h"
#include "verify_iso/verify_iso.h"
#include "verify_iso/advanced_verify_iso.h"

#ifndef VERIFY_ISO_NODELET_H
#define VERIFY_ISO_NODELET_H

namespace verify_iso {
/**
 * @brief The nodelet for faster communication between ROS components and the verification 
 */
class VerifyISONodelet : public nodelet::Nodelet {
 public:
  /**
   * @brief ROS subscriber to robot capsules
   */
  message_filters::Subscriber<custom_robot_msgs::CapsuleArray> robot_sub_;

  /**
   * @brief ROS subscriber to advanced robot capsules (with start and end configuration)
   */
  message_filters::Subscriber<custom_robot_msgs::StartGoalCapsuleArray> advanced_robot_sub_;
  
  /**
   * @brief ROS subscriber to human polycapsules (for human cylinder approach)
   */
  message_filters::Subscriber<custom_robot_msgs::PolycapsuleArray> human_sub_;

  /**
   * @brief ROS subscriber to human reachable set with position approach
   */
  message_filters::Subscriber<custom_robot_msgs::CapsuleArray> human_reach_sub_P_;
  
  /**
   * @brief ROS subscriber to human reachable set with velocity approach
   */
  message_filters::Subscriber<custom_robot_msgs::CapsuleArray> human_reach_sub_V_;

  /**
   * @brief ROS subscriber to human reachable set with acceleration approach
   */
  message_filters::Subscriber<custom_robot_msgs::CapsuleArray> human_reach_sub_A_;

  /**
   * @brief Type definition for robot and human reachable set synchronization
   * 
   * This is for classic verification with human_reach.
   */
  typedef message_filters::sync_policies::ApproximateTime<custom_robot_msgs::CapsuleArray, custom_robot_msgs::CapsuleArray, custom_robot_msgs::CapsuleArray, custom_robot_msgs::CapsuleArray> reach_policy;
  
  /**
   * @brief Messege synchronizer for robot and human reachable sets 
   * 
   * This is for classic verification with human_reach.
   */
  message_filters::Synchronizer<reach_policy>* reach_sync_;

  /**
   * @brief Type definition for robot and human reachable set synchronization
   * 
   * This is for advanced verification with human_reach.
   */
  typedef message_filters::sync_policies::ApproximateTime<custom_robot_msgs::StartGoalCapsuleArray, custom_robot_msgs::CapsuleArray, custom_robot_msgs::CapsuleArray, custom_robot_msgs::CapsuleArray> advanced_reach_policy;
  
  /**
   * @brief Messege synchronizer for robot and human reachable sets 
   * 
   * This is for classic verification with human_reach.
   */
  message_filters::Synchronizer<advanced_reach_policy>* advanced_reach_sync_;

  /**
   * @brief Type definition for robot and human reachable set synchronization
   * 
   * This is for classic verification with human_cylinder.
   */
  typedef message_filters::sync_policies::ApproximateTime<custom_robot_msgs::CapsuleArray, custom_robot_msgs::PolycapsuleArray> policy;
  
  /**
   * @brief Messege synchronizer for robot and human reachable sets 
   * 
   * This is for classic verification with human_cylinder.
   */
  message_filters::Synchronizer<policy>* sync_;

  /**
   * @brief The verify ISO object
   */
  VerifyISO verify_iso_;

  /**
   * @brief The advanced verify ISO object
   */
  AdvancedVerifyISO advanced_verify_iso_;

 private:
  /**
   * @brief Initialize the nodelet
   */
  virtual void onInit();
};
} // namespace verify_iso

#endif // VERIFY_ISO_NODELET_H