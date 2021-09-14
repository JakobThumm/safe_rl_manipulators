// -*- lsst-c++ -*-
/**
 * @file nodelet.h
 * @brief Define the class for robot reachability nodelet
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */

#include <vector>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "custom_robot_msgs/StartGoalCapsuleArray.h"
#include "custom_robot_msgs/CapsuleArray.h"
#include "robot_reach/robot_reach.h"

#ifndef ROBOT_REACH_NODELET_H
#define ROBOT_REACH_NODELET_H

namespace robot_reach {
/**
 * @brief The nodelet for faster communication between ROS components and the robot reachability
 */
class RobotReachNodelet : public nodelet::Nodelet {
 public:
  /**
   * @brief ROS subscriber to start and goal position
   */
  ros::Subscriber sub_start_goal_;

  /**
   * @brief The robot reachability object
   */
  RobotReach robot_reach_;

 private:
  /**
   * @brief Nodelet initialization.
   */
  virtual void onInit();
};
} // namespace robot_reach

#endif // ROBOT_REACH_NODELET_H