// -*- lsst-c++ -*/
/**
 * @file nodelet.h
 * @brief Defines the rviz visualization nodelet class
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "rviz_marker/rviz_marker.h"

#ifndef RVIZ_MARKER_NODELET_H
#define RVIZ_MARKER_NODELET_H

namespace rviz_marker {

/**
 * @brief The nodelet for faster communication between ROS components and the rviz visualization class.
 */
class RvizMarkerNodelet : public nodelet::Nodelet{
 private:
  /**
   * @brief class for visualization handling
   */
  RvizMarker rviz_marker_;

  /**
   * @brief ros subscriber to robot reachable set
   */
  ros::Subscriber robot_capsule_sub_;

  /**
   * @brief ros subscriber to advanced robot reachable set
   */
  ros::Subscriber advanced_robot_capsule_sub_;

  /**
   * @brief ros subscriber to robot reachable set
   */
  ros::Subscriber human_cylinder_sub_;

  /**
   * @brief ros subscriber to robot reachable set (position)
   */
  ros::Subscriber human_reach_sub_P_;

  /**
   * @brief ros subscriber to robot reachable set (velocity)
   */
  ros::Subscriber human_reach_sub_V_;

  /**
   * @brief ros subscriber to robot reachable set (acceleration)
   */
  ros::Subscriber human_reach_sub_A_;

  /**
   * @brief init function of ros nodelet
   */
  virtual void onInit();
};
} // namespace rviz_marker

#endif // RVIZ_MARKER_NODELET_H