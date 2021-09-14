// -*- lsst-c++ -*/
/**
 * @file nodelet.h
 * @brief Defines the human reach nodelet class
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */

#include <vector>
#include <string>
#include <map>

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <XmlRpc.h>

#include "custom_robot_msgs/CapsuleArray.h"
#include "human_reach/human_reach.h"

#ifndef HUMAN_REACH_NODELET_H
#define HUMAN_REACH_NODELET_H

namespace human_reach
{
/**
 * @brief The nodelet for faster communication between ROS components and the human reach class.
 */
class HumanReachNodelet : public nodelet::Nodelet
{
 public:
  /**
   * @brief Subscriber to the breaking time topic
   */
  ros::Subscriber sub_t_brake_;

  /**
   * @brief Subscriber to the human joint position measurement topic
   * 
   * The measurements stem from a motion capture system (or simulation plugin).
   */
  ros::Subscriber sub_human_joint_pos_;

  /**
   * @brief The human reachability calculation object
   */
  HumanReach human_reach_;

 private:
  /**
   * @brief Initializes the nodelet.
   * 
   * Sets up all ROS subscribers and publishers
   */
  virtual void onInit();
   
  /**
   * @brief Create the two maps describing a body part.
   * @param[in] bodies The root XmlRpc element read from ros param server
   * @param[in] joint_ids Maps the joint id (position in measurement) to the joint name
   * @param[out] body_link_joints Maps a proximal and distal joint to a body
   * @param[out] thickness Maps a radius to a body
   */
  void createBodies(XmlRpc::XmlRpcValue& bodies, 
      const std::map<std::string, int>& joint_ids,
      std::map<std::string, human_reach::jointPair>& body_link_joints, 
      std::map<std::string, double>& thickness);

  /**
   * @brief Create the lists for the extremities.
   * @param[in] bodies The root XmlRpc element read from ros param server
   * @param[in] joint_ids Maps the joint id (position in measurement) to the joint name
   * @param[out] shoulder_ids List of shoulder joint ids
   * @param[out] elbow_ids List of elbow joint ids
   * @param[out] wrist_names List of wrist joint names
   */
  void createExtremities(XmlRpc::XmlRpcValue& extemities, 
      const std::map<std::string, int>& joint_ids,
      std::vector<double>& shoulder_ids, 
      std::vector<double>& elbow_ids, 
      std::vector<std::string>& wrist_names);
};

} // namespace human_reach

#endif // HUMAN_REACH_NODELET_H