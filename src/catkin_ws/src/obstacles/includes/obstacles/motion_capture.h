// -*- lsst-c++ -*/
/**
 * @file motion_capture.h
 * @brief Defines the motion capture class
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <functional>
#include <filesystem>
#include <stdio.h>

#include <libxml/parser.h>
#include <libxml/tree.h>
#include <ignition/math.hh>
#include <ignition/math/Matrix4.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/util/system.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <boost/shared_ptr.hpp>
#include <boost/pointer_cast.hpp>

#include "custom_robot_msgs/PositionsHeadered.h"
#include "obstacles/skeleton_node.h"


#ifndef GAZEBO_PLUGINS_MOTION_CAPTURE_PLUGIN_H
#define GAZEBO_PLUGINS_MOTION_CAPTURE_PLUGIN_H

namespace obstacles {
/**
 * @brief Plugin that publishes the human joint positions to a ROS topic.
 * 
 * You can also control the human actor with this plugin.
 */
class GZ_PLUGIN_VISIBLE MotionCapturePlugin : public gazebo::ModelPlugin {

 private:
  /**
   * @brief A ROS callbackqueue that helps process messages
   */
  ros::CallbackQueue ros_queue_;

  /**
   * @brief A thread the keeps running the rosQueue
   */
  std::thread ros_queue_thread_;

  /**
   * @brief Pointer to the parent actor.
   */
  gazebo::physics::ActorPtr actor_;

  /**
   * @brief List of connections
   */
  std::vector<gazebo::event::ConnectionPtr> connections_;

  /**
   * @brief Root node of the custom skeleton of the actor. 
   */
  SkeletonNode* root_node_;

  /**
   * @brief A map describing which node position should be published at which position.
   */
  std::map<std::string, int> node_map_;

  /**
   * @brief Map joint cartesian position to joint name
   */
  std::map<std::string, ignition::math::Vector3d> joint_positions_;

  /**
   * @brief Counter for reducing debug output
   */
  int debug_count_ = 0;

  /**
   * @brief A node use for ROS transport
   */
  std::unique_ptr<ros::NodeHandle> ros_node_;

  /**
   * @brief ROS publisher that publishes the joint positions
   */
  ros::Publisher publisher_;

  /**
   * @brief Subscriber to the `/{model_name}/pose_cmd` topic
   */
  ros::Subscriber ros_subscriber_pose_;

  /**
   * @brief Subscriber to the `/{model_name}/start_stop_cmd` topic
   */
  ros::Subscriber ros_subscriber_start_stop_;

  /**
   * @brief Subscriber to the `/{model_name}/script_time_cmd` topic
   */
  ros::Subscriber ros_subscriber_script_time_;

  /**
   * @brief Time between update steps
   */
  double dt_update_;

  /**
   * @brief Last simulation timestep where update was executed
   */
  double last_update_time_;

  /**
   * @brief The position of the animation is shifted by these values.
   * 
   * Only position values are used.
   */
  ignition::math::Pose3d shift_pos_;

  /**
   * @brief Determines if the animation should be continued or not.
   */
  bool play_ = true;

  /**
   * @brief Animation time in seconds
   */
  double animation_time_ = 0.0;

 public:
  /**
   * @brief Constructor
   */
  MotionCapturePlugin();

  /**
   * @brief Load the actor plugin.
   * @param[in] _model Pointer to the parent model.
   * @param[in] _sdf Pointer to the plugin's SDF elements.
   */
  virtual void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /**
   * @brief Set the actor to the desired pose.
   * 
   * @param _msg Pose command
   */
  void OnRosPoseMsg(const geometry_msgs::PoseConstPtr &_msg);

  /**
   * @brief Set the script time of the actor. 
   * 
   * @param _msg Simulation time
   */
  void OnRosScriptTimeMsg(const std_msgs::Float64ConstPtr & _msg);

  /**
   * @brief Start or pauses the animation
   * 
   * @param _msg Bool: true = start, false = stop
   */
  void OnRosStartStopAnimation(const std_msgs::BoolConstPtr & _msg);

 private:
  /**
   * @brief ROS helper function that processes messages
   */
  void QueueThread();

  /**
   * @brief Function that is called every gazebo update cycle.
   * @param[in] _info Timing information
   */
  void OnUpdate(const gazebo::common::UpdateInfo &_info);

  /**
   * @brief Reset the trajectory info of the actor
   */
  void ResetTrajectoryInfo();

  /**
   * @brief Read in the library_visual_scenes node structure from a given dae file
   * @param[in] root Root xml node pointer
   */
  void ReadDaeSkeleton(xmlNode * root);

  /**
   * @brief Recursively fill skeleton node object
   * @param[in] xml_parent Parent xml node
   * @param[in] skeleton_parent Parent skeleton node
   */
  void FillSkeleton(xmlNode* xml_parent, SkeletonNode* skeleton_parent); 

  /**
   * @brief Get a child node of a xml node by its name.
   * @param[in] root The parent node
   * @param[in] child_name The identifier of the child.
   * 
   * @return The child node
   */
  xmlNode* GetChildByName(xmlNode * root, const char * child_name);

  /**
   * @brief Get a all children nodes of a xml node with the name.
   * @param[in] root The parent node
   * @param[in] child_name The identifier of the child.
   * 
   * @return The child node
   */
  std::vector<xmlNode*> GetAllChildrenByName(xmlNode * root, const char * child_name);

  /**
   * @brief Get the value of the attribute id of a xml node.
   * @param[in] root The parent node
   */
  inline std::string GetNodeId(xmlNode * root) {
    const unsigned char * id_str = reinterpret_cast<const unsigned char*>("id");
    return reinterpret_cast<const char*>(xmlGetProp(root, id_str));
  }

  /**
   * @brief Get the translation vector of a transformation matrix.
   * @param[in] transformation Transformation matrix
   */
  inline ignition::math::Vector3d GetPosOfTransformation(const ignition::math::Matrix4d& transformation) {
    return ignition::math::Vector3d(transformation(0, 3), transformation(1, 3), transformation(2, 3));
  }

  /**
   * @brief Calculate the cartesian position of the joints recursively. (Forward kinematics)
   * @param[in] joint_transformations Map containing the name of the joint and its transformation matrix.
   * @param[in] parent_transform Transformation matrix of parent joint (Identity for start)
   * @param[in] parent This skeleton node
   * @param[in] joint_positions Map containing the calculated output positions.
   */
  void CalculateJointPos(const std::map<std::string, ignition::math::Matrix4d>& joint_transformations,
                                  const ignition::math::Matrix4d& parent_transform,
                                  SkeletonNode* node,
                                  std::map<std::string, ignition::math::Vector3d>& joint_positions);
  };
} // namespace gazebo
#endif // GAZEBO_PLUGINS_MOTION_CAPTURE_PLUGIN_H
