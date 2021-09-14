// -*- lsst-c++ -*/
/**
 * @file nodelet.h
 * @brief Defines the nodelet class for the control command translator
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */

#ifndef CONTROL_COMMAND_TRANSLATOR_NODELET_H
#define CONTROL_COMMAND_TRANSLATOR_NODELET_H

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>

#include "control_command_translator/control_command_translator.h"

namespace control_command_translator {

/**
 * @brief The nodelet for faster communication between ROS components and the control command translator.
 */
class ControlCommandTranslatorNodelet : public nodelet::Nodelet {
 private:
  /**
   * @brief Translates a robot config commanded signal to a ROS group position controller signal.
   */
  ControlCommandTranslator* translator;

  /**
   * @brief Subscribes to the `/ns/robot_config_commanded` topic.
   */
  ros::Subscriber robot_config_commanded_sub;

  /**
   * @brief Init function that will be called when starting the nodelet.
   */
  virtual void onInit();
};
} // namespace control_command_translator
#endif // CONTROL_COMMAND_TRANSLATOR_NODELET_H