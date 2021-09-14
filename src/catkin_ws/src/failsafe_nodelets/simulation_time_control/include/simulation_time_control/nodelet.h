// -*- lsst-c++ -*/
/**
 * @file nodelet.h
 * @brief Describes the functionality of the simulation time control nodelet
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <gazebo_msgs/GetPhysicsProperties.h>
#include <gazebo_msgs/SetPhysicsProperties.h>

#include "custom_robot_msgs/BoolHeadered.h"
#include "simulation_time_control/simulation_time_control.h"


#ifndef SIMULATION_TIME_CONROL_NODELET_H
#define SIMULATION_TIME_CONROL_NODELET_H

namespace simulation_time_control {
/**
 * @brief The nodelet for faster communication between ROS components and the simulation time control class.
 */
class SimulationTimeControlNodelet : public nodelet::Nodelet {
 public:
  /**
   * @brief ROS subscriber to the simulation cycle finished signal
   */
  ros::Subscriber sub_cycle_finished_;

  /**
   * @brief The simulation time control object
   */
  SimulationTimeControl* sim_time_control_;

 private:
  /**
   * @brief Initialize the simulation time control nodelet
   */
  virtual void onInit();
};
} // namespace simulation_time_control

#endif // SIMULATION_TIME_CONROL_NODELET_H