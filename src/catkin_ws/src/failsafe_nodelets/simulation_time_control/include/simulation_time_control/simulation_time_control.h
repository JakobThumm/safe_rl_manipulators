// -*- lsst-c++ -*/
/**
 * @file simulation_time_control.h
 * @brief defined all the structure used in the simulation time control
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */


#include <exception>
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo_msgs/GetPhysicsProperties.h>
#include <gazebo_msgs/SetPhysicsProperties.h>
#include <std_msgs/Empty.h>

#include "custom_robot_msgs/BoolHeadered.h"

#ifndef SIMULATION_TIME_CONTROL_H
#define SIMULATION_TIME_CONTROL_H

namespace simulation_time_control {
/**
 * @brief Controls the speed of the gazebo simulation
 */
class SimulationTimeControl{
private:
  /**
   * @brief Publishes the start of the next online verification (OV) calculation cycle
   */
  ros::Publisher pub_start_next_cycle_;           
  
  /**
   * @brief Gazebo control node
   */
  gazebo::transport::NodePtr gazebonode_;

  /**
   * @brief Publish the step control information
   */
  gazebo::transport::PublisherPtr world_control_pub_; 

  /**
   * @brief The time when the loop begins
   */
  ros::Time cycle_begin_time_;

  /**
   * @brief The sample time of the online verification calcualation
   */
  double sample_time_;

  /**
   * @brief The current maximum update rate of the simulation
   */
  double max_update_rate_;

  /**
   * @brief The initial maximum update rate of the simulation
   */
  double initial_max_update_rate_;

  /**
   * @brief A FIFO queue holding the past n times where the OV calculation was slower than the simulation
   */
  std::queue<ros::Time> delay_queue_;

  /**
   * @brief Size of the delay_queue
   */
  int n_delays_queue_;

  /**
   * @brief Speedup/Slow down factor
   */
  double speed_factor_;

  /**
   * @brief Max simulation speed
   */
  double max_max_update_rate_;

  /**
   * @brief Number of simulation steps per OV calculation
   */
  int n_sim_steps_per_sample_time_;

  /**
   * @brief If the delay_queue(0)-delay_queue(5) < max_delay_time, Then reduce the simulation speed.
   */
  ros::Duration max_delay_time_;

  /**
   * @brief Do the first simulation speed reduction earliest after x seconds
   */
  ros::Time first_reduction_after_;

  /**
   * @brief The last time the simulation speed was reduced
   */
  ros::Time last_reduction_;

  /**
   * @brief The minimum time between two reductions (Simulation needs some time to reduce the speed)
   */
  ros::Duration time_between_reductions_;
 
  /**
   * @brief The minimum time between two speedups
   */
  ros::Duration time_between_speedups_;

  /**
   * @brief The set gazebo speed service client
   */
  ros::ServiceClient set_physics_properties_client_;

  /**
   * @brief The set gazebo speed service message
   */
  gazebo_msgs::SetPhysicsProperties set_physics_srv_;
  

public:
  /**
   * @brief Empty constructor
   */
  SimulationTimeControl(){}

  /**
   * @brief Contructor for simulation speed control
   *
   * @param[in] pub_start_next_cycle Publishes the start of the next online verification (OV) calculation cycle
   * @param[in] set_physics_properties_client The set gazebo speed service client
   * @param[in] set_physics_srv The set gazebo speed service message
   * @param[in] sample_time The sample time of the online verification calcualation
   * @param[in] initial_max_update_rate The initial maximum update rate of the simulation
   * @param[in] n_delays_queue Size of the delay_queue
   * @param[in] speed_factor Speedup/Slow down factor
   * @param[in] max_max_update_rate Max simulation speed
   * @param[in] max_delay_time If the delay_queue(0)-delay_queue(5) < max_delay_time, Then reduce the simulation speed.
   * @param[in] first_reduction_after Do the first simulation speed reduction earliest after x seconds
   * @param[in] time_between_reductions The minimum time between two reductions (Simulation needs some time to reduce the speed)
   * @param[in] time_between_speedups The minimum time between two speedups
   */
  SimulationTimeControl(const ros::Publisher &pub_start_next_cycle, 
                        const ros::ServiceClient& set_physics_properties_client,
                        gazebo_msgs::SetPhysicsProperties& set_physics_srv,
                        double sample_time=0.004, 
                        double initial_max_update_rate=10000,
                        int n_delays_queue=5,
                        double speed_factor=0.1,
                        double max_max_update_rate=15000,
                        double max_delay_time=0.5,
                        double first_reduction_after=10,
                        double time_between_reductions=1,
                        double time_between_speedups=10);

  /**
   * @brief Constructor for simulation step control
   * @param[in] pub_start_next_cycle Publisher of next simulation cycle begins
   * @param[in] sample_time Sample time simulation
   * @param[in] n_sim_steps_per_sample_time Fixed number of steps to do in each simulation cycle 
   */
  SimulationTimeControl(const ros::Publisher &pub_start_next_cycle, double sample_time, 
      int n_sim_steps_per_sample_time);

  
  /**
   * @brief Destructor
   */
  ~SimulationTimeControl() {}

  /**
   * @brief Control the simulation speed
   *
   * This is faster but less accurate than the step sim control
   * Slow down the simulation speed if the OV calculation is often slower than the simulation.
   * Speed up the simulation speed from time to time to see if the last slow down was unnecessary.
   * @param[in] calculation_finished The message coming from online verification that the calculation cycle is finished
   */
  void simControl(const std_msgs::EmptyConstPtr& calculation_finished);

  /**
   * @brief Step a fixed amount of steps in the simulation
   *
   * Control the simulation speed with a simulation step approach
   * This is slower but more accurate
   * @param[in] calculation_finished The message coming from online verification that the calculation cycle is finished
   */
  void stepSimControl(const std_msgs::EmptyConstPtr& calculation_finished);
};
} // namespace simulation_time_control
#endif // SIMULATION_TIME_CONTROL_H
