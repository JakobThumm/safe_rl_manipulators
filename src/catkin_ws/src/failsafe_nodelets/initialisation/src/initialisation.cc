// -*- lsst-c++ -*/
/**
 * @file initialisation.cc
 * @brief Main function for sending the init signal to online verification.
 * @version 0.1
 * @copyright MIT License. Please see package.xml for further detail.
 */

#include <string>
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include "modrob_workstation/RobotConfigMeasured.h"
#include "modrob_workstation/RobotStateCommanded.h"

/**
 * @brief Main function that waits for `startup_time` seconds and then sends an init signal.
 * 
 * The init signal is only read by the online verification object to start the motion planning and verification.
 */
int main(int argc, char **argv){
  ros::init(argc, argv, "initialization");
  ros::NodeHandle nh;
  ROS_INFO("Initialisation");

  ros::Publisher start_pub = nh.advertise<std_msgs::Empty>("/initialisation", 1000);

  double sleep_duration = 0.5;
  if (ros::param::has("/startup_time")) {
    ros::param::get("/startup_time", sleep_duration);
  }

  ROS_INFO_STREAM("Waiting for " << sleep_duration << "s to initialize the online verification.");
  int sleep_ms = (int)floor(sleep_duration*1000);
  std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
  std_msgs::Empty start_msg;
  start_pub.publish(start_msg);
  ROS_INFO("Sent initialization msg.");
  return 0;
}
