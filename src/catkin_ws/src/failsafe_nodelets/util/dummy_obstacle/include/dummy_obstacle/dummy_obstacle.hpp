#include <ros/ros.h>
#include <custom_robot_msgs/PolycapsuleArray.h>
#include <custom_robot_msgs/Positions.h>
#include <custom_robot_msgs/DoubleHeadered.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>


#pragma once

#ifndef DUMMY_OBSTACLE_H
#define DUMMY_OBSTACLE_H

/// A dummy sphere as an obstacle to test stuff.
class DummyObstacle{
private:
  ros::Publisher pub;
  ros::Publisher obs_pose_pub;
  double radius;  // Radius of the sphere
  geometry_msgs::Point pos;
  bool published_once = false;

public:
  /**
   * An empty HumanCapsule constructor 
   */
  DummyObstacle();

  /**
   * A basic HumanCapsule constructor 
   */
  DummyObstacle(ros::Publisher pub, ros::Publisher obs_pose_pub, double radius, double x, double y, double z);

  /**
   * A HumanCapsule destructor
   */
  ~DummyObstacle();

  /**
   * Reads the new potential buffer in the associated buffer and computes and sends the occupancy capsules of the robot
   */
  /**
   * Reads the ending time of the reachability analysis in the associated buffer and computes and sends the occupancy polycapsules of the humans
   *
   * @param data the message's data (contains t_end)
   */
    void reachabilityAnalysis(const custom_robot_msgs::DoubleHeaderedConstPtr& data);
};

#endif
