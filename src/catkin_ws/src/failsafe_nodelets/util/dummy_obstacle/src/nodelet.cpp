#include <ros/ros.h>
#include "custom_robot_msgs/Polycapsule.h"
#include <nodelet.hpp>

#include <pluginlib/class_list_macros.h>

namespace dummy_obstacle
{
  /* initialization */
  void DummyObstacleNodelet::onInit(){
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    ros::Publisher pub_sets = private_nh.advertise<custom_robot_msgs::PolycapsuleArray>("/human_polycapsules",1000);
    
    // get sensors layout
    double radius, x, y, z;
    radius = 0.1;
    x = 0.5;
    if(ros::param::has("/x_dummy")){
      ros::param::get("/x_dummy", x);
    }
    y = 0.5;
    if(ros::param::has("/y_dummy")){
      ros::param::get("/y_dummy", y);
    }
    z = 0.5;
    if(ros::param::has("/z_dummy")){
      ros::param::get("/z_dummy", z);
    }
    ROS_INFO("Publishing sphere pose.");
    ros::Publisher obs_pose_pub = private_nh.advertise<geometry_msgs::Pose>("/sphere_obs/pose_cmd", 1000);
    
    obstacle = DummyObstacle(pub_sets, obs_pose_pub, radius, x, y, z);
    sub_tbrake = private_nh.subscribe("/tbrake", 1000, &DummyObstacle::reachabilityAnalysis, &obstacle);
  }
}

PLUGINLIB_EXPORT_CLASS(dummy_obstacle::DummyObstacleNodelet, nodelet::Nodelet);
    
