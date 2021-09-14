#include "dummy_obstacle.hpp"
#include <geometry_msgs/Pose.h>
#include <global_library/global_library.h>

DummyObstacle::DummyObstacle(){}


DummyObstacle::DummyObstacle(ros::Publisher pub, ros::Publisher obs_pose_pub, double radius=1, double x=0.5, double y=0.5, double z=0.5):
  pub(pub),
  obs_pose_pub(obs_pose_pub),
  radius(radius)
{
  this->pos.x = x;
  this->pos.y = y;
  this->pos.z = z;
}

DummyObstacle::~DummyObstacle() {}

/**
 * Reads the new potential buffer in the associated buffer and computes and sends the occupancy capsules of the robot
 */
/**
 * Reads the ending time of the reachability analysis in the associated buffer and computes and sends the occupancy polycapsules of the humans
 *
 * @param data the message's data (contains t_end)
 */
void DummyObstacle::reachabilityAnalysis(const custom_robot_msgs::DoubleHeaderedConstPtr& data){
  if (!published_once){
    geometry_msgs::Pose obs_pose;
    obs_pose.position = pos;
    obs_pose.orientation.x = 0;
    obs_pose.orientation.y = 0;
    obs_pose.orientation.z = 0;
    obs_pose.orientation.w = 1;
    obs_pose_pub.publish(obs_pose);
    published_once = true;
    ROS_INFO_STREAM("Published sphere position to: " << pos.x << ", " << pos.y << ", " << pos.z);
  }
  
  custom_robot_msgs::PolycapsuleArrayPtr obstacle_pos(new custom_robot_msgs::PolycapsuleArray());
  custom_robot_msgs::Segment sphere_seg;
  sphere_seg.p = pos;
  sphere_seg.q = pos;
  custom_robot_msgs::Polycapsule sphere;
  sphere.polygon.push_back(sphere_seg);
  sphere.radius = radius;
  obstacle_pos->polycapsules.push_back(sphere);
  obstacle_pos->header.stamp = data->header.stamp;
  pub.publish(obstacle_pos);
}
