#include "rviz_marker/nodelet.h"


namespace rviz_marker {
  void RvizMarkerNodelet::onInit() {
    ros::NodeHandle& private_nh = getPrivateNodeHandle(); 
    // Define publishers
    ros::Publisher robot_visu_pub = private_nh.advertise<visualization_msgs::MarkerArray>("/robot_marker_array", 1000);
    ros::Publisher human_cylinder_visu_pub = private_nh.advertise<visualization_msgs::MarkerArray>(
        "/human_cylinder_marker_array", 1000);
    ros::Publisher human_reach_visu_pub_P = private_nh.advertise<visualization_msgs::MarkerArray>(
        "/human_reach_marker_array/P", 1000);
    ros::Publisher human_reach_visu_pub_V = private_nh.advertise<visualization_msgs::MarkerArray>(
        "/human_reach_marker_array/V", 1000);
    ros::Publisher human_reach_visu_pub_A = private_nh.advertise<visualization_msgs::MarkerArray>(
        "/human_reach_marker_array/A", 1000);

    // Create rviz marker object
    rviz_marker_ = RvizMarker(robot_visu_pub, 
        human_cylinder_visu_pub, 
        human_reach_visu_pub_P, 
        human_reach_visu_pub_V, 
        human_reach_visu_pub_A);

    // Define subscribers
    robot_capsule_sub_ = private_nh.subscribe("/robot_capsules", 1000, 
        &RvizMarker::robot_callback, &rviz_marker_);
    advanced_robot_capsule_sub_ = private_nh.subscribe("/advanced_robot_capsules", 1000, 
        &RvizMarker::advanced_robot_callback, &rviz_marker_);
    human_cylinder_sub_ = private_nh.subscribe("/human_polycapsules", 1000, 
        &RvizMarker::human_cylinder_callback, &rviz_marker_);
    human_reach_sub_P_ = private_nh.subscribe("/human_reach_capsules/P", 1000, 
        &RvizMarker::human_reach_callback_p, &rviz_marker_);
    human_reach_sub_V_ = private_nh.subscribe("/human_reach_capsules/V", 1000, 
        &RvizMarker::human_reach_callback_v, &rviz_marker_);
    human_reach_sub_A_ = private_nh.subscribe("/human_reach_capsules/A", 1000, 
        &RvizMarker::human_reach_callback_a, &rviz_marker_);
  }
} // namespace rviz_marker

PLUGINLIB_EXPORT_CLASS(rviz_marker::RvizMarkerNodelet, nodelet::Nodelet);
