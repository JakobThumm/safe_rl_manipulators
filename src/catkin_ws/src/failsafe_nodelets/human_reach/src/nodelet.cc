#include "human_reach/nodelet.h"

namespace human_reach {

void HumanReachNodelet::onInit() {
  ros::NodeHandle& private_nh = getPrivateNodeHandle();

  ros::Publisher pub_sets_P = private_nh.advertise<custom_robot_msgs::CapsuleArray>("/human_reach_capsules/P",1000);
  ros::Publisher pub_sets_V = private_nh.advertise<custom_robot_msgs::CapsuleArray>("/human_reach_capsules/V",1000);
  ros::Publisher pub_sets_A = private_nh.advertise<custom_robot_msgs::CapsuleArray>("/human_reach_capsules/A",1000);

  // get sensor information
  double measurement_error_pos = 0, measurement_error_vel = 0, delay = 0;
  if (ros::param::has("/motion_capture/measurement_error_pos")) {
    ros::param::get("/motion_capture/measurement_error_pos", measurement_error_pos);
  }
  if (ros::param::has("/motion_capture/measurement_error_vel")) {
    ros::param::get("/motion_capture/measurement_error_vel", measurement_error_vel);
  }
  if (ros::param::has("/motion_capture/delay")) {
    ros::param::get("/motion_capture/delay", delay);
  }

  std::vector<std::string> joint_names;
  if (ros::param::has("/motion_capture/joint_names")) {
    ros::param::get("/motion_capture/joint_names", joint_names);
  } else {
    ROS_ERROR("Parameter /motion_capture/joint_names not in parameter server!");
  }
  std::map<std::string, int> joint_ids;
  for (int i = 0; i < joint_names.size(); i++) {
      joint_ids[joint_names[i]] = i;
  }

  std::vector<double> joint_v_max;
  if (ros::param::has("/motion_capture/joint_v_max")) {
    ros::param::get("/motion_capture/joint_v_max", joint_v_max);
  } else {
    ROS_ERROR("Parameter /motion_capture/joint_v_max not in parameter server!");
  }

  std::vector<double> joint_a_max;
  if (ros::param::has("/motion_capture/joint_a_max")) {
    ros::param::get("/motion_capture/joint_a_max", joint_a_max);
  } else {
    ROS_ERROR("Parameter /motion_capture/joint_a_max not in parameter server!");
  }


  /// Create bodies
  std::map<std::string, jointPair> body_link_joints;
  std::map<std::string, double> thickness;
  XmlRpc::XmlRpcValue bodies;
  if (!ros::param::get("/motion_capture/bodies", bodies)) {
    ROS_ERROR("Parameter /motion_capture/bodies not in parameter server!");
  }
  createBodies(bodies, joint_ids, body_link_joints, thickness);
  
  /// Create extremities
  std::vector<double> shoulder_ids;
  std::vector<double> elbow_ids;
  std::vector<std::string> wrist_names;
  XmlRpc::XmlRpcValue extremities;
  if (!ros::param::get("/motion_capture/extremities", extremities)) {
    ROS_ERROR("Parameter /motion_capture/extremities not in parameter server!");
  }
  createExtremities(extremities, joint_ids, shoulder_ids, elbow_ids, wrist_names);

  human_reach_ = HumanReach(pub_sets_P, pub_sets_V, pub_sets_A, joint_names.size(), body_link_joints, 
                            thickness, joint_v_max, joint_a_max, 
                            shoulder_ids, elbow_ids, wrist_names,
                            measurement_error_pos, measurement_error_vel, delay);
  sub_t_brake_ = private_nh.subscribe("/tbrake", 1000, &HumanReach::humanReachabilityAnalysis, &human_reach_);
  sub_human_joint_pos_ = private_nh.subscribe("/human_joint_pos", 1000, &HumanReach::measurement, &human_reach_);
  
}


void HumanReachNodelet::createBodies(XmlRpc::XmlRpcValue& bodies, const std::map<std::string, int>& joint_ids, 
                                      std::map<std::string, human_reach::jointPair>& body_link_joints, std::map<std::string, double>& thickness) {
  ROS_ASSERT(bodies.getType()==XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i<bodies.size(); i++) {
    auto& body = bodies[i];
    ROS_ASSERT(body.getType()==XmlRpc::XmlRpcValue::TypeStruct);
    for (XmlRpc::XmlRpcValue::iterator info=body.begin(); info!=body.end(); ++info) {
      std::string body_name = info->first;
      int proximal_joint_id = -1;
      int distal_joint_id = -1;
      double r = 0.0;
      ROS_ASSERT(info->second.getType()==XmlRpc::XmlRpcValue::TypeArray);
      for (int j = 0; j<info->second.size(); j++) {
        auto& feature = info->second[j];
        ROS_ASSERT(feature.getType()==XmlRpc::XmlRpcValue::TypeStruct);
        for (XmlRpc::XmlRpcValue::iterator f=feature.begin(); f!=feature.end(); ++f) {
          std::string feature_name = f->first;
          if (feature_name == "proximal") {
            ROS_ASSERT(f->second.getType()==XmlRpc::XmlRpcValue::TypeString);
            std::string proximal = f->second;
            ROS_ASSERT(joint_ids.count(proximal) > 0);
            proximal_joint_id = joint_ids.at(proximal);
          } else if (feature_name == "distal") {
            ROS_ASSERT(f->second.getType()==XmlRpc::XmlRpcValue::TypeString);
            std::string distal = f->second;
            ROS_ASSERT(joint_ids.count(distal) > 0);
            distal_joint_id = joint_ids.at(distal);
          } else if (feature_name == "thickness") {
            ROS_ASSERT(f->second.getType()==XmlRpc::XmlRpcValue::TypeDouble);
            r = f->second;
          }
        }
      }
      // Create map entries
      ROS_INFO_STREAM("Creating body " << body_name << " with joint pair (" << proximal_joint_id << ", " << distal_joint_id << ") and thickness " << r);
      body_link_joints[body_name] = human_reach::jointPair(proximal_joint_id, distal_joint_id);
      thickness[body_name] = r;
    }
  }
}


void HumanReachNodelet::createExtremities(XmlRpc::XmlRpcValue& extemities, const std::map<std::string, int>& joint_ids,
                      std::vector<double>& shoulder_ids, std::vector<double>& elbow_ids, std::vector<std::string>& wrist_names) {
  ROS_ASSERT(extemities.getType()==XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i<extemities.size(); i++) {
    auto& extr = extemities[i];
    ROS_ASSERT(extr.getType()==XmlRpc::XmlRpcValue::TypeStruct);
    for (XmlRpc::XmlRpcValue::iterator info=extr.begin(); info!=extr.end(); ++info) {
      std::string extr_name = info->first;
      int shoulder_joint_id = -1;
      int elbow_joint_id = -1;
      std::string wrist_body_name = "";
      ROS_ASSERT(info->second.getType()==XmlRpc::XmlRpcValue::TypeArray);
      for (int j = 0; j<info->second.size(); j++) {
        auto& feature = info->second[j];
        ROS_ASSERT(feature.getType()==XmlRpc::XmlRpcValue::TypeStruct);
        for (XmlRpc::XmlRpcValue::iterator f=feature.begin(); f!=feature.end(); ++f) {
          std::string feature_name = f->first;
          if (feature_name == "shoulder") {
            ROS_ASSERT(f->second.getType()==XmlRpc::XmlRpcValue::TypeString);
            std::string shoulder = f->second;
            ROS_ASSERT(joint_ids.count(shoulder) > 0);
            shoulder_joint_id = joint_ids.at(shoulder);
          } else if (feature_name == "elbow") {
            ROS_ASSERT(f->second.getType()==XmlRpc::XmlRpcValue::TypeString);
            std::string elbow = f->second;
            ROS_ASSERT(joint_ids.count(elbow) > 0);
            elbow_joint_id = joint_ids.at(elbow);
          } else if (feature_name == "wristBody") {
            ROS_ASSERT(f->second.getType()==XmlRpc::XmlRpcValue::TypeString);
            std::string w = f->second;
            wrist_body_name = w;
          }
        }
      }
      // Create map entries
      ROS_INFO_STREAM("Creating extremity " << extr_name << " with shoulder joint " << shoulder_joint_id << ", elbow joint " << elbow_joint_id << " and body name " << wrist_body_name);
      shoulder_ids.push_back(shoulder_joint_id);
      elbow_ids.push_back(elbow_joint_id);
      wrist_names.push_back(wrist_body_name);
    }
  }
}
} // namespace human_reach

PLUGINLIB_EXPORT_CLASS(human_reach::HumanReachNodelet, nodelet::Nodelet);
    
