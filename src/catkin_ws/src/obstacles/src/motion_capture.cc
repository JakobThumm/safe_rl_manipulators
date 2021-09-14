#include "obstacles/motion_capture.h"

namespace obstacles {

GZ_REGISTER_MODEL_PLUGIN(MotionCapturePlugin)

#define ANIMATION "human_animation"

/////////////////////////////////////////////////
MotionCapturePlugin::MotionCapturePlugin()
{
}

/////////////////////////////////////////////////
void MotionCapturePlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->actor_ = boost::dynamic_pointer_cast<gazebo::physics::Actor>(_model);
  // Set custom trajectory
  ResetTrajectoryInfo();

  ROS_INFO("Looking for DAE file");
  assert((_sdf->HasElement("animation_file"), "The human actor model plugin does not define the <animation_file>path/to/animation.dae</animation_file>."));
  std::string dae_file = _sdf->Get<std::string>("animation_file");
  if(const char* sim_root = std::getenv("ROBOT_RL_SIM_ROOT")){
    dae_file = std::string(sim_root) + dae_file;
  } else {
    ROS_ERROR("You need to define your $ROBOT_RL_SIM_ROOT environment variable!");
    exit(-1);
  }
  bool found_file = std::filesystem::exists(dae_file);
  if (found_file){
    ROS_INFO("Found DAE file");
  } else {
    ROS_ERROR_STREAM("Could not find file " << dae_file);
    exit(-1);
  }

  double frame_rate = 100;
  if(_sdf->HasElement("frame_rate")){
    frame_rate = _sdf->Get<double>("frame_rate");
    if (frame_rate <= 0){
      frame_rate = 100;
      ROS_WARN("Selected frame rate for motion capture system is <= 0. Resetting to 100 Hz.");
    }
  }
  this->dt_update_ = 1/frame_rate;
  this->last_update_time_ = -1;
  
  ////// Read in dae file to extract the node structure. //////
  // initialize the library and check potential ABI mismatches
  xmlDoc *doc = NULL;
  xmlNode *root_element = NULL;
  LIBXML_TEST_VERSION
  std::string ns = "http://www.collada.org/2005/11/COLLADASchema";
  if ((doc = xmlReadFile(dae_file.c_str(), NULL, 0)) == NULL){
    ROS_ERROR_STREAM("Could not parse file " << dae_file.c_str());
    exit(-1);
  }
  root_element = xmlDocGetRootElement(doc);
  const xmlChar *name = root_element->name;
  // Read in skeleton
  ReadDaeSkeleton(root_element);
  
  // Cleanup
  xmlFreeDoc(doc);
  xmlCleanupParser();
  xmlMemoryDump();

  this->connections_.push_back(gazebo::event::Events::ConnectWorldUpdateBegin(
          std::bind(&MotionCapturePlugin::OnUpdate, this, std::placeholders::_1)));

  //////// Add collision boxes to the actor model //////////
  std::map<std::string, ignition::math::Vector3d> scaling;
  std::map<std::string, ignition::math::Pose3d> offsets;
  if (_sdf->HasElement("scaling")) {
    auto elem = _sdf->GetElement("scaling");
    while (elem) {
      if (!elem->HasAttribute("collision")) {
        gzwarn << "Skipping element without collision attribute" << std::endl;
        elem = elem->GetNextElement("scaling");
        continue;
      }
      auto name = elem->Get<std::string>("collision");

      if (elem->HasAttribute("scale")) {
        auto scale = elem->Get<ignition::math::Vector3d>("scale");
        scaling[name] = scale;
      }

      if (elem->HasAttribute("pose")) {
        auto pose = elem->Get<ignition::math::Pose3d>("pose");
        offsets[name] = pose;
      }
      elem = elem->GetNextElement("scaling");
    }
  }

  for (const auto &link : actor_->GetLinks()) {
    // Init the links, which in turn enables collisions
    link->Init();

    if (scaling.empty())
      continue;

    // Process all the collisions in all the links
    for (const auto &collision : link->GetCollisions()) {
      auto name = collision->GetName();
      ROS_INFO_STREAM(name);
      if (scaling.find(name) != scaling.end()) {
        auto boxShape = boost::dynamic_pointer_cast<gazebo::physics::BoxShape>(collision->GetShape());
        // Make sure we have a box shape.
        if (boxShape) {
          boxShape->SetSize(boxShape->Size() * scaling[name]);
        }
      }

      if (offsets.find(name) != offsets.end()) {
        collision->SetInitialRelativePose(offsets[name] + collision->InitialRelativePose());
      }
    }
  }

  // Initialize ros, if it has not already bee initialized.
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client",
        ros::init_options::NoSigintHandler);
  }
  // Create our ROS node. This acts in a similar manner to
  // the Gazebo node
  this->ros_node_.reset(new ros::NodeHandle("gazebo_client"));
  std::string topic_name = "/human_joint_pos";
  this->publisher_ = this->ros_node_->advertise<custom_robot_msgs::PositionsHeadered>(topic_name, 1000);

  // ROS Subscribers
  
  ros::SubscribeOptions so_pose =
    ros::SubscribeOptions::create<geometry_msgs::Pose>(
        "/" + actor_->GetName() + "/pose_cmd",
        1,
        boost::bind(&MotionCapturePlugin::OnRosPoseMsg, this, _1),
        ros::VoidPtr(), &ros_queue_);
  ros_subscriber_pose_ = ros_node_->subscribe(so_pose);

  ros::SubscribeOptions so_start_stop =
    ros::SubscribeOptions::create<std_msgs::Bool>(
        "/" + actor_->GetName() + "/start_stop_cmd",
        1,
        boost::bind(&MotionCapturePlugin::OnRosStartStopAnimation, this, _1),
        ros::VoidPtr(), &ros_queue_);
  ros_subscriber_start_stop_ = ros_node_->subscribe(so_start_stop);

  ros::SubscribeOptions so_script_time =
    ros::SubscribeOptions::create<std_msgs::Float64>(
        "/" + actor_->GetName() + "/script_time_cmd",
        1,
        boost::bind(&MotionCapturePlugin::OnRosScriptTimeMsg, this, _1),
        ros::VoidPtr(), &ros_queue_);
  ros_subscriber_script_time_ = ros_node_->subscribe(so_script_time);

  // Spin up the queue helper thread.
  ros_queue_thread_ = std::thread(std::bind(&MotionCapturePlugin::QueueThread, this));
  ROS_INFO("Motion capture plugin initialized.");
}

void MotionCapturePlugin::QueueThread()
{
  static const double timeout = 0.01;
  while (ros_node_->ok())
  {
    ros_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

/////////////////////////////////////////////////
void MotionCapturePlugin::OnUpdate(const gazebo::common::UpdateInfo &_info)
{
  gazebo::physics::TrajectoryInfoPtr traj_info =  actor_->CustomTrajectory();
  if (traj_info == nullptr) {
    ResetTrajectoryInfo();
  }
  // only update every dt_update (0.01) seconds.
  double t = _info.simTime.sec + _info.simTime.nsec * pow(10, -9);
  if (t < this->last_update_time_ + this->dt_update_){
    return;
  }
  if (play_) {
    animation_time_ += t - this->last_update_time_;
  }
  this->last_update_time_ = t;

  auto skelAnims = this->actor_->SkeletonAnimations();
  auto main_animation = skelAnims.find(ANIMATION);
  if (main_animation == skelAnims.end())
  {
    ROS_ERROR_STREAM("Skeleton animation " << ANIMATION << " not found.");
  }
  else
  {
    // Update the script time
    this->actor_->SetScriptTime(animation_time_);
    // Get the animation at script time
    std::map<std::string, ignition::math::Matrix4d> node_pose = main_animation->second->PoseAt(this->actor_->ScriptTime(), true);
    // Set the actor pose
    ignition::math::Matrix4d root_T = node_pose.at(this->root_node_->Name());
    ignition::math::Pose3d pose = root_T.Pose();
    pose += shift_pos_;
    actor_->SetWorldPose(pose, true, false);
    // Calculate joint information
    ignition::math::Matrix4d root_transform = ignition::math::Matrix4d::Identity;
    root_transform(0,3) = shift_pos_.X();
    root_transform(1,3) = shift_pos_.Y();
    root_transform(2,3) = shift_pos_.Z();
    // Recursively calculate cartesian joint positions.
    CalculateJointPos(node_pose, root_transform, this->root_node_, this->joint_positions_);
    custom_robot_msgs::PositionsHeadered joint_pos_msg;
    joint_pos_msg.header.stamp = ros::Time(_info.simTime.sec, _info.simTime.nsec);
    //int i = 0;
    for (const auto& jp : joint_positions_){
      geometry_msgs::Point p;
      p.x = jp.second.X();
      p.y = jp.second.Y();
      p.z = jp.second.Z();
      joint_pos_msg.data.push_back(p);
    }
    this->publisher_.publish(joint_pos_msg);
  }
}

void MotionCapturePlugin::ResetTrajectoryInfo() {
  gazebo::physics::TrajectoryInfoPtr trajectoryInfo(new gazebo::physics::TrajectoryInfo());
  trajectoryInfo->type = ANIMATION;
  trajectoryInfo->duration = 0.0;
  actor_->SetCustomTrajectory(trajectoryInfo);
}

void MotionCapturePlugin::CalculateJointPos(const std::map<std::string, ignition::math::Matrix4d>& joint_transformations,
                                    const ignition::math::Matrix4d& parent_transform,
                                    SkeletonNode* child,
                                    std::map<std::string, ignition::math::Vector3d>& joint_positions){
  // Calculate transformation matrix of child
  ignition::math::Matrix4d child_transform = parent_transform * joint_transformations.at(child->Name());
  // Extract position and save it in map
  joint_positions[child->Name()] = GetPosOfTransformation(child_transform);
  // Do this for all children of the child
  for (const auto& grand_child : child->GetChildren()){
    CalculateJointPos(joint_transformations, child_transform, grand_child, joint_positions);
  }
}

/////////////////////////////////////////////////
void MotionCapturePlugin::ReadDaeSkeleton(xmlNode * root){
  const char * lvs_str = "library_visual_scenes";
  xmlNode* lvs = GetChildByName(root, lvs_str);
  assert(lvs != NULL);
  const char * vs_str = "visual_scene";
  xmlNode* vs = GetChildByName(lvs, vs_str);
  assert(vs != NULL);
  const char * node_str = "node";
  xmlNode* animation_root = GetChildByName(vs, node_str);
  assert(animation_root != NULL);
  const char * matrix_str = "matrix";
  xmlNode* matrix = GetChildByName(animation_root, matrix_str);
  // Check if this is the correct animation root node
  // We do not use the matrix per se but it destiguishes to the other node in vs.
  assert(matrix != NULL);
  xmlNode* root_bone = GetChildByName(animation_root, node_str);
  assert(root_bone != NULL);
  std::string root_bone_name = GetNodeId(root_bone);
  this->root_node_ = new SkeletonNode(root_bone_name);
  // Search all subnodes
  FillSkeleton(root_bone, this->root_node_);
  ROS_INFO("Created animation skeleton!");
}

void MotionCapturePlugin::FillSkeleton(xmlNode* xml_parent, SkeletonNode* skeleton_parent){
  const char * node_str = "node";
  std::vector<xmlNode*> child_nodes = GetAllChildrenByName(xml_parent, node_str);
  for(xmlNode* child: child_nodes) {
    std::string child_name = GetNodeId(child);
    SkeletonNode* child_skeleton_node = new SkeletonNode(child_name);
    skeleton_parent->AddChild(child_skeleton_node);
    FillSkeleton(child, child_skeleton_node);
  }
}

xmlNode* MotionCapturePlugin::GetChildByName(xmlNode * root, const char * child_name){
  xmlNode *cur_node = NULL;
  for (cur_node = root->children; cur_node; cur_node = cur_node->next) {
    const char* name = reinterpret_cast<const char*>(cur_node->name);
    if (cur_node->type == XML_ELEMENT_NODE && strcmp(name, child_name)==0) {
      return cur_node;
    }
  }
  return NULL;
}

std::vector<xmlNode*> MotionCapturePlugin::GetAllChildrenByName(xmlNode * root, const char * child_name){
  std::vector<xmlNode*> children;
  xmlNode *cur_node = NULL;
  for (cur_node = root->children; cur_node; cur_node = cur_node->next) {
    const char* name = reinterpret_cast<const char*>(cur_node->name);
    if (cur_node->type == XML_ELEMENT_NODE && strcmp(name, child_name)==0) {
      children.push_back(cur_node);
    }
  }
  return children;
}

void MotionCapturePlugin::OnRosPoseMsg(const geometry_msgs::PoseConstPtr &_msg) {
  shift_pos_.SetX(_msg->position.x);
  shift_pos_.SetY(_msg->position.y);
  shift_pos_.SetZ(_msg->position.z);
}

void MotionCapturePlugin::OnRosScriptTimeMsg(const std_msgs::Float64ConstPtr & _msg) {
  animation_time_ = (double)_msg->data;
}

void MotionCapturePlugin::OnRosStartStopAnimation(const std_msgs::BoolConstPtr & _msg) {
  play_ = _msg->data;
}
} // namespace obstacles