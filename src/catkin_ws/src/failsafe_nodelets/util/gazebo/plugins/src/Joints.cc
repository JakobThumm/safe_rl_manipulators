#include "Joints.hh"
#include <gazebo/msgs/msgs.hh>
#include <string>
#include <math.h>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(JointsPlugin)

///////////////////////////////////////////////////////////
JointsPlugin::JointsPlugin() : ModelPlugin(){
}

///////////////////////////////////////////////////////////
JointsPlugin::~JointsPlugin(){
}

///////////////////////////////////////////////////////////
void JointsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){
  // Safety check
  if (_model->GetJointCount() == 0)
  {
    std::cerr << "Invalid joint count, Joints plugin not loaded\n";
    return;
  }

  if (!ros::isInitialized()){
    /*ROS_FATAL_STREAM_NAMED("A Ros node for Gazebo has not been initialized, unable to load plugin. " << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;*/
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_joint_plugin", ros::init_options::NoSigintHandler);
  }

  // Create our ros node
  this->nh.reset(new ros::NodeHandle("gazebo_joint_plugin"));

  // Store the model pointer for convenience.
  this->model = _model;

  // Setup a P-controller, with a gain of 0.1.
  int G = 50;
  double z = 5;
  this->pid = common::PID(G*G, 2*z*G, 0);

  // Apply the P-controller to the joint.

  /*for(int i = 1; i < /*_model->GetJointCount()*7; i++){
    if(ConvertJointType(this->model->GetJoints()[i]->GetMsgType()) == "revolute"){
      std::cout << "pid : " << this->model->GetJoints()[i]->GetScopedName() << std::endl;
      this->model->GetJointController()->SetPositionPID(
      this->model->GetJoints()[i]->GetScopedName(), this->pid);
    }
  }*/
  std::vector<double> q0;
  //initialize the node
  if(_sdf->HasElement("default_value")){
    if(nh->getParam("/"+_sdf->GetElement("default_value")->Get<std::string>(), q0)){
/**      int j=0;
      for(int i = 1; i < 7; i++){
        if(ConvertJointType(this->model->GetJoints()[i]->GetMsgType()) == "revolute"){
std::cout << "pid : " << this->model->GetJoints()[i]->GetScopedName() << " : " << q0[j] << std::endl;
          this->model->GetJointController()->SetPositionTarget(
          this->model->GetJoints()[i]->GetScopedName(), q0[j]);
          j += 1;
        }
      }*/
    }
    else{
      ROS_ERROR_STREAM_NAMED("joints_plugin","Parameter q0 is not set");
    }
  }

  if(_sdf->HasElement("joint_names")){
    std::vector<std::string> joints;
    if(nh->getParam("/"+_sdf->GetElement("joint_names")->Get<std::string>(), joints)){
      int i = 0;
      this->model->GetJointController()->SetPositionTarget("lwa4d::arm_podest_joint", 0.0);
      for(auto it = joints.begin(); it != joints.end(); ++it){
        joints_name.emplace(i, "lwa4d::" + *it);
        this->model->GetJointController()->SetPositionPID(joints_name.at(i), this->pid);
        this->model->GetJointController()->SetPositionTarget(joints_name.at(i), q0[i]);
        i++;
      }
    }
    else{
      ROS_ERROR_STREAM_NAMED("joints_plugin","Parameter joint_names is not set");
    }
  }
  

  std::string topic = "/chatter";
  if(_sdf->HasElement("joint_topic")){
    topic = _sdf->GetElement("joint_topic")->Get<std::string>();
  }

  ros::SubscribeOptions so =
  ros::SubscribeOptions::create<modrob_workstation::RobotConfigCommanded>(
      "/ns/robot_config_commanded", 1000,
      boost::bind(&JointsPlugin::callback, this, _1),
      ros::VoidPtr(), &this->rosQueue);
  this->rosSub = this->nh->subscribe(so);

// Spin up the queue helper thread.
this->rosQueueThread =
  std::thread(std::bind(&JointsPlugin::QueueThread, this));

}

/////////////////////////////////////////////////////////////////
void JointsPlugin::callback(const modrob_workstation::RobotConfigCommandedConstPtr &data){
  /*int j = 0;
  for(int i = 0; i < this->model->GetJointCount(); i++){
    if(ConvertJointType(this->model->GetJoints()[i]->GetMsgType()) == "revolute"){
      std::cout << this->model->GetJoints()[i]->GetScopedName() <<" : " << data->q[j] << std::endl;
      this->model->GetJointController()->SetPositionTarget(
      this->model->GetJoints()[i]->GetScopedName(), data->q[j]);
      j += 1;
    }
  }*/
  int i = 0;
  for(auto it = joints_name.begin(); it != joints_name.end(); ++it){
    /*if(i == 0){
      double angle = data->q[it->first] + M_PI;
      angle -= 2*M_PI*floor(angle/(2*M_PI));
      if(angle > M_PI){
        angle = angle - 2*M_PI;
      }
      std::cout << angle << std::endl;
      this->model->GetJointController()->SetPositionTarget(it->second, angle);
    }
    else{*/
      this->model->GetJointController()->SetPositionTarget(it->second, data->joint_moves[it->first].joint_angle);
    /*}
    i += 1;*/
  }
}

void JointsPlugin::QueueThread(){
  static const double timeout = 0.01;
  while (this->nh->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}
