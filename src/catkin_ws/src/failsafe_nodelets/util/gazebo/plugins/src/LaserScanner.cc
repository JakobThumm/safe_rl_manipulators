#include "LaserScanner.hh"
#include <geometry_msgs/Point.h>

using namespace gazebo;
GZ_REGISTER_WORLD_PLUGIN(LaserPlugin)

/////////////////////////////////////////////////
LaserPlugin::LaserPlugin() : WorldPlugin(){
}

/////////////////////////////////////////////////
LaserPlugin::~LaserPlugin(){
}

/////////////////////////////////////////////////
void LaserPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/){
  if (!ros::isInitialized()){
    /*ROS_FATAL_STREAM_NAMED("A Ros node for Gazebo has not been initialized, unable to load plugin. " << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;*/
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
  }

  // Create our ros node
  this->nh.reset(new ros::NodeHandle("gazebo_client"));

  this->world = _parent;

  // gazebo_7
  //this->model = _parent->GetModel("box");

  // gazebo_9
  this->model = _parent->ModelByName("box");

  // gazebo 7
  //this->model2 = _parent->GetModel("box2");
        
  //gazebo 9
  this->model2 = _parent->ModelByName("box2");

  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&LaserPlugin::OnUpdate, this));

  this->publish = nh->advertise<custom_robot_msgs::Positions>("/lasers_data", 1000);
 
  this->sample_time = 0.004;

  this->prev_time = -1;

  std::cout << world->ModelCount() << std::endl;
}

/////////////////////////////////////////////////
void LaserPlugin::OnUpdate(){

  double time = this->world->SimTime().Float();
  if(time - prev_time >= sample_time){
    custom_robot_msgs::Positions msg = custom_robot_msgs::Positions();
    prev_time = time;
    //gazebo 7
    /**gazebo::math::Pose pose = this->model->WorldPose();
    geometry_msgs::Point p1 = geometry_msgs::Point();
    p1.x = pose.pos.x;
    p1.y = pose.pos.y;
    msg.points.push_back(p1);
    if(time >= 10 && time <= 18){
      pose = this->model2->GetWorldPose();
      geometry_msgs::Point p2 = geometry_msgs::Point();
      p2.x = pose.pos.x;
      p2.y = pose.pos.y;
      msg.points.push_back(p2);
    }*/
    //gazebo 9
    ignition::math::Pose3d pose = this->model->WorldPose();
    geometry_msgs::Point p1 = geometry_msgs::Point();
    p1.x = pose.Pos().X();
    p1.y = pose.Pos().Y();
    msg.data.push_back(p1);
    if(time >= 10 && time <= 18){
      pose = this->model2->WorldPose();
      geometry_msgs::Point p2 = geometry_msgs::Point();
      p2.x = pose.Pos().X();
      p2.y = pose.Pos().Y();
      msg.data.push_back(p2);
    }
    publish.publish(msg);
  }
}
