#include "LightCurtainsContinuous.hh"
#include <sdf/sdf.hh>

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(LightCurtainPlugin)

///////////////////////////////////////////////////////////
LightCurtainPlugin::LightCurtainPlugin() : SensorPlugin(){
}

///////////////////////////////////////////////////////////
LightCurtainPlugin::~LightCurtainPlugin(){
}

///////////////////////////////////////////////////////////
void LightCurtainPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf){
  if (!ros::isInitialized()){
    /*ROS_FATAL_STREAM_NAMED("A Ros node for Gazebo has not been initialized, unable to load plugin. " << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;*/
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
  }

  // Create our ros node
  this->nh.reset(new ros::NodeHandle("gazebo_client"));

  // Get the parent sensor
  this->parentSensor = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);

  // Make sure the parent sensor is valid
  if (!this->parentSensor){
    gzerr << "LightCurtainsSensor requires a RaySensor. \n";
    return;
  }

  // Connect to the sensor update event
  this->newLaserScansConnection = this->parentSensor->ConnectUpdated(std::bind(&gazebo::LightCurtainPlugin::OnNewLaserScan, this));

  // Make sure the parent sensor is active
  this->parentSensor->SetActive(true);

  int id;

  if (!_sdf->HasElement("id")){
    // if parameter tag does NOT exist
    std::cout << "Missing parameter <id> in PluginName, default to standard" << std::endl;
    id = -1;
  }
    // if parameter tag exists, get its value
  else _sdf->GetElement("id")->GetValue()->Get(id);

  /*char s[20];
  sprintf(s, "/curtain_%d_data", id);
  this->publish = nh->advertise<std_msgs::Empty>(s, 1000);*/
  msg.data = id;

  this->publish = nh->advertise<std_msgs::Int8>("/curtains_data", 1000);

  this->is_crossing = false;

}

void LightCurtainPlugin::OnNewLaserScan(){
  if(parentSensor->RangeMax() - parentSensor->Range(0) > 1e-6){
    //std::cout << "Nb Rays : " << parentSensor->Range(0) << std::endl;
    //std::cout << msg.data << std::endl;
    //if(!is_crossing){
      publish.publish(msg);
      /*is_crossing = true;
    }
  }
  else{
    is_crossing = false;*/
  }
}
