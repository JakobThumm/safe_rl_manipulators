#include "PresenceSensor.hh"

using namespace gazebo;
GZ_REGISTER_SENSOR_PLUGIN(ContactPlugin)

/////////////////////////////////////////////////
ContactPlugin::ContactPlugin() : SensorPlugin()
{
}

/////////////////////////////////////////////////
ContactPlugin::~ContactPlugin()
{
}

/////////////////////////////////////////////////
void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr /*_sdf*/)
{
  if (!ros::isInitialized()){
    /*ROS_FATAL_STREAM_NAMED("A Ros node for Gazebo has not been initialized, unable to load plugin. " << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;*/
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
  }

  // Create our ros node
  this->nh.reset(new ros::NodeHandle("gazebo_client"));

  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    gzerr << "ContactPlugin requires a ContactSensor.\n";
    return;
  }

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&ContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);

  this->publish = nh->advertise<std_msgs::Bool>("/presence_data", 1000);

  std::vector<double> pos;
  int nb_curtains;
  nh->getParam("/nb_curtains", nb_curtains);
  char s[20];
  for(int i = 0; i < nb_curtains; i++){
    sprintf(s,"/curtains_pos_%d",i);
    ros::param::get(s,pos);
    curtains_pos.insert (curtains_pos.end(),pos.begin(),pos.end());
  }
//  nh->getParam("/curtains_pos", curtains_pos);
  msg = std_msgs::Bool();
  msg.data = false;

}

/////////////////////////////////////////////////
void ContactPlugin::OnUpdate()
{
  // Get all the contacts.
  msgs::Contacts contacts;
  contacts = this->parentSensor->Contacts();
  bool contact = false;
  int i = 0;
  int j = 0;
  int nb = 0;
  while (!contact && i < contacts.contact_size()){
    j = 0;
    while (!contact && j < contacts.contact(i).position_size()){
      if(isIn(contacts.contact(i).position(j))){
        contact = true;
      }
      j += 1;
    }
    i += 1;
  }
  if(contact != msg.data){
    msg.data = contact;
    publish.publish(msg);
  }
}

/////////////////////////////////////////////////////
bool ContactPlugin::isIn(const gazebo::msgs::Vector3d point){
  int j;
  bool c = false;

  std::vector<double>::iterator it = curtains_pos.begin();
  for(std::vector<double>::iterator it = curtains_pos.begin(); it != curtains_pos.end(); it=it+4){
    if((*(it+1) > point.y() != *(it+3) > point.y()) && 
       (point.x() < (*(it+2) - *it)*(point.y() - *(it+1))/(*(it+3)-*(it+1)) + *it)){
      c = !c;
    }
  }
  return c;
}
