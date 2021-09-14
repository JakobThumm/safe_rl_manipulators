#include "modrob_contact_plugin.hh"

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
void ContactPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Get the parent sensor.
  this->parentSensor =
    std::dynamic_pointer_cast<sensors::ContactSensor>(_sensor);

  // Make sure the parent sensor is valid.
  if (!this->parentSensor)
  {
    ROS_ERROR("ContactPlugin requires a ContactSensor!");
    return;
  }

  double frame_rate = 100;
  if(_sdf->HasElement("frame_rate")){
    frame_rate = _sdf->Get<double>("frame_rate");
    if (frame_rate <= 0){
      frame_rate = 100;
      ROS_WARN("Selected frame rate for motion capture system is <= 0. Resetting to 100 Hz.");
    }
  }
  this->dt_update = 1/frame_rate;
  this->last_update_time = -1;

  // Connect to the sensor update event.
  this->updateConnection = this->parentSensor->ConnectUpdated(
      std::bind(&ContactPlugin::OnUpdate, this));

  // Make sure the parent sensor is active.
  this->parentSensor->SetActive(true);

  /// Handling the ROS topic publication
  // Initialize ros, if it has not already bee initialized.
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client",
        ros::init_options::NoSigintHandler);
  }
  // Default namespace is modrob0
  std::string ns = "modrob0";

  // Check that the namespace element exists, then read the value
  if (_sdf->HasElement("name_space")){
    ns = _sdf->Get<std::string>("name_space");
  }
  std::string topic_name = ns + "/collisions";
  publisher = node_handle.advertise<modrob_simulation::Collisions>(topic_name, 100);
  //ros::spin();
}

/////////////////////////////////////////////////
void ContactPlugin::OnUpdate()
{
  // only update every dt_update (0.01) seconds.
  double t = ros::Time::now().toSec();
  if (t < this->last_update_time + this->dt_update){
    return;
  }
  this->last_update_time = t;
  // Get all the contacts.
  msgs::Contacts contacts;
  modrob_simulation::Collisions collisions;
  contacts = this->parentSensor->Contacts();
  // Return if no contact
  if(contacts.contact_size()==0){return;}
  for (unsigned int i = 0; i < contacts.contact_size(); ++i)
  {
    modrob_simulation::Collision collision;
    collision.parent_contact = contacts.contact(i).collision1();
    collision.obstacle_contact = contacts.contact(i).collision2();
    for (unsigned int j = 0; j < contacts.contact(i).position_size(); ++j)
    {
      geometry_msgs::Point position;
      position.x = contacts.contact(i).position(j).x();
      position.x = contacts.contact(i).position(j).y();
      position.x = contacts.contact(i).position(j).z();
      collision.position.push_back(position);
      geometry_msgs::Point normal;
      normal.x = contacts.contact(i).normal(j).x();
      normal.y = contacts.contact(i).normal(j).y();
      normal.z = contacts.contact(i).normal(j).z();
      collision.normal.push_back(normal);
      collision.depth.push_back(contacts.contact(i).depth(j));
    }
    collisions.collisions.push_back(collision);
  }
  publisher.publish(collisions);
}