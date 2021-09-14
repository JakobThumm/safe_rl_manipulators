//Code for coordiante calculation adapted from augmented reality them: dh_transformer

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <sstream>
#include <geometry_msgs/Point.h>
#include <modrob_simulation/RobotGroupDescription.h>
#include <modrob_simulation/RobotDescription.h>
#include <modrob_simulation/LinkDescription.h>
#include <modrob_simulation/JointDescription.h>
#include <modrob_simulation/LinkVisual.h>
#include <modrob_simulation/LinkSensor.h>
#include <modrob_simulation/LinkCollision.h>
#include <modrob_workstation/ModuleOrder.h>
#include "robot_description_publisher.hpp"
#include <math.h>

using namespace std;

enum Axis{ x, z, y } ;

//Robot link color
double robot_link_color_r = 1;
double robot_link_color_g = 1;
double robot_link_color_b = 1;
double robot_link_color_a = 1;

//Robot motor color
double robot_motor_color_r = 0.1;
double robot_motor_color_g = 0.1;
double robot_motor_color_b = 0.5;
double robot_motor_color_a = 1;

// Contact sensor information
std::string plugin_name = "modrob_contact_plugin";
std::string plugin_file = "libmodrob_contact_plugin.so";
bool always_on = true;

//Global variables
ros::Publisher publisher;
ros::Publisher group_publisher;
std::string database_path;
int part_count; 
uint order;      
int jointCount;
int counter;
modrob_workstation::ModuleOrder lastModuleOrder;
modrob_simulation::RobotDescription robotDescription;
modrob_simulation::RobotGroupDescription robotGroup;


// Array of base-positions
std::vector<geometry_msgs::Point> base_positions;

int main(int argc, char **argv){
  ros::init(argc, argv, "robot_description_publisher");
  ros::NodeHandle n;
  publisher = n.advertise<modrob_simulation::RobotDescription>("ns/robot_description", 1000);
  group_publisher = n.advertise<modrob_simulation::RobotGroupDescription>("ns/robot_group_description", 1000);

  // listen for a single robot description
  ros::Subscriber subcriberModuleOrder = n.subscribe("ns/robot_module_order", 1000, listenForModuleOrderAndGenerateAndPublishRobotDescriptionMessage);
  
  // listen for a group of robot descriptions
  ros::Subscriber subcriberGroupOrder = n.subscribe("ns/group_module_order", 1000, listenForGroupOrderAndPublishRobotGroupDescription);
  n.getParam("/modrob_simulation_package_path", database_path);
  database_path.append("/stl_files/module_db/Modules.csv");
  n.getParam("/modrob_simulation_package_path", stl_file_path);
  stl_file_path.append("/stl_files/");
  ros::spin();
  return 0;
}

// Publishes an entire group of robots
void listenForGroupOrderAndPublishRobotGroupDescription(const modrob_simulation::RobotGroupModuleOrder::ConstPtr& group){

  // array of future robot-descriptions
  modrob_simulation::RobotGroupDescription newGroupDescription;
  robotGroup = newGroupDescription;


  // Transform every ModuleOrder in "group" into an entry of groupDescription
  for(uint mem = 0; mem < group->robots.size(); mem++){

    // add the base-position to the list
    geometry_msgs::Point p;
    p.x = group->robots[mem].base_pos.x;
    p.y = group->robots[mem].base_pos.y;
    p.z = group->robots[mem].base_pos.z;
    if(std::count(base_positions.begin(),base_positions.end(),p)){
      p.x += 0.01*mem;
    }
    base_positions.push_back(p);

    // define the member 
    // start off with a blanck description
    modrob_simulation::RobotDescription desc;

    desc.header.stamp = ros::Time::now();
    desc.header.frame_id = std::to_string(mem);
    desc.name = "modrob" + std::to_string(mem);
    desc.base_pos.x = group->robots[mem].base_pos.x;
    desc.base_pos.y = group->robots[mem].base_pos.y;
    desc.base_pos.z = group->robots[mem].base_pos.z;

    robotDescription = desc;


    modrob_simulation::RobotModuleOrder newModuleOrder;

    // ID-paring per module-list
    order = 0;
    jointCount = 0;
    counter = 0;
    part_count = group->robots[mem].modules.size();
    for(auto const& it: group->robots[mem].modules){
      newModuleOrder.modules.push_back(it);
      parseID(it);
      //order++;
    }

    // Add the generated description to the group
    robotGroup.descriptions.push_back(robotDescription);

  }

  group_publisher.publish(robotGroup);
  ROS_INFO("%s", "Published Robot-Group description");
}

void listenForModuleOrderAndGenerateAndPublishRobotDescriptionMessage(const modrob_workstation::ModuleOrder::ConstPtr& module_order)
{

  //Check, if module order is different to last one
  if(lastModuleOrder.modules.size() == module_order->modules.size()){
    bool isEqual = true;
    for(int i=0;i < module_order->modules.size();i++){
      if(lastModuleOrder.modules[i] != module_order->modules[i]){
        isEqual = false;
        break;
      }
    }
    if(isEqual){
      return;
    }
  }
  

  modrob_simulation::RobotDescription newRobotDescription;
  robotDescription = newRobotDescription;

  // Header
  robotDescription.header.stamp = ros::Time::now();
  robotDescription.header.frame_id = "0";
  robotDescription.name = "modrob";

  
  part_count = module_order->modules.size();
  order = 0;
  jointCount = 0;
  modrob_workstation::ModuleOrder newModuleOrder;

  // Parse ID for each part id
  for(int i=0;i < module_order->modules.size();i++){
    newModuleOrder.modules.push_back(module_order->modules[i]);
    parseID(module_order->modules[i]); 
    order++;
  }


  lastModuleOrder = newModuleOrder;
  //Info, publish and loop
  publisher.publish(robotDescription);
  ROS_INFO("%s", "Published Robot description");

}



void parseID(int id){

  // ### Find database row, which corresponds to the id
  std::ifstream       file(database_path);
  CSVRow              row;
  file >> row;                  //Skipping first row (attributes-names)
  while(file >> row){           //Searching for line with matching id
      if(std::stoi(row[Database::ID]) == id) break;
  }
  ROS_INFO("Found %s with id %d",row[0].c_str(),id);
  if(!row[Database::Name].compare("")) ROS_ERROR("ID not found in database %d!",id);


  // ### Create urdf elements from row data
  switch (std::stoi(row[Database::typ])){
    case LINK_TYPE:   
      createLinkDescription(row);
      break;
    case MOTOR_TYPE:
      // part_count++; //Motor consistes of 2 parts disabled for renaming
      createMotorDescription(row);
      break;
    case BASE_TYPE:
      createLinkDescription(row);
      break;
    case END_EFFECTOR_TYPE:
      ROS_WARN("End effector does not exist in database");
      break;
    default:
      ROS_ERROR("Typ values of %s is not defined in database",row[Database::Name].c_str());

  }
}






// ############################################
// ######   Create Link Description    ########
// ############################################

void createLinkDescription(CSVRow row){
  // Link values
  modrob_simulation::LinkDescription link;

  // change the base position if desired | only applied in group setting
  link.origin_x     = stof(row[Database::r_com_pl_x]);
  link.origin_y     = stof(row[Database::r_com_pl_y]);
  link.origin_z     = stof(row[Database::r_com_pl_z]);
  link.origin_r     = 0;
  link.origin_p     = 0;
  link.origin_yy    = 0;
  link.mass         = stof(row[Database::m_pl]);
  link.intertia_ixx = stof(row[Database::I_pl_xx]);
  link.intertia_ixy = stof(row[Database::I_pl_xy]);
  link.intertia_ixz = stof(row[Database::I_pl_xz]);   
  link.intertia_iyy = stof(row[Database::I_pl_yy]);
  link.intertia_iyz = stof(row[Database::I_pl_yz]);
  link.intertia_izz = stof(row[Database::I_dl_zz]);

  if(order == 0){
    link.name = "part0";
  }else{
    link.name = "link" + to_string(order);
  }


  // Visual values
  rvizXYZRPYY _1, _2;
  Eigen::Matrix4d link_transform;
  rvizXYZRPYY coordinates = convertDatabaseCoordinatesToRvizCoordinates(row[Database::ID],row[Database::a_pl],row[Database::p_pl],row[Database::n_pl],row[Database::a_dl],row[Database::n_dl],row[Database::p_dl],row[Database::alpha_pl],row[Database::delta_pl],row[Database::delta_dl],row[Database::alpha_dl], _1, _2, link_transform);
  modrob_simulation::LinkVisual visual;
  visual.origin_x   = coordinates.x;
  visual.origin_y   = coordinates.y;
  visual.origin_z   = coordinates.z;
  visual.origin_r   = coordinates.r;
  visual.origin_p   = coordinates.p;
  visual.origin_yy  = coordinates.yy;
  std::string low_poly_file = stl_file_path + row[Database::Name] + "_low_poly.STL";
  if ( access( low_poly_file.c_str(), F_OK ) != -1 ){
    visual.type = row[Database::Name] + "_low_poly.STL";
  } else {
    visual.type       = row[Database::Name] + ".STL";
  }
  visual.color_name = "link_color-" + to_string(order);
  visual.color_r    = robot_link_color_r;
  visual.color_g    = robot_link_color_g;
  visual.color_b    = robot_link_color_b;
  visual.color_a    = robot_link_color_a;

  // add gazebo-material
  visual.gazebo_material = "Gazebo/DarkGrey";

  // Collision values
  modrob_simulation::LinkCollision collision;
  rvizXYZRPYY collision_coordinates;
  if (row.size() > Database::collision_size_x){
    collision.type = "box";
    collision.arg1 = stof(row[Database::collision_size_x]);
    collision.arg2 = stof(row[Database::collision_size_y]);
    collision.arg3 = stof(row[Database::collision_size_z]);
    Eigen::Matrix4d transform_coll_pos;
    transform_coll_pos << 1, 0, 0, stof(row[Database::collision_pos_x]),
                          0, 1, 0, stof(row[Database::collision_pos_y]),
                          0, 0, 1, stof(row[Database::collision_pos_z]),
                          0, 0, 0, 1;
    Eigen::Matrix4d collision_transform = link_transform * transform_coll_pos;
    collision_coordinates = transformationMatrix2RVIZ(collision_transform);
  } else {
    collision_coordinates = coordinates;
    std::string low_poly_file = stl_file_path + row[Database::Name] + "_low_poly.STL";
    if ( access( low_poly_file.c_str(), F_OK ) != -1 ) {
      collision.type = row[Database::Name] + "_low_poly.STL";
    } else {
      collision.type = row[Database::Name] + ".STL";
    }
  }
  collision.origin_x   = collision_coordinates.x;
  collision.origin_y   = collision_coordinates.y;
  collision.origin_z   = collision_coordinates.z;
  collision.origin_r   = collision_coordinates.r;
  collision.origin_p   = collision_coordinates.p;
  collision.origin_yy  = collision_coordinates.yy;
  // Add a contact sensor
  // Don't add contact sensor to base link.
  modrob_simulation::LinkSensor sensor;
  bool has_sensor = false;
  if (order > 0) {
    has_sensor = true;
    sensor.name = "link" + to_string(order) + "_contact_sensor";
    sensor.always_on = always_on;
    sensor.plugin_name = plugin_name;
    sensor.plugin_file = plugin_file;
    sensor.collision = "distal_link_of_joint" + to_string(jointCount-1) + "_NV_fixed_joint_lump__link" + to_string(order) + "_collision";
  }
  // Fixed joint  chaned order to count
  if(counter < part_count - 1){ // == is not the last part
    //Create fixed Joint
    modrob_simulation::JointDescription fixedJoint;

    if(order == 0){
      fixedJoint.parent_link = "part0";
      fixedJoint.name = "part0_joint" + to_string(jointCount);
    }else{
      fixedJoint.parent_link = "link" + to_string(order);
      fixedJoint.name = "link" + to_string(order) + "_" + "joint" + to_string(jointCount);
    }
    fixedJoint.child_link = "proximal_link_of_joint" + to_string(jointCount);
    fixedJoint.type             = "fixed"; 
    fixedJoint.origin_x         = coordinates.x;
    fixedJoint.origin_y         = coordinates.y;
    fixedJoint.origin_z         = coordinates.z;
    fixedJoint.origin_r         = coordinates.r;
    fixedJoint.origin_p         = coordinates.p;
    fixedJoint.origin_yy        = coordinates.yy;

    //Add everything to robotDescription
    robotDescription.joints.push_back(fixedJoint);

    
  }
  else{ //Create dummy object for tool
    //Create fixed Joint
    modrob_simulation::JointDescription fixedJoint;
    fixedJoint.parent_link      = "link" + to_string(order);
    fixedJoint.child_link       = "tool_dummy";
    fixedJoint.name             = "link" + to_string(order) + "_" + "tool_dummy";
    fixedJoint.type             = "fixed"; 
    fixedJoint.origin_x         = coordinates.x;
    fixedJoint.origin_y         = coordinates.y;
    fixedJoint.origin_z         = coordinates.z;
    fixedJoint.origin_r         = coordinates.r;
    fixedJoint.origin_p         = coordinates.p;
    fixedJoint.origin_yy        = coordinates.yy;


    modrob_simulation::LinkDescription tool_dummy;  
    tool_dummy.origin_x     = stof(row[Database::r_com_pl_x]);
    tool_dummy.origin_y     = stof(row[Database::r_com_pl_y]);
    tool_dummy.origin_z     = stof(row[Database::r_com_pl_z]);
    tool_dummy.name         = "tool_dummy";
    robotDescription.links.push_back(tool_dummy);
    

    //Add everything to robotDescription
    robotDescription.joints.push_back(fixedJoint);
  }
  //Add everything to robotDescription
  link.link_visual.push_back(visual);
  link.link_collision.push_back(collision);
  if (has_sensor){
    link.link_sensor.push_back(sensor);
  }
  robotDescription.links.push_back(link);
  counter++;
  order++;
}









// #############################################
// ######   Create Motor Description    ########
// #############################################

void createMotorDescription(CSVRow row){

  // Create proximal link
  modrob_simulation::LinkDescription link_proximal;  
  link_proximal.origin_x     = stof(row[Database::r_com_pl_x]);
  link_proximal.origin_y     = stof(row[Database::r_com_pl_y]);
  link_proximal.origin_z     = stof(row[Database::r_com_pl_z]);
  link_proximal.origin_r     = 0;
  link_proximal.origin_p     = 0;
  link_proximal.origin_yy    = 0;
  link_proximal.name         = "proximal_link_of_joint" + to_string(jointCount);
  link_proximal.mass         = stof(row[Database::m_pl]);
  link_proximal.intertia_ixx = stof(row[Database::I_pl_xx]);
  link_proximal.intertia_ixy = stof(row[Database::I_pl_xy]);
  link_proximal.intertia_ixz = stof(row[Database::I_pl_xz]);   
  link_proximal.intertia_iyy = stof(row[Database::I_pl_yy]);
  link_proximal.intertia_iyz = stof(row[Database::I_pl_yz]);
  link_proximal.intertia_izz = stof(row[Database::I_dl_zz]);

  // friction values
  link_proximal.kp         = 10000;//Stiffness
  link_proximal.kd         = 10;//Damping
  link_proximal.mu1        = 0.2;//link friction
  link_proximal.mu2        = 0.2;//link friction

  //Create visual
  modrob_simulation::LinkVisual visual;
  Eigen::Matrix4d joint_transform;
  rvizXYZRPYY proximal_coordinates, distal_coordinates;
  rvizXYZRPYY coordinates = convertDatabaseCoordinatesToRvizCoordinates(row[Database::ID],row[Database::a_pl],row[Database::p_pl],row[Database::n_pl],row[Database::a_dl],row[Database::n_dl],row[Database::p_dl],row[Database::alpha_pl],row[Database::delta_pl],row[Database::delta_dl],row[Database::alpha_dl], proximal_coordinates, distal_coordinates, joint_transform);
  visual.origin_x   = coordinates.x; 
  visual.origin_y   = coordinates.y; 
  visual.origin_z   = coordinates.z; 
  visual.origin_r   = coordinates.r; 
  visual.origin_p   = coordinates.p; 
  visual.origin_yy  = coordinates.yy;
  std::string low_poly_file = stl_file_path + row[Database::Name] + "_low_poly.STL";
  if ( access( low_poly_file.c_str(), F_OK ) != -1 ){
    visual.type = row[Database::Name] + "_low_poly.STL";
  } else {
    visual.type       = row[Database::Name] + ".STL";
  }
  visual.color_name = "joint_color-" + to_string(jointCount);
  visual.color_r    = robot_motor_color_r;
  visual.color_g    = robot_motor_color_g;
  visual.color_b    = robot_motor_color_b;
  visual.color_a    = robot_motor_color_a;
  
  // add gazebo material
  visual.gazebo_material = "Gazebo/Blue";

  //Create collision
  modrob_simulation::LinkCollision collision;
  rvizXYZRPYY collision_coordinates;
  if (row.size() > Database::collision_size_x){
    collision.type = "box";
    collision.arg1 = stof(row[Database::collision_size_x]);
    collision.arg2 = stof(row[Database::collision_size_y]);
    collision.arg3 = stof(row[Database::collision_size_z]);
    Eigen::Matrix4d transform_coll_pos;
    transform_coll_pos << 1, 0, 0, stof(row[Database::collision_pos_x]),
                          0, 1, 0, stof(row[Database::collision_pos_y]),
                          0, 0, 1, stof(row[Database::collision_pos_z]),
                          0, 0, 0, 1;
    Eigen::Matrix4d collision_transform = joint_transform * transform_coll_pos;
    collision_coordinates = transformationMatrix2RVIZ(collision_transform);
  } else {
    collision_coordinates = coordinates;
    std::string low_poly_file = stl_file_path + row[Database::Name] + "_low_poly.STL";
    if ( access( low_poly_file.c_str(), F_OK ) != -1 ) {
      collision.type = row[Database::Name] + "_low_poly.STL";
    } else {
      collision.type = row[Database::Name] + ".STL";
    }
  }
  collision.origin_x   = collision_coordinates.x;
  collision.origin_y   = collision_coordinates.y;
  collision.origin_z   = collision_coordinates.z;
  collision.origin_r   = collision_coordinates.r;
  collision.origin_p   = collision_coordinates.p;
  collision.origin_yy  = collision_coordinates.yy;

  // Add a contact sensor
  // Don't add contact sensor to the proximal link of joint 0 since it is still fixed with the base.
  modrob_simulation::LinkSensor sensor;
  bool has_sensor = false;
  if (jointCount > 0) {
    has_sensor = true;
    sensor.name = "proximal_link_of_joint" + to_string(jointCount) + "_contact_sensor";
    sensor.always_on = always_on;
    sensor.plugin_name = plugin_name;
    sensor.plugin_file = plugin_file;
    sensor.collision = "distal_link_of_joint" + to_string(jointCount-1) + "_NV_fixed_joint_lump__proximal_link_of_joint" + to_string(jointCount) + "_collision_1";
  }
  //Increase order removed !!
  //order++;

  //Create distal link
  modrob_simulation::LinkDescription link_distal;  
  link_distal.origin_x     = stof(row[Database::r_com_dl_x]);
  link_distal.origin_y     = stof(row[Database::r_com_dl_y]);
  link_distal.origin_z     = stof(row[Database::r_com_dl_z]);
  link_distal.origin_r     = 0;
  link_distal.origin_p     = 0;
  link_distal.origin_yy    = 0;
  link_distal.name         = "distal_link_of_joint" + to_string(jointCount) + "_NV";
  link_distal.mass         = stof(row[Database::m_dl]);
  link_distal.intertia_ixx = stof(row[Database::I_dl_xx]);
  link_distal.intertia_ixy = stof(row[Database::I_dl_xy]);
  link_distal.intertia_ixz = stof(row[Database::I_dl_xz]);   
  link_distal.intertia_iyy = stof(row[Database::I_dl_yy]);
  link_distal.intertia_iyz = stof(row[Database::I_dl_yz]);
  link_distal.intertia_izz = stof(row[Database::I_dl_zz]);

  // friction values
  link_proximal.kp         = 10000;//Stiffness
  link_proximal.kd         = 10;//Damping
  link_proximal.mu1        = 0.2;//link friction
  link_proximal.mu2        = 0.2;//link friction

  // Create moving joint between proximal and distal
  modrob_simulation::JointDescription movingJoint;
  movingJoint.parent_link      = "proximal_link_of_joint" + to_string(jointCount);
  movingJoint.child_link       = "distal_link_of_joint" + to_string(jointCount) + "_NV";
  movingJoint.name             = "joint" + to_string(jointCount);
  movingJoint.type             = "revolute"; 
  movingJoint.origin_x         = proximal_coordinates.x; 
  movingJoint.origin_y         = proximal_coordinates.y; 
  movingJoint.origin_z         = proximal_coordinates.z; //Put joint in the middle of motor
  movingJoint.origin_r         = proximal_coordinates.r; 
  movingJoint.origin_p         = proximal_coordinates.p; 
  movingJoint.origin_yy        = proximal_coordinates.yy; 
  movingJoint.axis_x           = 0;
  movingJoint.axis_y           = 0;
  movingJoint.axis_z           = 1;
  movingJoint.lower            = stof(row[Database::Ljl]);
  movingJoint.upper            = stof(row[Database::Ujl]);
  movingJoint.effort           = stof(row[Database::tau_lim]); // changed to tau_lim from ddq_lim
  movingJoint.velocity         = stof(row[Database::dq_lim]);//1.57;                           // set to 1.57 rad/s temporarily disabled the inference of this parameter since it is zero in the module database: stof(row[Database::dq_lim]);
  movingJoint.gear_ratio       = stof(row[Database::k_r]);     // added gear ratios for transmmissions in ros_control

  movingJoint.friction         = stof(row[Database::jbc]); //see Issue#4 Modules.csv friction coulomb --> s
  movingJoint.damping          = stof(row[Database::jbv]);; // Value taken from Althoff paper  

  // Fixed joint at the end of motor
  if(counter < part_count - 1){ // == is not the last part
    //Create fixed Joint
    modrob_simulation::JointDescription fixedJoint;
    fixedJoint.parent_link      = "distal_link_of_joint" + to_string(jointCount) + "_NV";
    fixedJoint.child_link       = "link" + to_string(order);
    fixedJoint.name             = "joint" + to_string(jointCount) + "_" + "link" + to_string(order);
    fixedJoint.type             = "fixed"; 
    fixedJoint.origin_x         = distal_coordinates.x; 
    fixedJoint.origin_y         = distal_coordinates.y;
    fixedJoint.origin_z         = distal_coordinates.z; //WAS: Half because we need to go from middle of motor (moving joint) to end of motor
    fixedJoint.origin_r         = distal_coordinates.r; 
    fixedJoint.origin_p         = distal_coordinates.p; 
    fixedJoint.origin_yy        = distal_coordinates.yy; 

    //Add fixedJoint to robot description
    robotDescription.joints.push_back(fixedJoint);
  }
  else{ //Create dummy object for tool
    //Create fixed Joint
    modrob_simulation::JointDescription fixedJoint;
    fixedJoint.parent_link      = "distal_link_of_joint" + to_string(jointCount) + "_NV";
    fixedJoint.child_link       = "tool_dummy";
    fixedJoint.name             = "joint" + to_string(jointCount) + "_" + "tool_dummy";
    fixedJoint.type             = "fixed"; 
    fixedJoint.origin_x         = distal_coordinates.x;
    fixedJoint.origin_y         = distal_coordinates.y;
    fixedJoint.origin_z         = distal_coordinates.z;
    fixedJoint.origin_r         = distal_coordinates.r;
    fixedJoint.origin_p         = distal_coordinates.p;
    fixedJoint.origin_yy        = distal_coordinates.yy;


    modrob_simulation::LinkDescription tool_dummy;  
    tool_dummy.origin_x     = stof(row[Database::r_com_pl_x]);
    tool_dummy.origin_y     = stof(row[Database::r_com_pl_y]);
    tool_dummy.origin_z     = stof(row[Database::r_com_pl_z]);
    tool_dummy.name         = "tool_dummy";
    robotDescription.links.push_back(tool_dummy);


    //Add everything to robotDescription
    robotDescription.joints.push_back(fixedJoint);
  }

  //Add everything to robot description
  link_proximal.link_visual.push_back(visual);
  link_proximal.link_collision.push_back(collision);
  if (has_sensor){
    link_proximal.link_sensor.push_back(sensor);
  }
  robotDescription.links.push_back(link_distal);
  robotDescription.links.push_back(link_proximal);
  robotDescription.joints.push_back(movingJoint);


  //Increase joint count
  jointCount++; 
  counter++;
}


//##########################################################
//###   Utility functions for coordinate transformation  ###
//##########################################################

// Functions adapted from augmented reality team
Eigen::Matrix4d T(Axis axis, double d)
{
  Eigen::Matrix4d trans;
  trans <<  1, 0 , 0, axis == Axis::x ? d : 0,
            0, 1 , 0, axis == Axis::y ? d : 0,
            0, 0 , 1, axis == Axis::z ? d : 0,
            0, 0 , 0, 1;
  return trans;
}

Eigen::Matrix4d R(Axis axis, double phi)
{
    Eigen::Matrix4d transform;
    switch (axis)
    {
        case Axis::z:
          transform <<  cos(phi) , -sin(phi) , 0, 0,
                        sin(phi) , cos(phi)  , 0, 0,
                        0        , 0         , 1, 0,
                        0        , 0         , 0, 1;
            break;
        case Axis::x:
            transform <<  1, 0        , 0         , 0,
                          0, cos(phi) , -sin(phi) , 0,
                          0, sin(phi) , cos(phi)  , 0,
                          0, 0        , 0         , 1;
            break;

        case Axis::y:
            transform <<  cos(phi)  , 0  , sin(phi)  , 0,
                          0         , 1  , 0         , 0,
                          -sin(phi) , 0  , cos(phi)  , 0,
                          0         , 0  , 0         , 1;
            break;
    }
    return transform;
}


Eigen::Matrix4d Rz(double d){
  return R(Axis::z, d);
}
Eigen::Matrix4d Rx(double d)
{
    return R(Axis::x, d);
}
Eigen::Matrix4d Tz(double d)
{
    return T(Axis::z, d);
}
Eigen::Matrix4d Tx(double d)
{
    return T(Axis::x, d);
}



// #####################################################
// ######   Translate Database to coordinates   ########
// #####################################################
rvizXYZRPYY convertDatabaseCoordinatesToRvizCoordinates(string id,string a,string p,string n,string ad, string nd, string pd, string al_pl, string de_pl, string de_dl, string al_dl){
  rvizXYZRPYY _1, _2;
  return convertDatabaseCoordinatesToRvizCoordinates(id, a, p, n, ad, nd, pd, al_pl, de_pl, de_dl, al_dl, _1, _2);
}

rvizXYZRPYY convertDatabaseCoordinatesToRvizCoordinates(string id,string a,string p,string n,string ad, string nd, string pd, string al_pl, string de_pl, string de_dl, string al_dl, rvizXYZRPYY& proximal_trafo, rvizXYZRPYY& distal_trafo){
  Eigen::Matrix4d _1;
  return convertDatabaseCoordinatesToRvizCoordinates(id, a, p, n, ad, nd, pd, al_pl, de_pl, de_dl, al_dl, proximal_trafo, distal_trafo, _1);
}

rvizXYZRPYY convertDatabaseCoordinatesToRvizCoordinates(string id,string a,string p,string n,string ad, string nd, string pd, string al_pl, string de_pl, string de_dl, string al_dl, rvizXYZRPYY& proximal_trafo, rvizXYZRPYY& distal_trafo, Eigen::Matrix4d& transform){

  double a_pl = stof(a);
  double p_pl = stof(p);
  double n_pl = stof(n);
  double a_dl = stof(ad);
  double n_dl = stof(nd);
  double p_dl = stof(pd);
  double alpha_pl = stof(al_pl);
  double delta_pl = stof(de_pl);
  double delta_dl = stof(de_dl);
  double alpha_dl = stof(al_dl);

  ROS_INFO("convertDatabaseCoordinatesToRvizCoordinates id, a_pl, p_pl, n_pl, a_dl, n_dl, p_dl, alpha_pl, delta_pl, alpha_dl, delta_dl");
  ROS_INFO_STREAM("" << id << ", " << a_pl << ", " << p_pl << ", " << n_pl << ", " << a_dl << ", " << n_dl << ", " << p_dl << ", " << alpha_pl << ", " << delta_pl << ", " << alpha_dl << ", " << delta_dl);
  // Multiplications adapted from augmented reality team
  Eigen::Matrix4d A_dj_j_min_1;
  Eigen::Matrix4d A_pl_j_plus_k;
  A_dj_j_min_1 = Tz(-p_dl) * Tx(a_dl) * Rx(alpha_dl) * Tz(n_dl) * Rz(delta_dl);
  A_pl_j_plus_k = Rz(-delta_pl) * Tz(-p_pl) * Tx(a_pl) * Rx(alpha_pl) * Tz(n_pl);
  transform =   A_pl_j_plus_k * A_dj_j_min_1;
  ROS_INFO_STREAM("Transformation matrix: \n" << 
                  transform(0,0) << " " << transform(0, 1) << " " << transform(0, 2) << " " << transform(0, 3) << "\n" <<
                  transform(1,0) << " " << transform(1, 1) << " " << transform(1, 2) << " " << transform(1, 3) << "\n" <<
                  transform(2,0) << " " << transform(2, 1) << " " << transform(2, 2) << " " << transform(2, 3) << "\n" <<
                  transform(3,0) << " " << transform(3, 1) << " " << transform(3, 2) << " " << transform(3, 3));

  proximal_trafo = transformationMatrix2RVIZ(A_pl_j_plus_k);
  distal_trafo = transformationMatrix2RVIZ(A_dj_j_min_1);
  rvizXYZRPYY coordinates = transformationMatrix2RVIZ(transform);
  return coordinates;
}


///// Convert a 4d transformation matrix into x,y,z + roll, pitch, yaw values
rvizXYZRPYY transformationMatrix2RVIZ(const Eigen::Matrix4d& transform){
  rvizXYZRPYY coordinates;
  // Calculate RPY
  Eigen::Matrix3d rotationMatrix;
  for (int i = 0; i < 3; i++)
  {
    for (int k = 0; k < 3; k++)
    {
      rotationMatrix(i,k) = transform(i,k);
    }
  }
  Eigen::Vector3d angles = rotationMatrix.eulerAngles(2,1,0); 
  // Fill Rviz coordinates
  coordinates.x = transform(0,3);
  coordinates.y = transform(1,3);
  coordinates.z = transform(2,3);
  coordinates.r = angles(2);
  coordinates.p = angles(1);
  coordinates.yy = angles(0);
  return coordinates;
}