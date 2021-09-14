//Code for coordinate calculation adapted from augmented reality them: dh_transformer

//gen2 json database support added by
//Author: Maximilian Körsten (ge46hug@mytum.de)

#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include <string>
#include <sstream>
#include "robot_description_publisher.hpp"
#include <math.h>
#include "Eigen/Dense"
#include "json_database_parser.hpp"

using namespace std;

enum Axis
{
	x,
	z,
	y
};

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

//Global variables
ros::Publisher publisher; //To be accessible in the listener function
std::string database_path;
int part_count;
int order;
int jointCount;
modrob_workstation::ModuleOrder lastModuleOrder;
modrob_workstation::RobotDescription robotDescription;

JSON_Database_Parser *gen2_parser;
int lastGen;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "robot_description_publisher");
	ros::NodeHandle n("~");
	publisher = n.advertise<modrob_workstation::RobotDescription>("/ns/robot_description", 1000);
	ros::Subscriber subcriberModuleOrder = n.subscribe("/ns/robot_module_order", 1000, listenForModuleOrderAndGenerateAndPublishRobotDescriptionMessage);

	// Create private rosparameter "~generation", if it doesn't exist yet
	if (!n.hasParam("generation"))
	{
		n.setParam("generation", DEFAULT_GENERATION);
		ROS_INFO_STREAM("Generation set to " << DEFAULT_GENERATION << " by default. If a different robot is used, please change the parameter accordingly!");
	}
	n.getParam("generation", lastGen);

	database_path = ros::package::getPath("modrob_resources");
	database_path.append("/stl_files/module_db/Modules.csv");
	ros::spin();
	return 0;
}

void listenForModuleOrderAndGenerateAndPublishRobotDescriptionMessage(const modrob_workstation::ModuleOrder::ConstPtr &module_order)
{
	ros::NodeHandle n("~");
	int selectedGen = 0;
	n.getParam("generation", selectedGen);
	// Check for valid generation parameter
	if(selectedGen != 1 && selectedGen != 2) {
		ROS_WARN("Generation parameter was set to invalid value. Default generation will be used!");
		selectedGen = DEFAULT_GENERATION;
	}

	//Check, if module order is different to last one
	if (lastModuleOrder.modules.size() == module_order->modules.size())
	{
		bool isEqual = true;
		for (int i = 0; i < module_order->modules.size(); i++)
		{
			if (lastModuleOrder.modules[i] != module_order->modules[i])
			{
				isEqual = false;
				break;
			}
		}
		if (isEqual)
		{

			if(lastGen == selectedGen)
				return;
		}
	}

	modrob_workstation::RobotDescription newRobotDescription;
	robotDescription = newRobotDescription;

	// Header
	robotDescription.header.stamp = ros::Time::now();
	robotDescription.header.frame_id = "0";
	robotDescription.name = "modular-robot";

	part_count = module_order->modules.size();
	order = 0;
	jointCount = 0;
	modrob_workstation::ModuleOrder newModuleOrder;

	// Parse ID for each part id
	for (int i = 0; i < module_order->modules.size(); i++)
	{
		newModuleOrder.modules.push_back(module_order->modules[i]);
		switch (selectedGen)
		{
		case 2:
			//Create the parser if it hasn't been created yet (this ensures that the json database is only searched if gen2 is selected)
			if(!gen2_parser)
				gen2_parser = new JSON_Database_Parser();
				
			if(!( gen2_parser->Parse_ID(robotDescription, part_count, jointCount, order, module_order->modules[i]) ))
			{
				//Parse of part failed. Therefore creating the robotdescription failed!
				ROS_ERROR_STREAM("Robot Description could not be created, because the part at order "<<order<<" could not be created!"<<std::endl);
				//Stop the parse function
				return;
			}
			break;
		case 1:
		default:
			parseID(module_order->modules[i]);
			break;
		}
		order++;
	}

	lastModuleOrder = newModuleOrder;
	//Info, publish and loop
	publisher.publish(robotDescription);
	ROS_INFO("Published Robot description");

	// Set /nrJoints parameter for other Modules to see
	ros::param::set("/nrJoints", jointCount);
	// Update lastgen
	lastGen = selectedGen;
}

void parseID(int id)
{

	// ### Find database row, which corresponds to the id
	std::ifstream file(database_path);
	CSVRow row;
	file >> row; //Skipping first row (attributes-names)
	while (file >> row)
	{ //Searching for line with matching id
		if (std::stoi(row[Database::ID]) == id)
			break;
	}
	ROS_INFO("Found %s with id %d", row[0].c_str(), id);
	if (!row[Database::Name].compare(""))
		ROS_ERROR("ID not found in database %d!", id);

	// ### Create urdf elements from row data
	switch (std::stoi(row[Database::typ]))
	{
	case LINK_TYPE:
		createLinkDescription(row);
		break;
	case MOTOR_TYPE:
		part_count++; //Motor consistes of 2 parts
		createMotorDescription(row);
		break;
	case BASE_TYPE:
		createLinkDescription(row);
		break;
	case END_EFFECTOR_TYPE:
		ROS_WARN("End effector does not exist in database");
		break;
	default:
		ROS_ERROR("Typ values of %s is not defined in database", row[Database::Name].c_str());
	}
}

// ############################################
// ######   Create Link Description    ########
// ############################################

void createLinkDescription(CSVRow row)
{
	// Link values
	modrob_workstation::LinkDescription link;
	link.origin_x = stof(row[Database::r_com_pl_x]);
	link.origin_y = stof(row[Database::r_com_pl_y]);
	link.origin_z = stof(row[Database::r_com_pl_z]);
	link.origin_r = 0;
	link.origin_p = 0;
	link.origin_yy = 0;
	link.name = "part" + to_string(order);
	link.mass = stof(row[Database::m_pl]);
	link.intertia_ixx = stof(row[Database::I_pl_xx]);
	link.intertia_ixy = stof(row[Database::I_pl_xy]);
	link.intertia_ixz = stof(row[Database::I_pl_xz]);
	link.intertia_iyy = stof(row[Database::I_pl_yy]);
	link.intertia_iyz = stof(row[Database::I_pl_yz]);
	link.intertia_izz = stof(row[Database::I_dl_zz]);

	// Visual values
	rvizXYZRPYY coordinates = convertDatabaseCoordinatesToRvizCoordinates(row[Database::ID], row[Database::a_pl], row[Database::p_pl], row[Database::n_pl], row[Database::a_dl], row[Database::n_dl], row[Database::p_dl], row[Database::alpha_pl], row[Database::delta_pl], row[Database::delta_dl], row[Database::alpha_dl]);
	modrob_workstation::LinkVisual visual;
	visual.origin_x = coordinates.x;
	visual.origin_y = coordinates.y;
	visual.origin_z = coordinates.z;
	visual.origin_r = coordinates.r;
	visual.origin_p = coordinates.p;
	visual.origin_yy = coordinates.yy;
	visual.type = "stl_files/" + row[Database::Name] + ".STL";
	visual.color_name = "color-" + to_string(order);
	visual.color_r = robot_link_color_r;
	visual.color_g = robot_link_color_g;
	visual.color_b = robot_link_color_b;
	visual.color_a = robot_link_color_a;

	// Collision values
	modrob_workstation::LinkCollision collision;
	collision.origin_x = coordinates.x;
	collision.origin_y = coordinates.y;
	collision.origin_z = coordinates.z;
	collision.origin_r = coordinates.r;
	collision.origin_p = coordinates.p;
	collision.origin_yy = coordinates.yy;
	collision.type = "stl_files/" + row[Database::Name] + ".STL";

	// Fixed joint
	if (order < part_count - 1)
	{ // == is not the last part
		//Create fixed Joint
		modrob_workstation::JointDescription fixedJoint;
		fixedJoint.parent_link = "part" + to_string(order);
		fixedJoint.child_link = "part" + to_string(order + 1);
		fixedJoint.name = "part" + to_string(order) + "_" + "part" + to_string(order + 1);
		fixedJoint.type = "fixed";
		fixedJoint.origin_x = coordinates.x;
		fixedJoint.origin_y = coordinates.y;
		fixedJoint.origin_z = coordinates.z;
		fixedJoint.origin_r = coordinates.r;
		fixedJoint.origin_p = coordinates.p;
		fixedJoint.origin_yy = coordinates.yy;

		//Add everything to robotDescription
		robotDescription.joints.push_back(fixedJoint);
	}
	else
	{ //Create dummy object for tool
		//Create fixed Joint
		modrob_workstation::JointDescription fixedJoint;
		fixedJoint.parent_link = "part" + to_string(order);
		fixedJoint.child_link = "tool_dummy";
		fixedJoint.name = "part" + to_string(order) + "_" + "tool_dummy";
		fixedJoint.type = "fixed";
		fixedJoint.origin_x = coordinates.x;
		fixedJoint.origin_y = coordinates.y;
		fixedJoint.origin_z = coordinates.z;
		fixedJoint.origin_r = coordinates.r;
		fixedJoint.origin_p = coordinates.p;
		fixedJoint.origin_yy = coordinates.yy;

		modrob_workstation::LinkDescription tool_dummy;
		tool_dummy.origin_x = stof(row[Database::r_com_pl_x]);
		tool_dummy.origin_y = stof(row[Database::r_com_pl_y]);
		tool_dummy.origin_z = stof(row[Database::r_com_pl_z]);
		tool_dummy.name = "tool_dummy";
		robotDescription.links.push_back(tool_dummy);

		//Add everything to robotDescription
		robotDescription.joints.push_back(fixedJoint);
	}

	//Add everything to robotDescription
	link.link_visual.push_back(visual);
	link.link_collision.push_back(collision);
	robotDescription.links.push_back(link);
}

// #############################################
// ######   Create Motor Description    ########
// #############################################

void createMotorDescription(CSVRow row)
{

	// Create proximal link
	modrob_workstation::LinkDescription link_proximal;
	link_proximal.origin_x = stof(row[Database::r_com_pl_x]);
	link_proximal.origin_y = stof(row[Database::r_com_pl_y]);
	link_proximal.origin_z = stof(row[Database::r_com_pl_z]);
	link_proximal.origin_r = 0;
	link_proximal.origin_p = 0;
	link_proximal.origin_yy = 0;
	link_proximal.name = "part" + to_string(order);
	link_proximal.mass = stof(row[Database::m_pl]);
	link_proximal.intertia_ixx = stof(row[Database::I_pl_xx]);
	link_proximal.intertia_ixy = stof(row[Database::I_pl_xy]);
	link_proximal.intertia_ixz = stof(row[Database::I_pl_xz]);
	link_proximal.intertia_iyy = stof(row[Database::I_pl_yy]);
	link_proximal.intertia_iyz = stof(row[Database::I_pl_yz]);
	link_proximal.intertia_izz = stof(row[Database::I_dl_zz]);

	//Create visual
	modrob_workstation::LinkVisual visual;
	rvizXYZRPYY coordinates = convertDatabaseCoordinatesToRvizCoordinates(row[Database::ID], row[Database::a_pl], row[Database::p_pl], row[Database::n_pl], row[Database::a_dl], row[Database::n_dl], row[Database::p_dl], row[Database::alpha_pl], row[Database::delta_pl], row[Database::delta_dl], row[Database::alpha_dl]);
	visual.origin_x = coordinates.x;
	visual.origin_y = coordinates.y;
	visual.origin_z = coordinates.z;
	visual.origin_r = coordinates.r;
	visual.origin_p = coordinates.p;
	visual.origin_yy = coordinates.yy;
	visual.type = "stl_files/" + row[Database::Name] + ".STL";
	visual.color_name = "color-" + to_string(order);
	visual.color_r = robot_motor_color_r;
	visual.color_g = robot_motor_color_g;
	visual.color_b = robot_motor_color_b;
	visual.color_a = robot_motor_color_a;

	//Create collision
	modrob_workstation::LinkCollision collision;
	collision.origin_x = coordinates.x;
	collision.origin_y = coordinates.y;
	collision.origin_z = coordinates.z;
	collision.origin_r = coordinates.r;
	collision.origin_p = coordinates.p;
	collision.origin_yy = coordinates.yy;
	collision.type = "stl_files/" + row[Database::Name] + ".STL";

	//Increase order
	order++;

	//Create distal link
	modrob_workstation::LinkDescription link_distal;
	link_distal.origin_x = stof(row[Database::r_com_dl_x]);
	link_distal.origin_y = stof(row[Database::r_com_dl_y]);
	link_distal.origin_z = stof(row[Database::r_com_dl_z]);
	link_distal.origin_r = 0;
	link_distal.origin_p = 0;
	link_distal.origin_yy = 0;
	link_distal.name = "part" + to_string(order);
	link_distal.mass = stof(row[Database::m_dl]);
	link_distal.intertia_ixx = stof(row[Database::I_dl_xx]);
	link_distal.intertia_ixy = stof(row[Database::I_dl_xy]);
	link_distal.intertia_ixz = stof(row[Database::I_dl_xz]);
	link_distal.intertia_iyy = stof(row[Database::I_dl_yy]);
	link_distal.intertia_iyz = stof(row[Database::I_dl_yz]);
	link_distal.intertia_izz = stof(row[Database::I_dl_zz]);

	// Create moving joint between proximal and distal
	modrob_workstation::JointDescription movingJoint;
	movingJoint.parent_link = "part" + to_string(order - 1);
	movingJoint.child_link = "part" + to_string(order);
	movingJoint.name = "joint" + to_string(jointCount);
	movingJoint.type = "revolute";
	movingJoint.origin_x = coordinates.x * 0.5;
	movingJoint.origin_y = coordinates.y * 0.5;
	movingJoint.origin_z = coordinates.z * 0.5; //Put joint in the middle of motor
	movingJoint.origin_r = coordinates.r;
	movingJoint.origin_p = coordinates.p;
	movingJoint.origin_yy = coordinates.yy;
	movingJoint.axis_x = 0;
	movingJoint.axis_y = 0;
	movingJoint.axis_z = 1;
	movingJoint.lower = stof(row[Database::Ljl]);
	movingJoint.upper = stof(row[Database::Ujl]);
	movingJoint.effort = stof(row[Database::ddq_lim]);
	movingJoint.velocity = stof(row[Database::dq_lim]);

	// Fixed joint at the end of motor
	if (order < part_count - 1)
	{ // == is not the last part
		//Create fixed Joint
		modrob_workstation::JointDescription fixedJoint;
		fixedJoint.parent_link = "part" + to_string(order);
		fixedJoint.child_link = "part" + to_string(order + 1);
		fixedJoint.name = "part" + to_string(order) + "_" + "part" + to_string(order + 1);
		fixedJoint.type = "fixed";
		fixedJoint.origin_x = coordinates.x * 0.5;
		fixedJoint.origin_y = coordinates.y * 0.5;
		fixedJoint.origin_z = coordinates.z * 0.5; //Half because we need to go from middle of motor (moving joint) to end of motor
		fixedJoint.origin_r = coordinates.r;
		fixedJoint.origin_p = coordinates.p;
		fixedJoint.origin_yy = coordinates.y;

		//Add fixedJoint to robot description
		robotDescription.joints.push_back(fixedJoint);
	}
	else
	{ //Create dummy object for tool
		//Create fixed Joint
		modrob_workstation::JointDescription fixedJoint;
		fixedJoint.parent_link = "part" + to_string(order);
		fixedJoint.child_link = "tool_dummy";
		fixedJoint.name = "part" + to_string(order) + "_" + "tool_dummy";
		fixedJoint.type = "fixed";
		fixedJoint.origin_x = coordinates.x;
		fixedJoint.origin_y = coordinates.y;
		fixedJoint.origin_z = coordinates.z;
		fixedJoint.origin_r = coordinates.r;
		fixedJoint.origin_p = coordinates.p;
		fixedJoint.origin_yy = coordinates.yy;

		modrob_workstation::LinkDescription tool_dummy;
		tool_dummy.origin_x = stof(row[Database::r_com_pl_x]);
		tool_dummy.origin_y = stof(row[Database::r_com_pl_y]);
		tool_dummy.origin_z = stof(row[Database::r_com_pl_z]);
		tool_dummy.name = "tool_dummy";
		robotDescription.links.push_back(tool_dummy);

		//Add everything to robotDescription
		robotDescription.joints.push_back(fixedJoint);
	}

	//Add everything to robot description
	link_proximal.link_visual.push_back(visual);
	link_proximal.link_collision.push_back(collision);
	robotDescription.links.push_back(link_distal);
	robotDescription.links.push_back(link_proximal);
	robotDescription.joints.push_back(movingJoint);

	//Increase joint count
	jointCount++;
}

//##########################################################
//###   Utility functions for coordinate transformation  ###
//##########################################################

// Functions adapted from augmented reality team
Eigen::Matrix4d T(Axis axis, double d)
{
	Eigen::Matrix4d trans;
	trans << 1, 0, 0, axis == Axis::x ? d : 0,
		0, 1, 0, axis == Axis::y ? d : 0,
		0, 0, 1, axis == Axis::z ? d : 0,
		0, 0, 0, 1;
	return trans;
}

Eigen::Matrix4d R(Axis axis, double phi)
{
	Eigen::Matrix4d transform;
	switch (axis)
	{
	case Axis::z:
		transform << cos(phi), -sin(phi), 0, 0,
			sin(phi), cos(phi), 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;
		break;
	case Axis::x:
		transform << 1, 0, 0, 0,
			0, cos(phi), -sin(phi), 0,
			0, sin(phi), cos(phi), 0,
			0, 0, 0, 1;
		break;

	case Axis::y:
		transform << cos(phi), 0, sin(phi), 0,
			0, 1, 0, 0,
			-sin(phi), 0, cos(phi), 0,
			0, 0, 0, 1;
		break;
	}
	return transform;
}

Eigen::Matrix4d Rz(double d)
{
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

rvizXYZRPYY convertDatabaseCoordinatesToRvizCoordinates(string id, string a, string p, string n, string ad, string nd, string pd, string al_pl, string de_pl, string de_dl, string al_dl)
{

	using namespace Eigen;

	rvizXYZRPYY coordinates;

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

	// Multiplications adapted from augmented reality team
	Matrix4d A_dj_j_min_1;
	Matrix4d A_pl_j_plus_k;
	A_dj_j_min_1 = Tz(-p_dl) * Tx(a_dl) * Rx(alpha_dl) * Tz(n_dl) * Rz(delta_dl);
	A_pl_j_plus_k = Rz(-delta_pl) * Tz(-p_pl) * Tx(a_pl) * Rx(alpha_pl) * Tz(n_pl);
	Matrix4d transform = A_pl_j_plus_k * A_dj_j_min_1;

	Eigen::Matrix3d rotationMatrix;
	for (int i = 0; i < 3; i++)
	{
		for (int k = 0; k < 3; k++)
		{
			rotationMatrix(i, k) = transform(i, k);
		}
	}

	Eigen::Vector3d angles = rotationMatrix.eulerAngles(2, 1, 0);

	coordinates.x = transform(0, 3);
	coordinates.y = transform(1, 3);
	coordinates.z = transform(2, 3);
	coordinates.r = angles(2);
	coordinates.p = angles(1);
	coordinates.yy = angles(0);

	return coordinates;
}
