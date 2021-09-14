// ===============================
// AUTHOR       : Max Körsten (ge46hug@mytum.de)
// CREATE DATE  : 22.12.2020
// PURPOSE      : Read Part information from a JSON Database and fill in fields of a robot_description Message
// ===============================

#include "json_database_parser.hpp"

#include "ros/ros.h"
#include <ros/package.h>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <exception>
#include "Eigen/Dense"
#include <math.h>


// Short alias for this namespace
namespace pt = boost::property_tree;

JSON_Database_Parser::JSON_Database_Parser()
{
    this->UpdateModuleFileList();
}

bool JSON_Database_Parser::Parse_ID(modrob_workstation::RobotDescription &robot_description, int &part_count, int &joint_count, int &order, int id)
{
	// Search for id in this->moduleJSONFilesIDs
	std::list<int>::iterator idListIt = this->moduleJSONFilesIDs.begin();
	std::list<std::string>::iterator fileListIt = this->moduleJSONFiles.begin();
	
	// Advance both iterators until the id was found or the list ends
	while(idListIt != this->moduleJSONFilesIDs.end() && *idListIt != id)
	{
		std::advance(idListIt, 1);
		std::advance(fileListIt, 1);
	}
	if(idListIt == this->moduleJSONFilesIDs.end())
	{
		// no file with the correct id found
		ROS_ERROR_STREAM("No file containing part with id " << id << " found in ModuleLibrary!" << std::endl);
		return false;
	}

	// Open file at position of found id in this->moduleJSONFiles 
	std::string filename = *fileListIt;

	pt::ptree json;
	try
	{
		std::fstream file;
		file.open(filename, std::ios::in);
		pt::read_json(file, json);
		file.close();
	}
	catch(const std::exception& e)
	{
		ROS_ERROR_STREAM("Could not open json file: " << filename << std::endl << "Error Message:\n" << e.what() << std::endl);
		return false;
	}
	this->currentFile = fileListIt;
	
	//determine type (Base, Strut, Motor or End effector)
	std::string partType = json.get<std::string>("type");

	if(partType == "Base") {
		try {
			this->createLinkDescription(robot_description, json, part_count, order);
		}
		catch(std::exception &e) {
			ROS_ERROR_STREAM("JSON parse failed with message:\n" << e.what() << std::endl);
			return false;
		}
	}
	else if(partType == "RevoluteJoint") {
		try {
			part_count++; //Motor consists of 2 parts
			this->createMotorDescription(robot_description, json, part_count, joint_count, order);
		}
		catch(std::exception &e) {
			ROS_ERROR_STREAM("JSON parse failed with message:\n" << e.what() << std::endl);
			return false;
		}

	}
	else if(partType == "Strut") {
		try {
			this->createLinkDescription(robot_description, json, part_count, order);
		}
		catch(std::exception &e) {
			ROS_ERROR_STREAM("JSON parse failed with message:\n" << e.what() << std::endl);
			return false;
		}
	}
	else if(partType == "Endeffector") {
		
	}
	else {
		ROS_ERROR_STREAM("Part is of type " << partType << ". This type is unknown to the parser!");
		return false;
	}

	return true;
}

void JSON_Database_Parser::UpdateModuleFileList()
{
    using namespace boost::filesystem;

    std::string database_path = ros::package::getPath(DATABASE_PACKAGE_NAME);
	database_path.append("/ModuleLibrary/");
    this->moduleJSONFiles.clear();
    this->moduleJSONFilesIDs.clear();

    recursive_directory_iterator dir(database_path), end;
    if(dir == end)
    {
        //database_path doesn't exist
        ROS_WARN_STREAM("The gen2 json database was not found! Expected path: " << database_path << std::endl);
    }
    while (dir != end)
    {
        // find .json files
        if (dir->path().filename().extension() == ".json")
        {
            int id;
			std::string fullpath = boost::filesystem::canonical(dir->path(), path("~/")).string();

            if(this->getPartIDofJSONFile(fullpath, id))
            {
				// if the file is valid, add it to known files stored in moduleJSONFiles and add the corresponding part id to moduleJSONFilesIDs
                this->moduleJSONFiles.push_back(fullpath);
                this->moduleJSONFilesIDs.push_back(id);
            }
        }
        ++dir;
    }
    ROS_INFO_STREAM(this->moduleJSONFiles.size() << " .json files found in ModuleLibrary.");
}

void JSON_Database_Parser::createLinkDescription(modrob_workstation::RobotDescription &robot_description, pt::ptree &link_json, int part_count, int order)
{
    // Link values
	modrob_workstation::LinkDescription link;

    auto& pl_r_com = link_json.get_child("dynamics.pl_r_com");
    pt::ptree::const_iterator row = pl_r_com.begin();

	link.origin_x = row->second.get_value<double>();row++;
	link.origin_y = row->second.get_value<double>(); row++;
	link.origin_z = row->second.get_value<double>();

	link.origin_r = 0;
	link.origin_p = 0;
	link.origin_yy = 0;

	link.name = "part" + std::to_string(order);
	link.mass = link_json.get<double>("dynamics.pl_Mass");

	//Iterate through the Inertia matrix
	auto& pl_Inertia = link_json.get_child("dynamics.pl_Inertia");
	row = pl_Inertia.begin(); //row is an iterator of the row objects
	pt::ptree::const_iterator cell = row->second.begin(); //cell is an iterator over the row elements
	link.intertia_ixx = cell->second.get_value<double>(); cell++;
	link.intertia_ixy = cell->second.get_value<double>(); cell++;
	link.intertia_ixz = cell->second.get_value<double>(); row++;
	//update cell to the first element of the new row
	cell = row->second.begin(); cell++;
	link.intertia_iyy = cell->second.get_value<double>(); cell++;
	link.intertia_iyz = cell->second.get_value<double>(); row++;
	//update cell to the first element of the new row
	cell = row->second.begin(); cell++; cell ++;
	link.intertia_izz = cell->second.get_value<double>();


	// Visual values
	modrob_workstation::LinkVisual visual;
	visual.origin_x = 0;
	visual.origin_y = 0;
	visual.origin_z = 0;
	visual.origin_r = 0;
	visual.origin_p = 0;
	visual.origin_yy = 0;

	// Collision values
	modrob_workstation::LinkCollision collision;
	collision.origin_x = 0;
	collision.origin_y = 0;
	collision.origin_z = 0;
	collision.origin_r = 0;
	collision.origin_p = 0;
	collision.origin_yy = 0;
	

	// Visual and Collision types
	std::string collisiontype = link_json.get<std::string>("collisions.pl_obj.Type");
	if(collisiontype == "Mesh") {
		std::string currentFilePath = *(this->currentFile);
		std::string filePathRelativeToDatabase = currentFilePath.erase(0,((ros::package::getPath(DATABASE_PACKAGE_NAME)).size()+1));
		std::string stlFilePathRelToDatabase = boost::filesystem::path(filePathRelativeToDatabase).parent_path().string()+ "/" + link_json.get<std::string>("collisions.pl_obj.file");
		
		visual.type = stlFilePathRelToDatabase;
		collision.type = stlFilePathRelToDatabase;
	}
	else 
	{
		//Here we could add support for other types/shapes (Box, Cylinder, etc...)
		ROS_WARN_STREAM("Part (order " << order << ") has '" << collisiontype << "' defined for collision type. Only 'Mesh' is supported at the moment.\nNo visual will be created.");

	}
	visual.color_name = "color-" + std::to_string(order);
	visual.color_r = robot_link_color_r;
	visual.color_g = robot_link_color_g;
	visual.color_b = robot_link_color_b;
	visual.color_a = robot_link_color_a;

	// Fixed joint
	Eigen::Matrix4d transform;
	int x = 0;
	for (pt::ptree::value_type &row : link_json.get_child("kinematics.pl_Transformation"))
	{
		int y = 0;
		for (pt::ptree::value_type &cell : row.second)
		{
			transform(x,y) = cell.second.get_value<double>();
			y++;
		}
		x++;
	}
	Eigen::Vector3d angles = (transform.block<3,3>(0,0)).eulerAngles(2, 1, 0);

	if (order < part_count - 1)
	{ // == is not the last part
		//Create fixed Joint
		modrob_workstation::JointDescription fixedJoint;
		fixedJoint.parent_link = "part" + std::to_string(order);
		fixedJoint.child_link = "part" + std::to_string(order + 1);
		fixedJoint.name = "part" + std::to_string(order) + "_" + "part" + std::to_string(order + 1);
		fixedJoint.type = "fixed";
		fixedJoint.origin_x = transform(0, 3);
		fixedJoint.origin_y = transform(1, 3);
		fixedJoint.origin_z = transform(2, 3);
		fixedJoint.origin_r = angles(2);
		fixedJoint.origin_p = angles(1);
		fixedJoint.origin_yy = angles(0);

		//Add everything to robotDescription
		robot_description.joints.push_back(fixedJoint);
	}
	else
	{ //Create dummy object for tool
		//Create fixed Joint
		modrob_workstation::JointDescription fixedJoint;
		fixedJoint.parent_link = "part" + std::to_string(order);
		fixedJoint.child_link = "tool_dummy";
		fixedJoint.name = "part" + std::to_string(order) + "_" + "tool_dummy";
		fixedJoint.type = "fixed";
		fixedJoint.origin_x = transform(0, 3);
		fixedJoint.origin_y = transform(1, 3);
		fixedJoint.origin_z = transform(2, 3);
		fixedJoint.origin_r = angles(2);
		fixedJoint.origin_p = angles(1);
		fixedJoint.origin_yy = angles(0);

		modrob_workstation::LinkDescription tool_dummy;
		auto& pl_r_com = link_json.get_child("dynamics.pl_r_com");
    	pt::ptree::const_iterator row = pl_r_com.begin();
		tool_dummy.origin_x = row->second.get_value<double>(); row++;
		tool_dummy.origin_y = row->second.get_value<double>(); row++;
		tool_dummy.origin_z = row->second.get_value<double>();
		tool_dummy.name = "tool_dummy";
		robot_description.links.push_back(tool_dummy);

		//Add everything to robotDescription
		robot_description.joints.push_back(fixedJoint);
	}

	//Add everything to robotDescription
	link.link_visual.push_back(visual);
	link.link_collision.push_back(collision);
	robot_description.links.push_back(link);
}

void JSON_Database_Parser::createMotorDescription(modrob_workstation::RobotDescription &robot_description, pt::ptree &motor_json, int part_count, int &joint_count, int &order)
{
	// Create proximal link
	modrob_workstation::LinkDescription link_proximal;

	auto& pl_r_com = motor_json.get_child("dynamics.pl_r_com");
    pt::ptree::const_iterator row = pl_r_com.begin();

	link_proximal.origin_x = row->second.get_value<double>();row++;
	link_proximal.origin_y = row->second.get_value<double>();row++;
	link_proximal.origin_z = row->second.get_value<double>();

	link_proximal.origin_r = 0;
	link_proximal.origin_p = 0;
	link_proximal.origin_yy = 0;
	link_proximal.name = "part" + std::to_string(order);
	link_proximal.mass = motor_json.get<double>("dynamics.pl_Mass");
	
	//Iterate through the Inertia matrix
	auto& pl_Inertia = motor_json.get_child("dynamics.pl_Inertia");
	row = pl_Inertia.begin(); //row is an iterator of the row objects
	pt::ptree::const_iterator cell = row->second.begin(); //cell is an iterator over the row elements
	link_proximal.intertia_ixx = cell->second.get_value<double>(); cell++;
	link_proximal.intertia_ixy = cell->second.get_value<double>(); cell++;
	link_proximal.intertia_ixz = cell->second.get_value<double>(); row++;
	//update cell to the first element of the new row
	cell = row->second.begin(); cell++;
	link_proximal.intertia_iyy = cell->second.get_value<double>(); cell++;
	link_proximal.intertia_iyz = cell->second.get_value<double>(); row++;
	//update cell to the first element of the new row
	cell = row->second.begin(); cell++; cell ++;
	link_proximal.intertia_izz = cell->second.get_value<double>();

	//Create visual_pl
	modrob_workstation::LinkVisual visual_pl;
	visual_pl.origin_x = 0;
	visual_pl.origin_y = 0;
	visual_pl.origin_z = 0;
	visual_pl.origin_r = 0;
	visual_pl.origin_p = 0;
	visual_pl.origin_yy = 0;

	visual_pl.color_name = "color-" + std::to_string(order);
	visual_pl.color_r = robot_motor_color_r;
	visual_pl.color_g = robot_motor_color_g;
	visual_pl.color_b = robot_motor_color_b;
	visual_pl.color_a = robot_motor_color_a;

	//Create collision_pl
	modrob_workstation::LinkCollision collision_pl;
	collision_pl.origin_x = 0;
	collision_pl.origin_y = 0;
	collision_pl.origin_z = 0;
	collision_pl.origin_r = 0;
	collision_pl.origin_p = 0;
	collision_pl.origin_yy = 0;

	// Visual and Collision types
	std::string collisiontype = motor_json.get<std::string>("collisions.pl_obj.Type");
	if(collisiontype == "Mesh") {
		std::string currentFilePath = *(this->currentFile);
		std::string filePathRelativeToDatabase = currentFilePath.erase(0,((ros::package::getPath(DATABASE_PACKAGE_NAME)).size()+1));
		std::string stlFilePathRelToDatabase = boost::filesystem::path(filePathRelativeToDatabase).parent_path().string()+ "/" + motor_json.get<std::string>("collisions.pl_obj.file");
		
		visual_pl.type = stlFilePathRelToDatabase;
		collision_pl.type = stlFilePathRelToDatabase;

		link_proximal.link_visual.push_back(visual_pl);
		link_proximal.link_collision.push_back(collision_pl);
	}
	else 
	{
		//Here we could add support for other types/shapes (Box, Cylinder, etc...)
		ROS_WARN_STREAM("Part (order " << order << ") has '" << collisiontype << "' defined for collision type. Only 'Mesh' is supported at the moment.\nNo visual will be created.");

	}

	//Increase order
	order++;

	// Create moving joint between proximal and distal
	Eigen::Matrix4d transform;
	int x = 0;
	for (pt::ptree::value_type &row : motor_json.get_child("kinematics.pl_Transformation"))
	{
		int y = 0;
		for (pt::ptree::value_type &cell : row.second)
		{
			transform(x,y) = cell.second.get_value<double>();
			y++;
		}
		x++;
	}

	Eigen::Vector3d angles = (transform.block<3,3>(0,0)).eulerAngles(2, 1, 0);

	modrob_workstation::JointDescription movingJoint;
	movingJoint.parent_link = "part" + std::to_string(order - 1);
	movingJoint.child_link = "part" + std::to_string(order);
	movingJoint.name = "joint" + std::to_string(joint_count);
	movingJoint.type = "revolute";
	movingJoint.origin_x = transform(0, 3);
	movingJoint.origin_y = transform(1, 3);
	movingJoint.origin_z = transform(2, 3);
	movingJoint.origin_r = angles(2);
	movingJoint.origin_p = angles(1);
	movingJoint.origin_yy = angles(0);
	movingJoint.axis_x = 0;
	movingJoint.axis_y = 0;
	movingJoint.axis_z = 1;
	movingJoint.lower = motor_json.get<double>("limits.q_lower");
	movingJoint.upper = motor_json.get<double>("limits.q_upper");
	movingJoint.effort = M_PI;
	movingJoint.velocity = motor_json.get<double>("limits.velocity");


	//Create distal link
	modrob_workstation::LinkDescription link_distal;

	x = 0;
	for (pt::ptree::value_type &row : motor_json.get_child("kinematics.dl_Transformation"))
	{
		int y = 0;
		for (pt::ptree::value_type &cell : row.second)
		{
			transform(x,y) = cell.second.get_value<double>();
			y++;
		}
		x++;
	}
	angles = (transform.block<3,3>(0,0)).eulerAngles(2, 1, 0);



	auto& dl_r_com = motor_json.get_child("dynamics.dl_r_com");
    row = dl_r_com.begin();
	link_distal.origin_x = row->second.get_value<double>();row++;
	link_distal.origin_y = row->second.get_value<double>();row++;
	link_distal.origin_z = row->second.get_value<double>();
	link_distal.origin_r = 0;
	link_distal.origin_p = 0;
	link_distal.origin_yy = 0;
	link_distal.name = "part" + std::to_string(order);
	link_distal.mass = motor_json.get<double>("dynamics.dl_Mass");

	//Iterate through the Inertia matrix
	auto& dl_Inertia = motor_json.get_child("dynamics.dl_Inertia");
	row = dl_Inertia.begin(); //row is an iterator of the row objects
	cell = row->second.begin(); //cell is an iterator over the row elements
	link_distal.intertia_ixx = cell->second.get_value<double>(); cell++;
	link_distal.intertia_ixy = cell->second.get_value<double>(); cell++;
	link_distal.intertia_ixz = cell->second.get_value<double>(); row++;
	//update cell to the first element of the new row
	cell = row->second.begin(); cell++;
	link_distal.intertia_iyy = cell->second.get_value<double>(); cell++;
	link_distal.intertia_iyz = cell->second.get_value<double>(); row++;
	//update cell to the first element of the new row
	cell = row->second.begin(); cell++; cell ++;
	link_distal.intertia_izz = cell->second.get_value<double>();

	//Create visual_dl
	modrob_workstation::LinkVisual visual_dl;
	visual_dl.origin_x = transform(0, 3);
	visual_dl.origin_y = transform(1, 3);
	visual_dl.origin_z = transform(2, 3);
	visual_dl.origin_r = angles(2);
	visual_dl.origin_p = angles(1);
	visual_dl.origin_yy = angles(0);

	visual_dl.color_name = "color-" + std::to_string(order);
	visual_dl.color_r = robot_motor_color_r;
	visual_dl.color_g = robot_motor_color_g;
	visual_dl.color_b = robot_motor_color_b;
	visual_dl.color_a = robot_motor_color_a;

	//Create collision_pl
	modrob_workstation::LinkCollision collision_dl;
	collision_dl.origin_x = transform(0, 3);
	collision_dl.origin_y = transform(1, 3);
	collision_dl.origin_z = transform(2, 3);
	collision_dl.origin_r = angles(2);
	collision_dl.origin_p = angles(1);
	collision_dl.origin_yy = angles(0);

	// Visual and Collision types
	collisiontype = motor_json.get<std::string>("collisions.dl_obj.Type");
	if(collisiontype == "Mesh") {
		std::string currentFilePath = *(this->currentFile);
		std::string filePathRelativeToDatabase = currentFilePath.erase(0,((ros::package::getPath(DATABASE_PACKAGE_NAME)).size()+1));
		std::string stlFilePathRelToDatabase = boost::filesystem::path(filePathRelativeToDatabase).parent_path().string()+ "/" + motor_json.get<std::string>("collisions.dl_obj.file");
		
		visual_dl.type = stlFilePathRelToDatabase;
		collision_dl.type = stlFilePathRelToDatabase;

		link_distal.link_visual.push_back(visual_dl);
		link_distal.link_collision.push_back(collision_dl);
	}
	else 
	{
		//Here we could add support for other types/shapes (Box, Cylinder, etc...)
		ROS_WARN_STREAM("Part (order " << order << ") has '" << collisiontype << "' defined for collision type. Only 'Mesh' is supported at the moment.\nNo visual will be created.");

	}


	// Fixed joint at the end of motor
	if (order < part_count - 1)
	{ // == is not the last part
		//Create fixed Joint
		modrob_workstation::JointDescription fixedJoint;
		fixedJoint.parent_link = "part" + std::to_string(order);
		fixedJoint.child_link = "part" + std::to_string(order + 1);
		fixedJoint.name = "part" + std::to_string(order) + "_" + "part" + std::to_string(order + 1);
		fixedJoint.type = "fixed";
		fixedJoint.origin_x = transform(0, 3);
		fixedJoint.origin_y = transform(1, 3);
		fixedJoint.origin_z = transform(2, 3);
		fixedJoint.origin_r = angles(2);
		fixedJoint.origin_p = angles(1);
		fixedJoint.origin_yy = angles(0);

		//Add fixedJoint to robot description
		robot_description.joints.push_back(fixedJoint);
	}
	else
	{ //Create dummy object for tool
		//Create fixed Joint
		modrob_workstation::JointDescription fixedJoint;
		fixedJoint.parent_link = "part" + std::to_string(order);
		fixedJoint.child_link = "tool_dummy";
		fixedJoint.name = "part" + std::to_string(order) + "_" + "tool_dummy";
		fixedJoint.type = "fixed";
		fixedJoint.origin_x = transform(0, 3);
		fixedJoint.origin_y = transform(1, 3);
		fixedJoint.origin_z = transform(2, 3);
		fixedJoint.origin_r = angles(2);
		fixedJoint.origin_p = angles(1);
		fixedJoint.origin_yy = angles(0);

		modrob_workstation::LinkDescription tool_dummy;
		//auto& pl_r_com = motor_json.get_child("dynamics.pl_r_com");
		row = pl_r_com.begin();
		tool_dummy.origin_x = row->second.get_value<double>();row++;
		tool_dummy.origin_y = row->second.get_value<double>();row++;
		tool_dummy.origin_z = row->second.get_value<double>();
		tool_dummy.name = "tool_dummy";
		robot_description.links.push_back(tool_dummy);

		//Add everything to robotDescription
		robot_description.joints.push_back(fixedJoint);
	}

	//Add everything to robot description
	robot_description.links.push_back(link_distal);
	robot_description.links.push_back(link_proximal);
	robot_description.joints.push_back(movingJoint);

	//Increase joint count
	joint_count++;
}

bool JSON_Database_Parser::getPartIDofJSONFile(std::string file, int& out_id)
{
    pt::ptree json;
    try
    {
        pt::read_json(file, json);
        out_id = json.get<int>("ID");
    }
    catch(std::exception &e)
    {
        ROS_WARN_STREAM(e.what() << std::endl);
        return false;
    }
    return true;
}