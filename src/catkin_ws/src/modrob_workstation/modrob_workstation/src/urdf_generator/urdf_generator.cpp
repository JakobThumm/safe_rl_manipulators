
#include "ros/ros.h"
#include <ros/console.h>
#include <bits/stdc++.h> 
#include <iostream> 
#include <sys/stat.h> 
#include <sys/types.h> 
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <string>
#include <modrob_workstation/RobotDescription.h>
#include <modrob_workstation/LinkDescription.h>
#include <modrob_workstation/JointDescription.h>
#include <modrob_workstation/LinkVisual.h>
#include <modrob_workstation/LinkCollision.h>

#define URDF_ROSPARAM_PATH "/urdf_file"

void callback_RobotDescription(const modrob_workstation::RobotDescription::ConstPtr& msg)
{
  std::string name = msg->name;
  std::string data = "<?xml version=\"1.0\"?>\n<robot name=\""+ name +"\">\n";

  // iterate over all links
  int size_links = msg->links.size();
  for (int i = 0; i < size_links; i++ ){
	std::string link_description = "\t<link name=\"";
      	modrob_workstation::LinkDescription link = msg->links[i];

	std::string link_name = link.name;
	link_description = link_description + link_name + "\">\n";
	
	if(link_name.compare("part0")==0){}
	else{
		link_description = link_description + "\t\t<inertial>\n";

		double link_origin_x = link.origin_x;
		double link_origin_y = link.origin_y;
		double link_origin_z = link.origin_z;

		double link_origin_r = link.origin_r;
		double link_origin_p = link.origin_p;
		double link_origin_yy = link.origin_yy;

		link_description = link_description + "\t\t\t<origin rpy=\"" + std::to_string(link_origin_r) + " " + std::to_string(link_origin_p) + " " + std::to_string(link_origin_yy) +"\" xyz=\""+ std::to_string(link_origin_x) + " " + std::to_string(link_origin_y) + " " + std::to_string(link_origin_z) +"\"/>\n";

		double link_mass = link.mass;

		link_description = link_description + "\t\t\t<mass value=\"" + std::to_string(link_mass) +"\"/>\n";

		double link_intertia_ixx = link.intertia_ixx;
		double link_intertia_ixy = link.intertia_ixy; 
		double link_intertia_ixz = link.intertia_ixz;
		double link_intertia_iyy = link.intertia_iyy;
		double link_intertia_iyz = link.intertia_iyz;
		double link_intertia_izz = link.intertia_izz;

		link_description = link_description + "\t\t\t<inertia ixx=\"" + std::to_string(link_intertia_ixx) + "\" ixy=\"" + std::to_string(link_intertia_ixy) + "\" ixz=\"" + std::to_string(link_intertia_ixz) + "\" iyy=\"" + std::to_string(link_intertia_iyy) + "\" iyz=\"" + std::to_string(link_intertia_iyz) + "\" izz=\"" + std::to_string(link_intertia_izz) + "\"/>\n";


		link_description = link_description + "\t\t</inertial>\n";
	}

	// iterate over all visuals
	int size_vis = link.link_visual.size();
	for (int i = 0; i < size_vis; i++ ){
		link_description = link_description + "\t\t<visual>\n";

		modrob_workstation::LinkVisual link_vis = link.link_visual[i];

		double link_vis_origin_x = link_vis.origin_x;
		double link_vis_origin_y = link_vis.origin_y;
		double link_vis_origin_z = link_vis.origin_z;

		double link_vis_origin_r = link_vis.origin_r;
		double link_vis_origin_p = link_vis.origin_p;
		double link_vis_origin_yy = link_vis.origin_yy;

		link_description = link_description + "\t\t\t<origin rpy=\"" + std::to_string(link_vis_origin_r) + " " + std::to_string(link_vis_origin_p) + " " + std::to_string(link_vis_origin_yy) +"\" xyz=\""+ std::to_string(link_vis_origin_x) + " " + std::to_string(link_vis_origin_y) + " " + std::to_string(link_vis_origin_z) +"\"/>\n";
		
		std::string link_vis_type = link_vis.type;
		
		if(link_vis_type.compare("sphere") == 0){
			double link_vis_sphere_radius = link_vis.arg1;
			link_description = link_description + "\t\t\t<geometry>\n\t\t\t\t<sphere radius=\"" + std::to_string(link_vis_sphere_radius) + "\"/>\n\t\t\t</geometry>\n";
		} else if(link_vis_type.compare("cylinder") == 0){
			double link_vis_cylinder_radius = link_vis.arg1;
			double link_vis_cylinder_length = link_vis.arg2;
			link_description = link_description + "\t\t\t<geometry>\n\t\t\t\t<cylinder length=\"" + std::to_string(link_vis_cylinder_length) + "\" radius=\"" + std::to_string(link_vis_cylinder_radius) + "\"/>\n\t\t\t</geometry>\n";
		} else if(link_vis_type.compare("box") == 0){
			double link_vis_box_x = link_vis.arg1;
			double link_vis_box_y = link_vis.arg2;
			double link_vis_box_z = link_vis.arg3;
			link_description = link_description + "\t\t\t<geometry>\n\t\t\t\t<box size=\"" + std::to_string(link_vis_box_x) + " " + std::to_string(link_vis_box_y) + " " + std::to_string(link_vis_box_z) + "\"/>\n\t\t\t</geometry>\n";
		} else {
			std::string url = link_vis_type;
			link_description = link_description + "\t\t\t<geometry>\n\t\t\t\t<mesh filename=\"package://modrob_resources/" + url + "\"/>\n\t\t\t</geometry>\n";
		} 


		std::string link_vis_color_name = link_vis.color_name;
		double link_vis_color_r = link_vis.color_r;
		double link_vis_color_b = link_vis.color_b;
		double link_vis_color_g = link_vis.color_g;
		double link_vis_color_a = link_vis.color_a;
		link_description = link_description + "\t\t\t<material name=\"" + link_vis_color_name + "\">\n\t\t\t\t<color rgba=\"" + std::to_string(link_vis_color_r) + " " + std::to_string(link_vis_color_g) + " " + std::to_string(link_vis_color_b) + " " + std::to_string(link_vis_color_a) + "\"/>\n\t\t\t</material>\n";

		std::string link_vis_texture = link_vis.texture;
		
		link_description = link_description + "\t\t</visual>\n";
	}

	// iterate over all collisions
	int size_col = link.link_collision.size();
	for (int i = 0; i < size_col; i++ ){

		link_description = link_description + "\t\t<collision>\n";

		modrob_workstation::LinkCollision link_col = link.link_collision[i];

		double link_vis_origin_x = link_col.origin_x;
		double link_vis_origin_y = link_col.origin_y;
		double link_vis_origin_z = link_col.origin_z;

		double link_vis_origin_r = link_col.origin_r;
		double link_vis_origin_p = link_col.origin_p;
		double link_vis_origin_yy = link_col.origin_yy;

		link_description = link_description + "\t\t\t<origin rpy=\"" + std::to_string(link_vis_origin_r) + " " + std::to_string(link_vis_origin_p) + " " + std::to_string(link_vis_origin_yy) +"\" xyz=\""+ std::to_string(link_vis_origin_x) + " " + std::to_string(link_vis_origin_y) + " " + std::to_string(link_vis_origin_z) +"\"/>\n";
		
		std::string link_vis_type = link_col.type;
		
		if(link_vis_type.compare("sphere") == 0){
			double link_vis_sphere_radius = link_col.arg1;
			link_description = link_description + "\t\t\t<geometry>\n\t\t\t\t<sphere radius=\"" + std::to_string(link_vis_sphere_radius) + "\"/>\n\t\t\t</geometry>\n";
		} else if(link_vis_type.compare("cylinder") == 0){
			double link_vis_cylinder_radius = link_col.arg1;
			double link_vis_cylinder_length = link_col.arg2;
			link_description = link_description + "\t\t\t<geometry>\n\t\t\t\t<cylinder length=\"" + std::to_string(link_vis_cylinder_length) + "\" radius=\"" + std::to_string(link_vis_cylinder_radius) + "\"/>\n\t\t\t</geometry>\n";
		} else if(link_vis_type.compare("box") == 0){
			double link_vis_box_x = link_col.arg1;
			double link_vis_box_y = link_col.arg2;
			double link_vis_box_z = link_col.arg3;
			link_description = link_description + "\t\t\t<geometry>\n\t\t\t\t<box size=\"" + std::to_string(link_vis_box_x) + " " + std::to_string(link_vis_box_y) + " " + std::to_string(link_vis_box_z) + "\"/>\n\t\t\t</geometry>\n";
		} else {
			std::string url = link_vis_type;
			link_description = link_description + "\t\t\t<geometry>\n\t\t\t\t<mesh filename=\"package://modrob_resources/" + url + "\"/>\n\t\t\t</geometry>\n";
		} 
		
		link_description = link_description + "\t\t</collision>\n";
	}
	link_description = link_description + "\t</link>\n";
	data = data + link_description;
  }


  // iterate over all joints
  int size_joints = msg->joints.size();
  for (int i = 0; i < size_joints; i++ ){
	std::string joint_description = "\t<joint name=\"";

	modrob_workstation::JointDescription joint = msg->joints[i];

       	std::string joint_name = joint.name;
  	std::string joint_type = joint.type;

	joint_description = joint_description + joint_name + "\" type=\"" + joint_type + "\">\n";

	double joint_origin_x = joint.origin_x;
	double joint_origin_y = joint.origin_y;
	double joint_origin_z = joint.origin_z;

	double joint_origin_r = joint.origin_r;
	double joint_origin_p = joint.origin_p;
	double joint_origin_yy = joint.origin_yy;

	joint_description = joint_description + "\t\t<origin xyz=\"" + std::to_string(joint_origin_x) + " " + std::to_string(joint_origin_y) + " " + std::to_string(joint_origin_z) +"\" rpy=\""+ std::to_string(joint_origin_r) + " " + std::to_string(joint_origin_p) + " " + std::to_string(joint_origin_yy) +"\"/>\n";

	std::string joint_parent_link = joint.parent_link;
  	std::string joint_child_link = joint.child_link;

	joint_description = joint_description + "\t\t<parent link=\"" + joint_parent_link + "\"/>\n\t\t<child link=\"" + joint_child_link +"\"/>\n";

	double joint_axis_x = joint.axis_x;
	double joint_axis_y = joint.axis_y;
	double joint_axis_z = joint.axis_z;

	joint_description = joint_description + "\t\t<axis xyz=\"" + std::to_string(joint_axis_x) + " " + std::to_string(joint_axis_y) + " " + std::to_string(joint_axis_z) + "\"/>\n";

	double joint_damping = joint.damping;
	double joint_friction = joint.friction;


	//joint_description = joint_description + "\t\t<dynamics damping=\"" + std::to_string(joint_damping) + "\" friction=\"" + std::to_string(joint_friction) +"\"/>\n";

	double joint_lower = joint.lower;
	double joint_upper = joint.upper;
	double joint_effort = joint.effort;
	double joint_velocity = joint.velocity;

	if(!joint_type.compare("revolute")){
		joint_description = joint_description + "\t\t<limit effort=\"" + std::to_string(joint_effort) + "\" velocity=\"" + std::to_string(joint_velocity) + "\" lower=\"" + std::to_string(joint_lower) + "\" upper=\"" + std::to_string(joint_upper) +"\"/>\n";
	}

	double joint_soft_lower_limit = joint.soft_lower_limit;
	double joint_soft_upper_limit = joint.soft_upper_limit;
	double joint_k_position = joint.k_position;
	double joint_k_velocity = joint.k_velocity;

	//joint_description = joint_description + "\t\t<safety_controller k_velocity=\"" + std::to_string(joint_k_velocity) + "\" k_position=\"" + std::to_string(joint_k_position) + "\" soft_lower_limit=\"" + std::to_string(joint_soft_lower_limit) + "\" soft_upper_limit=\"" + std::to_string(joint_soft_upper_limit) +"\"/>\n";

	joint_description = joint_description + "\t</joint>\n";
	data = data + joint_description;
  }

  data = data + "</robot>\n";

  ros::NodeHandle nn("~");
  nn.setParam(URDF_ROSPARAM_PATH, data);
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "urdf_generator");
  ros::NodeHandle n("~");
  ros::Subscriber sub_RobotDescription = n.subscribe("/ns/robot_description", 1000, callback_RobotDescription);

  ros::spin();

  return 0;
}
