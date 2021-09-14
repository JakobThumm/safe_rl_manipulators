#include "ros/ros.h"
#include <ros/console.h>
#include <bits/stdc++.h> 
#include <iostream> 
#include <sys/stat.h> 
#include <sys/types.h> 
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <string>
#include <modrob_simulation/RobotGroupDescription.h>
#include <modrob_simulation/RobotDescription.h>
#include <modrob_simulation/LinkDescription.h>
#include <modrob_simulation/JointDescription.h>
#include <modrob_simulation/LinkVisual.h>
#include <modrob_simulation/LinkCollision.h>
#include <geometry_msgs/Point.h>
#include "gripper_integration.hpp"

#include <sstream>
#include <iostream>
#include <fstream>
using namespace std;

typedef std::shared_ptr<modrob_simulation::RobotDescription> RoboPtr;

// modrob_simulation path
std::string package_path;

string urdf_file_path;
string display_launch_filename;
const string default_display_launch_filename = "display.launch";
string rvizconfig_file;

// location-data and naming of control-based files 
std::string yaml_file_path;
std::string control_launch_file_path;
const std::string default_control_launch_file_name = "control.launch";
const std::string default_yaml_name = "config.yaml";

// Parameters
uint num_robots;
std::vector<uint> modrob_joint_nums;

void writeToFile(string filename, string data){
  ofstream OutputFile(filename);
  OutputFile << data;
  OutputFile.close();
}

void groupCallback(const modrob_simulation::RobotGroupDescription::ConstPtr& group){

	// add a list of all control-launch-file-names
	std::vector<std::string> control_launch_names;

	// set robot independent parameters
	num_robots = group->descriptions.size();

	// Generate general_config.yaml
	std::string general_conf = "num_robots: " + std::to_string(num_robots) + "\n";

	for(int it = 0; it < num_robots; it++){
		modrob_simulation::RobotDescription msg = group->descriptions.at(it);
		std::string name = msg.name;
		string data = "<?xml version=\"1.0\"?>\n<robot name=\""+ name +"\">\n\n";

		// adding a head to config.yaml
		std::string config = name + ":\n  # Publish all joint states --------------------------------\n  joint_state_controller:\n    type: joint_state_controller/JointStateController\n    publish_rate: 100\n\n  # Position Controllers ------------------------------------\n";
		int num_non_fixed_joints;

		// add a world frame for fixation
		std::string world_frame = "\t<link name=\"world\"/>\n\n";
		data = data + world_frame;

		// add the ros_control_plugin
		std::string ros_control_plugin = "\t<gazebo>\n\t\t<plugin name=\"gazebo_ros_control\" filename=\"libgazebo_ros_control.so\">\n\t\t\t<robotNamespace>/" + name + "</robotNamespace>\n\t\t\t<robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>\n\t\t</plugin>\n\t</gazebo>\n\n";
		data = data + ros_control_plugin;

		// check whether a gripper is desired
		bool gripper = false;
		if(ros::param::has("/gripper")){
			ros::param::get("/gripper", gripper);
			general_conf = general_conf + "modrob" + std::to_string(it) + "_has_gripper: true\n";
		}else{
			general_conf = general_conf + "modrob" + std::to_string(it) + "_has_gripper: false\n";
		}

		// iterate over all links
		int size_links = msg.links.size();
		int last_joint_number = -1;
		for (int i = 0; i < size_links; i++ ){
			string link_description = "\t<link name=\"";
			modrob_simulation::LinkDescription link = msg.links[i];

			std::string link_name = link.name;
			link_description = link_description + link_name + "\">\n";
			
			if(link_name.compare("part0")==0){
			}else{
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

			
			// keep track of the Gazebo/Material
			std::string gazebo_material;

			// add gazebo tag for material
			// std::string material = "\t<gazebo reference=\"" + link_name + "\">\n\t\t<material>" + std::to_string() + "</material>\n\t</gazebo>\n";

			// iterate over all visuals
			int size_vis = link.link_visual.size();
			for (int i = 0; i < size_vis; i++ ){
				link_description = link_description + "\t\t<visual>\n";

				modrob_simulation::LinkVisual link_vis = link.link_visual[i];

				double link_vis_origin_x = link_vis.origin_x;
				double link_vis_origin_y = link_vis.origin_y;
				double link_vis_origin_z = link_vis.origin_z;

				double link_vis_origin_r = link_vis.origin_r;
				double link_vis_origin_p = link_vis.origin_p;
				double link_vis_origin_yy = link_vis.origin_yy;

				// memorize gazebo_material
				gazebo_material = link_vis.gazebo_material;

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
					link_description = link_description + "\t\t\t<geometry>\n\t\t\t\t<mesh filename=\"package://modrob_simulation/stl_files/" + url + "\"/>\n\t\t\t</geometry>\n";
				} 


				std::string link_vis_color_name = link_vis.color_name;
				double link_vis_color_r = link_vis.color_r;
				double link_vis_color_b = link_vis.color_b;
				double link_vis_color_g = link_vis.color_g;
				double link_vis_color_a = link_vis.color_a;
				//link_description = link_description + "\t\t\t<material name=\"" + link_vis_color_name + "\">\n\t\t\t\t<color rgba=\"" + std::to_string(link_vis_color_r) + " " + std::to_string(link_vis_color_g) + " " + std::to_string(link_vis_color_b) + " " + std::to_string(link_vis_color_a) + "\"/>\n\t\t\t</material>\n";
			
				std::string link_vis_texture = link_vis.texture;
				
				link_description = link_description + "\t\t</visual>\n";
			}

			// iterate over all collisions
			int size_col = link.link_collision.size();
			for (int i = 0; i < size_col; i++ ){

				link_description = link_description + "\t\t<collision>\n";

				modrob_simulation::LinkCollision link_col = link.link_collision[i];

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
					link_description = link_description + "\t\t\t<geometry>\n\t\t\t\t<mesh filename=\"package://modrob_simulation/stl_files/" + url + "\"/>\n\t\t\t</geometry>\n";
				} 
				
				link_description = link_description + "\t\t</collision>\n";
			}
			link_description = link_description + "\t</link>\n\n";
			link_description = link_description + "\t<gazebo reference=\"" + link_name + "\">\n\t\t<material>" + gazebo_material + "</material>\n\t</gazebo>\n\n";

			// kp, kd, mu1, and mu2 gazebo reference
			double link_kp 	= link.kp;
			double link_kd 	= link.kd;
			double link_mu1 = link.mu1;
			double link_mu2 = link.mu2;
			link_description = link_description + "\t<gazebo reference=\"" + link_name + "\">\n" + "\t\t<kp>" + std::to_string(link_kp) + "</kp>\n\t\t<kd>" + std::to_string(link_kd) + "</kd>\n\t\t<mu1>" + std::to_string(link_mu1) + "</mu1>\n\t\t<mu2>" + std::to_string(link_mu2) + "</mu2>\n\t</gazebo>\n\n";
			
			// iterate over all sensors
			for (int i = 0; i < link.link_sensor.size(); i++ ){
				modrob_simulation::LinkSensor link_sensor = link.link_sensor[i];
				// Add contact sensors
				link_description = link_description + "\t<gazebo reference=\"" + link_name + "\">\n";
				//  <sensor name='modrob0_contact_sensor_proximal_link_of_joint1' type='contact'>
				link_description = link_description + "\t\t<sensor name=\"" + link_sensor.name + "\" type=\"contact\">\n";
				// <always_on>true</always_on>
				link_description = link_description	+ "\t\t\t<always_on>" + to_string(link_sensor.always_on) + "</always_on>\n";
				// <plugin name="modrob_contact_plugin" filename="libmodrob_contact_plugin.so"/>
				link_description = link_description + "\t\t\t<plugin name=\"" + link_sensor.plugin_name + 
									"\" filename=\"" + link_sensor.plugin_file + "\">\n";
				link_description = link_description + "\t\t\t\t<name_space>modrob" + to_string(it) +"</name_space>\n\t\t\t</plugin>\n";
				// <contact>
				link_description = link_description + "\t\t\t<contact>\n";
                //<collision>distal_link_of_joint0_NV_fixed_joint_lump__proximal_link_of_joint1_collision_1</collision>
				link_description = link_description + "\t\t\t\t<collision>" + link_sensor.collision + "</collision>\n";
          		// </contact></sensor></gazebo>
				link_description = link_description + "\t\t\t</contact>\n" + "\t\t</sensor>\n" + "\t</gazebo>\n\n";
			}
			data = data + link_description;

			//Hack to find last regular joint number
			int pos = link_name.find("distal_link_of_joint");
			if (pos != std::string::npos){
				last_joint_number = std::stoi(link_name.substr(pos+20, 1));
			}
		}	

		int size_joints = msg.joints.size();
		//here add gripper_links() after if clause (if last link and gripper=true)
		if(gripper){
			data = data + gripper_links(last_joint_number, it);
		}

		// connect the base and the world-link
		std::string fixed_base =  "\t<joint name=\"fixed\" type=\"fixed\">\n\t\t<parent link=\"world\"/>\n\t\t<child link=\"part0\"/>\n\t</joint>\n";
		data = data+fixed_base;

		// iterate over all joints
		num_non_fixed_joints = 0;
		for (int i = 0; i < size_joints; i++ ){
			
			string joint_description = "\t<joint name=\"";

			modrob_simulation::JointDescription joint = msg.joints[i];

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


			joint_description = joint_description + "\t\t<dynamics damping=\"" + std::to_string(joint_damping) + "\" friction=\"" + std::to_string(joint_friction) +"\"/>\n";

			double joint_lower = joint.lower;
			double joint_upper = joint.upper;
			double joint_effort = joint.effort;
			double joint_velocity = joint.velocity;

			// get joint-id as std::string
			std::stringstream ss;
			std::string joint_num;
			ss << joint.name.back();
			ss >> joint_num;

			// constant update-rate for force_torque sensors
			int update_rate = 100;

			if(!joint_type.compare("revolute")){
				joint_description = joint_description + "\t\t<limit effort=\"" + std::to_string(joint_effort) + "\" velocity=\"" + std::to_string(joint_velocity) + "\" lower=\"" + std::to_string(joint_lower) + "\" upper=\"" + std::to_string(joint_upper) +"\"/>\n";

				// add torque/force-sensors
				std::string force_torque = "\t\t<sensor name=\"force_torque_joint" + joint_num + "\" type=\"force_torque\">\n\t\t\t<update_rate>" + std::to_string(update_rate) + "</update_rate>\n\t\t</sensor>\n";
				joint_description = joint_description + force_torque;

				// count non-fixed joints
				num_non_fixed_joints++;
			}

			double joint_soft_lower_limit = joint.soft_lower_limit;
			double joint_soft_upper_limit = joint.soft_upper_limit;
			double joint_k_position = joint.k_position;
			double joint_k_velocity = joint.k_velocity;

			//joint_description = joint_description + "\t\t<safety_controller k_velocity=\"" + std::to_string(joint_k_velocity) + "\" k_position=\"" + std::to_string(joint_k_position) + "\" soft_lower_limit=\"" + std::to_string(joint_soft_lower_limit) + "\" soft_upper_limit=\"" + std::to_string(joint_soft_upper_limit) +"\"/>\n";

			joint_description = joint_description + "\t</joint>\n\n";

			if(!joint_type.compare("revolute")){
				// add the implicitDamper flag --> currently disabled
				std::string implicit_damper = "\t<gazebo reference=\"" + joint.name + "\">\n\t\t<implicitSpringDamper>1</implicitSpringDamper/>\n\t</gazebo>\n\n";
				// joint_description = joint_description + implicit_damper;

				// add a transmission-element for every joint
				std::string hardware_interface = "<hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>\n\t\t";
				std::string transmission = "\t<transmission name=\"trans" + joint_num + "\">\n\t\t<type>transmission_interface/SimpleTransmission</type>\n\t\t<joint name=\"" + joint.name + "\">\n\t\t\t" + hardware_interface + "</joint>\n\t\t<actuator name=\"motor" + joint_num + "\">\n\t\t\t" + hardware_interface + "\t<mechanicalReduction>" + std::to_string(joint.gear_ratio) + "</mechanicalReduction>\n\t\t</actuator>\n\t</transmission>\n\n";
				joint_description = joint_description + transmission;

				// add a joint_position_controller for every revolute joint
				std::string controller = "  joint" + joint_num + "_position_controller:\n    type: effort_controllers/JointPositionController\n    joint: " + joint_name + "\n    pid: {p: 100.0, i: 0.01, d: 10.0}\n";
				config = config + controller;
					
			}

			data = data + joint_description;

			//add gripper_joints here after if clause (if last joint and gripper true)
			if(gripper & i==size_joints-1){
				//adjust 3 to flexible
				data = data + gripper_joints();
				std::string fingers_hardware_interface = "<hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>\n\t\t";
				std::string finger1_transmission = "\t<transmission name=\"trans_hand_to_finger1\">\n\t\t<type>transmission_interface/SimpleTransmission</type>\n\t\t<joint name=\"hand_to_finger1\">\n\t\t\t" + fingers_hardware_interface + "</joint>\n\t\t<actuator name=\"motor_hand_to_finger1\">\n\t\t\t" + fingers_hardware_interface + "\t<mechanicalReduction>" + std::to_string(160) + "</mechanicalReduction>\n\t\t</actuator>\n\t</transmission>\n\n";
				std::string finger2_transmission = "\t<transmission name=\"trans_hand_to_finger2\">\n\t\t<type>transmission_interface/SimpleTransmission</type>\n\t\t<joint name=\"hand_to_finger2\">\n\t\t\t" + fingers_hardware_interface + "</joint>\n\t\t<actuator name=\"motor_hand_to_finger2\">\n\t\t\t" + fingers_hardware_interface + "\t<mechanicalReduction>" + std::to_string(160) + "</mechanicalReduction>\n\t\t</actuator>\n\t</transmission>\n\n";
				data = data + finger1_transmission + finger2_transmission;
			}

		}

		data = data + "</robot>\n";

		// set robot dependent parameters
		modrob_joint_nums.push_back(num_non_fixed_joints);

		std::string control_launch = std::string("<launch>\n\t") + 
			"<!-- Determine the robot for which controllers are to be launched -->\n\t" +
			"<arg name=\"name_space\" default=\"modrob0\"/>\n\t" +
			"<arg name=\"controller_description_file\" default=\"$(find modrob_simulation)/config/modrob0_control.yaml\"/>\n\t" +
			"<!-- Load joint controller configurations from config.yaml to parameter-server -->\n\t" +
			"<rosparam file=\"$(arg controller_description_file)\" command=\"load\"/>\n\n";
		control_launch = control_launch + "\t<!-- Loading the joint_position_controllers -->\n\t" +
						"<node name=\"controller_spawner\" pkg=\"controller_manager\" type=\"spawner\" respawn=\"false\" output=\"screen\" ns=\"$(arg name_space)\" args=\"{eval get_controller_names.py}\"/>\n";
		
		// adding the joint_position controllers for each joint as arguments of the contoller_spawner
		/*
		std::string args;
		for(int i=0; i < num_non_fixed_joints; i++){
			args = args + "joint" + std::to_string(i) + "_position_controller ";
		}
		*/
		// adding transmissions and position controllers for the end-effector fingers
		if(gripper){
			//args = args + "hand_to_finger1_position_controller hand_to_finger2_position_controller ";
			config = config + "  hand_to_finger1_position_controller:\n    type: effort_controllers/JointPositionController\n    joint: hand_to_finger1\n    pid: {p: 1000.0, i: 0.0, d: 0.0}\n";
			config = config + "  hand_to_finger2_position_controller:\n    type: effort_controllers/JointPositionController\n    joint: hand_to_finger2\n    pid: {p: 1000.0, i: 0.0, d: 0.0}\n";
		}

		//control_launch = control_launch + args + "joint_state_controller\"/>\n";

		// adding motion_publisher for this robot
		control_launch = control_launch + "\t<!-- Loading the motion publisher for this robot -->\n\t<node name=\"motion" + std::to_string(it) + "_publisher\" pkg=\"modrob_simulation\" type=\"motion_publisher\" args=\"" + std::to_string(it) + "\"/>\n</launch>";

		// add launchfile for controllers if necessary and write current control.launch
		control_launch_names.push_back(control_launch_file_path + "/launch/" + name + "_control.launch");
		ROS_INFO("Wrinting to control.launch");
		writeToFile(control_launch_file_path + "/launch/" + name + "_control.launch", control_launch);


		// add config file for pid values and controller inits
		ROS_INFO("Writing to config file");
		writeToFile(yaml_file_path + "/config/" + name + "_control.yaml", config);

		// Convert the urdf into a proper sdf-model
		// system(("gz sdf -p " + package_path + "/urdf/" + name + ".urdf > " + package_path + "/models/modrob/modular-robot.sdf").c_str());

 
		ROS_INFO("Writing urdf to file");
		writeToFile(package_path + "/urdf/" + name + ".urdf", data);

		ROS_INFO("URDF created successfully!");
		
	}

	// Update general_config.yaml with the number of joints per robot
	for(int j = 0; j < num_robots; j ++){
		general_conf = general_conf + "modrob" + std::to_string(j) + "_joint_num: " + std::to_string(modrob_joint_nums[j]) + "\n";
	}

	writeToFile(control_launch_file_path + "/config/general_config.yaml",general_conf);

	// Generate multiple_gazebo.launch
	std::string multiple_gazebo_launch = "<launch>\n\t<!-- Check wether controllers should be launched -->\n\t<arg name=\"control\" default=\"false\" />\n\n";

	// Iterate over all robots
	std::vector<std::string> list_of_name_spaces;

	// Check wether there are any overlapping bases

	// Get all base-positions
	std::vector<geometry_msgs::Vector3> base_list;
	std::vector<geometry_msgs::Vector3> base_list_failed;

	for(int bases = 0; bases < group->descriptions.size(); bases++){
		base_list.push_back(group->descriptions[bases].base_pos);
	}

	double safety_distance;
	// Get safety_distance parameter
	if(ros::param::has("/safety_distance")){
		double safety;
		ros::param::get("/safety_distance", safety);
		safety_distance = safety;
		ROS_INFO("Received safety parameter  %f!",safety_distance);
	}


	// Try as long as there are collisions within the range of min_dist(globally defined)
	ROS_INFO("Checking for conflicts of base-origins");
	bool conflict = true;
	double dist_bases;
	while(conflict){
		conflict = false;
		for(int it1 = 0; it1 < base_list.size(); it1++){
			for(int it2 = 0; it2 < base_list.size(); it2++){
				if(group->descriptions[it1].name == group->descriptions[it2].name){
					continue;
				}

				dist_bases = sqrt(pow(base_list[it1].x-base_list[it2].x,2) + pow(base_list[it1].y-base_list[it2].y,2));
				//std::cout<<pow(base_list[it1].x-base_list[it2].x,2)<<"  "<<base_list[it1].y-base_list[it2].y<<"\n";
				std::cout<<"Distance is: " << dist_bases << " Minimum distance: " << safety_distance << "\n";
				if(dist_bases < safety_distance){
					conflict = true;
					ROS_INFO("Conflict found; Distance between %d and %d is %f. This is less than the minimum required distance of %f",it1,it2,dist_bases,safety_distance);
					
					if(dist_bases == 0.0){
						// Update the origins in the original list
						base_list[it2].x += safety_distance;
						base_list[it2].y += 0.0;
					}else{
						// Update the origins
						base_list[it2].x = base_list[it1].x + (safety_distance/dist_bases)*(base_list[it2].x - base_list[it1].x);
						base_list[it2].y = base_list[it1].y + (safety_distance/dist_bases)*(base_list[it2].y - base_list[it1].y);
					}
					
					ROS_INFO("The base position of %d has been changed to: x: %f y: %f z: %f",it2,base_list[it2].x,base_list[it2].y,base_list[it2].z);
				}
			}
		}
	}

	for(int it = 0; it < control_launch_names.size(); it++){
		multiple_gazebo_launch = multiple_gazebo_launch + "\t<group ns=\"modrob" + std::to_string(it) + "\">\n\t\t<!-- Loading the robot model and its descriptions -->\n\t\t<arg name=\"model\" default=\"$(find modrob_simulation)/urdf/modrob" + std::to_string(it) + ".urdf\"/>\n\t\t<arg name=\"gui\" default=\"false\" />\n\t\t<param name=\"robot_description\" command=\"$(find xacro)/xacro $(arg model)\" />";
		multiple_gazebo_launch = multiple_gazebo_launch + "\n\n\t\t<!-- Launching robot-/joint-state-->\n\t\t<include file=\"$(find modrob_simulation)/launch/modrob_move.launch\" >\n\t\t\t<arg name=\"model\" value=\"$(arg model)\"/>\n\t\t\t<arg name=\"tf_prefix\" value=\"robot" + std::to_string(it) +  "_tf\" />\n\t\t\t<arg name=\"gui\" value=\"$(arg gui)\" />\n\t\t</include>";
		
		// Find the origin of the base
		geometry_msgs::Vector3 base_pos  = base_list[it];


		std::string string_pos = "<arg name=\"init_pose\" value=\"-x " + std::to_string(base_pos.x) + " -y " + std::to_string(base_pos.y) + " -z " + std::to_string(base_pos.z) + "\" />\n";
		multiple_gazebo_launch = multiple_gazebo_launch + "\n\n\t\t<!-- Spawning the robot -->\n\t\t<include file=\"$(find modrob_simulation)/launch/modrob.launch\" >\n\t\t\t<arg name=\"robot_name\" value=\"modrob" + std::to_string(it) +  "\" />\n\t\t\t" + string_pos + "\t\t</include>\n\t</group>\n\n";

		// Create robot namespace
		list_of_name_spaces.push_back("<arg name=\"name_space\" value=\"modrob" + std::to_string(it) + "\" />\n");
	}

	// Controller launching in multiple_gazebo.launch
	multiple_gazebo_launch = multiple_gazebo_launch + "\t<!-- Launching position controllers if desired -->\n\t<group if=\"$(arg control)\">\n";
	for(auto const & it: control_launch_names){
		multiple_gazebo_launch = multiple_gazebo_launch + "\t\t<include file=\"" + it +"\" >\n\t\t\t" + list_of_name_spaces[0] + "\t\t</include>\n";
		list_of_name_spaces.erase(list_of_name_spaces.begin());
	}
	multiple_gazebo_launch = multiple_gazebo_launch + "\t</group>\n</launch>";

	// Update multiple_gazebo.launch
	writeToFile(control_launch_file_path + "/launch/multiple_gazebo.launch",multiple_gazebo_launch);
	ROS_INFO("multiple_gazebo.launch: file created!");
	ROS_INFO("All URDFs created successfully; Shutting down!");

	// Checking whether simulation is desired
	if(ros::param::has("/gazebo")){
		bool gazebo;
		ros::param::get("/gazebo",gazebo);
        if(!gazebo){
			ros::shutdown();
			return;
		}
    }

	// Checking whether a simulation gui is desired
	bool no_gui = false;
	std::string gui = "";
	if(ros::param::has("/no_gui")){
		ros::param::get("/no_gui",no_gui);
    }else{
		no_gui = false;
	}
	if(no_gui){
		gui = gui + "no_gui:=\"true\"";
	}else{
		gui = gui + "no_gui:=\"false\"";
	}

	bool control = false;
	std::string controller_str = "";
	if(ros::param::has("/control")){
		ros::param::get("/control",control);
    }else{
		control = false;
	}
	if(control){
		controller_str = controller_str + "control:=\"true\"";
	}else{
		controller_str = controller_str + "control:=\"false\"";
	}

	display_launch_filename = "main_multi.launch";
	ROS_INFO("Executing: roslaunch modrob_simulation %s %s %s &", display_launch_filename.c_str(), gui.c_str(), controller_str.c_str());
	system(("roslaunch modrob_simulation " + display_launch_filename + " " + gui + " " + controller_str + " &").c_str() );
	ros::shutdown();
}




int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n("~");

  // Receives the group of robot descriptions
  ros::Subscriber group_sub = n.subscribe("/ns/robot_group_description", 1000, &groupCallback);

  if(n.getParam("/modrob_simulation_package_path", package_path)){
    ROS_INFO("Got param /modrob_simulation_package_path: %s", package_path.c_str());
  } else {
    ROS_ERROR("Failed to get param '/modrob_simulation_package_path'");
  }
  control_launch_file_path = package_path;
  yaml_file_path = package_path;
  urdf_file_path = package_path;

  n.param("display_launch_filename", display_launch_filename, default_display_launch_filename);
  ROS_INFO("Value for display_launch_filename: %s", display_launch_filename.c_str());


  ros::spin();

  return 0;
}
