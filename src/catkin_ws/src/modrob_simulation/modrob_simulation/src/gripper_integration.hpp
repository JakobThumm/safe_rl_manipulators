//#include "GripperIntegration.h"  
#include <string>

using namespace std;

std::string gripper_links(int last_joint, int robot_num)
{
	std::string gripperLinkDes;
	gripperLinkDes += "\n\t<link name=\"hand\">\n";
	gripperLinkDes += "\t\t<inertial>\n";
	gripperLinkDes += "\t\t\t<origin rpy=\"0.000000 0.000000 0.000000\" xyz=\"0.000000 0.000000 0.06250\"/>\n";
	gripperLinkDes += "\t\t\t<mass value=\"" + std::to_string(0.184000) + "\"/>\n";
	gripperLinkDes += "\t\t\t<inertia ixx=\"0.550000\" ixy=\"-0.000002\" ixz=\"-0.000003\" iyy=\"0.930000\" iyz=\"0.000005\" izz=\"0.550000\"/>\n";
	gripperLinkDes += "\t\t</inertial>\n";
	gripperLinkDes += "\t\t<visual>\n";
	gripperLinkDes += "\t\t\t<origin rpy=\"0.000000 -0.000000 0.000000\" xyz=\"0.000000 0.000000 0.06250\"/>\n";
	gripperLinkDes += "\t\t<geometry>\n";
	gripperLinkDes += "\t\t\t\t<mesh filename=\"package://modrob_simulation/stl_files/E_Schimaneck.STL\"/>\n";
	gripperLinkDes += "\t\t\t</geometry>\n";
	gripperLinkDes += "\t\t</visual>\n";
	gripperLinkDes += "\t\t<collision>\n";
	gripperLinkDes += "\t\t\t<origin rpy=\"0.000000 -0.000000 0.000000\" xyz=\"0.002343744 0.000000 0.02165501\"/>\n";
	gripperLinkDes += "\t\t\t<geometry>\n";
	gripperLinkDes += "\t\t\t\t<box size=\"0.08531006 0.08485281 0.1616900\"/>\n";
	gripperLinkDes += "\t\t\t</geometry>\n";
	gripperLinkDes += "\t\t\t</collision>\n";
	gripperLinkDes += "\t</link>\n\n";
	gripperLinkDes += "\t<gazebo reference=\"hand\">\n";
	gripperLinkDes += "\t\t<material>Gazebo/DarkGrey</material>\n";
	gripperLinkDes += "\t</gazebo>\n\n";
	gripperLinkDes += "\t<gazebo reference=\"hand\">\n";
	gripperLinkDes += "\t\t<kp>" + std::to_string(0.000000) + "</kp>\n";
	gripperLinkDes += "\t\t<kd>" + std::to_string(0.000000) + "</kd>\n";
	gripperLinkDes += "\t\t<mu1>" + std::to_string(0.000000) + "</mu1>\n";
	gripperLinkDes += "\t\t<mu2>" + std::to_string(0.000000) + "</mu2>\n";
	gripperLinkDes += "\t</gazebo>\n\n";
	// Add contact sensors
	gripperLinkDes += "\t<gazebo reference=\"hand\">\n";
	gripperLinkDes += "\t\t<sensor name=\"hand_collision_sensor\" type=\"contact\">\n";
	gripperLinkDes += "\t\t\t<always_on>true</always_on>\n";
	gripperLinkDes += "\t\t\t<plugin name=\"modrob_contact_plugin\" filename=\"libmodrob_contact_plugin.so\">\n";
	gripperLinkDes += "\t\t\t\t<name_space>modrob" + std::to_string(robot_num) + "</name_space>\n\t\t\t</plugin>\n";
	gripperLinkDes += "\t\t\t<contact>\n";
	gripperLinkDes += "\t\t\t\t<collision>distal_link_of_joint" + std::to_string(last_joint) + "_NV_fixed_joint_lump__hand_collision</collision>\n";
	gripperLinkDes += "\t\t\t</contact>\n\t\t</sensor>\n\t</gazebo>\n\n";
	gripperLinkDes += "\t<link name=\"finger1\">\n";
	gripperLinkDes += "\t\t<inertial>\n";
	gripperLinkDes += "\t\t\t<origin rpy=\"0.00000 0.000000 0.0\" xyz=\"0.0 -0.000 0.001500\"/>\n";
	gripperLinkDes += "\t\t\t<mass value=\"" + std::to_string(0.006944)+ "\"/>\n";
	gripperLinkDes += "\t\t\t<inertia ixx=\"0.000004629\" ixy=\"0.000000486\" ixz=\"0.000000972\" iyy=\"0.000004157\" iyz=\"0.000001389\" izz=\"0.00000138\"/>\n";
	gripperLinkDes += "\t\t</inertial>\n";
	gripperLinkDes += "\t\t<visual>\n";
	gripperLinkDes += "\t\t\t<origin rpy=\"0.000000 0.000000 0.0\" xyz=\"0.0 -0.000 0.001500\"/>\n";
	gripperLinkDes += "\t\t\t<geometry>\n";
	gripperLinkDes += "\t\t\t\t<mesh filename=\"package://modrob_simulation/stl_files/finger_Birne_Gimatic_m.STL\"/>\n";
	gripperLinkDes += "\t\t\t</geometry>\n";
	gripperLinkDes += "\t\t</visual>\n";
	/*
	gripperLinkDes += "\t\t<collision>\n";
	gripperLinkDes += "\t\t\t<origin rpy=\"0.000000 0.000000 0.0\" xyz=\"0.0 -0.000 0.001500\"/>\n";
	gripperLinkDes += "\t\t\t<geometry>\n";
	gripperLinkDes += "\t\t\t\t<mesh filename=\"package://modrob_simulation/stl_files/finger_Birne_Gimatic_m.STL\"/>\n";
	gripperLinkDes += "\t\t\t</geometry>\n";
	gripperLinkDes += "\t\t</collision>\n";
	*/
	gripperLinkDes += "\t</link>\n\n";
	gripperLinkDes += "\t<gazebo reference=\"finger1\">\n";
	gripperLinkDes += "\t\t<material>Gazebo/Blue</material>\n";
	gripperLinkDes += "\t</gazebo>\n\n";
	gripperLinkDes += "\t<gazebo reference=\"finger1\">\n";
	gripperLinkDes += "\t\t<kp>110.000000</kp>\n";
	gripperLinkDes += "\t\t<kd>10.000000</kd>\n";
	gripperLinkDes += "\t\t<mu1>100.00</mu1>\n";	// was 0.38
	gripperLinkDes += "\t\t<mu2>100.00</mu2>\n";	// was 0.38
	gripperLinkDes += "\t</gazebo>\n\n";
	// Add contact sensors
	/*
	gripperLinkDes += "\t<gazebo reference=\"finger1\">\n";
	gripperLinkDes += "\t\t<sensor name=\"finger1_collision_sensor\" type=\"contact\">\n";
	gripperLinkDes += "\t\t\t<always_on>true</always_on>\n";
	gripperLinkDes += "\t\t\t<plugin name=\"modrob_contact_plugin\" filename=\"libmodrob_contact_plugin.so\">\n";
	gripperLinkDes += "\t\t\t\t<name_space>modrob" + std::to_string(robot_num) + "</name_space>\n\t\t\t</plugin>\n";
	gripperLinkDes += "\t\t\t<contact>\n";
	gripperLinkDes += "\t\t\t\t<collision>finger1_collision</collision>\n";
	gripperLinkDes += "\t\t\t</contact>\n\t\t</sensor>\n\t</gazebo>\n\n";
	*/
	gripperLinkDes += "\t<link name=\"finger2\">\n";
	gripperLinkDes += "\t\t<inertial>\n";
	gripperLinkDes += "\t\t\t<origin rpy=\"0.00000 0.000000 0.0\" xyz=\"0.000 0.00 0.001500\"/>\n";
	gripperLinkDes += "\t\t\t<mass value=\"0.006944\"/>\n";
	gripperLinkDes += "\t\t\t<inertia ixx=\"0.000004629\" ixy=\"0.000000486\" ixz=\"0.000000972\" iyy=\"0.000004157\" iyz=\"0.000001389\" izz=\"0.00000138\"/>\n";
	gripperLinkDes += "\t\t</inertial>\n";
	gripperLinkDes += "\t\t<visual>\n";
	gripperLinkDes += "\t\t\t<origin rpy=\"0.000000 0.000000 0.0\" xyz=\"0.000 0.00 0.001500\"/>\n";
	gripperLinkDes += "\t\t\t<geometry>\n";
	gripperLinkDes += "\t\t\t\t<mesh filename=\"package://modrob_simulation/stl_files/finger_Birne_Gimatic_m.STL\"/>\n";
	gripperLinkDes += "\t\t\t</geometry>\n";
	gripperLinkDes += "\t\t</visual>\n";
	/*
	gripperLinkDes += "\t\t<collision>\n";
	gripperLinkDes += "\t\t\t<origin rpy=\"0.000000 0.000000 0.0\" xyz=\"0.000 0.00 0.001500\"/>\n";
	gripperLinkDes += "\t\t\t<geometry>\n";
	gripperLinkDes += "\t\t\t\t<mesh filename=\"package://modrob_simulation/stl_files/finger_Birne_Gimatic_m.STL\"/>\n";
	gripperLinkDes += "\t\t\t</geometry>\n";
	gripperLinkDes += "\t\t</collision>\n";
	*/
	gripperLinkDes += "\t</link>\n\n";
	gripperLinkDes += "\t<gazebo reference=\"finger2\">\n";
	gripperLinkDes += "\t\t<material>Gazebo/Blue</material>\n";
	gripperLinkDes += "\t</gazebo>\n\n";
	gripperLinkDes += "\t<gazebo reference=\"finger2\">\n";
	gripperLinkDes += "\t\t<kp>110.000000</kp>\n";
	gripperLinkDes += "\t\t<kd>10.000000</kd>\n";
	gripperLinkDes += "\t\t<mu1>100.00</mu1>\n";	// was 0.38
	gripperLinkDes += "\t\t<mu2>100.00</mu2>\n";	// was 0.38
	gripperLinkDes += "\t</gazebo>\n\n";
	// Add contact sensors
	/*
	gripperLinkDes += "\t<gazebo reference=\"finger2\">\n";
	gripperLinkDes += "\t\t<sensor name=\"finger2_collision_sensor\" type=\"contact\">\n";
	gripperLinkDes += "\t\t\t<always_on>true</always_on>\n";
	gripperLinkDes += "\t\t\t<plugin name=\"modrob_contact_plugin\" filename=\"libmodrob_contact_plugin.so\">\n";
	gripperLinkDes += "\t\t\t\t<name_space>modrob" + std::to_string(robot_num) + "</name_space>\n\t\t\t</plugin>\n";
	gripperLinkDes += "\t\t\t<contact>\n";
	gripperLinkDes += "\t\t\t\t<collision>finger2_collision</collision>\n";
	gripperLinkDes += "\t\t\t</contact>\n\t\t</sensor>\n\t</gazebo>\n\n";
	*/
    return gripperLinkDes;

}

std::string gripper_joints()
{
    std::string gripperJointDes; 
    gripperJointDes += "\n\t<joint name=\"fixed_gripper_to_arm_link\" type=\"fixed\">\n";
    gripperJointDes += "\t\t<origin rpy=\"0.000000 0.000000 0.000000\" xyz=\"0.000000 0.000000 0.059775\"/>\n";
	gripperJointDes += "\t\t<parent link=\"tool_dummy\"/>\n";
    gripperJointDes += "\t\t<child link=\"hand\"/>\n";
  	gripperJointDes += "\t</joint>\n\n";
    gripperJointDes += "\t<joint name=\"hand_to_finger1\" type=\"prismatic\">\n";
	gripperJointDes += "\t\t<origin rpy=\"0.000000 0.000000 +1.570796327\" xyz=\"0.00700 -0.03200 0.06200\"/>\n";
	gripperJointDes += "\t\t<parent link=\"hand\"/>\n";
	gripperJointDes += "\t\t<child link=\"finger1\"/>\n";
	gripperJointDes += "\t\t<dynamics damping=\"12.590000\" friction=\"0.400000\"/>\n";
	gripperJointDes += "\t\t<limit effort=\"3.0\" lower=\"-0.0\" upper=\"0.0100\" velocity=\"0.5\"/> <!-- was 0.38 and 0.0 -->\n";
  	gripperJointDes += "\t</joint>\n\n";
  	gripperJointDes += "\t<joint name=\"hand_to_finger2\" type=\"prismatic\">\n";
	gripperJointDes += "\t\t<origin rpy=\"0.000000 0.000000 -1.570796327\" xyz=\"-0.00700 0.0620 0.06200\"/>\n";
	gripperJointDes += "\t\t<parent link=\"hand\"/>\n";
	gripperJointDes += "\t\t<child link=\"finger2\"/>\n";
	gripperJointDes += "\t\t<dynamics damping=\"12.590000\" friction=\"0.400000\"/>\n";
	gripperJointDes += "\t\t<limit effort=\"3.0\" lower=\"0.03\" upper=\"0.04\" velocity=\"0.5\"/>\n";
  	gripperJointDes += "\t</joint>\n\n";
	return gripperJointDes;
}