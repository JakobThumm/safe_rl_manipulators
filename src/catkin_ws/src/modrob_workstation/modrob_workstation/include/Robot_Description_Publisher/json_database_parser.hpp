// ===============================
// AUTHOR       : Max Körsten (ge46hug@mytum.de)
// CREATE DATE  : 22.12.2020
// PURPOSE      : Read Part information from a JSON Database and fill in fields of a robot_description Message
// ===============================

#ifndef JSON_DATABASE_PARSER_HPP
#define JSON_DATABASE_PARSER_HPP

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <modrob_workstation/RobotDescription.h>
#include <modrob_workstation/LinkDescription.h>
#include <modrob_workstation/JointDescription.h>
#include <modrob_workstation/LinkVisual.h>
#include <modrob_workstation/LinkCollision.h>

//The class looks for a folder called ModuleLibrary in the DATABASE_PACKAGE_NAME package and enumerates all .json files in any subfolders of the ModuleLibrary
#define DATABASE_PACKAGE_NAME "modrob_resources"

// Short alias for this namespace
namespace pt = boost::property_tree;

class JSON_Database_Parser
{
public:
    JSON_Database_Parser();

    bool Parse_ID(modrob_workstation::RobotDescription &robot_description, int &part_count, int &joint_count, int &order, int id);

    void UpdateModuleFileList();

private:
    std::list<std::string> moduleJSONFiles;
    std::list<int> moduleJSONFilesIDs;
    std::list<std::string>::iterator currentFile;

    void createLinkDescription(modrob_workstation::RobotDescription &robot_description, pt::ptree &link_json, int part_count, int order);
    void createMotorDescription(modrob_workstation::RobotDescription &robot_description, pt::ptree &motor_json, int part_count, int &joint_count, int &order);

    bool getPartIDofJSONFile(std::string file, int& out_id);

    //Robot link color
    const double robot_link_color_r = 1;
    const double robot_link_color_g = 1;
    const double robot_link_color_b = 1;
    const double robot_link_color_a = 1;

    //Robot motor color
    const double robot_motor_color_r = 0.1;
    const double robot_motor_color_g = 0.1;
    const double robot_motor_color_b = 0.5;
    const double robot_motor_color_a = 1;

};


#endif