#include <ros/ros.h>
#include "custom_robot_msgs/Polycapsule.h"
#include <nodelet.hpp>

#include <pluginlib/class_list_macros.h>

namespace human_cylinder
{
  /* initialization */
  void HumanCylinderNodelet::onInit(){
    ros::NodeHandle& private_nh = getPrivateNodeHandle();

    ros::Publisher pub_sets = private_nh.advertise<custom_robot_msgs::PolycapsuleArray>("/human_polycapsules",1000);

    // get sensors layout
    int nb_curtains  = 0, nb_presence_sensor  = 0, nb_laser_scanner = 0;
    int sensor_type = -1;
    if(ros::param::has("/nb_curtains")){
      ros::param::get("/nb_curtains", nb_curtains);
    }
    if(ros::param::has("/nb_presence_sensors")){
      ros::param::get("/with_camera", nb_presence_sensor);
    }

    if(ros::param::has("/nb_laser_scanners")){
      ros::param::get("/nb_laser_scanners", nb_laser_scanner);
    }

    // get sensors caracteristics
    double delay = 0, pos_uncertainty = 0, human_maximum_thickness = 0;
    std::vector<LaserScanner> lasers;
   if (nb_laser_scanner > 0) {
      int id;
      double range = 0;
      std::vector<double> origin;
      HalfCircle hc;
      char s[20];
      for (int i = 0; i < nb_laser_scanner; i++) {
          sprintf(s, "/laser_scanner_%d", i);
          if (ros::param::has(s)) {
              ros::param::get(std::string(s) + "/id", id);
              ros::param::get(std::string(s) + "/range", range);
              ros::param::get(std::string(s) + "/delay", delay);
              ros::param::get(std::string(s) + "/uncertainty", pos_uncertainty);
              ros::param::get(std::string(s) + "/human_maximum_thickness", human_maximum_thickness);
              ros::param::get(std::string(s) + "/origin", origin);

              geometry_msgs::Point p;
              p.x = origin[0];
              p.y = origin[1];
              hc = HalfCircle(p, range, origin[2]);
              lasers.push_back(LaserScanner(id, delay, pos_uncertainty, human_maximum_thickness, hc));
          }
          else {
              NODELET_DEBUG("The parameter %s is missing", s);
          }
      }
    } 

    std::vector<Curtain> curtains;
    if (nb_curtains > 0) {
      int id;
      std::vector<double> pos;
      // get the curtains caracteristics
      char s[20];
      for (int i = 0; i < nb_curtains; i++) {
        sprintf(s, "/curtain_%d", i);
        if (ros::param::has(s)) {
          ros::param::get(std::string(s) + "/id", id);
          ros::param::get(std::string(s) + "/pos", pos);
          ros::param::get(std::string(s) + "/delay", delay);
          ros::param::get(std::string(s) + "/pos_uncertainty", pos_uncertainty);
          ros::param::get(std::string(s) + "/human_maximum_thickness", human_maximum_thickness);
          if (pos.size() == 4) {
            std::vector<geometry_msgs::Point> points;
            for (auto p = pos.begin(); p != pos.end(); p++) {
              geometry_msgs::Point point;
              point.x = *p;
              p++;
              point.y = *p;
              points.push_back(point);
            }
          curtains.push_back(Curtain(id, delay, pos_uncertainty, human_maximum_thickness, points[0], points[1]));
          }
          else {
            NODELET_DEBUG("The parameter /curtain_%d/pos should be [p1x,p1y,p2x,p2y]", i);
          }
        }
        else {
          NODELET_DEBUG("The parameter %s is missing", s);
        }
      }
    }

    PresenceSensor presence_sensor;
    if (nb_presence_sensor > 0) {
        int id;
        if (ros::param::has("/presence_sensor/id")) {
          ros::param::get("/presence_sensor/id", id);
          presence_sensor = PresenceSensor(id);
         }
         else {
                NODELET_DEBUG("The parameter /presence_sensor/id is missing");
         }
    }

    // get human parameters
    double max_speed, sample_time = 0;
    if (ros::param::has("/sample_time")) {
        ros::param::get("/sample_time", sample_time);
    }
    if (ros::param::has("/max_speed")) {
        ros::param::get("/max_speed", max_speed);
    }

    HumanCylinder human = HumanCylinder(max_speed);

    MultiSensorsAnalysis* analyzer;
    if (nb_curtains > 0) {
        CurtainsAnalysis* curtains_analyzer;
        if (nb_presence_sensor > 0) {
            CurtainsWithPresenceSensorsAnalysis* presence_analyzer = new CurtainsWithPresenceSensorsAnalysis(human, presence_sensor, sample_time, curtains); 
            sub_presence_sensors = private_nh.subscribe("/presence_data", 1000, &CurtainsWithPresenceSensorsAnalysis::presence_callback, presence_analyzer);
            curtains_analyzer = presence_analyzer;
        }
        else {
            curtains_analyzer = new CurtainsAnalysis(human, sample_time, curtains);
        }
        if (nb_laser_scanner > 0) {
            LaserScannerAnalysis laser_analyzer = LaserScannerAnalysis(lasers, human, sample_time);
            CurtainsWithLaserScannerAnalysis* test = new CurtainsWithLaserScannerAnalysis(*curtains_analyzer, laser_analyzer);
sub_curtains = private_nh.subscribe("/curtains_data", 1000, &CurtainsWithLaserScannerAnalysis::curtain_callback, test);
            sub_lasers = private_nh.subscribe("/lasers_data", 1000, &CurtainsWithLaserScannerAnalysis::laser_callback, test);
analyzer = test;
        }
        else {
sub_curtains = private_nh.subscribe("/curtains_data", 1000, &CurtainsAnalysis::callback, curtains_analyzer);
            analyzer = curtains_analyzer;
        }
    }
    else if (nb_laser_scanner > 0) {
        LaserScannerAnalysis* laser_analyzer = new LaserScannerAnalysis(lasers, human, sample_time);
        sub_lasers = private_nh.subscribe("/lasers_data", 1000, &LaserScannerAnalysis::callback, laser_analyzer);
        analyzer = laser_analyzer;
    }
    else {
        ROS_ERROR("There is no sensors");
    }

    human_computation = HumanCylinderComputation(human, analyzer, pub_sets);
    sub_tbrake = private_nh.subscribe("/tbrake", 1000, &HumanCylinderComputation::humanReachabilityAnalysis, &human_computation);

    analyzer->checkLayout();
  }
}

PLUGINLIB_EXPORT_CLASS(human_cylinder::HumanCylinderNodelet, nodelet::Nodelet);
    
