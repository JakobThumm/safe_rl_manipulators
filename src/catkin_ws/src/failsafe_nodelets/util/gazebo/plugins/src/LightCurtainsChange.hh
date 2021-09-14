#ifndef GAZEBO_ROS_LIGHT_CURTAIN_HPP
#define GAZEBO_ROS_LIGHT_CURTAIN_HPP

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <string>
#include "ros/ros.h"
#include "std_msgs/Int8.h"


namespace gazebo{
  class LightCurtainPlugin : public SensorPlugin{
    public: LightCurtainPlugin();
    
    public: virtual ~LightCurtainPlugin();

    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    public: virtual void OnNewLaserScan();

    private: std_msgs::Int8 msg;

    private: sensors::RaySensorPtr parentSensor;
    
    private: event::ConnectionPtr newLaserScansConnection;

    private: std::unique_ptr<ros::NodeHandle> nh;

    private: ros::Publisher publish;
    
    private: std::string topic;

    private: bool is_crossing;
  };
}

#endif
