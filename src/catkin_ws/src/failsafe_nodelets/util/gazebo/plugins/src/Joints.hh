#ifndef _JOINTS_PLUGIN_HH_
#define _JOINTS_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "modrob_workstation/RobotConfigCommanded.h"
#include <string>
#include <map>

namespace gazebo{

  class JointsPlugin : public ModelPlugin{
    
    public: JointsPlugin();

    public: virtual ~JointsPlugin();

    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    public: void callback(const modrob_workstation::RobotConfigCommandedConstPtr &data);

    private: void QueueThread();

    private: std::unique_ptr<ros::NodeHandle> nh;

    private: ros::Subscriber rosSub;

    /// \brief A ROS callbackqueue that helps process messages
private: ros::CallbackQueue rosQueue;

/// \brief A thread the keeps running the rosQueue
private: std::thread rosQueueThread;

    private: physics::ModelPtr model;

    private: common::PID pid;

    private: std::map<int, std::string> joints_name;
  };
}

#endif
