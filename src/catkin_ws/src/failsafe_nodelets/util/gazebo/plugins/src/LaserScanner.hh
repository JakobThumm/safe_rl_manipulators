#ifndef _GAZEBO_LASER_PLUGIN_HH_
#define _GAZEBO_LASER_PLUGIN_HH_

#include <string>

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include "ros/ros.h"
#include "custom_robot_msgs/Positions.h"

namespace gazebo
{
  /// \brief An example plugin for a contact sensor.
  class LaserPlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: LaserPlugin();

    /// \brief Destructor.
    public: virtual ~LaserPlugin();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the contact sensor's update signal.
    private: virtual void OnUpdate();

    //Pointer to the world
    private: physics::WorldPtr world;

    // Pointer to the model
    private: physics::ModelPtr model;

    private: physics::ModelPtr model2;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: double sample_time;

    private: double prev_time;

    private: std::unique_ptr<ros::NodeHandle> nh;

    private: ros::Publisher publish;
  };
}
#endif
