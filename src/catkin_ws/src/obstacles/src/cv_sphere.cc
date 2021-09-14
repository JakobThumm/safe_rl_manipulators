#ifndef _VELODYNE_PLUGIN_HH_
#define _VELODYNE_PLUGIN_HH_

#include <random>
#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose.h"
#include <geometry_msgs/Point.h>
#include <obstacles/cv_model.h>
#include <Eigen/Dense>

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class CV_Sphere : public ModelPlugin
  {
    /// \brief Constructor
    public: CV_Sphere() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
      
      // Store the model pointer for convenience.
      this->model = _model;
      // Check that the velocity element exists, then read the value
      this->desired_y_velo = 0;
      if (_sdf->HasElement("velocity"))
        this->desired_y_velo = _sdf->Get<double>("velocity");

      ignition::math::Vector3d start_vel = ignition::math::Vector3d(0, this->desired_y_velo, 0);
      this->SetVelocity(start_vel);
      this->sigma_model_noise = 0.001;
      if (_sdf->HasElement("sigma_model_noise"))
        this->sigma_model_noise = _sdf->Get<double>("sigma_model_noise");
      this->sigma_sensor_noise = 0.01;
      if (_sdf->HasElement("sigma_sensor_noise"))
        this->sigma_sensor_noise = _sdf->Get<double>("sigma_sensor_noise");
      // Define random generator with Gaussian distribution
      const double mean = 0.0;
      this->model_noise = std::normal_distribution<double>(mean, this->sigma_model_noise);
      this->sensor_noise = std::normal_distribution<double>(mean, this->sigma_sensor_noise);
      // Set up tracking with cv model
      this->cv_model = CVModel(1, this->sigma_model_noise, this->sigma_sensor_noise);
      Eigen::Vector3d start_pos;
      start_pos << this->model->WorldPose().X()+this->sensor_noise(this->generator),
              this->model->WorldPose().Y()+this->sensor_noise(this->generator),
              this->model->WorldPose().Z()+this->sensor_noise(this->generator);
      Eigen::Vector3d s_v;
      s_v << 0.0, this->desired_y_velo, 0.0;
      this->cv_model.init_target(start_pos, s_v, this->sigma_model_noise, this->sigma_sensor_noise);
      // Create the node
      this->node = transport::NodePtr(new transport::Node());
      #if GAZEBO_MAJOR_VERSION < 8
      this->node->Init(this->model->GetWorld()->GetName());
      #else
      this->node->Init(this->model->GetWorld()->Name());
      #endif

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&CV_Sphere::OnUpdate, this));

      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so_vel =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            "/" + this->model->GetName() + "/vel_cmd",
            1,
            boost::bind(&CV_Sphere::OnRosVelMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so_vel);

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so_pose =
        ros::SubscribeOptions::create<geometry_msgs::Pose>(
            "/" + this->model->GetName() + "/pose_cmd",
            1,
            boost::bind(&CV_Sphere::OnRosPoseMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSubPose = this->rosNode->subscribe(so_pose);

      std::string topic_name = "/" + this->model->GetName() + "/position";
      publisher = this->rosNode->advertise<geometry_msgs::Point>(topic_name, 100);

      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&CV_Sphere::QueueThread, this));

      std::cout << "Fyling sphere plugin connected.";
      ROS_INFO("Fyling sphere plugin connected.");
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Simulate a noisy measurement
      
      Eigen::Vector3d true_pos;
      true_pos << this->model->WorldPose().X(),
                  this->model->WorldPose().Y(),
                  this->model->WorldPose().Z();
      
      Eigen::Vector3d meas;
      meas << true_pos(0)+this->sensor_noise(this->generator),
              true_pos(1)+this->sensor_noise(this->generator),
              true_pos(2)+this->sensor_noise(this->generator);
      // Use the CV KF to estimate the true position
      // Currently the KF uses quite a bit of performance, so we turned it off.
      /*
      this->cv_model.perform_prediction_update(meas);
      Eigen::Vector3d pos = this->cv_model.get_pos();
      */
      // Publish the filtered sphere position
      geometry_msgs::Point pos_msg;
      pos_msg.x = meas(0);
      pos_msg.y = meas(1);
      pos_msg.z = meas(2);
      publisher.publish(pos_msg);
      
      // Debugging
      // Difference between estimated pos and true pos should be lower than difference between measured pos and true pos on average.
      /*
      Eigen::Vector3d diff_est = (pos - true_pos).cwiseAbs();
      Eigen::Vector3d diff_meas = (meas - true_pos).cwiseAbs();
      Eigen::Vector3d best = diff_meas - diff_est;
      ROS_INFO_STREAM("Difference between erros diff_meas - diff_est, these should be positive most of the time. x: " << best(0) << 
                      ", y: " << best(1) << 
                      ", z: " << best(2) << "\n");
      */
      // Add model noise
      // Generate gaussian noise
      ignition::math::Vector3d current_vel = this->model->WorldLinearVel();
      this->desired_y_velo += this->model_noise(this->generator);
      current_vel.Set(
        current_vel.X()+this->model_noise(this->generator),
        this->desired_y_velo,
        current_vel.Z()+this->model_noise(this->generator)
      );
      // Apply a small linear velocity to the model.
      this->model->SetLinearVel(current_vel);
      
    }

    /// \brief Set the velocity of the Velodyne
    /// \param[in] _vel New target velocity
    public: void SetVelocity(const ignition::math::Vector3d &_vel)
    {
      // Set the joint's target velocity.
      this->model->SetLinearVel(_vel);
    }

    /// \brief Set the pose of the Velodyne
    /// \param[in] _pose New pose
    public: void SetPose(const ignition::math::Pose3d &_pose)
    {
      ignition::math::Pose3d curr_pose = this->model->WorldPose();
      this->model->SetWorldPose(_pose);
    }

    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set the y velocity
    /// of the sphere.
    public: void OnRosVelMsg(const std_msgs::Float32ConstPtr &_msg)
    {
      this->desired_y_velo = _msg->data;
      this->SetVelocity(ignition::math::Vector3d(0, this->desired_y_velo, 0));
    }

    /// \brief Handle an incoming pose message from ROS
    /// \param[in] _msg A geometry_msgs::Pose
    public: void OnRosPoseMsg(const geometry_msgs::PoseConstPtr &_msg)
    {
      std::cerr << "Pose msg received.";
      ignition::math::Pose3d _pose = ignition::math::Pose3d(
        _msg->position.x,
        _msg->position.y,
        _msg->position.z,
        _msg->orientation.w,
        _msg->orientation.x,
        _msg->orientation.y,
        _msg->orientation.z
      );
      std::cerr << "Pose created.";
      this->SetPose(_pose);
    }

    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }

    /// \brief Pointer to the model.
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// \brief Gaussian noise of velocity noise
    private: double sigma_model_noise;

    /// \brief Gaussian noise of position sensor
    private: double sigma_sensor_noise;

    private: std::default_random_engine generator;
    private: std::normal_distribution<double> model_noise;
    private: std::normal_distribution<double> sensor_noise;
    /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;

    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr pose_sub;

    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSubPose;

    /// \brief ROS publisher that publishes a gazebo collision
    private: ros::Publisher publisher;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;

    /// \brief Desired y velocity (needed for initial velo jump)
    private: double desired_y_velo;

    /// \brief CV KF tracker
    private: CVModel cv_model;

  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_MODEL_PLUGIN(CV_Sphere)
}
#endif