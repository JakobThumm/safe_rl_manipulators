#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include "ros/ros.h"

namespace gazebo{
  class ModelPush : public ModelPlugin{
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){
      // gazebo_7
      //this->model = _parent->GetModel("box");

      // gazebo_9
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));

      int nb_joints;
      double begin,end,speed;
      int id;
      
      ros::param::get("/nb_actor",nb_joints);
      char s[20];
      for(int i = 0; i < nb_joints; i++){
        sprintf(s,"/actor_%d",i);
        ros::param::get(std::string(s) + "/begin",begin);
        begins.push_back(begin);
        ros::param::get(std::string(s) + "/end",end);
        ends.push_back(end);
        ros::param::get(std::string(s) + "/speed",speed);
        speeds.push_back(-speed);
        ros::param::get(std::string(s) + "/id",id);
        ROS_ERROR("%s", s);
        joints.push_back(this->model->GetJoint(s));
      }

      nb_it = 0;
    }

    // Called by the world update start event
    public: void OnUpdate(){
      auto speed = speeds.begin();
      auto begin = begins.begin();
      auto end = ends.begin();
      for(auto joint = joints.begin(); joint != joints.end(); joint++, speed++, begin++, end++){
        if(nb_it >= *begin*1e3 && nb_it <= (*begin+1.6)*1e3){
          (*joint)->SetVelocity(0, *speed);
        }
        else if(nb_it >= *end*1e3){
          (*joint)->SetVelocity(0, -*speed);
        }
        else{
          (*joint)->SetVelocity(0, 0);
        }
      }
      nb_it++;
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    private: std::vector<physics::JointPtr> joints;
    private: std::vector<double> speeds;
    private: std::vector<double> begins;
    private: std::vector<double> ends;
    //private: physics::JointPtr joint2;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: int nb_it;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
