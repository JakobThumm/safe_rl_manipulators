#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo{
  class ModelPush : public WorldPlugin{
    public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf){
      // Store the pointer to the model
      this->world = _parent;

      // gazebo_7
      //this->model = _parent->GetModel("box");

      // gazebo_9
      this->model = _parent->ModelByName("box");
      
      if(_sdf->HasElement("camera")){
        presence_sensor = _sdf->GetElement("camera")->Get<bool>();
      }
      else{ 
        presence_sensor = false;
      }

      if(presence_sensor){
        // gazebo 7
        //this->model2 = _parent->GetModel("box2");
        
        //gazebo 9
        this->model2 = _parent->ModelByName("box2");
      }

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));

      if(_sdf->HasElement("speed")){
        speed = _sdf->GetElement("speed")->Get<double>();
      }
      else{ 
        speed = 1.6;
      }
      if(_sdf->HasElement("begin")){
        begin = _sdf->GetElement("begin")->Get<double>();
      }
      else{ 
        begin = 12;
      }
    }

    // Called by the world update start event
    public: void OnUpdate(){
      double time = this->world->SimTime().Float();
      // Apply a small linear velocity to the model.
      if (time >= begin+3){
        this->model->SetLinearVel(ignition::math::Vector3d(-speed, 0, 0));
      }
      else if(time >= begin+1.6 || time <= begin){
        this->model->SetLinearVel(ignition::math::Vector3d(0,0,0));
      }
      else{
        this->model->SetLinearVel(ignition::math::Vector3d(speed, 0, 0));
      }

      if(presence_sensor){
        if (time >= begin+4){
          this->model2->SetLinearVel(ignition::math::Vector3d(speed, 0, 0));
        }
        else if(time >= begin+2.6 || time <= begin+1){
          this->model2->SetLinearVel(ignition::math::Vector3d(0,0,0));
        }
        else{
          this->model2->SetLinearVel(ignition::math::Vector3d(-speed, 0, 0));
        }
      }
    }

    //Pointer to the world
    private: physics::WorldPtr world;

    // Pointer to the model
    private: physics::ModelPtr model;

    private: physics::ModelPtr model2;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: double speed;

    private: double begin;

    private: bool presence_sensor;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(ModelPush)
}
