#include "workstation_adapter.hpp"
#include <vector>
#include <stdint.h>
#include <unistd.h>


WorkstationAdapter::WorkstationAdapter(const char *targetIP, const std::function<void(RobotConfigMeasured)>& receiveConfigMeasured, const std::function<void(RobotModuleOrder)>& receiveModuleOrder)
:workstation(targetIP,receiveConfigMeasured,receiveModuleOrder){

}

void WorkstationAdapter::sendConfig(const modrob_workstation::RobotConfigCommanded& config){
    std::cout << "RobotConfigCommanded received.." << std::endl;
    // parse config to RobotAngleCommanded
    RobotAngleCommanded command = parseToRobotAngleCommanded(config);
    // send the config to the Robot Computer
    workstation.send(&command);
}

void WorkstationAdapter::sendState(const modrob_workstation::RobotStateCommanded& state){
    std::cout << "RobotStateCommanded received.." << std::endl;
    // parse config to RobotStateCommanded
    RobotStateCommanded command = parseToRobotStateCommanded(state);
    // send the state to the Robot Computer
    workstation.send(&command);
}

RobotAngleCommanded WorkstationAdapter::parseToRobotAngleCommanded(const modrob_workstation::RobotConfigCommanded& config) {
  
   std::vector<JointAngleCommanded> joint_moves;

   // initialize the joint_moves array of the RobotAngleCommanded
   for(int i = 0; i < config.joint_moves.size(); i++) {
      JointAngleCommanded joint(config.joint_moves[i].joint_angle, config.joint_moves[i].joint_velocity, config.joint_moves[i].joint_acceleration);
      joint_moves.push_back(joint);
   }

   // initialize the RobotAngleCommanded
   RobotAngleCommanded out(config.tool_activation, joint_moves);

   return out;
}

RobotTorqueCommanded parseToRobotTorqueCommanded(const modrob_workstation::RobotConfigCommanded& config) {
  
   std::vector<JointTorqueCommanded> joint_moves;
   // initialize the joint_moves array of the RobotTorqueCommanded
   for(int i = 0; i < config.joint_moves.size(); i++) {
      JointTorqueCommanded joint(config.joint_moves[i].joint_torque);
      joint_moves.push_back(joint);
   }

   // initialize the RobotTorqueCommanded
   RobotTorqueCommanded out(config.tool_activation, joint_moves);

   return out;
}

RobotStateCommanded WorkstationAdapter::parseToRobotStateCommanded(const modrob_workstation::RobotStateCommanded& state){
  // initialize the RobotStateCommanded
  RobotStateCommanded out(state.state);
  return out;
}


modrob_workstation::RobotConfigMeasured WorkstationAdapter::parseToConfigMeasured(RobotConfigMeasured config){
  
  std::vector<modrob_workstation::JointConfigMeasured> joint_configuration;

  modrob_workstation::RobotConfigMeasured out;

  // initialize the joint_configuration Vector of RobotConfigMeasured
  for(int i = 0; i < config.getJointMeasurements().size(); i++) {



      modrob_workstation::JointConfigMeasured joint;

      joint.joint_angle = config.getJointMeasurements()[i].getJointAngle();
      joint.joint_velocity = config.getJointMeasurements()[i].getJointVelocity();
      joint.joint_acceleration = config.getJointMeasurements()[i].getJointAcceleration();
      joint.joint_torque = config.getJointMeasurements()[i].getJointTorque();
      joint.joint_temperature = config.getJointMeasurements()[i].getJointTemperature();

      
      if (i == 0){
         // for the first joint the Vector needs to be initialized
        out.joint_config_measured = {joint};

      } else {
         // for all other joints the jointConifigMeasured needs to be pushed
        out.joint_config_measured.push_back(joint);
      }
  }
  
  // intitialize Measured_robot_state
  out.measured_robot_state = config.getState();
  // initialize tool_activation
  out.tool_activation = config.getToolActivation();

  return out;
}



modrob_workstation::ModuleOrder WorkstationAdapter::parseToModuleOrder(RobotModuleOrder moduleOrder){
  
   modrob_workstation::ModuleOrder out;

   // initialize the moduleOrder Vector of ModuleOrder
   for(int i = 0; i < moduleOrder.getModules().size(); i++) {

       int8_t module = moduleOrder.getModules()[i];

       if (i == 0){
        // for the first module the Vector needs to be initialized
        out.modules = {module};
       } else {
       // for all other modules the module needs to be pushed
        out.modules.push_back(module);
       }
   }

   return out;
}




















