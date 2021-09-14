#include <utility>
#include "RobotTorqueCommanded.hpp"

#define TORQUE_COMMAND_MESSAGE_TYPE 8

char* RobotTorqueCommanded::serialize(char *result) {
    *result++ = this->toolActivation;

    for (auto &jointTorque : this->jointTorques) {
        result = jointTorque.serialize(result);
    }

    return result;
}

uint16_t RobotTorqueCommanded::getSize() {
    return sizeof(bool) + jointTorques.size() * JointTorqueCommanded::getSize();
}

RobotTorqueCommanded::RobotTorqueCommanded(bool toolActivation, std::vector<JointTorqueCommanded> jointTorques) {
    this->toolActivation = toolActivation;
    this->jointTorques = std::move(jointTorques);
    this->type = TORQUE_COMMAND_MESSAGE_TYPE;
}
