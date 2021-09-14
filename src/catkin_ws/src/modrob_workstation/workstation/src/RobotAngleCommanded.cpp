#include <utility>
#include "RobotAngleCommanded.hpp"

#define ANGLE_COMMAND_MESSAGE_TYPE 7

char* RobotAngleCommanded::serialize(char *result) {
    *result++ = this->toolActivation;

    for (auto &jointAngle : this->jointAngles) {
        result = jointAngle.serialize(result);
    }

    return result;
}

uint16_t RobotAngleCommanded::getSize() {
    return sizeof(bool) + this->jointAngles.size() * JointAngleCommanded::getSize();
}

RobotAngleCommanded::RobotAngleCommanded(bool toolActivation, std::vector<JointAngleCommanded> jointAngles) {
    this->toolActivation = toolActivation;
    this->jointAngles = std::move(jointAngles);
    this->type = ANGLE_COMMAND_MESSAGE_TYPE;
}
