#include "RobotConfigMeasured.hpp"

std::vector<JointConfigMeasured> RobotConfigMeasured::getJointMeasurements() {
    return this->jointMeasurements;
}

uint8_t RobotConfigMeasured::getState() {
    return this->state;
}

bool RobotConfigMeasured::getToolActivation() {
    return this->toolActivation;
}

uint16_t RobotConfigMeasured::getReadSize() {
    return sizeof(bool) + sizeof(uint8_t) + this->jointMeasurements.size() * JointConfigMeasured::getReadSize();
}

RobotConfigMeasured::RobotConfigMeasured(bool toolActivation, uint8_t state, std::vector<JointConfigMeasured> jointMeasurements) {
    this->toolActivation = toolActivation;
    this->state = state;
    this->jointMeasurements = jointMeasurements;

}

RobotConfigMeasured RobotConfigMeasured::deserialize(char *source, uint8_t numberOfJoints) {
    uint8_t state = *((uint8_t *) source++);
    bool toolActivation = *((bool *) source++);
    std::vector<JointConfigMeasured> jointMeasurements;

    for (int i = 0; i < numberOfJoints; i++) {
        jointMeasurements.push_back(JointConfigMeasured::deserialize(source));
        source += JointConfigMeasured::getReadSize();
    }
    
    return RobotConfigMeasured(toolActivation, state, jointMeasurements);
}