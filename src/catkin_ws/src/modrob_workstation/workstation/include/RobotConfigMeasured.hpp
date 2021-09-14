#include "RobotCommanded.hpp"
#include "JointConfigMeasured.hpp"
#include <vector>
#include <cstdint>
#ifndef WORKSTAION_ROBOTCONFIGMEASURED_H
#define WORKSTAION_ROBOTCONFIGMEASURED_H


class RobotConfigMeasured {
private:
    bool toolActivation;
    uint8_t state;
    std::vector<JointConfigMeasured> jointMeasurements;

public:

    bool getToolActivation();

    uint8_t getState();

    std::vector<JointConfigMeasured> getJointMeasurements();

    static RobotConfigMeasured deserialize(char * source, uint8_t payloadLength);

    RobotConfigMeasured(bool toolActivation, uint8_t state, std::vector<JointConfigMeasured> jointMeasurements);
    

    uint16_t getReadSize();

};


#endif
