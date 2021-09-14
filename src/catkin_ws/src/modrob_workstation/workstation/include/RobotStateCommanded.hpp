#ifndef WORKSTAION_ROBOTSTATECOMMANDED_H
#define WORKSTAION_ROBOTSTATECOMMANDED_H

#include <cstdint>
#include "RobotCommanded.hpp"

class RobotStateCommanded : public RobotCommanded {
    uint8_t state;
public:

    RobotStateCommanded(uint8_t state);

    char *serialize(char *result) override;

    uint16_t getSize() override;
};

#endif