#include <cstdint>

#ifndef WORKSTAION_ROBOTCOMMANDED_H
#define WORKSTAION_ROBOTCOMMANDED_H



class RobotCommanded {
protected:
    uint8_t type;

public:

    uint8_t getType();

    virtual char *serialize(char *result) = 0;

    virtual uint16_t getSize() = 0;
};

#endif