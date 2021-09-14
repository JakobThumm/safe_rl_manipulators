#include <cstdint>
#include <vector>

#ifndef WORKSTAION_MODULEORDER_H
#define WORKSTAION_MODULEORDER_H



class RobotModuleOrder {
private:
    std::vector<uint8_t> modules;

public:

    std::vector<uint8_t> getModules();

    static RobotModuleOrder deserialize(char *source, uint8_t numberOfJoints);

    uint16_t getReadSize();

    RobotModuleOrder(std::vector<uint8_t> modules);
};

#endif