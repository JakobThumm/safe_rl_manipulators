#include "RobotModuleOrder.hpp"

uint16_t RobotModuleOrder::getReadSize() {
    return this->modules.size();
}

std::vector<uint8_t> RobotModuleOrder::getModules() {
    return this->modules;
}

RobotModuleOrder::RobotModuleOrder(std::vector<uint8_t> modules) {
    this->modules = modules;
}

RobotModuleOrder RobotModuleOrder::deserialize(char *source, uint8_t numberOfJoints) {
    std::vector<uint8_t> modules;
    for (int i = 0; i < numberOfJoints; i++) {
        modules.push_back(*((uint8_t *) source++));
    }

    return RobotModuleOrder(modules);
}