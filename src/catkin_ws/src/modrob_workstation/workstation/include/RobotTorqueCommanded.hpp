//
// Created by lukas on 28.01.20.
//

#include "RobotCommanded.hpp"
#include "JointTorqueCommanded.hpp"
#include <vector>

#ifndef WORKSTAION_ROBOTTORQUECOMMAND_H
#define WORKSTAION_ROBOTTORQUECOMMAND_H


class RobotTorqueCommanded : public RobotCommanded {
private:
    bool toolActivation;
    std::vector<JointTorqueCommanded> jointTorques;

public:

    RobotTorqueCommanded(bool toolActivation, std::vector<JointTorqueCommanded> jointTorques);

    char *serialize(char * result) override;

    uint16_t getSize() override;

};


#endif //WORKSTAION_ROBOTTORQUECOMMAND_H
