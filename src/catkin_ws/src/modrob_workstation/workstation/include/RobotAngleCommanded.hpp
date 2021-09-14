//
// Created by lukas on 28.01.20.
//
#include "RobotCommanded.hpp"
#include "JointAngleCommanded.hpp"
#include <vector>
#ifndef WORKSTAION_ROBOTANGLECOMMAND_H
#define WORKSTAION_ROBOTANGLECOMMAND_H


class RobotAngleCommanded : public RobotCommanded {
private:
    bool toolActivation;
    std::vector<JointAngleCommanded> jointAngles;
public:

public:

    RobotAngleCommanded(bool toolActivation, std::vector<JointAngleCommanded> jointTorques);

    char *serialize(char * result) override;

    uint16_t getSize() override;

};


#endif //WORKSTAION_ROBOTANGLECOMMAND_H
