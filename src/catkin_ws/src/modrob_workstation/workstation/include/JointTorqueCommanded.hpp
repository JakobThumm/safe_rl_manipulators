//
// Created by lukas on 28.01.20.
//

#ifndef WORKSTAION_JOINTTORQUECOMMAND_H
#define WORKSTAION_JOINTTORQUECOMMAND_H


class JointTorqueCommanded {
private:
    double jointTorque;

public:

    JointTorqueCommanded(double jointTorque);

    char *serialize(char *result);

    static int getSize();
};


#endif //WORKSTAION_JOINTTORQUECOMMAND_H
