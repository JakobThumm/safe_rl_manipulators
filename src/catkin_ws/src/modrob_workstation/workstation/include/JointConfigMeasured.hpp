#include <cstdint>

#ifndef WORKSTAION_JOINTCONFIGMEASURED_H
#define WORKSTAION_JOINTCONFIGMEASURED_H


class JointConfigMeasured {
private:
    double jointAngle;
    double jointVelocity;
    double jointAcceleration;
    double jointTorque;
    double jointTemperature;

public:

    JointConfigMeasured(double jointAngle, double jointVelocity, double jointAcceleration, double jointTorque, double jointTemperature);

    double getJointAngle();

    double getJointVelocity();

    double getJointAcceleration();

    double getJointTorque();

    double getJointTemperature();

    static JointConfigMeasured deserialize(char *result);

    static uint16_t getReadSize();
};


#endif //WORKSTAION_JOINTANGLECOMMAND_H
