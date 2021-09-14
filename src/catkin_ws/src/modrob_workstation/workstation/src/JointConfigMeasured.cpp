#include "JointConfigMeasured.hpp"

uint16_t JointConfigMeasured::getReadSize() {
    return 4 * sizeof(double) + sizeof(float);
}

double JointConfigMeasured::getJointAcceleration() {
    return this->jointAcceleration;
}

double JointConfigMeasured::getJointAngle() {
    return this->jointAngle;
}

double JointConfigMeasured::getJointVelocity() {
    return this->jointVelocity;
}

double JointConfigMeasured::getJointTorque() {
    return this->jointTorque;
}

double JointConfigMeasured::getJointTemperature() {
    return this->jointTemperature;
}

JointConfigMeasured JointConfigMeasured::deserialize(char *source) {
    double jointAngle = *((double *) source + 0 * sizeof(double));
    double jointVelocity = *((double *) source + 1 * sizeof(double));
    double jointAcceleration = *((double *) source + 2 * sizeof(double));
    double jointTorque = *((double *) source + 3 * sizeof(double));
    double jointTemperature = (double) *((float *) source + 4 * sizeof(double));
    return JointConfigMeasured(jointAngle, jointVelocity, jointAcceleration, jointTorque, jointTemperature);
}

JointConfigMeasured::JointConfigMeasured(double jointAngle, double jointVelocity, double jointAcceleration, double jointTorque, double jointTemperature) {
    this->jointAngle = jointAngle;
    this->jointVelocity = jointVelocity;
    this->jointAcceleration = jointAcceleration;
    this->jointTorque = jointTorque;
    this->jointTemperature = jointTemperature;
}