//
// Created by lukas on 28.01.20.
//

#include "JointTorqueCommanded.hpp"

int JointTorqueCommanded::getSize() {
    return sizeof(double);
}

char* JointTorqueCommanded::serialize(char *result) {
    double *d_result = ((double *) result);
    *d_result++ = this->jointTorque;
    return (char *) d_result;
}

JointTorqueCommanded::JointTorqueCommanded(double jointTorque)  {
    this->jointTorque = jointTorque;
}