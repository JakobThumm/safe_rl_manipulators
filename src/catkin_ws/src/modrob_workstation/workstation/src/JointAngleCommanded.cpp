//
// Created by lukas on 28.01.20.
//

#include "JointAngleCommanded.hpp"

int JointAngleCommanded::getSize() {
    return 3 * sizeof(double);
}

char* JointAngleCommanded::serialize(char *result) {
    double * d_result = ((double *) result);
    *d_result++ = jointAngle;
    *d_result++ = jointVelocity;
    *d_result++ = jointAcceleration;
    return (char *) d_result;
}

JointAngleCommanded::JointAngleCommanded(double jointAngle, double jointVelocity, double jointAcceleration) {
    this->jointAngle = jointAngle;
    this->jointVelocity = jointVelocity;
    this->jointAcceleration = jointAcceleration;
}