//
// Created by lukas on 28.01.20.
//

#ifndef WORKSTAION_JOINTANGLECOMMAND_H
#define WORKSTAION_JOINTANGLECOMMAND_H


class JointAngleCommanded {
private:
    double jointAngle;
    double jointVelocity;
    double jointAcceleration;

public:

    JointAngleCommanded(double jointAngle, double jointVelocity, double jointAcceleration);

    char *serialize(char *result);

    static int getSize();

};


#endif //WORKSTAION_JOINTANGLECOMMAND_H
