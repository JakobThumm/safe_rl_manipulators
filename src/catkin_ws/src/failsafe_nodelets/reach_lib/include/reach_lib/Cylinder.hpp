#include "Point.hpp"

#ifndef CYLINDER_H
#define CYLINDR_H

class Cylinder{
    private:

    public:
        Point p1;
        Point p2;
        double r;

        Cylinder();

        Cylinder(Point p1, Point p2, double r);

        ~Cylinder();

        std::string to_string();

};
#endif