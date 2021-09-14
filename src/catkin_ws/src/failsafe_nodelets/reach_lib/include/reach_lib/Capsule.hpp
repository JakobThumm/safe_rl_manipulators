#include <string>
#include <vector>
#include <tuple>
#include <iostream>
#include <cmath>
#include "Point.hpp"
#include "custom_robot_msgs/Capsule.h"

#ifndef CAPSULE_H
#define CAPSULE_H




class Capsule{
    private:
        static std::tuple<double,double> alphaBeta(double ri, double rj, Point p);

        static Point pk(Point pj, Point x, double b);

        static bool point_in_capsule(Capsule c, Point p, double r=0.0);

        static bool point_in_ball(Capsule c, Point p, double r=0.0);

        static bool capsule_capsule_intersection(Capsule c1, Capsule c2, double s=0.0);
        
    public:
        Point p1;
        Point p2;
        double r;

        Capsule();

        Capsule(Point p1, Point p2, double r);

        ~Capsule();

        static Capsule minkowski(Capsule c1, Capsule c2);

        static Capsule capsuleEnclosure(Capsule c1, Capsule c2);

        static Capsule ballEnclosure(Capsule c1, Capsule c2);

        static bool intersection(Capsule c1, Capsule c2);

        bool point_in_reach(Point p, double r=0.0);

        std::string to_string();

        custom_robot_msgs::Capsule toCapsuleMsg();      
};


#endif