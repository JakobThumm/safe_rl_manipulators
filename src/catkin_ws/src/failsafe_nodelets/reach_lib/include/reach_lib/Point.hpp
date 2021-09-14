#include <string>
#include <cmath>
#include <tuple>
#include "geometry_msgs/Point.h"

#ifndef POINT_H
#define POINT_H



class Point{
private:
public:
    double x;
    double y;
    double z;

    Point();
    Point(double x, double y, double z);
    ~Point();

    static Point diff(Point p1, Point p2);

    static Point add(Point p1, Point p2);

    static Point origin(Point p1, Point p2);

    static double norm(Point p1);

    static double norm(Point p1, Point p2);

    static double inner_dot(Point p1, Point p2);

    static Point cross(Point p1, Point p2);

    static std::tuple<double,double,double,double> orientation(Point p1, Point p2);

    std::string to_string();

    friend bool operator== (const Point& p1, const Point& p2){
        if(p1.x == p2.x && p1.y == p2.y && p1.z == p2.z){
            return true;
        }else{
            return false;
        }
    }

    geometry_msgs::Point toPointMsg();
};


#endif