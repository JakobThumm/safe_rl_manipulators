#include "reach_lib/Cylinder.hpp"


Cylinder::Cylinder(){
    this->p1 = Point();
    this->p2 = Point();
    this->r = 0.0;
}

Cylinder::Cylinder(Point p1, Point p2, double r){
    this->p1 = p1;
    this->p2 = p2;
    this->r = r;
}

Cylinder::~Cylinder(){};

std::string Cylinder::to_string(){
    return "\np1: " + this->p1.to_string() + "\np2: " + this->p2.to_string() + "\nr: " + std::to_string(this->r);
}