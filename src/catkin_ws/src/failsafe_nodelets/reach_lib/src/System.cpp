#include "reach_lib/System.hpp"


System::System(){
    this->measurement_error_pos = 0.0;
    this->measurement_error_vel = 0.0;
    this->delay = 0.0;
}

System::System(double measurement_error_pos, double measurement_error_vel, double delay){
    this->measurement_error_pos = measurement_error_pos;
    this->measurement_error_vel = measurement_error_vel;
    this->delay = delay;
}

System::~System(){};