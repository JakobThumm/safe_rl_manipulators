#include "reach_lib/Articulated.hpp"


Articulated::~Articulated(){
    
}

Articulated::Articulated(double measurement_error_pos, double measurement_error_vel, double delay, 
                         std::map<std::string, human_reach::jointPair>& joint_pair_map):
    measurement_error_pos(measurement_error_pos),
    measurement_error_vel(measurement_error_vel),
    delay(delay),
    joint_pair_map(joint_pair_map)
{
    // Nothing to do here.
}