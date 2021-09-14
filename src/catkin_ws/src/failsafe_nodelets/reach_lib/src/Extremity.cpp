#include "reach_lib/Extremity.hpp"

Capsule Extremity::CalculateReach(const Point& p_s, const Point& p_e, const Point& p_w, 
                           double t_break, double measurement_error_pos, double delay){
    // Movement of the shoulder
    double radius = this->max_v_proximal * (t_break + delay);
    // Current measured upper arm length
    radius += Point::norm(p_s, p_e);
    // Current measured lower arm length
    radius += Point::norm(p_e, p_w);
    // Measurement error
    radius += measurement_error_pos;
    // Size of hand
    radius += this->thickness;
    return Capsule(p_s, p_s, radius);                    
}