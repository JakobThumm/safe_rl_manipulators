#include "reach_lib/BodyPart_V.hpp"


Capsule BodyPart_V::CalculateReach(const Point& p1, const Point& p2, const Point& v1, const Point& v2,
                           double t_break, double measurement_error_pos, double measurement_error_vel, double delay){
    Capsule reach_set;
    // Calculate ball enclosure of proximal joint
    Capsule rp1_t1 = ReachCapsule(p1, 0, this->max_v_proximal, measurement_error_pos);
    Capsule rp1_t2 = ReachCapsule(p1, t_break + delay, this->max_v_proximal, measurement_error_pos);
    Capsule b1 = Capsule::ballEnclosure(rp1_t1, rp1_t2);

    // If capsule, also calculate ball enclosure of distal joint
    if(!(p1 == p2)){
        Capsule rp2_t1 = ReachCapsule(p2, 0, this->max_v_distal, measurement_error_pos);
        Capsule rp2_t2 = ReachCapsule(p2, t_break + delay, this->max_v_distal, measurement_error_pos);
        Capsule b2 = Capsule::ballEnclosure(rp2_t1, rp2_t2);

        // Add min thickness
        if(b1.r >= b2.r){
            b1.r += this->thickness;
        }else{
            b2.r += this->thickness;
        }            

        reach_set = Capsule::capsuleEnclosure(b1, b2);
    }else{
        reach_set = rp1_t1;
        // Add min thickness
        reach_set.r += this->thickness;
    }
    return reach_set;
}

Capsule BodyPart_V::ReachCapsule(const Point& p, double t, double max_v, double measurement_error_pos){
    return Capsule(p, p, measurement_error_pos + max_v * t);
}