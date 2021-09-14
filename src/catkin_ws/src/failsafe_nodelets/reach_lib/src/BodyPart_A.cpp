#include "reach_lib/BodyPart_A.hpp"


Capsule BodyPart_A::CalculateReach(const Point& p1, const Point& p2, const Point& v1, const Point& v2,
                           double t_break, double measurement_error_pos, double measurement_error_vel, double delay){
    Capsule reach_set;
    // Calculate ball enclosure of proximal joint
    Capsule rp1_t1 = ReachCapsule(p1, v1, 0, this->max_a_proximal, measurement_error_pos, measurement_error_vel);
    Capsule rp1_t2 = ReachCapsule(p1, v1, t_break + delay, this->max_a_proximal, measurement_error_pos, measurement_error_vel);
    Capsule b1 = Capsule::ballEnclosure(rp1_t1, rp1_t2);

    // If capsule, also calculate ball enclosure of distal joint
    if(!(p1 == p2)){
        Capsule rp2_t1 = ReachCapsule(p2, v2, 0, this->max_a_distal, measurement_error_pos, measurement_error_vel);
        Capsule rp2_t2 = ReachCapsule(p2, v2, t_break + delay, this->max_a_distal, measurement_error_pos, measurement_error_vel);
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

Capsule BodyPart_A::ReachCapsule(const Point& p, const Point& v, double t, double max_a, double measurement_error_pos, double measurement_error_vel){
    // Calculate distance traveled in time t assuming constant velocity.
    Point d_v_const;
    d_v_const.x = v.x * t;
    d_v_const.y = v.y * t;
    d_v_const.z = v.z * t;
    Point zero = Point();
    // Include the measurement error
    Capsule sphere_start = Capsule(p, p, measurement_error_pos);
    // Integrate over velocity meas error
    Capsule sphere_end = Capsule(d_v_const, d_v_const, measurement_error_vel * t);
    // Assume max acceleration
    Capsule acceleration_sphere = Capsule(zero, zero, max_a / 2.0 * pow(t, 2));
    // Minkowski add all motions and errors.
    Capsule Bydy = Capsule::minkowski(sphere_start, sphere_end);
    return Capsule::minkowski(Bydy, acceleration_sphere);
}