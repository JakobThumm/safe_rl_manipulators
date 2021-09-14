#include <cmath>
#include <string>
#include <tuple>
#include <iostream>
#include <vector>
#include <map>
#include <stdexcept>
#include "Capsule.hpp"
#include "Point.hpp"

#ifndef BODYPART_H
#define BODYPART_H


/*
 * Base class of Body Part.
 * This class calculates the motion of a human body part.
 * A human body part consists of two points and a thickness creating a capsule.
 */
class BodyPart{
protected:
    /// \brief The name of the body part.
    std::string name;

    /// \brief The radius of the body part enclosing capsule.
    double thickness = 1;

    /// \brief Max cartesian velocity of proximal joint.  
    double max_v_proximal = 2;

    /// \brief Max cartesian velocity of distal joint.  
    double max_v_distal = 2;
    
public:    
    /// \brief Empty constructor
    BodyPart(){}

    /// \brief BodyPart constructor
    /// \param[in] name The name of the body part.
    /// \param[in] thickness The radius of the body part enclosing capsule.
    /// \param[in] max_v_proximal Max cartesian velocity of proximal joint.
    /// \param[in] max_v_distal Max cartesian velocity of distal joint.
    BodyPart(std::string name, double thickness, double max_v_proximal, double max_v_distal);

    /// \brief Destructor
    ~BodyPart(){}

    /// \brief Calculate the reachable set of a single human body part
    ///         according to the Cartesian reachablity approach presented in
    ///         https://doi-org.eaccess.ub.tum.de/10.1109/IROS.2017.8206314
    ///         Velocity and acceleration approach only.
    /// \param[in] p1 The proximal joint position
    /// \param[in] p2 The distal joint position
    /// \param[in] v1 The proximal joint velocity
    /// \param[in] v2 The distal joint velocity
    /// \param[in] t_break Time horizon of reachability analysis
    /// \param[in] measurement_error_pos Maximal positional measurement error
    /// \param[in] measurement_error_vel Maximal velocity measurement error
    /// \param[in] delay Delay in measurement processing pipeline
    Capsule CalculateReach(const Point& p1, const Point& p2, const Point& v1, const Point& v2,
                             double t_break, double measurement_error_pos = 0, double measurement_error_vel = 0, double delay = 0){
        throw "Function CalculateReach not defined.";
    }

    /// \brief Calculate the reachable set of a single human body part
    ///         according to the Cartesian reachablity approach presented in
    ///         https://doi-org.eaccess.ub.tum.de/10.1109/IROS.2017.8206314
    ///         Position approach only
    /// \param[in] p_s The shoulder joint position
    /// \param[in] p_e The elbow joint position
    /// \param[in] p_w The wrist joint position
    /// \param[in] t_break Time horizon of reachability analysis
    /// \param[in] measurement_error_pos Maximal positional measurement error
    /// \param[in] delay Delay in measurement processing pipeline
    Capsule CalculateReach(const Point& p_s, const Point& p_e, const Point& p_w, 
                           double t_break, double measurement_error_pos = 0, double delay = 0){
        throw "Function CalculateReach not defined.";
    }

    /// \brief Return body part name
    std::string Name(){
        return name;
    }
};
#endif