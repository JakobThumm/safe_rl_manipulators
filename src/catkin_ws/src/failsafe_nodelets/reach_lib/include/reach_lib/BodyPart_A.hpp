#include "BodyPart.hpp"

#ifndef BODYPART_A_H
#define BODYPART_A_H


/*
 * Body part modeled with a maximum acceleration approach.
 */
class BodyPart_A : public BodyPart{
private:
    /// \brief Max cartesian acceleration of proximal joint.  
    double max_a_proximal = 1;

    /// \brief Max cartesian acceleration of distal joint.  
    double max_a_distal = 1;

    /// \brief Calculate the reachable set of a single capsule
    ///         according to the Cartesian reachablity approach presented in
    ///         https://doi-org.eaccess.ub.tum.de/10.1109/IROS.2017.8206314
    ///         See chapter III) B) 2)
    /// \param[in] p The joint cartesian position
    /// \param[in] p The joint cartesian velocity
    /// \param[in] t Time horizon of reachability analysis
    /// \param[in] max_a The max acceleration of this joint
    /// \param[in] measurement_error_pos Maximal positional measurement error
    /// \param[in] measurement_error_vel Maximal velocity measurement error
    Capsule ReachCapsule(const Point& p, const Point& v, double t, double max_a, double measurement_error_pos = 0, double measurement_error_vel = 0);

public:    
    /// \brief Empty constructor
    BodyPart_A():BodyPart(){}

    /// \brief BodyPart_A constructor
    /// \param[in] name The name of the body part.
    /// \param[in] thickness The radius of the body part enclosing capsule.
    /// \param[in] max_v_proximal Max cartesian velocity of proximal joint.
    /// \param[in] max_v_distal Max cartesian velocity of distal joint.
    BodyPart_A(std::string name, double thickness, double max_v_proximal, double max_v_distal, 
                           double max_a_proximal, double max_a_distal):
        BodyPart(name, thickness, max_v_proximal, max_v_distal),
        max_a_proximal(max_a_proximal),
        max_a_distal(max_a_distal)
        {}

    /// \brief Destructor
    ~BodyPart_A(){}

    /// \brief Calculate the reachable set of a single human body part
    ///         according to the Cartesian reachablity approach presented in
    ///         https://doi-org.eaccess.ub.tum.de/10.1109/IROS.2017.8206314
    ///         See chapter III) B) 2)
    /// \param[in] p1 The proximal joint position
    /// \param[in] p2 The distal joint position
    /// \param[in] v1 The proximal joint velocity
    /// \param[in] v2 The distal joint velocity
    /// \param[in] t_break Time horizon of reachability analysis
    /// \param[in] measurement_error_pos Maximal positional measurement error
    /// \param[in] measurement_error_vel Maximal velocity measurement error
    /// \param[in] delay Delay in measurement processing pipeline
    Capsule CalculateReach(const Point& p1, const Point& p2, const Point& v1, const Point& v2,
                           double t_break, double measurement_error_pos = 0, double measurement_error_vel = 0, double delay = 0);
};
#endif