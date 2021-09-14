#include "BodyPart.hpp"

#ifndef BODYPART_V_H
#define BODYPART_V_H


/*
 * Body part modeled with a maximum velocity approach.
 */
class BodyPart_V : public BodyPart{
private:
    /// \brief Calculate the reachable set of a single capsule
    ///         according to the Cartesian reachablity approach presented in
    ///         https://doi-org.eaccess.ub.tum.de/10.1109/IROS.2017.8206314
    ///         See chapter III) B) 3)
    /// \param[in] p The joint cartesian position
    /// \param[in] t Time horizon of reachability analysis
    /// \param[in] max_v The max velocity of this joint
    /// \param[in] measurement_error_pos Maximal positional measurement error
    Capsule ReachCapsule(const Point& p, double t, double max_v, double measurement_error_pos = 0);

public:    
    /// \brief Empty constructor
    BodyPart_V():BodyPart(){}

    /// \brief BodyPart_V constructor
    /// \param[in] name The name of the body part.
    /// \param[in] thickness The radius of the body part enclosing capsule.
    /// \param[in] max_v_proximal Max cartesian velocity of proximal joint.
    /// \param[in] max_v_distal Max cartesian velocity of distal joint.
    BodyPart_V(std::string name, double thickness, double max_v_proximal, double max_v_distal):
        BodyPart(name, thickness, max_v_proximal, max_v_distal){}

    /// \brief Destructor
    ~BodyPart_V(){}

    /// \brief Calculate the reachable set of a single human body part
    ///         according to the Cartesian reachablity approach presented in
    ///         https://doi-org.eaccess.ub.tum.de/10.1109/IROS.2017.8206314
    ///         See chapter III) B) 3)
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