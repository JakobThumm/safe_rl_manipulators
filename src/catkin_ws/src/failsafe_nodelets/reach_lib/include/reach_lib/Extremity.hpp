#include "BodyPart.hpp"

#ifndef EXTREMITY_H
#define EXTREMITY_H


/*
 * Body part modeled with a maximum position approach (Basically arm length).
 */
class Extremity : public BodyPart{
private:

public:    
    /// \brief Empty constructor
    Extremity():BodyPart(){}

    /// \brief Extremity constructor
    /// \param[in] name The name of the body part.
    /// \param[in] thickness The radius of the hand (we recommend 0.205).
    /// \param[in] max_v_proximal Max velocity of the shoulder(!!!) joint.
    Extremity(std::string name, double thickness, double max_v_proximal):
        BodyPart(name, thickness, max_v_proximal, 0){}

    /// \brief Destructor
    ~Extremity(){}

    /// \brief Calculate the reachable set of a single human body part
    ///         according to the Cartesian reachablity approach presented in
    ///         https://doi-org.eaccess.ub.tum.de/10.1109/IROS.2017.8206314
    ///         See chapter III) B) 4)
    /// \param[in] p_s The shoulder joint position
    /// \param[in] p_e The elbow joint position
    /// \param[in] p_w The wrist joint position
    /// \param[in] t_break Time horizon of reachability analysis
    /// \param[in] measurement_error_pos Maximal positional measurement error
    /// \param[in] delay Delay in measurement processing pipeline
    Capsule CalculateReach(const Point& p_s, const Point& p_e, const Point& p_w, 
                           double t_break, double measurement_error_pos = 0, double delay = 0);
};
#endif