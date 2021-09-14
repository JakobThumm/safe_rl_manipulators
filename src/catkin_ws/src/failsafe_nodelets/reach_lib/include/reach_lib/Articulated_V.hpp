#include "Articulated.hpp"
#include "BodyPart_V.hpp"

#ifndef ARTICULATED_V_H
#define ARTICULATED_V_H

/*
 * This class calculates the human reachability using a 
 * maximum velocity approach of the joints.
 */
class Articulated_V : public Articulated{
private:
    /// \brief All body parts (links) for which we want to calculate the reachability analysis
    std::vector<BodyPart_V> bodies;

    /// \brief The maximum velocity of the joints
    std::vector<double> max_v;

public:
    /// \brief Empty constructor
    Articulated_V():Articulated(){}

    /// \brief Constructor
    /// \param[in] measurement_error_pos Maximal positional measurement error
    /// \param[in] measurement_error_vel Maximal velocity measurement error
    /// \param[in] delay Delay in measurement processing pipeline
    /// \param[in] joint_pair_map Maps the proximal and distal joint to a body part identified by a string (key: Name of body part, value: Proximal and distal joint index)
    /// \param[in] thickness Defines the thickness of the body parts (key: Name of body part, value: Thickness of body part)
    /// \param[in] max_v The maximum velocity of the joints
    Articulated_V(double measurement_error_pos, double measurement_error_vel, double delay, 
                std::map<std::string, human_reach::jointPair>& joint_pair_map, std::map<std::string, double>& thickness, 
                std::vector<double>& max_v);

    /// \brief Destructor
    ~Articulated_V(){}

    /// \brief Return reachability analysis mode
    std::string get_mode() override {
        return "VEL";
    }

    /// \brief Return the body parts
    std::vector<BodyPart_V> get_body(){
        return this->bodies;
    }

    /// \brief Calculate the reachable set of the total human body with the velocity approach.
    /// \param[in] p The joint positions in cartesian world coordinate frame (x, y, z) 
    /// \param[in] t_break The time horizon of the reachability analysis
    /// \param[in] v The joint velocities (difference between positions)
    std::vector<Capsule> update(std::vector<Point> p, double t_break, std::vector<Point> v={}) override;
};
#endif