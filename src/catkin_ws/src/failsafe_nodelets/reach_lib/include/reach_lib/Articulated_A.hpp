#include "Articulated.hpp"
#include "BodyPart_A.hpp"


#ifndef ARTICULATED_A_H
#define ARTICULATED_A_H

/*
 * This class calculates the human reachability using a 
 * maximum acceleration approach of the joints.
 */
class Articulated_A  : public Articulated{
private:
    /// \brief All body parts (links) for which we want to calculate the reachability analysis
    std::vector<BodyPart_A> bodies;

    /// \brief The maximum velocity of the joints
    std::vector<double> max_v;

    /// \brief The maximum acceleration of the joints
    std::vector<double> max_a;

public:
    /// \brief Empty constructor
    Articulated_A():Articulated(){}

    /// \brief Constructor
    /// \param[in] measurement_error_pos Maximal positional measurement error
    /// \param[in] measurement_error_vel Maximal velocity measurement error
    /// \param[in] delay Delay in measurement processing pipeline
    /// \param[in] joint_pair_map Maps the proximal and distal joint to a body part identified by a string (key: Name of body part, value: Proximal and distal joint index)
    /// \param[in] thickness Defines the thickness of the body parts (key: Name of body part, value: Thickness of body part)
    /// \param[in] max_v The maximum velocity of the joints
    /// \param[in] max_a The maximum acceleration of the joints
    Articulated_A(double measurement_error_pos, double measurement_error_vel, double delay, 
                std::map<std::string, human_reach::jointPair>& joint_pair_map, std::map<std::string, double>& thickness, 
                std::vector<double>& max_v, std::vector<double>& max_a);

    ~Articulated_A(){}

    /// \brief Return reachability analysis mode
    std::string get_mode() override {
        return "ACCEL";
    }

    /// \brief Return the body parts
    std::vector<BodyPart_A> get_body(){
        return this->bodies;
    }

    /// \brief Calculate the reachable set of the total human body with the acceleration approach.
    /// \param[in] p The joint positions in cartesian world coordinate frame (x, y, z) 
    /// \param[in] t_break The time horizon of the reachability analysis
    /// \param[in] v The joint velocities (difference between positions)
    std::vector<Capsule> update(std::vector<Point> p, double t_break, std::vector<Point> v={}) override;
};
#endif