#include "Articulated.hpp"
#include "Extremity.hpp"


#ifndef ARTICULATED_P_H
#define ARTICULATED_P_H

/*
 * This class calculates the human reachability using the 
 * max reachable position of extremeties (basically arm length)
 */
class Articulated_P : public Articulated{
private:
    /// \brief All extremities for which we want to calculate the reachability analysis
    std::vector<Extremity> extremities;

    /// \brief The indices of the left and right shoulder joints 
    std::vector<double> shoulder_ids;

    /// \brief The indices of the left and right elbow joints
    std::vector<double> elbow_ids;

public:
    /// \brief Empty constructor
    Articulated_P():Articulated(){}

    /// \brief Constructor of position Articulator
    /// \param[in] measurement_error_pos Maximal positional measurement error
    /// \param[in] measurement_error_vel Maximal velocity measurement error
    /// \param[in] delay Delay in measurement processing pipeline
    /// \param[in] joint_pair_map Maps the proximal and distal joint to a body part identified by a string (key: Name of body part, value: Proximal and distal joint index)
    /// \param[in] thickness Defines the thickness of the body parts (key: Name of body part, value: Thickness of body part)
    /// \param[in] max_v The maximum velocity of the joints
    /// \param[in] shoulder_ids The indices of the left and right shoulder joints 
    /// \param[in] elbow_ids The indices of the left and right elbow joints 
    /// \param[in] wrist_names The name identifiers of the two hands
    Articulated_P(double measurement_error_pos, double measurement_error_vel, double delay, 
                std::map<std::string, human_reach::jointPair>& joint_pair_map, std::map<std::string, double>& thickness, 
                std::vector<double>& max_v, std::vector<double>& shoulder_ids, std::vector<double>& elbow_ids, std::vector<std::string>& wrist_names);

    /// \brief Descructor
    ~Articulated_P(){}

    /// \brief Return reachability analysis mode
    std::string get_mode() override {
        return "POS";
    }

    /// \brief Return the extremities
    std::vector<Extremity> get_extremities(){
        return this->extremities;
    }
    
    /// \brief Calculate the reachable set of the total human body with the max position approach.
    /// \param[in] p The joint positions in cartesian world coordinate frame (x, y, z) 
    /// \param[in] t_break The time horizon of the reachability analysis
    std::vector<Capsule> update(std::vector<Point> p, double t_break, std::vector<Point> v={}) override;
};
#endif