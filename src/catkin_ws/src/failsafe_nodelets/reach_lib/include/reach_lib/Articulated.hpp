#include <vector>
#include <map>
#include <assert.h>
#include "BodyPart.hpp"
#include "Extremity.hpp"
#include "System.hpp"


#ifndef ARTICULATED_H
#define ARTICULATED_H

namespace human_reach{
  /// \brief Each body part has a proximal and a distal joint (They can be the same.)
  typedef std::pair<int, int> jointPair;
}

/*
 * Articulated Base Class.
 * This class describes the movement of a human body.
 * Here, we calculate the reachable sets of the human.
 * We distinguish between three modes of reachability analysis:
 * 1) "POS": Max reachable position of extremeties (basically arm length)
 * 2) "VEL": Max reachable set based on a maximum velocity model
 * 3) "ACCEL": Max reachable set based on a maximum acceleration model
 */
class Articulated{
protected:
    /// \brief Maximal positional measurement error
    double measurement_error_pos = 0.0;

    /// \brief Maximal velocity measurement error
    double measurement_error_vel = 0.0;

    /// \brief Delay in measurement processing pipeline
    double delay = 0.0;

    /// \brief Maps the proximal and distal joint to a body part identified by a string
    std::map<std::string, human_reach::jointPair> joint_pair_map;

public:

    /// \brief Dummy constructor
    Articulated(){}

    /// \brief Constructor of base class
    /// \param[in] measurement_error_pos Maximal positional measurement error
    /// \param[in] measurement_error_vel Maximal velocity measurement error
    /// \param[in] delay Delay in measurement processing pipeline
    /// \param[in] joint_pair_map Maps the proximal and distal joint to a body part identified by a string
    Articulated(double measurement_error_pos, double measurement_error_vel, double delay, 
                std::map<std::string, human_reach::jointPair>& joint_pair_map);

    /// \brief Destructor
    ~Articulated();

    /// \brief Return reachability analysis mode
    virtual std::string get_mode() = 0;

    /// \brief Return the mapping from the body part identifier to the joint pair ids.
    std::map<std::string, human_reach::jointPair> get_joint_pair_map(){
        return joint_pair_map;
    }
    
    /// \brief Calculate the reachable set of the total human body.
    ///       This function is called in every verification step.
    ///       This function calls BodyPart::CalculateReach() to get the reachable set of each body part.
    /// \param[in] p The joint positions in cartesian world coordinate frame (x, y, z) 
    /// \param[in] t_break The time horizon of the reachability analysis
    /// \param[in] v The joint velocities (difference between positions)
    virtual std::vector<Capsule> update(std::vector<Point> p, double t_break, std::vector<Point> v={}) = 0;
};

#endif