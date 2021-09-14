#ifndef _CV_MODEL_HH_
#define _CV_MODEL_HH_

#include <math.h> 
#include <Eigen/Dense>
#include <obstacles/kf_model.h>


/// \brief The constant velocity Kalman filter super class
class CVModel : public KFModel
{

    public: CVModel();

    /// \brief Constructor.
    /// \param[in] dt The time difference between two measurements (Use 1 for timestep dependent tracking)
    /// \param[in] sigma_w sigma of system noise
    /// \param[in] sigma_v sigma of measurement noise
    public: CVModel(double dt, double sigma_w=0.001, double sigma_v=0.01);

    /// \brief Destructor.
    public: virtual ~CVModel(){}

    /// \brief Reset the target
    /// \param[in] initial_pos The initial position of the target
    /// \param[in] initial_vel The initial velocity of the target
    /// \param[in] sigma_pos The unvertainty in the initial position
    /// \param[in] sigma_v The uncertainty in the initial velocity
    public: void init_target(Eigen::Vector3d initial_pos, Eigen::Vector3d initial_vel, double sigma_pos, double sigma_v);

    /// \brief Execute the prediction and update step of the Kalman filter.
    /// \param[in] measurement The measured target position
    public: void perform_prediction_update(Eigen::Vector3d measurement);

    /// \brief Return the target position
    public: Eigen::Vector3d get_pos();

    /// \brief Retrun the target velocity
    public: Eigen::Vector3d get_vel();
    //////////////// ARGS ////////////////
    //// *** PRIVATE *** /////

};

namespace Eigen {
    typedef Eigen::Matrix< double , 6 , 6 > Matrix6d;
    typedef Eigen::Matrix< double , 6 , 1 > Vector6d;
}


#endif