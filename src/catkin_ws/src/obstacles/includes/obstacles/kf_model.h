#ifndef _KF_MODEL_HH_
#define _KF_MODEL_HH_

#include <Eigen/Dense>
 
using namespace Eigen;

/// \brief The Kalman filter super class
class KFModel
{
    /// \brief Default constructor
    public: KFModel();

    /// \brief Constructor.
    /// \param[in] F Discrete system matrix
    /// \param[in] C_w System noise matrix
    /// \param[in] H Measurement matrix
    /// \param[in] C_v Measurement noise matrix
    public: KFModel(MatrixXd F, MatrixXd C_w, MatrixXd H, MatrixXd C_v);

    /// \brief Destructor.
    public: virtual ~KFModel(){}

    public: void set_variables(MatrixXd F, MatrixXd C_w, MatrixXd H, MatrixXd C_v);

    /// \brief Reset the target
    /// \param[in] initial_state The estimated initial state of the target
    /// \param[in] C_p The initial state uncertainty matrix
    public: void init_target(VectorXd initial_state, MatrixXd C_p);

    /// \brief Execute the prediction step of the Kalman filter.
    public: void predict();

    /// \brief Execute update step of the Kalman filter.
    /// \param[in] measurement sensor measurement
    public: void update(VectorXd measurement);


    //////////////// ARGS ////////////////
    //// *** PROTECTED *** /////
    /// \brief Discrete system matrix
    protected: MatrixXd F;
    /// \brief Discrete system matrix transposed
    protected: MatrixXd F_t;
    /// \brief System noise matrix
    protected: MatrixXd C_w;
    /// \brief Measurement matrix
    protected: Matrix<double, Dynamic, Dynamic> H;
    /// \brief Measurement matrix transposed
    protected: Matrix<double, Dynamic, Dynamic> H_t;
    /// \brief Measurement noise matrix
    protected: MatrixXd C_v;
    /// \brief The predicted covariance matrix
    protected: MatrixXd C_p;
    /// \brief The estimated covariance matrix
    protected: MatrixXd C_e;
    /// \brief The state vector
    protected: VectorXd state;
    /// \brief Calculation variable
    protected: MatrixXd M1;
    /// \brief Calculation variable
    protected: MatrixXd K;
};


#endif