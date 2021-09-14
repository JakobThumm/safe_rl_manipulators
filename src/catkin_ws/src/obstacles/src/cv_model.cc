#include <obstacles/cv_model.h>
 
using namespace Eigen;

CVModel::CVModel() : KFModel(){
    
}

CVModel::CVModel(double dt, double sigma_w, double sigma_v)
    : KFModel()
{
    // Transition matrix
    this->F = Matrix6d();
    F << 1, dt, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 1, dt, 0, 0,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, dt,
         0, 0, 0, 0, 0, 1;
    this->F_t = this->F.transpose();
    // Prediction covariance matrix
    this->C_w = Matrix6d();
    C_w << pow(dt,3)/3, pow(dt,2)/2, 0, 0, 0, 0,
           pow(dt,2)/2, dt, 0, 0, 0, 0,
           0, 0, pow(dt,3)/3, pow(dt,2)/2, 0, 0,
           0, 0, pow(dt,2)/2, dt, 0, 0,
           0, 0, 0, 0, pow(dt,3)/3, pow(dt,2)/2,
           0, 0, 0, 0, pow(dt,2)/2, dt;
    this->C_w = pow(sigma_w, 2) * this->C_w;
    // Measurement matrix
    this->H = Matrix<double, 3, 6>();
    H << 1, 0, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0,
         0, 0, 0, 0, 1, 0;
    this->H_t = this->H.transpose();
    // Measurement covariance matrix
    this->C_v = Matrix3d();
    this->C_v = pow(sigma_v, 2) * Matrix3d::Identity();
}

void CVModel::init_target(Vector3d initial_pos, Vector3d initial_vel, double sigma_pos, double sigma_v){
    this->state = Vector6d();
    this->state << initial_pos(0), initial_vel(0),
                   initial_pos(1), initial_vel(1),
                   initial_pos(2), initial_vel(2);
    // Prediction covariance matrix
    this->C_p = Matrix6d();
    this->C_p << pow(sigma_pos, 2), 0, 0, 0, 0, 0,
           0, pow(sigma_v, 2), 0, 0, 0, 0,
           0, 0, pow(sigma_pos, 2), 0, 0, 0,
           0, 0, 0, pow(sigma_v, 2), 0, 0,
           0, 0, 0, 0, pow(sigma_pos, 2), 0,
           0, 0, 0, 0, 0, pow(sigma_v, 2);
    this->C_e = this->C_p;
}

void CVModel::perform_prediction_update(Vector3d measurement){
    this->predict();
    this->update(measurement);
}

Vector3d CVModel::get_pos(){
    Vector3d pos;
    pos << this->state(0), this->state(2), this->state(4);
    return pos;
}

Vector3d CVModel::get_vel(){
    Vector3d vel;
    vel << this->state(1), this->state(3), this->state(5);
    return vel;
}