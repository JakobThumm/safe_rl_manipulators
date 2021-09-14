#include <obstacles/kf_model.h>

using namespace Eigen;

KFModel::KFModel(){

}

/////////////////////////////////////////////////
KFModel::KFModel(MatrixXd F, MatrixXd C_w, MatrixXd H, MatrixXd C_v)
{
    this->F = F;
    this->C_w = C_w;
    this->H = H;
    this->C_v = C_v;
}

void KFModel::set_variables(MatrixXd F, MatrixXd C_w, MatrixXd H, MatrixXd C_v){
    this->F = F;
    this->C_w = C_w;
    this->H = H;
    this->C_v = C_v;
}

void KFModel::init_target(VectorXd initial_state, MatrixXd C_p){
    this->state = initial_state;
    this->C_p = C_p;
    this->C_e = C_p;
}

void KFModel::predict(){
    // Predict state x_p = F * x_e
    this->state = this->F * this->state;
    // Predict C_p = F * C_e * F' + C_w
    this->C_p = this->F * this->C_e * this->F_t + this->C_w;
}

void KFModel::update(VectorXd measurement){
    // Update (filter) step from last time step
    // Calculate gain K = C_p * H' * inv(C_v + H * C_p * H')
    this->M1 = this->C_v + this->H * this->C_p * this->H_t;
    this->K = this->C_p * this->H_t * M1.inverse();
    // Update state x_e = x_p + K * (y - H * x_p)
    this->state = this->state + K * (measurement - this->H * this->state);
    // Update covariance matrix C_e = C_p - K * H * C_p
    this->C_e = this->C_p - K * this->H * this->C_p;
}