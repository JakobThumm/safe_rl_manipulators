#include <modrob_simulation/RobotGroupModuleOrder.h>
#include <modrob_simulation/RobotDescription.h>
#include <modrob_simulation/LinkDescription.h>
#include <modrob_simulation/JointDescription.h>
#include <modrob_simulation/LinkVisual.h>
#include <modrob_simulation/LinkCollision.h>
#include <modrob_workstation/ModuleOrder.h>
#include "database_access.cpp"
#include <string>
#include "Eigen/Dense"

using namespace std;


// Meaning of database index (index -> attribute-name)
// Example, Name == 0, ID == 1, typ == 2 etc....
enum Database { Name, ID, typ, Cplx, CANidTX, CANidRX, a_pl, alpha_pl, p_pl, 
                n_pl, delta_pl, a_dl, alpha_dl, p_dl, n_dl, delta_dl, jt, 
                delta_j, Ujl, Ljl, m_pl, I_pl_xx, I_pl_xy, I_pl_xz, I_pl_yx, 
                I_pl_yy, I_pl_yz, I_pl_zx, I_pl_zy, I_pl_zz, r_com_pl_x, 
                r_com_pl_y, r_com_pl_z, m_dl, I_dl_xx, I_dl_xy, I_dl_xz, 
                I_dl_yx, I_dl_yy, I_dl_yz, I_dl_zx, I_dl_zy, I_dl_zz, 
                r_com_dl_x, r_com_dl_y, r_com_dl_z, Im, jbc, jbv, k_tau, 
                k_r, tau_lim, curr_lim, dq_lim, ddq_lim, InputConnectorSize, 
                OutputConnectorSize, encRes, currFactor,
                collision_size_x, collision_size_y, collision_size_z,
                collision_pos_x, collision_pos_y, collision_pos_z};

// Meaning of type variable in database
enum ID_TYPES {END_EFFECTOR_TYPE, MOTOR_TYPE, LINK_TYPE, BASE_TYPE};    //END_EFFECTOR_TYPE does not exist, only 3 types in database: 1,2,3

struct rvizXYZRPYY{
    float x;
    float y;
    float z;
    float r;
    float p;
    float yy;
};

// Function definitions
void listenForGroupOrderAndPublishRobotGroupDescription(const modrob_simulation::RobotGroupModuleOrder::ConstPtr&);
void listenForModuleOrderAndGenerateAndPublishRobotDescriptionMessage(const modrob_workstation::ModuleOrder::ConstPtr&);
void parseID(int);
void createLinkDescription(CSVRow);
void createMotorDescription(CSVRow);
rvizXYZRPYY convertDatabaseCoordinatesToRvizCoordinates(string,string,string,string,string,string,string,string,string,string,string);
rvizXYZRPYY convertDatabaseCoordinatesToRvizCoordinates(string id,string a,string p,string n,string ad, string nd, string pd, string al_pl, string de_pl, string de_dl, string al_dl, rvizXYZRPYY& proximal_trafo, rvizXYZRPYY& distal_trafo);
rvizXYZRPYY convertDatabaseCoordinatesToRvizCoordinates(string id,string a,string p,string n,string ad, string nd, string pd, string al_pl, string de_pl, string de_dl, string al_dl, rvizXYZRPYY& proximal_trafo, rvizXYZRPYY& distal_trafo, Eigen::Matrix4d& transform);
rvizXYZRPYY transformationMatrix2RVIZ(const Eigen::Matrix4d& transform);

std::string stl_file_path;