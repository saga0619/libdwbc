#include <chrono>
#include "dwbc.h"
#include <unistd.h>
#include <iomanip>
#include <fstream>

#ifndef URDF_DIR
#define URDF_DIR ""
#endif

using namespace DWBC;
using namespace std;
#define COMPARISON_EPS 1.0E-6

template <class Matrix>
void wrtie_binary(const char *filename, const Matrix &matrix)
{
    std::string r_path = RESOURCE_DIR;

    std::string t_path = r_path + filename;

    std::ofstream out(t_path, std::ios::out | std::ios::binary | std::ios::trunc);
    typename Matrix::Index rows = matrix.rows(), cols = matrix.cols();
    out.write((char *)(&rows), sizeof(typename Matrix::Index));
    out.write((char *)(&cols), sizeof(typename Matrix::Index));
    out.write((char *)matrix.data(), rows * cols * sizeof(typename Matrix::Scalar));
    out.close();
}

int main(void)
{
     RobotData rd_;
    std::string resource_path = URDF_DIR;
    std::string urdf_name = "/dyros_tocabi.urdf";
    std::string urdf_path = resource_path + urdf_name;

    rd_.LoadModelData(urdf_path, true, false);
    VectorXd q = VectorXd::Zero(rd_.system_dof_ + 1);
    VectorXd qdot = VectorXd::Zero(rd_.system_dof_);
    VectorXd qddot = VectorXd::Zero(rd_.system_dof_);

    VectorXd fstar_1;
    fstar_1.setZero(6);

    VectorXd fstar_2;
    fstar_2.setZero(3);
    std::string case_set = "";

    // q << 0, 0, 0.92983, 0, 0, 0,
    //     0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
    //     0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
    //     0, 0, 0,
    //     0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
    //     0, 0,
    //     -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;

    // fstar_1 << 0.1, 4.0, 0.1, 0.1, -0.1, 0.1;
    // fstar_2 << 0.1, -0.1, 0.1;

    // case_set = "/1";

    q << 0, 0, 0.92983, 0, 0, 0,
        0.1, 0.0, -0.24, 0.5, -0.6, 0.0,
        0.05, 0.0, -0.21, 0.7, -0.31, 0.0,
        0, 0, 0,
        0.2, 0.5, 1.5, -1.27, -1.2, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 1.3, 0.1, 1.3, 0, 1;

    fstar_1 << 0.4, 2.0, 0.1, 0.3, -0.1, 0.1;
    fstar_2 << 0.1, 0.1, 0.1;

    case_set = "/2";


    rd_.UpdateKinematics(q, qdot, qddot);

    int left_foot_id = 6;
    int right_foot_id = 12;

    rd_.AddContactConstraint(left_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    rd_.AddContactConstraint(right_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    rd_.AddContactConstraint(23, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);
    rd_.AddContactConstraint(31, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);

    rd_.AddTaskSpace(0, TASK_LINK_6D, 0, Vector3d::Zero());
    rd_.AddTaskSpace(1, TASK_LINK_ROTATION, 15, Vector3d::Zero());

    VectorXd tlim;
    tlim.setConstant(rd_.model_dof_, 300);

    rd_.SetTorqueLimit(tlim);

    rd_.UpdateKinematics(q, qdot, qddot);

    // rd_.AddContactConstraint(left_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    // rd_.AddContactConstraint(right_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);

    // rd_.AddTaskSpace(TASK_LINK_6D, 0, Vector3d::Zero());
    // rd_.AddTaskSpace(TASK_LINK_ROTATION, 15, Vector3d::Zero());

    rd_.SetContact(true, true);
    rd_.CalcContactConstraint();

    std::string f_name = "/J_C";

    wrtie_binary((case_set + f_name).c_str(), rd_.J_C);

    f_name = "/A_inv_";
    wrtie_binary((case_set + f_name).c_str(), rd_.A_inv_);

    f_name = "/Lambda_contact";
    wrtie_binary((case_set + f_name).c_str(), rd_.Lambda_contact);

    f_name = "/J_C_INV_T";
    wrtie_binary((case_set + f_name).c_str(), rd_.J_C_INV_T);

    f_name = "/N_C";
    wrtie_binary((case_set + f_name).c_str(), rd_.N_C);

    f_name = "/W";
    wrtie_binary((case_set + f_name).c_str(), rd_.W);

    f_name = "/NwJw";
    wrtie_binary((case_set + f_name).c_str(), rd_.NwJw);

    f_name = "/W_inv";
    wrtie_binary((case_set + f_name).c_str(), rd_.W_inv);

    rd_.SetTaskSpace(0, fstar_1);

    rd_.SetTaskSpace(1, fstar_2);

    f_name = "/torque_grav_";
    wrtie_binary((case_set + f_name).c_str(), rd_.CalcGravCompensation());

    rd_.CalcTaskControlTorque(true);

    f_name = "/torque_task_";
    wrtie_binary((case_set + f_name).c_str(), rd_.torque_task_);

    rd_.CalcContactRedistribute(true);

    f_name = "/torque_contact_";
    wrtie_binary((case_set + f_name).c_str(), rd_.torque_contact_);
}
