#include <chrono>
#include "dwbc.h"
#include <unistd.h>
#include <iomanip>

#ifndef URDF_DIR
#define URDF_DIR ""
#endif

#undef COMPARISON_EPS
#define COMPARISON_EPS 1.0E-12

using namespace DWBC;

int main(void)
{
    srand((unsigned int)time(0));
    bool verbose = false;
    int repeat_time = 1000;
    RobotData rd_;
    std::string resource_path = URDF_DIR;
    std::string urdf_name = "/dyros_tocabi.urdf";
    std::string urdf_path = resource_path + urdf_name;
    VectorXd tlim, q, qdot, qddot, fstar_1, fstar_2;

    rd_.LoadModelData(urdf_path, true, false);

    q.setZero(rd_.model_.q_size);
    qdot.setZero(rd_.model_.qdot_size);
    qddot.setZero(rd_.model_.qdot_size);
    qdot.setRandom(rd_.model_.qdot_size);
    qdot.segment(0, 6).setZero();
    tlim.setConstant(rd_.model_dof_, 500);

    fstar_1.setZero(6);
    fstar_1 << 0.1, 2.0, 0.1, 0.1, -0.1, 0.1;
    // fstar_1 << 0.1, 2.0, 0.1; //, 0.1, -0.1, 0.1;

    rd_.SetTorqueLimit(tlim);
    // verbose = true;
    q << 0, 0, 0.92983, 0, 0, 0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0, 0, 0,
        0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 0, 0, 0, 0, 1;

    rd_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    rd_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    // rd_.AddTaskSpace(TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero(), verbose);
    rd_.AddTaskSpace(TASK_LINK_6D, "pelvis_link", Vector3d::Zero(), verbose);
    rd_.SetTaskSpace(0, fstar_1);

    rd_.UpdateKinematics(q, qdot, qddot);
    rd_.SetContact(true, true);
    rd_.CalcGravCompensation();
    rd_.CalcTaskControlTorque(true);
    rd_.CalcContactRedistribute(true);

    MatrixXd W_inv1 = rd_.W_inv;

    VectorXd qd_rand;
    qd_rand = 0.2 * VectorXd::Random(rd_.model_dof_);

    VectorXd q2;
    q2 = q;
    q2.segment(6, rd_.model_dof_) = q.segment(6, rd_.model_dof_) + qd_rand;

    rd_.UpdateKinematics(q2, qdot, qddot);
    rd_.SetContact(true, true);
    rd_.CalcGravCompensation();
    rd_.CalcTaskControlTorque(true);
    rd_.CalcContactRedistribute(true);

    MatrixXd W_inv2 = rd_.W_inv;

    std::cout << qd_rand.transpose() << std::endl;
    std::cout << "-------------------------------------------" << std::endl;
    std::cout << W_inv2 - W_inv1 << std::endl;

    MatrixXd W_diff = W_inv2 - W_inv1;

    for (int i = 0; i < W_diff.cols(); i++)
    {
        for (int j = 0; j < W_diff.rows(); j++)
        {
            if (abs(W_diff(i, j)) < 0.0001)
            {
                W_diff(i, j) = 0;
            }
            else{
                W_diff(i, j) = 1;
            }
        }
    }
    std::cout <<"--------------------------------------------------"<<std::endl;
    std::cout << W_diff << std::endl;

    return 0;
}
