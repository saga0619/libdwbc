#include <chrono>
#include "dwbc.h"
#include <unistd.h>
#include <iomanip>

#ifndef URDF_DIR
#define URDF_DIR ""
#endif

using namespace DWBC;
using namespace std;

int main(void)
{
    bool contact1 = true;
    bool contact2 = true;
    bool contact3 = false;

    double rot_z = 0;
    // double rot_z = M_PI_2;
    int repeat = 10;

    bool use_hqp = true;

    std::string urdf2_file = std::string(URDF_DIR) + "/dyros_tocabi.urdf";

    std::string desired_control_target = "COM";
    std::string desired_control_target2 = "pelvis_link";
    std::string desired_control_target3 = "upperbody_link";
    std::string desired_control_target4 = "L_Wrist2_Link";
    std::string desired_control_target5 = "R_Wrist2_Link";

    RobotData rd2_;
    rd2_.LoadModelData(urdf2_file, true, false);

    VectorXd t2lim;
    VectorXd q2 = VectorXd::Zero(rd2_.model_.q_size);
    VectorXd q2dot = VectorXd::Zero(rd2_.model_.qdot_size);
    VectorXd q2ddot = VectorXd::Zero(rd2_.model_.qdot_size);

    q2 << -0.0325, -0.0579, 0.7273, 0.0194, -0.0118, -0.0008,
        -0.0006, 0.0698, -0.7835, 1.6487, -0.8420, -0.0911,
        -0.0007, 0.0767, -0.7963, 1.6742, -0.8549, -0.1150,
        -0.0001, -0.0003, 0.0204,
        0.2998, 0.3001, 1.5000, -1.2701, -1.0507, 0.0000, -1.0000, 0.0000,
        -0.0000, 0.0003, -0.2998, -0.3060, -1.5001, 1.2700, 1.0848, 0.0000,
        1.0000, 0.0000, 0.9997;

    VectorXd fstar_0(3);
    VectorXd fstar_1(3);
    VectorXd fstar_2(3);
    VectorXd fstar_3(12);

    fstar_0 << 0.3142, -1.8202, -1.7750;
    fstar_1 << -17.8677, 8.4977, 1.0850;
    fstar_2 << -8.5340, 8.5992, 1.2655;
    fstar_3 << 4.0251, 3.9975, 7.5672, -8.2841, 30.3652, 0.8954, 2.7585, 3.7898, 9.3234, -9.5724, 43.8036, 2.5202;

    bool verbose = false;
    rd2_.UpdateKinematics(q2, q2dot, q2ddot);
    rd2_.AddContactConstraint("l_ankleroll_link", DWBC::CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.12, 0.06);
    rd2_.AddContactConstraint("r_ankleroll_link", DWBC::CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.12, 0.06);
    rd2_.AddContactConstraint(23, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);
    rd2_.AddContactConstraint(31, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);

    rd2_.AddTaskSpace(0, TASK_LINK_POSITION, desired_control_target.c_str(), Vector3d::Zero(), verbose);
    rd2_.AddTaskSpace(1, TASK_LINK_ROTATION, desired_control_target2.c_str(), Vector3d::Zero(), verbose);
    rd2_.AddTaskSpace(2, TASK_LINK_ROTATION, desired_control_target3.c_str(), Vector3d::Zero(), verbose);
    rd2_.AddTaskSpace(3, TASK_LINK_6D, desired_control_target4.c_str(), Vector3d::Zero(), verbose);
    rd2_.AddTaskSpace(3, TASK_LINK_6D, desired_control_target5.c_str(), Vector3d::Zero(), verbose);
    // rd2_.AddTaskSpace(TASK_CUSTOM, 12);

    rd2_.SetContact(contact1, contact2, contact3, false);
    rd2_.CalcContactConstraint();

    int lh_id = rd2_.getLinkID("L_Wrist2_Link");
    int rh_id = rd2_.getLinkID("R_Wrist2_Link");

    rd2_.SetTaskSpace(0, fstar_0);
    rd2_.SetTaskSpace(1, fstar_1);
    rd2_.SetTaskSpace(2, fstar_2);
    rd2_.SetTaskSpace(3, fstar_3);

    rd2_.CalcGravCompensation();
    rd2_.CalcTaskControlTorque(use_hqp, true);
    rd2_.CalcContactRedistribute(use_hqp, true);
    auto t10 = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < repeat; i++)
    {
        rd2_.SetContact(contact1, contact2, contact3);
        rd2_.CalcContactConstraint();
        rd2_.CalcGravCompensation();
    }
    auto t11 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < repeat; i++)
    {
        rd2_.CalcTaskSpace();
    }

    int taskqp_err = 0;
    auto t12 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < repeat; i++)
    {
        taskqp_err += rd2_.CalcTaskControlTorque(use_hqp, false, false);
    }
    auto t13 = std::chrono::high_resolution_clock::now();
    int contactqp_err = 0;
    for (int i = 0; i < repeat; i++)
    {
        contactqp_err += rd2_.CalcContactRedistribute(use_hqp, false);
    }
    auto t14 = std::chrono::high_resolution_clock::now();

    std::cout << "-----------------------------------------------------------------" << std::endl;

    double time_original_us = std::chrono::duration_cast<std::chrono::microseconds>(t14 - t10).count();
    double time_original1_us = std::chrono::duration_cast<std::chrono::microseconds>(t11 - t10).count();
    double time_original2_us = std::chrono::duration_cast<std::chrono::microseconds>(t12 - t11).count();
    double time_original3_us = std::chrono::duration_cast<std::chrono::microseconds>(t13 - t12).count();
    double time_original4_us = std::chrono::duration_cast<std::chrono::microseconds>(t14 - t13).count();

    std::cout << "Original Dynamics Model  TOTAL CONSUMPTION : " << (float)(time_original_us / repeat) << " us" << std::endl;
    std::cout << "Original Dynamics Model 1 - Contact Constraint Calculation : " << (int)(time_original1_us / repeat) << " us" << std::endl;
    std::cout << "Original Dynamics Model 2 - Task Space Calculation         : " << (int)(time_original2_us / repeat) << " us" << std::endl;
    std::cout << "Original Dynamics Model 3 - Task Torque Calculation        : " << (int)(time_original3_us / repeat) << " us" << std::endl;
    std::cout << "Original Dynamics Model 4 - Contact Redistribution Calcula : " << (int)(time_original4_us / repeat) << " us" << std::endl;

    std::cout << "-----------------------------------------------------------------" << std::endl;

    std::cout << "Task QP Error : " << taskqp_err << std::endl;
    std::cout << "Contact QP Error : " << contactqp_err << std::endl;

    rd2_.SetContact(1,1,1);
    rd2_.CalcContactConstraint();
    rd2_.ReducedDynamicsCalculate();

    std::cout << "J_R:"<<std::endl;
    std::cout << rd2_.J_R << std::endl;




    // std::cout << " QP Fstar Original : " << rd2_.ts_[0].f_star_.transpose() << std::endl;
    // std::cout << " QP Fstar modified : " << rd2_.ts_[0].f_star_.transpose() + rd2_.ts_[0].f_star_qp_.transpose() << std::endl;

    // std::cout << "Result Task Torque : " << std::endl;

    // VectorXd torque_task_original = rd2_.torque_task_;
    // std::cout << rd2_.torque_task_.transpose() << std::endl;

    // std::cout << "Result grav Torque : " << std::endl;
    // std::cout << rd2_.torque_grav_.transpose() << std::endl;

    // VectorXd torque_grav_original = rd2_.torque_grav_;

    // std::cout << "contact torque : " << std::endl;
    // std::cout << rd2_.torque_contact_.transpose() << std::endl;

    // VectorXd torque_contact_original = rd2_.torque_contact_;

    // MatrixXd s_k = MatrixXd::Zero(rd2_.model_dof_, rd2_.model_dof_ + 6);
    // s_k.block(0, 6, rd2_.model_dof_, rd2_.model_dof_) = MatrixXd::Identity(rd2_.model_dof_, rd2_.model_dof_);
    // MatrixXd give_me_fstar = rd2_.ts_[0].J_task_ * rd2_.A_inv_ * rd2_.N_C * s_k.transpose();

    // std::cout << "fstar recalc : " << (give_me_fstar * rd2_.torque_task_).transpose() << std::endl;

    // if (use_hqp)
    // {
    //     std::cout << "-----------------------------------------------------------------" << std::endl;

    //     for (int i = 0; i < rd2_.ts_.size(); i++)
    //     {
    //         std::cout << "task " << i << " fstar qp  : " << rd2_.ts_[i].f_star_qp_.transpose() << std::endl;
    //         std::cout << "contact qp : " << rd2_.ts_[i].contact_qp_.transpose() << std::endl;
    //     }
    //     std::cout << "contact qp final : " << rd2_.cf_redis_qp_.transpose() << std::endl;
    // }

    // VectorXd contact_force;

    // VectorXd command_torque = rd2_.torque_task_ + rd2_.torque_grav_ + rd2_.torque_contact_;

    // contact_force = rd2_.getContactForce(command_torque);

    // Vector3d zmp_pos = rd2_.getZMP(contact_force);

    // std::cout << "res contact force : " << contact_force.transpose() << std::endl;

    // std::cout << "lf cp pos : " << rd2_.cc_[0].xc_pos.transpose() << std::endl;
    // std::cout << "rf cp vel : " << rd2_.cc_[1].xc_pos.transpose() << std::endl;

    // std::cout << " zmp pos : " << zmp_pos.transpose() << std::endl;

    return 0;
}
