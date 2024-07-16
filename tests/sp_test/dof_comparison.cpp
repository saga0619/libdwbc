#include <chrono>
#include "dwbc.h"
#include <unistd.h>
#include <iomanip>
#include <iostream>
#include <fstream>

#ifndef URDF_DIR
#define URDF_DIR ""
#endif

using namespace DWBC;
using namespace std;

// int main(void)

int main(int argc, char **argv)
{
    // retreive int arguments
    int option = 0;
    if (argc > 1)
    {
        option = std::stoi(argv[1]);
    }
    // std::cout << "option : " << option << std::endl;

    bool contact1 = true;
    bool contact2 = true;
    bool contact3 = false;

    double rot_z = 0;
    // double rot_z = M_PI_2;
    int repeat = 10000;

    bool use_hqp = true;

    std::string urdf2_file = std::string(URDF_DIR) + "/dyros_tocabi.urdf";

    std::string desired_control_target = "COM";
    std::string desired_control_target2 = "pelvis_link";
    // std::string desired_control_target3 = "upperbody_link";
    // std::string desired_control_target4 = "L_Wrist2_Link";
    std::string desired_control_target5 = "EE";

    // VectorXd fstar_1;
    // fstar_1.setZero(6);
    // fstar_1 << 0.5, 0.3, 0.2, 0.12, -0.11, 0.05; // based on local frmae.

    // fstar_1

    RobotData rd2_;
    rd2_.LoadModelData(urdf2_file, true, false);

    VectorXd t2lim;
    VectorXd q2 = VectorXd::Zero(rd2_.model_.q_size);
    VectorXd q2dot = VectorXd::Zero(rd2_.model_.qdot_size);
    VectorXd q2ddot = VectorXd::Zero(rd2_.model_.qdot_size);

    // rd_.ChangeLinkToFixedJoint("head_link", verbose);
    VectorXd tlim;
    tlim.setConstant(rd2_.model_dof_, 500);
    // rd2_.SetTorqueLimit(tlim);
    // verbose = true;

    Vector3d euler_rotation(0, 0, rot_z);

    // get quaternion from euler angle in radian, euler_rotation
    Eigen::Quaterniond qu = Eigen::AngleAxisd(euler_rotation[0], Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(euler_rotation[1], Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(euler_rotation[2], Eigen::Vector3d::UnitZ());

    // Eigen::Quaterniond q2_()

    // std::cout << "quaternion : " << qu.w() << " " << qu.x() << " " << qu.y() << " " << qu.z() << std::endl;

    q2 << 0, 0, 0, qu.x(), qu.y(), qu.z(),
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0, 0, 0,
        0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, qu.w();

    // q2 << 0, 0, 0.92983, 0, 0, 0,
    //     0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
    //     0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
    //     0, 0, 0,
    //     0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
    //     0, 0,
    //     -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;

    bool verbose = false;
    rd2_.UpdateKinematics(q2, q2dot, q2ddot);
    rd2_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.13, 0.06, verbose);
    rd2_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.13, 0.06, verbose);
    // rd2_.AddContactConstraint(23, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);
    // rd2_.AddContactConstraint(31, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);

    rd2_.AddTaskSpace(0, TASK_LINK_6D, desired_control_target.c_str(), Vector3d::Zero(), verbose);
    // rd2_.AddTaskSpace(1, TASK_LINK_ROTATION, desired_control_target2.c_str(), Vector3d::Zero(), verbose);
    // rd2_.AddTaskSpace(2, TASK_LINK_ROTATION, desired_control_target3.c_str(), Vector3d::Zero(), verbose);
    // rd2_.AddTaskSpace(3, TASK_LINK_6D, desired_control_target4.c_str(), Vector3d::Zero(), verbose);
    // rd2_.AddTaskSpace(3, TASK_LINK_6D, desired_control_target5.c_str(), Vector3d::Zero(), verbose);
    // rd2_.AddTaskSpace(TASK_CUSTOM, 12);

    rd2_.SetContact(contact1, contact2);
    rd2_.CalcContactConstraint();

    // fstar_1.segment(0, 3) = rd2_.link_[0].rotm * fstar_1.segment(0, 3);
    // fstar_1.segment(3, 3) = rd2_.link_[0].rotm * fstar_1.segment(3, 3);

    // MatrixXd J_task3;
    // VectorXd f_star3;

    int lh_id = rd2_.getLinkID("L_Wrist2_Link");
    int rh_id = rd2_.getLinkID("R_Wrist2_Link");

    // f_star3.setZero(12);
    // f_star3.segment(0, 6) = 0.5 * fstar_1;
    // f_star3.segment(6, 6) = 0.2 * fstar_1;

    // f_star3(3) = 1.0;
    // f_star3(9) = 1.0;

    // Vector6d f_star0;
    // f_star0 << 0.4, 2, 0.2, 0, 0, 0;

    Vector3d fstar_0;
    fstar_0 << 0.4, 2, 0.2;

    Vector3d fstar_1;
    fstar_1 << 0.2, 0.1, 0.1;

    Vector6d fstar_2;
    fstar_2 << 0.4, 0.3, -0.4, 1, 0.3, 0.2;

    // rd2_.SetTaskSpace(0, f_star0);
    // rd2_.SetTaskSpace(1, fstar_1.segment(3, 3));
    // rd2_.SetTaskSpace(2, -fstar_1.segment(3, 3));
    // rd2_.SetTaskSpace(3, f_star3);

    // rd2_.CalcGravCompensation();
    // rd2_.CalcTaskControlTorque(use_hqp, true);
    // rd2_.CalcContactRedistribute(use_hqp, true);
    // auto t10 = std::chrono::high_resolution_clock::now();

    // for (int i = 0; i < repeat; i++)
    // {
    //     rd2_.SetContact(contact1, contact2, contact3);
    //     rd2_.CalcContactConstraint();
    //     rd2_.CalcGravCompensation();
    // }
    // auto t11 = std::chrono::high_resolution_clock::now();
    // for (int i = 0; i < repeat; i++)
    // {
    //     rd2_.CalcTaskSpace();
    // }
    // auto t12 = std::chrono::high_resolution_clock::now();
    // for (int i = 0; i < repeat; i++)
    // {
    //     rd2_.CalcTaskControlTorque(use_hqp, false, false);
    // }

    // auto t13 = std::chrono::high_resolution_clock::now();
    // for (int i = 0; i < repeat; i++)
    // {
    //     if (!rd2_.CalcContactRedistribute(use_hqp, false))
    //     {
    //         std::cout << "Contact Redistribution Failed at step : " << i << std::endl;
    //     }
    // }
    // auto t14 = std::chrono::high_resolution_clock::now();

    // // std::cout << "-----------------------------------------------------------------" << std::endl;

    // double time_original_us = std::chrono::duration_cast<std::chrono::microseconds>(t14 - t10).count();
    // double time_original1_us = std::chrono::duration_cast<std::chrono::microseconds>(t11 - t10).count();
    // double time_original2_us = std::chrono::duration_cast<std::chrono::microseconds>(t12 - t11).count();
    // double time_original3_us = std::chrono::duration_cast<std::chrono::microseconds>(t13 - t12).count();
    // double time_original4_us = std::chrono::duration_cast<std::chrono::microseconds>(t14 - t13).count();

    // std::cout << "Original Dynamics Model  TOTAL CONSUMPTION : " << (float)(time_original_us / repeat) << " us" << std::endl;
    // std::cout << "Original Dynamics Model 1 - Contact Constraint Calculation : " << (int)(time_original1_us / repeat) << " us" << std::endl;
    // std::cout << "Original Dynamics Model 2 - Task Space Calculation         : " << (int)(time_original2_us / repeat) << " us" << std::endl;
    // std::cout << "Original Dynamics Model 3 - Task Torque Calculation        : " << (int)(time_original3_us / repeat) << " us" << std::endl;
    // std::cout << "Original Dynamics Model 4 - Contact Redistribution Calcula : " << (int)(time_original4_us / repeat) << " us" << std::endl;

    std::cout << "-----------------------------------------------------------------" << std::endl;

    // for (int i = 0; i < rd2_.ts_.size(); i++)
    // {
    //     std::cout << "task " << i << " fstar qp  : " << rd2_.ts_[i].f_star_qp_.transpose() << std::endl;
    //     std::cout << "contact qp : " << rd2_.ts_[i].contact_qp_.transpose() << std::endl;
    // }
    // std::cout << "contact qp final : " << rd2_.cf_redis_qp_.transpose() << std::endl;

    // std::cout << "-----------------------------------------------------------------" << std::endl;

    int q_dof = rd2_.model_.q_size;
    int qdot_dof = rd2_.model_.qdot_size;
    int qddot_dof = qdot_dof;

    // for (int k = 0; k < 10; k++)
    // {
    MatrixXd give_me_fstar;

    VectorXd q3 = q2;
    VectorXd q3dot = q2dot;
    VectorXd q3ddot = q2ddot;

    // if (k > 0)
    // {

    int k = option;

    if (k > 0)
    {
        q_dof -= k;
        qdot_dof -= k;
        qddot_dof -= k;

        q3.setZero(q_dof);
        q3 = q2.segment(0, q_dof);
        q3(q_dof - 1) = q2(q_dof - 1);

        q3dot.setZero(qdot_dof);
        q3dot = q2dot.segment(0, qdot_dof);

        q3ddot.setZero(qddot_dof);
        q3ddot = q2ddot.segment(0, qddot_dof);
    }
    else if (k == 0)
    {
        q3 = q2;
        q3dot = q2dot;
        q3ddot = q2ddot;
    }
    else if (k < 0)
    {
        q_dof -= k;
        qdot_dof -= k;
        qddot_dof -= k;

        q3.setZero(q_dof);
        q3.segment(0, q_dof + k) = q2.segment(0, q_dof + k);

        q3.segment(q_dof + k, -k) = q2.segment(q_dof + k - 8, -k);

        // q3.segment(q_dof - 1, 1) = q2.segment(q_dof - 1, 1);

        q3(q_dof - 1) = q2(q_dof + k - 1);

        q3dot.setZero(qdot_dof);
        q3ddot.setZero(qddot_dof);
    }
    // }
    int dof = 33 - option;

    std::string urdf3_file = std::string(URDF_DIR) + "/dof_test/dyros_tocabi_dof" + std::to_string(33 - option) + ".urdf";

    std::cout << "URDF : " << urdf3_file << std::endl;

    urdf2_file = std::string(URDF_DIR) + "/dyros_tocabi_dof31.urdf";
    bool t1_success = true;
    bool t2_success = true;

    double t1_time = 0;
    double t2_time = 0;
    double t2_dynamics_calc = 0;
    double t2_task_calc = 0;
    double t2_nctask_calc = 0;

    {
        RobotData rd_;
        rd_.LoadModelData(urdf3_file, true, false);

        rd_.UpdateKinematics(q3, q3dot, q3ddot);
        rd_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.13, 0.06, verbose);
        rd_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.13, 0.06, verbose);

        rd_.AddTaskSpace(0, TASK_LINK_POSITION, desired_control_target.c_str(), Vector3d::Zero(), verbose);
        rd_.AddTaskSpace(1, TASK_LINK_ROTATION, desired_control_target2.c_str(), Vector3d::Zero(), verbose);
        rd_.AddTaskSpace(2, TASK_LINK_6D, desired_control_target5.c_str(), Vector3d::Zero(), verbose);
        rd_.SetContact(contact1, contact2);
        // rd_.CalcContactConstraint();

        // rd_.ReducedDynamicsCalculate();

        rd_.SetTaskSpace(0, fstar_0);
        rd_.SetTaskSpace(1, fstar_1);
        rd_.SetTaskSpace(2, fstar_2);

        rd_.UpdateTaskSpace();

        DWBC::HQP hqp_;
        int acc_size = rd_.system_dof_;
        int torque_size = rd_.model_dof_;
        int contact_size = rd_.contact_dof_;
        int variable_size = acc_size + contact_size;

        int acc_idx = 0;
        // int toruqe_idx = acc_size;
        int contact_idx = acc_size + 0;
        hqp_.initialize(acc_size, 0, contact_size);

        MatrixXd cost_h = MatrixXd::Zero(acc_size + contact_size, acc_size + contact_size);
        VectorXd cost_g = VectorXd::Zero(acc_size + contact_size);

        cost_h.block(0, 0, acc_size, acc_size) = rd_.A_ / rd_.A_.norm();

        // Priority 1
        int ineq_constraint_size = torque_size * 2; // torque limit
        int eq_constraint_size = 6;                 // newton euler

        hqp_.addHierarchy(ineq_constraint_size, eq_constraint_size);
        // hqp_.hqp_hs_[0].y_ans_.head(variable_size).setZero();

        MatrixXd A = MatrixXd::Zero(ineq_constraint_size, variable_size);
        VectorXd a = VectorXd::Zero(ineq_constraint_size);

        MatrixXd B = MatrixXd::Zero(eq_constraint_size, variable_size);
        VectorXd b = VectorXd::Zero(eq_constraint_size);

        B.block(0, 0, eq_constraint_size, acc_size) = rd_.A_.topRows(eq_constraint_size);
        B.block(0, acc_size, eq_constraint_size, contact_size) = rd_.J_C.transpose().topRows(eq_constraint_size);
        b.head(eq_constraint_size) = rd_.B_.head(eq_constraint_size);

        VectorXd tlim = VectorXd::Constant(torque_size, 200);

        A.block(0, 0, torque_size, acc_size) = rd_.A_.bottomRows(torque_size);
        A.block(0, acc_size, torque_size, contact_size) = rd_.J_C.transpose().bottomRows(torque_size);
        A.block(torque_size, 0, torque_size, acc_size) = -rd_.A_.bottomRows(torque_size);
        A.block(torque_size, acc_size, torque_size, contact_size) = -rd_.J_C.transpose().bottomRows(torque_size);

        // A.block(0, acc_size, torque_size, torque_size) = MatrixXd::Identity(torque_size, torque_size);            // torque limit
        // A.block(torque_size, acc_size, torque_size, torque_size) = -MatrixXd::Identity(torque_size, torque_size); // torque limit
        a.head(torque_size) = -tlim + rd_.B_.tail(torque_size);
        a.tail(torque_size) = -tlim - rd_.B_.tail(torque_size);

        // std::cout << "test0 " << hqp_.hqp_hs_[0].ineq_const_size_ << std::endl;
        // std::cout << "A :rows : " << A.rows() << " cols : " << A.cols() << std::endl;

        hqp_.hqp_hs_[0].updateConstraintMatrix(A, a, B, b);
        hqp_.hqp_hs_[0].normalizeConstraintMatrix();
        // Solve
        hqp_.hqp_hs_[0].v_ans_.setZero(ineq_constraint_size);
        hqp_.hqp_hs_[0].w_ans_.setZero(eq_constraint_size);
        hqp_.hqp_hs_[0].y_ans_.head(acc_size) = -rd_.A_inv_ * rd_.B_;

        // Priority 2
        int contact_constraint_size = rd_.contact_link_num_ * 10;
        ineq_constraint_size = contact_constraint_size + torque_size * 2;
        eq_constraint_size = contact_size;

        hqp_.addHierarchy(ineq_constraint_size, eq_constraint_size);
        A.setZero(ineq_constraint_size, variable_size);
        a.setZero(ineq_constraint_size);
        A.block(0, acc_size, contact_constraint_size, contact_size) = rd_.getContactConstraintMatrix();
        A.block(contact_constraint_size, 6, torque_size, torque_size) = MatrixXd::Identity(torque_size, torque_size);
        A.block(contact_constraint_size + torque_size, 6, torque_size, torque_size) = -MatrixXd::Identity(torque_size, torque_size);

        VectorXd alim = VectorXd::Constant(torque_size, 5);
        a.segment(contact_constraint_size, torque_size) = -alim;
        a.segment(contact_constraint_size + torque_size, torque_size) = -alim;

        B.setZero(eq_constraint_size, variable_size);
        b.setZero(eq_constraint_size);
        B.block(0, 0, contact_size, acc_size) = rd_.J_C;

        // std::cout << "test1" << std::endl;
        hqp_.hqp_hs_[1].updateConstraintMatrix(A, a, B, b);
        hqp_.hqp_hs_[1].updateCostMatrix(cost_h, cost_g);
        hqp_.hqp_hs_[1].normalizeConstraintMatrix();

        // Priority 3
        ineq_constraint_size = 0;
        eq_constraint_size = 6;

        hqp_.addHierarchy(ineq_constraint_size, eq_constraint_size);

        B.setZero(eq_constraint_size, variable_size);
        b.setZero(eq_constraint_size);

        MatrixXd J_task = MatrixXd::Zero(6, acc_size);
        VectorXd f_star = VectorXd::Zero(6);

        J_task.topRows(3) = rd_.ts_[0].J_task_;
        J_task.bottomRows(3) = rd_.ts_[1].J_task_;

        f_star.head(3) = rd_.ts_[0].f_star_;
        f_star.tail(3) = rd_.ts_[1].f_star_;

        B.block(0, 0, 6, acc_size) = J_task;
        b.head(6) = -f_star;

        // std::cout << "J task for h2 : \n"
        //           << J_task << std::endl;
        // std::cout << "f star for h2 : \n"
        //           << f_star << std::endl;

        // std::cout << "test2" << std::endl;
        hqp_.hqp_hs_[2].updateConstraintMatrix(A, a, B, b);
        hqp_.hqp_hs_[2].updateCostMatrix(cost_h, cost_g);
        hqp_.hqp_hs_[2].normalizeConstraintMatrix();

        // Priority 4
        ineq_constraint_size = 0;
        eq_constraint_size = 6;

        B.setZero(eq_constraint_size, variable_size);
        b.setZero(eq_constraint_size);

        B.block(0, 0, 6, acc_size) = rd_.ts_[2].J_task_;
        b.head(6) = -rd_.ts_[2].f_star_;
        // B.block(6, 0, acc_size, acc_size) = rd_.A_;

        // std::cout << "test3" << std::endl;
        hqp_.addHierarchy(ineq_constraint_size, eq_constraint_size);
        hqp_.hqp_hs_[3].updateConstraintMatrix(A, a, B, b);
        hqp_.hqp_hs_[3].updateCostMatrix(cost_h, cost_g);
        hqp_.hqp_hs_[3].normalizeConstraintMatrix();

        // eq_constraint_size = 6;
        // ineq_constraint_size = 0;

        hqp_.prepare();

        auto t2_0 = std::chrono::high_resolution_clock::now();
        bool init_qp = true;
        for (int i = 0; i < repeat; i++)
        {
            hqp_.solveSequential(init_qp);

            init_qp = false;
        }
        auto t2_1 = std::chrono::high_resolution_clock::now();

        double time_original_us2 = std::chrono::duration_cast<std::chrono::microseconds>(t2_1 - t2_0).count();

        t1_time = time_original_us2 / repeat;

        std::cout << "HERZOG Dynamics Model COMP 2 TOTAL CONSUMPTION : " << (float)(time_original_us2 / repeat) << " us" << std::endl;
    }

    std::cout << "-----------------------------------------------------------------" << std::endl;
    {
        RobotData rd_;
        rd_.LoadModelData(urdf3_file, true, false);

        rd_.UpdateKinematics(q3, q3dot, q3ddot);
        rd_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.13, 0.06, verbose);
        rd_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.13, 0.06, verbose);

        rd_.AddTaskSpace(0, TASK_LINK_POSITION, desired_control_target.c_str(), Vector3d::Zero(), verbose);
        rd_.AddTaskSpace(1, TASK_LINK_ROTATION, desired_control_target2.c_str(), Vector3d::Zero(), verbose);
        rd_.AddTaskSpace(2, TASK_LINK_6D, desired_control_target5.c_str(), Vector3d::Zero(), verbose);
        rd_.SetContact(contact1, contact2);
        // rd_.CalcContactConstraint();

        rd_.ReducedDynamicsCalculate();

        rd_.SetTaskSpace(0, fstar_0);
        rd_.SetTaskSpace(1, fstar_1);
        rd_.SetTaskSpace(2, fstar_2);
        rd_.UpdateTaskSpace();
        DWBC::HQP hqp_;
        int acc_size = rd_.reduced_system_dof_;
        int torque_size = rd_.reduced_model_dof_;
        int contact_size = rd_.contact_dof_;
        int variable_size = acc_size + contact_size;

        int acc_idx = 0;
        // int toruqe_idx = acc_size;
        int contact_idx = acc_size + 0;
        hqp_.initialize(acc_size, 0, contact_size);

        MatrixXd cost_h = MatrixXd::Zero(acc_size + contact_size, acc_size + contact_size);
        VectorXd cost_g = VectorXd::Zero(acc_size + contact_size);

        cost_h.block(0, 0, acc_size, acc_size) = rd_.A_R / rd_.A_.norm();

        // Priority 1
        int ineq_constraint_size = torque_size * 2; // torque limit
        int eq_constraint_size = 6;                 // newton euler

        hqp_.addHierarchy(ineq_constraint_size, eq_constraint_size);
        // hqp_.hqp_hs_[0].y_ans_.head(variable_size).setZero();

        MatrixXd A = MatrixXd::Zero(ineq_constraint_size, variable_size);
        VectorXd a = VectorXd::Zero(ineq_constraint_size);

        MatrixXd B = MatrixXd::Zero(eq_constraint_size, variable_size);
        VectorXd b = VectorXd::Zero(eq_constraint_size);

        B.block(0, 0, eq_constraint_size, acc_size) = rd_.A_R.topRows(eq_constraint_size);
        B.block(0, acc_size, eq_constraint_size, contact_size) = (rd_.J_CR.transpose()).topRows(eq_constraint_size);
        b.head(eq_constraint_size) = rd_.G_R.head(eq_constraint_size);

        VectorXd tlim = VectorXd::Constant(torque_size, 200);
        tlim.tail(6).setConstant(1000);
        // tlim(torque_size - 1 - 3) = ;

        A.block(0, 0, torque_size, acc_size) = rd_.A_R.bottomRows(torque_size);
        A.block(0, acc_size, torque_size, contact_size) = rd_.J_CR.transpose().bottomRows(torque_size);
        A.block(torque_size, 0, torque_size, acc_size) = -rd_.A_R.bottomRows(torque_size);
        A.block(torque_size, acc_size, torque_size, contact_size) = -rd_.J_CR.transpose().bottomRows(torque_size);

        // A.block(0, acc_size, torque_size, torque_size) = MatrixXd::Identity(torque_size, torque_size);            // torque limit
        // A.block(torque_size, acc_size, torque_size, torque_size) = -MatrixXd::Identity(torque_size, torque_size); // torque limit
        a.head(torque_size) = -tlim + rd_.G_R.tail(torque_size);
        a.tail(torque_size) = -tlim - rd_.G_R.tail(torque_size);

        // std::cout << "test0 " << hqp_.hqp_hs_[0].ineq_const_size_ << std::endl;
        // std::cout << "A :rows : " << A.rows() << " cols : " << A.cols() << std::endl;

        hqp_.hqp_hs_[0].updateConstraintMatrix(A, a, B, b);
        hqp_.hqp_hs_[0].normalizeConstraintMatrix();
        // Solve
        hqp_.hqp_hs_[0].v_ans_.setZero(ineq_constraint_size);
        hqp_.hqp_hs_[0].w_ans_.setZero(eq_constraint_size);
        hqp_.hqp_hs_[0].y_ans_.head(acc_size) = -rd_.A_R_inv * rd_.G_R;

        // Priority 2
        int contact_constraint_size = rd_.contact_link_num_ * 10;
        ineq_constraint_size = contact_constraint_size + torque_size * 2;
        eq_constraint_size = contact_size;

        hqp_.addHierarchy(ineq_constraint_size, eq_constraint_size);
        A.setZero(ineq_constraint_size, variable_size);
        a.setZero(ineq_constraint_size);
        A.block(0, acc_size, contact_constraint_size, contact_size) = rd_.getContactConstraintMatrix();
        A.block(contact_constraint_size, 6, torque_size, torque_size) = MatrixXd::Identity(torque_size, torque_size);
        A.block(contact_constraint_size + torque_size, 6, torque_size, torque_size) = -MatrixXd::Identity(torque_size, torque_size);

        VectorXd alim = VectorXd::Constant(torque_size, 5);
        a.segment(contact_constraint_size, torque_size) = -alim;
        a.segment(contact_constraint_size + torque_size, torque_size) = -alim;

        B.setZero(eq_constraint_size, variable_size);
        b.setZero(eq_constraint_size);
        B.block(0, 0, contact_size, acc_size) = rd_.J_CR;

        // std::cout << "test1" << std::endl;
        hqp_.hqp_hs_[1].updateConstraintMatrix(A, a, B, b);
        hqp_.hqp_hs_[1].updateCostMatrix(cost_h, cost_g);
        hqp_.hqp_hs_[1].normalizeConstraintMatrix();

        // Priority 3
        ineq_constraint_size = 0;
        eq_constraint_size = 6;

        hqp_.addHierarchy(ineq_constraint_size, eq_constraint_size);

        B.setZero(eq_constraint_size, variable_size);
        b.setZero(eq_constraint_size);

        MatrixXd J_task = MatrixXd::Zero(6, acc_size);
        VectorXd f_star = VectorXd::Zero(6);

        J_task.topRows(3) = rd_.ts_[0].J_task_ * rd_.J_R_INV_T.transpose();
        J_task.bottomRows(3) = rd_.ts_[1].J_task_ * rd_.J_R_INV_T.transpose();

        f_star.head(3) = rd_.ts_[0].f_star_;
        f_star.tail(3) = rd_.ts_[1].f_star_;

        B.block(0, 0, 6, acc_size) = J_task;
        b.head(6) = -f_star;

        // std::cout << "J task for h2 : \n"
        //           << J_task << std::endl;
        // std::cout << "f star for h2 : \n"
        //           << f_star << std::endl;

        hqp_.hqp_hs_[2].updateConstraintMatrix(A, a, B, b);
        hqp_.hqp_hs_[2].updateCostMatrix(cost_h, cost_g);
        hqp_.hqp_hs_[2].normalizeConstraintMatrix();

        // Priority 4
        ineq_constraint_size = 0;
        eq_constraint_size = 6;

        B.setZero(eq_constraint_size, variable_size);
        b.setZero(eq_constraint_size);

        // B.block(0, 0, 6, acc_size) = rd_.ts_[2].J_task_;
        // b.head(6) = -rd_.ts_[2].f_star_;
        // B.block(6, 0, acc_size, acc_size) = rd_.A_;

        // std::cout << "test3" << std::endl;
        // hqp_.addHierarchy(ineq_constraint_size, eq_constraint_size);
        // hqp_.hqp_hs_[3].updateConstraintMatrix(A, a, B, b);
        // hqp_.hqp_hs_[3].updateCostMatrix(cost_h, cost_g);

        // eq_constraint_size = 6;
        // ineq_constraint_size = 0;
        hqp_.prepare();

        rd_.SetContact(contact1, contact2);

        rd_.UpdateTaskSpace();

        auto t2_0 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < repeat; i++)
        {

            rd_.ReducedDynamicsCalculate();
        }
        auto t2_1 = std::chrono::high_resolution_clock::now();

        bool init_qp = true;
        for (int i = 0; i < repeat; i++)
        {
            hqp_.solveSequential(init_qp);

            init_qp = false;
        }
        auto t2_2 = std::chrono::high_resolution_clock::now();

        VectorXd fstar_gnc = VectorXd::Zero(6);
        fstar_gnc = hqp_.hqp_hs_[2].y_ans_.segment(acc_size - 6, 6);
        VectorXd fstar_base = VectorXd::Zero(6);
        fstar_base = hqp_.hqp_hs_[2].y_ans_.head(6);

        MatrixXd jtask_gnc = rd_.J_I_nc_;

        HQP hqp_nc_;
        int acc_size_nc = rd_.nc_dof;
        int torque_size_nc = rd_.nc_dof;
        variable_size = acc_size_nc;

        MatrixXd cost_h_nc = MatrixXd::Zero(variable_size, variable_size);
        VectorXd cost_g_nc = VectorXd::Zero(variable_size);

        cost_h_nc = rd_.A_NC.bottomRightCorner(acc_size_nc, acc_size_nc) / rd_.A_NC.bottomRightCorner(acc_size_nc, acc_size_nc).norm();

        hqp_nc_.initialize(acc_size_nc, 0, 0);

        eq_constraint_size = 6;
        ineq_constraint_size = 2 * acc_size_nc;

        hqp_nc_.addHierarchy(ineq_constraint_size, eq_constraint_size);

        MatrixXd A_nc = MatrixXd::Zero(ineq_constraint_size, variable_size);
        VectorXd a_nc = VectorXd::Zero(ineq_constraint_size);

        MatrixXd B_nc = MatrixXd::Zero(eq_constraint_size, variable_size);
        VectorXd b_nc = VectorXd::Zero(eq_constraint_size);

        // gcom const
        B_nc.block(0, 0, eq_constraint_size, acc_size_nc) = jtask_gnc;
        b_nc.head(eq_constraint_size) = -fstar_gnc;

        VectorXd tlim_nc = VectorXd::Constant(torque_size_nc, 200);
        A_nc.block(0, 0, acc_size_nc, acc_size_nc) = rd_.A_NC.bottomRightCorner(acc_size_nc, acc_size_nc);
        a_nc.segment(0, acc_size_nc) = -tlim_nc + rd_.G_NC;
        A_nc.block(1 * acc_size_nc, 0, acc_size_nc, acc_size_nc) = -rd_.A_NC.bottomRightCorner(acc_size_nc, acc_size_nc);
        a_nc.segment(1 * acc_size_nc, acc_size_nc) = -tlim_nc - rd_.G_NC;

        // // torque lim
        // A_nc.block(2 * acc_size_nc, 0, acc_size_nc, acc_size_nc) = rd_.A_NC.bottomRightCorner(acc_size_nc, acc_size_nc);
        // a_nc.segment(2 * acc_size_nc, acc_size_nc) = -tlim_nc + rd_.G_NC;

        // A_nc.block(3 * acc_size_nc, 0, acc_size_nc, acc_size_nc) = -rd_.A_NC.bottomRightCorner(acc_size_nc, acc_size_nc);

        // a_nc.segment(3 * acc_size_nc, acc_size_nc) = -tlim_nc - rd_.G_NC;

        hqp_nc_.hqp_hs_[0].updateConstraintMatrix(A_nc, a_nc, B_nc, b_nc);
        hqp_nc_.hqp_hs_[0].updateCostMatrix(cost_h_nc, cost_g_nc);

        eq_constraint_size = 6;
        ineq_constraint_size = 2 * acc_size_nc;

        hqp_nc_.addHierarchy(ineq_constraint_size, eq_constraint_size);

        MatrixXd Ja = MatrixXd::Identity(6, 6);
        Ja.block(0, 3, 3, 3) = skew(rd_.link_[rd_.ts_[2].task_link_[0].link_id_].xpos - rd_.link_[0].xpos);

        VectorXd fstar_local = Ja * (rd_.ts_[2].f_star_ - fstar_base);

        B_nc.setZero(eq_constraint_size, variable_size);
        b_nc.setZero(eq_constraint_size);

        B_nc.block(0, 0, 6, acc_size_nc) = rd_.ts_[2].J_task_.rightCols(rd_.nc_dof);
        b_nc.head(6) = -fstar_local;

        // acc lim
        A_nc.block(0, 0, acc_size_nc, acc_size_nc) = MatrixXd::Identity(acc_size_nc, acc_size_nc);
        a_nc.segment(0, acc_size_nc) = -VectorXd::Constant(acc_size_nc, 5);

        A_nc.block(acc_size_nc, 0, acc_size_nc, acc_size_nc) = -MatrixXd::Identity(acc_size_nc, acc_size_nc);
        a_nc.segment(acc_size_nc, acc_size_nc) = -VectorXd::Constant(acc_size_nc, 5);

        hqp_nc_.hqp_hs_[1].updateConstraintMatrix(A_nc, a_nc, B_nc, b_nc);
        hqp_nc_.hqp_hs_[1].updateCostMatrix(cost_h_nc, cost_g_nc);

        hqp_nc_.prepare();

        // auto t2_3 = std::chrono::high_resolution_clock::now();

        init_qp = true;
        for (int i = 0; i < repeat; i++)
        {
            hqp_nc_.solvefirst(init_qp);
            hqp_nc_.solveSequential(init_qp);

            init_qp = false;
        }

        auto t2_3 = std::chrono::high_resolution_clock::now();

        double time_original_us2 = std::chrono::duration_cast<std::chrono::microseconds>(t2_3 - t2_0).count();
        double time_original_us21 = std::chrono::duration_cast<std::chrono::microseconds>(t2_1 - t2_0).count();
        double time_original_us22 = std::chrono::duration_cast<std::chrono::microseconds>(t2_2 - t2_1).count();
        double time_original_us23 = std::chrono::duration_cast<std::chrono::microseconds>(t2_3 - t2_2).count();

        t2_time = time_original_us2 / repeat;
        t2_dynamics_calc = time_original_us21 / repeat;
        t2_task_calc = time_original_us22 / repeat;
        t2_nctask_calc = time_original_us23 / repeat;

        std::cout << "Reduced Dynamics Model COMP 2 TOTAL CONSUMPTION : " << (float)(time_original_us2 / repeat) << " us" << std::endl;
        std::cout << "Reduced Dynamics Model 1 - Reduced Dynamics calculation : " << (float)(time_original_us21 / repeat) << " us" << std::endl;
        std::cout << "Reduced Dynamics Model 2 - Task Space Calculation         : " << (float)(time_original_us22 / repeat) << " us" << std::endl;
        std::cout << "Reduced Dynamics Model 2 - NC Space Calculation         : " << (float)(time_original_us23 / repeat) << " us" << std::endl;

        std::cout << "-----------------------------------------------------------------" << std::endl;

        std::cout << rd_.model_dof_ << "DOF : total comparison : " << t2_time / t1_time * 100 << " %" << std::endl;
    }
    // }

    if (t1_success && t2_success)
    {
        std::ofstream outputFile("/home/dyros/saga_ws/libdwbc/tests/sp_test/output.txt", std::ios::app);
        outputFile << std::fixed << std::setprecision(4);
        outputFile << dof << " " << t1_time << " " << t2_time << " " << t2_dynamics_calc << " " << t2_task_calc << " " << t2_nctask_calc << " " << t2_time / t1_time << std::endl;
        outputFile.close();
    }
    else
    {
        std::cout << "Fail" << std::endl;
    }

    return 0;
}
