#include <chrono>
#include "dwbc.h"
#include <unistd.h>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <limits.h>

#ifndef URDF_DIR
#define URDF_DIR ""
#endif

using namespace DWBC;
using namespace std;

// int main(void)
std::string getExecutablePath()
{
    char result[PATH_MAX];
    ssize_t count = readlink("/proc/self/exe", result, PATH_MAX);
    if (count == -1)
    {
        // 에러 처리
        perror("readlink");
        exit(EXIT_FAILURE);
    }
    return std::string(result, count);
}

std::string getExecutableDir()
{
    std::string path = getExecutablePath();
    size_t lastSlash = path.find_last_of('/');
    if (lastSlash == std::string::npos)
    {
        return ""; // 디렉토리 구분자가 없음
    }
    return path.substr(0, lastSlash);
}

int compare(int dof_input, int repeat_v)
{
    bool contact1 = true;
    bool contact2 = true;

    double rot_z = 0;
    int repeat = repeat_v;

    double tlim_val;
    double alim_val;

    tlim_val = 200;
    alim_val = 5;

    bool use_hqp = true;

    std::string urdf2_file = std::string(URDF_DIR) + "/dyros_tocabi.urdf";

    std::string desired_control_target = "COM";
    std::string desired_control_target2 = "pelvis_link";
    std::string desired_control_target5 = "EE";

    VectorXd t2lim;
    VectorXd q2 = VectorXd::Zero(40);
    VectorXd q2dot = VectorXd::Zero(39);
    VectorXd q2ddot = VectorXd::Zero(39);

    Vector3d euler_rotation(0, 0, rot_z);

    // get quaternion from euler angle in radian, euler_rotation
    Eigen::Quaterniond qu = Eigen::AngleAxisd(euler_rotation[0], Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(euler_rotation[1], Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(euler_rotation[2], Eigen::Vector3d::UnitZ());

    q2 << 0, 0, 0, qu.x(), qu.y(), qu.z(),
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0, 0, 0,
        0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, qu.w();

    Vector6d fstar_0;
    fstar_0 << 0.4, 2, 0.2, 0.2, 0.1, 0.1;

    Vector3d fstar_1;
    fstar_1 << 0.2, 0.1, 0.1;

    Vector6d fstar_2;
    fstar_2 << 0.4, 0.3, -0.4, 1, 0.3, 0.2;

    std::cout << "-----------------------------------------------------------------" << std::endl;

    // for (int i = 0; i < rd2_.ts_.size(); i++)
    // {
    //     std::cout << "task " << i << " fstar qp  : " << rd2_.ts_[i].f_star_qp_.transpose() << std::endl;
    //     std::cout << "contact qp : " << rd2_.ts_[i].contact_qp_.transpose() << std::endl;
    // }
    // std::cout << "contact qp final : " << rd2_.cf_redis_qp_.transpose() << std::endl;

    // std::cout << "-----------------------------------------------------------------" << std::endl;

    int q_dof = 40;
    int qdot_dof = 39;
    int qddot_dof = 39;

    // for (int k = 0; k < 10; k++)
    // {
    MatrixXd give_me_fstar;

    VectorXd q3;     // = q2;
    VectorXd q3dot;  // = q2dot;
    VectorXd q3ddot; // = q2ddot;

    int option = 33 - dof_input;

    int k = option;

    q3dot.setZero(dof_input + 6);
    q3ddot.setZero(dof_input + 6);

    q3.setZero(dof_input + 7);

    q3.head(dof_input) = q2.head(dof_input);

    q3.tail(1) = q2.tail(1);

    std::string urdf3_file = std::string(URDF_DIR) + "/dof_test/dyros_tocabi_dof" + std::to_string(dof_input) + ".urdf";

    std::cout << "URDF : " << urdf3_file << std::endl;

    urdf2_file = std::string(URDF_DIR) + "/dyros_tocabi_dof31.urdf";
    bool t1_success = true;
    bool t2_success = true;

    double t0_time = 0;
    double t1_time = 0;
    double t2_time = 0;
    double t2_dynamics_calc = 0;
    double t2_task_calc = 0;
    double t2_nctask_calc = 0;

    double original_solve_time = 0;
    double proposed_solve_time = 0;

    bool verbose = false;
    {
        RobotData rd_;
        rd_.LoadModelData(urdf3_file, true, false);

        rd_.UpdateKinematics(q3, q3dot, q3ddot);
        rd_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.13, 0.06, verbose);
        rd_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.13, 0.06, verbose);

        rd_.AddTaskSpace(0, TASK_LINK_POSITION, desired_control_target.c_str(), Vector3d::Zero(), verbose);
        rd_.AddTaskSpace(0, TASK_LINK_ROTATION, desired_control_target2.c_str(), Vector3d::Zero(), verbose);
        rd_.AddTaskSpace(1, TASK_LINK_6D, desired_control_target5.c_str(), Vector3d::Zero(), verbose);
        rd_.SetContact(contact1, contact2);

        rd_.SetTaskSpace(0, fstar_0);
        rd_.SetTaskSpace(1, fstar_2);
        rd_.UpdateTaskSpace();

        DWBC::HQP hqp_;
        rd_.ConfigureLQP(hqp_);
        auto t1 = std::chrono::high_resolution_clock::now();
        rd_.CalcControlTorqueLQP(hqp_, true);

        double total_time = 0;
        double prep_time = 0;
        double solve_time = 0;
        for (int i = 0; i < repeat; i++)
        {
            rd_.CalcControlTorqueLQP(hqp_, false);
            total_time += hqp_.total_time_step_;
            prep_time += hqp_.update_time_step_;
            solve_time += hqp_.solve_time_step_;
        }
        auto t2 = std::chrono::high_resolution_clock::now();

        double time_original_us = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();

        t1_time = time_original_us / repeat;
        original_solve_time = solve_time / repeat;

        std::cout << "  ORIGINAL LQP TOTAL CONSUMPTION : " << t1_time << " us (" << hqp_.total_time_max_ << " us )" << std::endl;
        std::cout << "            Internal update time : " << prep_time / repeat << " us (" << hqp_.update_time_max_ << " us )" << std::endl;
        std::cout << "          Internal QP solve time : " << solve_time / repeat << " us (" << hqp_.solve_time_max_ << " us )" << std::endl;
    }

    std::cout << "-----------------------------------------------------------------" << std::endl;
    {
        RobotData rd_;
        rd_.LoadModelData(urdf3_file, true, false);

        rd_.UpdateKinematics(q3, q3dot, q3ddot);
        rd_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.13, 0.06, verbose);
        rd_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.13, 0.06, verbose);

        rd_.AddTaskSpace(0, TASK_LINK_POSITION, desired_control_target.c_str(), Vector3d::Zero(), verbose);
        rd_.AddTaskSpace(0, TASK_LINK_ROTATION, desired_control_target2.c_str(), Vector3d::Zero(), verbose);
        rd_.AddTaskSpace(1, TASK_LINK_6D, desired_control_target5.c_str(), Vector3d::Zero(), verbose);
        rd_.SetContact(contact1, contact2);
        // rd_.CalcContactConstraint();

        rd_.ReducedDynamicsCalculate();

        rd_.SetTaskSpace(0, fstar_0);
        rd_.SetTaskSpace(1, fstar_2);
        rd_.UpdateTaskSpace();
        DWBC::HQP hqp_;
        rd_.ConfigureLQP_R(hqp_);

        auto t2_0 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < repeat; i++)
        {
            rd_.ReducedDynamicsCalculate();
        }
        double lqp1_total_time = 0;
        double lqp1_prep_time = 0;
        double lqp1_solve_time = 0;

        double lqp2_total_time = 0;
        double lqp2_prep_time = 0;
        double lqp2_solve_time = 0;

        auto t2_1 = std::chrono::high_resolution_clock::now();

        rd_.CalcControlTorqueLQP_R(hqp_, true);
        for (int i = 0; i < repeat; i++)
        {
            rd_.CalcControlTorqueLQP_R(hqp_, false);
            lqp1_total_time += hqp_.total_time_step_;
            lqp1_prep_time += hqp_.update_time_step_;
            lqp1_solve_time += hqp_.solve_time_step_;
        }
        auto t2_2 = std::chrono::high_resolution_clock::now();

        VectorXd jacc = hqp_.hqp_hs_.back().y_ans_.head(hqp_.acceleration_size_);
        DWBC::HQP hqp_nc_;
        rd_.ConfigureLQP_R_NC(hqp_nc_, jacc);

        rd_.CalcControlTorqueLQP_R_NC(hqp_nc_, true);
        for (int i = 0; i < repeat; i++)
        {
            rd_.CalcControlTorqueLQP_R_NC(hqp_nc_, false);
            lqp2_total_time += hqp_nc_.total_time_step_;
            lqp2_prep_time += hqp_nc_.update_time_step_;
            lqp2_solve_time += hqp_nc_.solve_time_step_;
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

        proposed_solve_time = lqp1_solve_time / repeat + lqp2_solve_time / repeat;

        std::cout << "Reduced Dynamics Model COMP 2 TOTAL CONSUMPTION : " << (float)(time_original_us2 / repeat) << " us" << std::endl;
        std::cout << "Reduced Dynamics Model 1 - Reduced Dyn calc     : " << (float)(time_original_us21 / repeat) << " us" << std::endl;
        std::cout << "Reduced Dynamics Model 2 - LQP 1                : " << (float)(lqp1_total_time / repeat) << " us" << std::endl;
        std::cout << "Reduced Dynamics Model 2 - Update + Solve       : " << (float)(lqp1_prep_time / repeat) << " us | " << (float)(lqp1_solve_time / repeat) << " us" << std::endl;
        std::cout << "Reduced Dynamics Model 2 - LQP 2                : " << (float)(lqp2_total_time / repeat) << " us" << std::endl;
        std::cout << "Reduced Dynamics Model 2 - Update + Solve       : " << (float)(lqp2_prep_time / repeat) << " us | " << (float)(lqp2_solve_time / repeat) << " us" << std::endl;

        std::cout << "-----------------------------------------------------------------" << std::endl;

        std::cout << rd_.model_dof_ << "DOF : total comparison : " << t2_time / t1_time * 100 << " % (lower is better)" << std::endl;
        std::cout << rd_.model_dof_ << "DOF :    qp comparison : " << proposed_solve_time / original_solve_time * 100 << " % (lower is better)" << std::endl;
    }

    // {
    //     RobotData rd_;
    //     rd_.LoadModelData(urdf3_file, true, false);

    //     rd_.UpdateKinematics(q3, q3dot, q3ddot);
    //     rd_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.13, 0.06, verbose);
    //     rd_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.13, 0.06, verbose);

    //     rd_.AddTaskSpace(0, TASK_LINK_POSITION, desired_control_target.c_str(), Vector3d::Zero(), verbose);
    //     rd_.AddTaskSpace(0, TASK_LINK_ROTATION, desired_control_target2.c_str(), Vector3d::Zero(), verbose);
    //     rd_.AddTaskSpace(1, TASK_LINK_6D, desired_control_target5.c_str(), Vector3d::Zero(), verbose);
    //     rd_.SetContact(contact1, contact2);
    //     // rd_.CalcContactConstraint();

    //     // rd_.ReducedDynamicsCalculate();

    //     rd_.SetTaskSpace(0, fstar_0);
    //     rd_.SetTaskSpace(1, fstar_2);

    //     rd_.UpdateTaskSpace();

    //     DWBC::HQP hqp_;
    //     int acc_size = rd_.system_dof_;
    //     int torque_size = rd_.model_dof_;
    //     int contact_size = rd_.contact_dof_;
    //     int variable_size = acc_size + contact_size;

    //     int acc_idx = 0;
    //     int contact_idx = acc_size + 0;
    //     hqp_.initialize(acc_size, 0, contact_size);

    //     MatrixXd cost_h = MatrixXd::Zero(acc_size + contact_size, acc_size + contact_size);
    //     VectorXd cost_g = VectorXd::Zero(acc_size + contact_size);

    //     cost_h.block(0, 0, acc_size, acc_size) = rd_.A_ / rd_.A_.norm() * 5;

    //     // Priority 1
    //     int ineq_constraint_size = torque_size * 2; // torque limit
    //     int eq_constraint_size = 6;                 // newton euler

    //     hqp_.addHierarchy(ineq_constraint_size, eq_constraint_size);
    //     // hqp_.hqp_hs_[0].y_ans_.head(variable_size).setZero();

    //     MatrixXd A = MatrixXd::Zero(ineq_constraint_size, variable_size);
    //     VectorXd a = VectorXd::Zero(ineq_constraint_size);

    //     MatrixXd B = MatrixXd::Zero(eq_constraint_size, variable_size);
    //     VectorXd b = VectorXd::Zero(eq_constraint_size);

    //     B.block(0, 0, eq_constraint_size, acc_size) = rd_.A_.topRows(eq_constraint_size);
    //     B.block(0, acc_size, eq_constraint_size, contact_size) = rd_.J_C.transpose().topRows(eq_constraint_size);
    //     b.head(eq_constraint_size) = rd_.B_.head(eq_constraint_size);

    //     VectorXd tlim = VectorXd::Constant(torque_size, 200);

    //     A.block(0, 0, torque_size, acc_size) = rd_.A_.bottomRows(torque_size);
    //     A.block(0, acc_size, torque_size, contact_size) = rd_.J_C.transpose().bottomRows(torque_size);
    //     A.block(torque_size, 0, torque_size, acc_size) = -rd_.A_.bottomRows(torque_size);
    //     A.block(torque_size, acc_size, torque_size, contact_size) = -rd_.J_C.transpose().bottomRows(torque_size);

    //     // A.block(0, acc_size, torque_size, torque_size) = MatrixXd::Identity(torque_size, torque_size);            // torque limit
    //     // A.block(torque_size, acc_size, torque_size, torque_size) = -MatrixXd::Identity(torque_size, torque_size); // torque limit
    //     a.head(torque_size) = -tlim + rd_.B_.tail(torque_size);
    //     a.tail(torque_size) = -tlim - rd_.B_.tail(torque_size);

    //     // std::cout << "test0 " << hqp_.hqp_hs_[0].ineq_const_size_ << std::endl;
    //     // std::cout << "A :rows : " << A.rows() << " cols : " << A.cols() << std::endl;

    //     hqp_.hqp_hs_[0].updateConstraintMatrix(A, a, B, b);
    //     hqp_.hqp_hs_[0].normalizeConstraintMatrix();
    //     // Solve
    //     hqp_.hqp_hs_[0].v_ans_.setZero(ineq_constraint_size);
    //     hqp_.hqp_hs_[0].w_ans_.setZero(eq_constraint_size);
    //     hqp_.hqp_hs_[0].y_ans_.head(acc_size) = -rd_.A_inv_ * rd_.B_;

    //     // Priority 2
    //     int contact_constraint_size = rd_.contact_link_num_ * 10;
    //     ineq_constraint_size = contact_constraint_size + torque_size * 2;
    //     eq_constraint_size = contact_size;

    //     hqp_.addHierarchy(ineq_constraint_size, eq_constraint_size);

    //     A.setZero(ineq_constraint_size, variable_size);
    //     a.setZero(ineq_constraint_size);
    //     A.block(0, acc_size, contact_constraint_size, contact_size) = rd_.getContactConstraintMatrix();
    //     A.block(contact_constraint_size, 6, torque_size, torque_size) = MatrixXd::Identity(torque_size, torque_size);
    //     A.block(contact_constraint_size + torque_size, 6, torque_size, torque_size) = -MatrixXd::Identity(torque_size, torque_size);

    //     VectorXd alim = VectorXd::Constant(torque_size, alim_val);
    //     a.segment(contact_constraint_size, torque_size) = -alim;
    //     a.segment(contact_constraint_size + torque_size, torque_size) = -alim;

    //     B.setZero(eq_constraint_size, variable_size);
    //     b.setZero(eq_constraint_size);
    //     B.block(0, 0, contact_size, acc_size) = rd_.J_C;

    //     hqp_.hqp_hs_[1].updateConstraintMatrix(A, a, B, b);
    //     hqp_.hqp_hs_[1].updateCostMatrix(cost_h, cost_g);
    //     hqp_.hqp_hs_[1].normalizeConstraintMatrix();

    //     // Priority 3
    //     ineq_constraint_size = 0;
    //     eq_constraint_size = 6;

    //     hqp_.addHierarchy(ineq_constraint_size, eq_constraint_size);

    //     B.setZero(eq_constraint_size, variable_size);
    //     b.setZero(eq_constraint_size);

    //     // MatrixXd J_task = MatrixXd::Zero(6, acc_size);
    //     // VectorXd f_star = VectorXd::Zero(6);

    //     // J_task.topRows(3) = rd_.ts_[0].J_task_;
    //     // J_task.bottomRows(3) = rd_.ts_[1].J_task_;

    //     // f_star.head(3) = rd_.ts_[0].f_star_;
    //     // f_star.tail(3) = rd_.ts_[1].f_star_;

    //     B.block(0, 0, 6, acc_size) = rd_.ts_[0].J_task_;
    //     b.head(6) = -rd_.ts_[0].f_star_;

    //     hqp_.hqp_hs_[2].updateConstraintMatrix(A, a, B, b);
    //     hqp_.hqp_hs_[2].updateCostMatrix(cost_h, cost_g);
    //     hqp_.hqp_hs_[2].normalizeConstraintMatrix();

    //     // Priority 4
    //     ineq_constraint_size = 0;
    //     eq_constraint_size = 6;

    //     B.setZero(eq_constraint_size, variable_size);
    //     b.setZero(eq_constraint_size);

    //     B.block(0, 0, 6, acc_size) = rd_.ts_[1].J_task_;
    //     b.head(6) = -rd_.ts_[1].f_star_;
    //     // B.block(6, 0, acc_size, acc_size) = rd_.A_;

    //     // std::cout << "test3" << std::endl;
    //     hqp_.addHierarchy(ineq_constraint_size, eq_constraint_size);
    //     hqp_.hqp_hs_[3].updateConstraintMatrix(A, a, B, b);
    //     hqp_.hqp_hs_[3].updateCostMatrix(cost_h, cost_g);
    //     hqp_.hqp_hs_[3].normalizeConstraintMatrix();

    //     // eq_constraint_size = 6;
    //     // ineq_constraint_size = 0;

    //     hqp_.prepare();

    //     auto t2_0 = std::chrono::high_resolution_clock::now();
    //     bool init_qp = true;
    //     for (int i = 0; i < repeat; i++)
    //     {
    //         hqp_.solveSequential(init_qp);

    //         init_qp = false;
    //     }
    //     auto t2_1 = std::chrono::high_resolution_clock::now();

    //     double time_original_us2 = std::chrono::duration_cast<std::chrono::microseconds>(t2_1 - t2_0).count();

    //     t1_time = time_original_us2 / repeat;

    //     std::cout << "ORINGAL HERZOG Dynamics Model COMP 2 TOTAL CONSUMPTION : " << (float)(time_original_us2 / repeat) << " us" << std::endl;

    //     double prep_time = 0;
    //     double solve_time = 0;
    //     for (int i = 0; i < hqp_.hqp_hs_.size(); i++)
    //     {
    //         prep_time += hqp_.hqp_hs_[i].qp_update_time_;
    //         solve_time += hqp_.hqp_hs_[i].qp_solve_time_;

    //         std::cout << " Task " << i << " : " << hqp_.hqp_hs_[i].qp_update_time_ / repeat << " us | " << hqp_.hqp_hs_[i].qp_solve_time_ / repeat << std::endl;
    //     }

    //     std::cout << "PREP TIME : " << prep_time / repeat << " us" << std::endl;
    //     std::cout << "SOLVE TIME : " << solve_time / repeat << " us" << std::endl;
    // }

    // std::cout << "-----------------------------------------------------------------" << std::endl;
    // {
    //     RobotData rd_;
    //     rd_.LoadModelData(urdf3_file, true, false);

    //     rd_.UpdateKinematics(q3, q3dot, q3ddot);
    //     rd_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.13, 0.06, verbose);
    //     rd_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.13, 0.06, verbose);

    //     rd_.AddTaskSpace(0, TASK_LINK_POSITION, desired_control_target.c_str(), Vector3d::Zero(), verbose);
    //     rd_.AddTaskSpace(1, TASK_LINK_ROTATION, desired_control_target2.c_str(), Vector3d::Zero(), verbose);
    //     rd_.AddTaskSpace(2, TASK_LINK_6D, desired_control_target5.c_str(), Vector3d::Zero(), verbose);
    //     rd_.SetContact(contact1, contact2);
    //     // rd_.CalcContactConstraint();

    //     rd_.ReducedDynamicsCalculate();

    //     rd_.SetTaskSpace(0, fstar_0);
    //     rd_.SetTaskSpace(1, fstar_1);
    //     rd_.SetTaskSpace(2, fstar_2);
    //     rd_.UpdateTaskSpace();
    //     DWBC::HQP hqp_;
    //     int acc_size = rd_.reduced_system_dof_;
    //     int torque_size = rd_.reduced_model_dof_;
    //     int contact_size = rd_.contact_dof_;
    //     int variable_size = acc_size + contact_size;

    //     int acc_idx = 0;
    //     // int toruqe_idx = acc_size;
    //     int contact_idx = acc_size + 0;
    //     hqp_.initialize(acc_size, 0, contact_size);

    //     MatrixXd cost_h = MatrixXd::Zero(acc_size + contact_size, acc_size + contact_size);
    //     VectorXd cost_g = VectorXd::Zero(acc_size + contact_size);

    //     cost_h.block(0, 0, acc_size, acc_size) = rd_.A_R / rd_.A_.norm();

    //     // Priority 1
    //     int ineq_constraint_size = torque_size * 2; // torque limit
    //     int eq_constraint_size = 6;                 // newton euler

    //     hqp_.addHierarchy(ineq_constraint_size, eq_constraint_size);
    //     // hqp_.hqp_hs_[0].y_ans_.head(variable_size).setZero();

    //     MatrixXd A = MatrixXd::Zero(ineq_constraint_size, variable_size);
    //     VectorXd a = VectorXd::Zero(ineq_constraint_size);

    //     MatrixXd B = MatrixXd::Zero(eq_constraint_size, variable_size);
    //     VectorXd b = VectorXd::Zero(eq_constraint_size);

    //     B.block(0, 0, eq_constraint_size, acc_size) = rd_.A_R.topRows(eq_constraint_size);
    //     B.block(0, acc_size, eq_constraint_size, contact_size) = (rd_.J_CR.transpose()).topRows(eq_constraint_size);
    //     b.head(eq_constraint_size) = rd_.G_R.head(eq_constraint_size);

    //     VectorXd tlim = VectorXd::Constant(torque_size, tlim_val);
    //     tlim(torque_size - 4) += 400;

    //     A.block(0, 0, torque_size, acc_size) = rd_.A_R.bottomRows(torque_size);
    //     A.block(0, acc_size, torque_size, contact_size) = rd_.J_CR.transpose().bottomRows(torque_size);
    //     A.block(torque_size, 0, torque_size, acc_size) = -rd_.A_R.bottomRows(torque_size);
    //     A.block(torque_size, acc_size, torque_size, contact_size) = -rd_.J_CR.transpose().bottomRows(torque_size);

    //     // A.block(0, acc_size, torque_size, torque_size) = MatrixXd::Identity(torque_size, torque_size);            // torque limit
    //     // A.block(torque_size, acc_size, torque_size, torque_size) = -MatrixXd::Identity(torque_size, torque_size); // torque limit
    //     a.head(torque_size) = -tlim + rd_.G_R.tail(torque_size);
    //     a.tail(torque_size) = -tlim - rd_.G_R.tail(torque_size);

    //     // std::cout << "test0 " << hqp_.hqp_hs_[0].ineq_const_size_ << std::endl;
    //     // std::cout << "A :rows : " << A.rows() << " cols : " << A.cols() << std::endl;

    //     hqp_.hqp_hs_[0].updateConstraintMatrix(A, a, B, b);
    //     hqp_.hqp_hs_[0].normalizeConstraintMatrix();
    //     // Solve
    //     hqp_.hqp_hs_[0].v_ans_.setZero(ineq_constraint_size);
    //     hqp_.hqp_hs_[0].w_ans_.setZero(eq_constraint_size);
    //     hqp_.hqp_hs_[0].y_ans_.head(acc_size) = -rd_.A_R_inv * rd_.G_R;

    //     // Priority 2
    //     int contact_constraint_size = rd_.contact_link_num_ * 10;
    //     ineq_constraint_size = contact_constraint_size + acc_size * 2;
    //     eq_constraint_size = contact_size;

    //     hqp_.addHierarchy(ineq_constraint_size, eq_constraint_size);
    //     A.setZero(ineq_constraint_size, variable_size);
    //     a.setZero(ineq_constraint_size);
    //     A.block(0, acc_size, contact_constraint_size, contact_size) = rd_.getContactConstraintMatrix();
    //     A.block(contact_constraint_size, 0, acc_size, acc_size) = MatrixXd::Identity(acc_size, acc_size);
    //     A.block(contact_constraint_size + acc_size, 0, acc_size, acc_size) = -MatrixXd::Identity(acc_size, acc_size);

    //     VectorXd alim = VectorXd::Constant(acc_size, alim_val);
    //     a.segment(contact_constraint_size, acc_size) = -alim;
    //     a.segment(contact_constraint_size + acc_size, acc_size) = -alim;

    //     B.setZero(eq_constraint_size, variable_size);
    //     b.setZero(eq_constraint_size);
    //     B.block(0, 0, contact_size, acc_size) = rd_.J_CR;

    //     // std::cout << "test1" << std::endl;
    //     hqp_.hqp_hs_[1].updateConstraintMatrix(A, a, B, b);
    //     hqp_.hqp_hs_[1].updateCostMatrix(cost_h, cost_g);
    //     hqp_.hqp_hs_[1].normalizeConstraintMatrix();

    //     // Priority 3
    //     ineq_constraint_size = 0;
    //     eq_constraint_size = 6;

    //     hqp_.addHierarchy(ineq_constraint_size, eq_constraint_size);

    //     B.setZero(eq_constraint_size, variable_size);
    //     b.setZero(eq_constraint_size);

    //     MatrixXd J_task = MatrixXd::Zero(6, acc_size);
    //     VectorXd f_star = VectorXd::Zero(6);

    //     J_task.topRows(3) = rd_.ts_[0].J_task_ * rd_.J_R_INV_T.transpose();
    //     J_task.bottomRows(3) = rd_.ts_[1].J_task_ * rd_.J_R_INV_T.transpose();

    //     f_star.head(3) = rd_.ts_[0].f_star_;
    //     f_star.tail(3) = rd_.ts_[1].f_star_;

    //     B.block(0, 0, 6, acc_size) = J_task;
    //     b.head(6) = -f_star;

    //     // std::cout << "J task for h2 : \n"
    //     //           << J_task << std::endl;
    //     // std::cout << "f star for h2 : \n"
    //     //           << f_star << std::endl;

    //     hqp_.hqp_hs_[2].updateConstraintMatrix(A, a, B, b);
    //     hqp_.hqp_hs_[2].updateCostMatrix(cost_h, cost_g);
    //     hqp_.hqp_hs_[2].normalizeConstraintMatrix();

    //     // Priority 4
    //     ineq_constraint_size = 0;
    //     eq_constraint_size = 6;

    //     B.setZero(eq_constraint_size, variable_size);
    //     b.setZero(eq_constraint_size);

    //     // B.block(0, 0, 6, acc_size) = rd_.ts_[2].J_task_;
    //     // b.head(6) = -rd_.ts_[2].f_star_;
    //     // B.block(6, 0, acc_size, acc_size) = rd_.A_;

    //     // std::cout << "test3" << std::endl;
    //     // hqp_.addHierarchy(ineq_constraint_size, eq_constraint_size);
    //     // hqp_.hqp_hs_[3].updateConstraintMatrix(A, a, B, b);
    //     // hqp_.hqp_hs_[3].updateCostMatrix(cost_h, cost_g);

    //     // eq_constraint_size = 6;
    //     // ineq_constraint_size = 0;
    //     hqp_.prepare();

    //     rd_.SetContact(contact1, contact2);

    //     rd_.UpdateTaskSpace();

    //     auto t2_0 = std::chrono::high_resolution_clock::now();
    //     for (int i = 0; i < repeat; i++)
    //     {

    //         rd_.ReducedDynamicsCalculate();
    //     }
    //     auto t2_1 = std::chrono::high_resolution_clock::now();

    //     bool init_qp = true;
    //     for (int i = 0; i < repeat; i++)
    //     {
    //         hqp_.solveSequential(init_qp);

    //         init_qp = false;
    //     }
    //     auto t2_2 = std::chrono::high_resolution_clock::now();

    //     VectorXd fstar_gnc = VectorXd::Zero(6);
    //     fstar_gnc = hqp_.hqp_hs_[2].y_ans_.segment(acc_size - 6, 6);
    //     VectorXd fstar_base = VectorXd::Zero(6);
    //     fstar_base = hqp_.hqp_hs_[2].y_ans_.head(6);

    //     MatrixXd jtask_gnc = rd_.J_I_nc_;

    //     HQP hqp_nc_;
    //     int acc_size_nc = rd_.nc_dof;
    //     int torque_size_nc = rd_.nc_dof;
    //     variable_size = acc_size_nc;

    //     MatrixXd cost_h_nc = MatrixXd::Zero(variable_size, variable_size);
    //     VectorXd cost_g_nc = VectorXd::Zero(variable_size);

    //     cost_h_nc = rd_.A_NC.bottomRightCorner(acc_size_nc, acc_size_nc) / rd_.A_NC.bottomRightCorner(acc_size_nc, acc_size_nc).norm();

    //     hqp_nc_.initialize(acc_size_nc, 0, 0);

    //     eq_constraint_size = 6;
    //     ineq_constraint_size = 2 * acc_size_nc;

    //     hqp_nc_.addHierarchy(ineq_constraint_size, eq_constraint_size);

    //     MatrixXd A_nc = MatrixXd::Zero(ineq_constraint_size, variable_size);
    //     VectorXd a_nc = VectorXd::Zero(ineq_constraint_size);

    //     MatrixXd B_nc = MatrixXd::Zero(eq_constraint_size, variable_size);
    //     VectorXd b_nc = VectorXd::Zero(eq_constraint_size);

    //     // gcom const
    //     B_nc.block(0, 0, eq_constraint_size, acc_size_nc) = jtask_gnc;
    //     b_nc.head(eq_constraint_size) = -fstar_gnc;

    //     VectorXd tlim_nc = VectorXd::Constant(torque_size_nc, 200);
    //     A_nc.block(0, 0, acc_size_nc, acc_size_nc) = rd_.A_NC.bottomRightCorner(acc_size_nc, acc_size_nc);
    //     a_nc.segment(0, acc_size_nc) = -tlim_nc + rd_.G_NC;
    //     A_nc.block(1 * acc_size_nc, 0, acc_size_nc, acc_size_nc) = -rd_.A_NC.bottomRightCorner(acc_size_nc, acc_size_nc);
    //     a_nc.segment(1 * acc_size_nc, acc_size_nc) = -tlim_nc - rd_.G_NC;

    //     // // torque lim
    //     // A_nc.block(2 * acc_size_nc, 0, acc_size_nc, acc_size_nc) = rd_.A_NC.bottomRightCorner(acc_size_nc, acc_size_nc);
    //     // a_nc.segment(2 * acc_size_nc, acc_size_nc) = -tlim_nc + rd_.G_NC;

    //     // A_nc.block(3 * acc_size_nc, 0, acc_size_nc, acc_size_nc) = -rd_.A_NC.bottomRightCorner(acc_size_nc, acc_size_nc);

    //     // a_nc.segment(3 * acc_size_nc, acc_size_nc) = -tlim_nc - rd_.G_NC;

    //     hqp_nc_.hqp_hs_[0].updateConstraintMatrix(A_nc, a_nc, B_nc, b_nc);
    //     hqp_nc_.hqp_hs_[0].updateCostMatrix(cost_h_nc, cost_g_nc);

    //     eq_constraint_size = 6;
    //     ineq_constraint_size = 2 * acc_size_nc;

    //     hqp_nc_.addHierarchy(ineq_constraint_size, eq_constraint_size);

    //     MatrixXd Ja = MatrixXd::Identity(6, 6);
    //     Ja.block(0, 3, 3, 3) = skew(rd_.link_[rd_.ts_[2].task_link_[0].link_id_].xpos - rd_.link_[0].xpos);

    //     VectorXd fstar_local = Ja * (rd_.ts_[2].f_star_ - fstar_base);

    //     B_nc.setZero(eq_constraint_size, variable_size);
    //     b_nc.setZero(eq_constraint_size);

    //     B_nc.block(0, 0, 6, acc_size_nc) = rd_.ts_[2].J_task_.rightCols(rd_.nc_dof);
    //     b_nc.head(6) = -fstar_local;

    //     // acc lim
    //     A_nc.block(0, 0, acc_size_nc, acc_size_nc) = MatrixXd::Identity(acc_size_nc, acc_size_nc);
    //     a_nc.segment(0, acc_size_nc) = -VectorXd::Constant(acc_size_nc, 5);

    //     A_nc.block(acc_size_nc, 0, acc_size_nc, acc_size_nc) = -MatrixXd::Identity(acc_size_nc, acc_size_nc);
    //     a_nc.segment(acc_size_nc, acc_size_nc) = -VectorXd::Constant(acc_size_nc, 5);

    //     hqp_nc_.hqp_hs_[1].updateConstraintMatrix(A_nc, a_nc, B_nc, b_nc);
    //     hqp_nc_.hqp_hs_[1].updateCostMatrix(cost_h_nc, cost_g_nc);

    //     hqp_nc_.prepare();

    //     // auto t2_3 = std::chrono::high_resolution_clock::now();

    //     init_qp = true;
    //     for (int i = 0; i < repeat; i++)
    //     {
    //         hqp_nc_.solvefirst(init_qp);
    //         hqp_nc_.solveSequential(init_qp);

    //         init_qp = false;
    //     }

    //     auto t2_3 = std::chrono::high_resolution_clock::now();

    //     double time_original_us2 = std::chrono::duration_cast<std::chrono::microseconds>(t2_3 - t2_0).count();
    //     double time_original_us21 = std::chrono::duration_cast<std::chrono::microseconds>(t2_1 - t2_0).count();
    //     double time_original_us22 = std::chrono::duration_cast<std::chrono::microseconds>(t2_2 - t2_1).count();
    //     double time_original_us23 = std::chrono::duration_cast<std::chrono::microseconds>(t2_3 - t2_2).count();

    //     t2_time = time_original_us2 / repeat;
    //     t2_dynamics_calc = time_original_us21 / repeat;
    //     t2_task_calc = time_original_us22 / repeat;
    //     t2_nctask_calc = time_original_us23 / repeat;

    //     std::cout << "Reduced Dynamics Model COMP 2 TOTAL CONSUMPTION : " << (float)(time_original_us2 / repeat) << " us" << std::endl;
    //     std::cout << "Reduced Dynamics Model 1 - Reduced Dynamics calculation : " << (float)(time_original_us21 / repeat) << " us" << std::endl;
    //     std::cout << "Reduced Dynamics Model 2 - Task Space Calculation         : " << (float)(time_original_us22 / repeat) << " us" << std::endl;
    //     std::cout << "Reduced Dynamics Model 2 - NC Space Calculation         : " << (float)(time_original_us23 / repeat) << " us" << std::endl;

    //     std::cout << "-----------------------------------------------------------------" << std::endl;

    //     std::cout << rd_.model_dof_ << "DOF : total comparison : " << t2_time / t1_time * 100 << " %" << std::endl;
    // }
    // }

    if (t1_success && t2_success)
    {
        std::string execDir = getExecutableDir();
        std::string filePath = execDir + "/../../../output.txt";
        std::ofstream outputFile(filePath, std::ios::app);
        outputFile << std::fixed << std::setprecision(4);
        outputFile << dof_input << " " << t1_time << " " << t2_time << " " << t2_dynamics_calc << " " << t2_task_calc << " " << t2_nctask_calc << " " << t2_time / t1_time << std::endl;
        outputFile.close();
    }
    else
    {
        std::cout << "Fail" << std::endl;
    }

    return 0;
}

int main(int argc, char **argv)
{
    int dof_option = -1; // 초기화: 옵션이 제공되지 않으면 -1로 남음
    int repeat_option = 1000;

    int opt;
    while ((opt = getopt(argc, argv, "d:r:")) != -1)
    {
        switch (opt)
        {
        case 'd':
            dof_option = std::stoi(optarg);
            break;
        case 'r':
            repeat_option = std::stoi(optarg);
            break;
        default:
            std::cerr << "Usage: " << argv[0] << " -d <dof> -r <repeatr>\n";
            return 1;
        }
    }

    std::cout << "dof_option = " << dof_option << std::endl;
    std::cout << "repeat_option = " << repeat_option << std::endl;

    // int option = 0;
    // if (argc > 1)
    // {
    //     option = std::stoi(argv[1]);
    //     compare(dof_option);
    // }
    // else
    // {
    //     for (int i = 20; i < 46; i++)
    //     {
    //         compare(i);
    //     }
    // }

    if (dof_option != -1)
    {
        compare(dof_option, repeat_option);
    }
    else
    {
        for (int i = 20; i < 46; i++)
        {
            compare(i, repeat_option);
        }
    }

    return 0;
}
