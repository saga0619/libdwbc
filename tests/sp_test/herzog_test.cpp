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
    double rot_z = 0;
    int repeat = 1000;

    bool use_hqp = true;

    std::string urdf2_file = std::string(URDF_DIR) + "/dyros_tocabi.urdf";

    std::string desired_control_target = "COM";
    std::string desired_control_target2 = "pelvis_link";
    std::string desired_control_target3 = "upperbody_link";
    std::string desired_control_target4 = "L_Wrist2_Link";
    std::string desired_control_target5 = "R_Wrist2_Link";

    VectorXd fstar_1;
    fstar_1.setZero(6);
    fstar_1 << 0.5, 0.3, 0.2, 0.12, -0.11, 0.05; // based on local frmae.

    // fstar_1

    RobotData rd2_;
    rd2_.LoadModelData(urdf2_file, true, false);

    VectorXd t2lim;
    VectorXd q2 = VectorXd::Zero(rd2_.model_.q_size);
    VectorXd q2dot = VectorXd::Zero(rd2_.model_.qdot_size);
    VectorXd q2ddot = VectorXd::Zero(rd2_.model_.qdot_size);

    // rd_.ChangeLinkToFixedJoint("head_link", verbose);
    // VectorXd tlim;
    // tlim.setConstant(rd2_.model_dof_, 500);
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

    // q2 << 0, 0, 0, qu.x(), qu.y(), qu.z(),
    //     0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
    //     0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
    //     0, 0, 0, 0,0,0, qu.w();

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

    rd2_.SetContact(true, true);
    rd2_.CalcContactConstraint();

    fstar_1.segment(0, 3) = rd2_.link_[0].rotm * fstar_1.segment(0, 3);
    fstar_1.segment(3, 3) = rd2_.link_[0].rotm * fstar_1.segment(3, 3);

    MatrixXd J_task3;
    VectorXd f_star3;

    // int lh_id = rd2_.getLinkID("L_Wrist2_Link");
    // int rh_id = rd2_.getLinkID("R_Wrist2_Link");

    f_star3.setZero(12);
    f_star3.segment(0, 6) = 0.5 * fstar_1;
    f_star3.segment(6, 6) = 0.2 * fstar_1;

    // f_star3(3) = 1.0;
    // f_star3(9) = 1.0;

    Vector6d f_star0;
    f_star0 << -2, -2.2, 0.2, 0.5, 0.4, -0.6;

    rd2_.SetTaskSpace(0, f_star0);
    // rd2_.SetTaskSpace(1, fstar_1.segment(3, 3));
    // rd2_.SetTaskSpace(2, -fstar_1.segment(3, 3));
    // rd2_.SetTaskSpace(3, f_star3);

    // rd2_.cc_[0].fz_limiter_switch_ = true;
    // rd2_.cc_[1].fz_limiter_switch_ = true;

    // rd2_.cc_[0].fz_limit_ = -1000;
    // rd2_.cc_[1].fz_limit_ = -1000;

    rd2_.CalcGravCompensation();
    rd2_.CalcTaskControlTorque(use_hqp, true);
    rd2_.CalcContactRedistribute(use_hqp, true);
    auto t10 = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < repeat; i++)
    {
        rd2_.SetContact(true, true);
        rd2_.CalcContactConstraint();
        rd2_.CalcGravCompensation();
    }
    auto t11 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < repeat; i++)
    {
        rd2_.CalcTaskSpace();
    }
    auto t12 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < repeat; i++)
    {
        rd2_.CalcTaskControlTorque(use_hqp, false, false);
    }
    auto t13 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < repeat; i++)
    {
        rd2_.CalcContactRedistribute(use_hqp, false);
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
    std::cout << " QP Fstar Original : " << rd2_.ts_[0].f_star_.transpose() << std::endl;
    std::cout << " QP Fstar modified : " << rd2_.ts_[0].f_star_.transpose() + rd2_.ts_[0].f_star_qp_.transpose() << std::endl;

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

    // MatrixXd jtask = rd2_.ts_[3].J_task_.bottomRightCorner(6, 8);

    // std::cout << "effective mass : \n"
    //           << (jtask * rd2_.A_.bottomRightCorner(8, 8).inverse() * jtask.transpose()).inverse() << std::endl;

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

    // std::cout << "lf cp pos : " << rd2_.cc_[0].zmp_pos.transpose() - rd2_.cc_[0].xc_pos.transpose() << std::endl;
    // std::cout << "rf cp vel : " << rd2_.cc_[1].zmp_pos.transpose() - rd2_.cc_[1].xc_pos.transpose() << std::endl;

    // std::cout << " zmp pos : " << zmp_pos.transpose() << std::endl;

    // std::cout << "J_C" << std::endl;
    // std::cout << rd2_.J_C << std::endl;

    // qpOASES

    DWBC::CQuadraticProgram qp_herzog1_;

    /**
     * initialize herzog qp h1
     * h1 is the highest priority task
     * y = qacc, torque, contact force
     * v = inequality constraint slack variable
     * w = equality constraint slack variable
     *
     * h1 constraint : dynamics, torque limit.
     *
     *
     *  */

    // int status_size = rd2_.model_dof_ * 2 + 6 + 12; // qacc, torque, contact force size

    // int ineq_constraint_size = rd2_.model_dof_ * 2; // torque limit.
    // int slace_v_size = ineq_constraint_size;

    // int eq_constraint_size = rd2_.model_dof_ + 6;
    // int slace_w_size = eq_constraint_size;

    // int total_opt_variable_size = status_size + slace_v_size + slace_w_size;
    // int total_constraint_size = ineq_constraint_size + eq_constraint_size;

    // qp_herzog1_.InitializeProblemSize(total_opt_variable_size, total_constraint_size);

    // MatrixXd A = MatrixXd::Zero(ineq_constraint_size, status_size);
    // VectorXd a = VectorXd::Zero(ineq_constraint_size);

    // MatrixXd B = MatrixXd::Zero(eq_constraint_size, status_size);
    // VectorXd b = VectorXd::Zero(eq_constraint_size);

    // // for torque limit ineqaility constraint
    // VectorXd tlim_lower = VectorXd::Constant(rd2_.model_dof_, -200);
    // VectorXd tlim_upper = VectorXd::Constant(rd2_.model_dof_, 200);

    // A.topRightCorner(rd2_.model_dof_, rd2_.model_dof_) = MatrixXd::Identity(rd2_.model_dof_, rd2_.model_dof_);
    // A.bottomRightCorner(rd2_.model_dof_, rd2_.model_dof_) = -MatrixXd::Identity(rd2_.model_dof_, rd2_.model_dof_);

    // a.head(rd2_.model_dof_) = -tlim_upper;
    // a.tail(rd2_.model_dof_) = -tlim_lower;

    // B.block(0, 0, rd2_.system_dof_, rd2_.system_dof_) = rd2_.A_;
    // B.block(0, rd2_.system_dof_, rd2_.system_dof_, rd2_.contact_dof_) = -rd2_.J_C.transpose();
    // B.block(6, rd2_.system_dof_ + rd2_.contact_dof_, rd2_.model_dof_, rd2_.model_dof_) = -MatrixXd::Identity(rd2_.model_dof_, rd2_.model_dof_);

    // b = rd2_.B_;

    // MatrixXd H = MatrixXd::Zero(total_opt_variable_size, total_opt_variable_size);
    // VectorXd g = VectorXd::Zero(total_opt_variable_size);

    // H.bottomRightCorner(slace_v_size + slace_w_size, slace_v_size + slace_w_size) = MatrixXd::Identity(slace_v_size + slace_w_size, slace_v_size + slace_w_size);

    // qp_herzog1_.UpdateMinProblem(H, g);

    // // combine A, B matrix to constraint matrix

    // MatrixXd qpA = MatrixXd::Zero(total_constraint_size, total_opt_variable_size);

    // qpA.block(0, 0, ineq_constraint_size, status_size) = A;
    // qpA.block(0, status_size, ineq_constraint_size, ineq_constraint_size) = -MatrixXd::Identity(ineq_constraint_size, ineq_constraint_size);

    // qpA.block(ineq_constraint_size, 0, eq_constraint_size, status_size) = B;
    // qpA.block(ineq_constraint_size, status_size + ineq_constraint_size, eq_constraint_size, eq_constraint_size) = -MatrixXd::Identity(eq_constraint_size, eq_constraint_size);

    // VectorXd lbA = VectorXd::Zero(total_constraint_size);
    // VectorXd ubA = VectorXd::Zero(total_constraint_size);

    // ubA.head(ineq_constraint_size) = -a;
    // ubA.tail(eq_constraint_size) = -b;

    // lbA.head(ineq_constraint_size) = -VectorXd::Constant(ineq_constraint_size, INFTY);
    // lbA.tail(eq_constraint_size) = -b;

    // qp_herzog1_.UpdateSubjectToAx(qpA, lbA, ubA);
    // qp_herzog1_.DeleteSubjectToX();

    // VectorXd qpres = VectorXd::Zero(total_opt_variable_size);

    // qp_herzog1_.SolveQPoases(1000, qpres);

    // int idx = 0;
    // std::cout << "qacc : " << std::endl;
    // std::cout << qpres.segment(idx, rd2_.model_dof_ + 6).transpose() << std::endl;
    // idx += rd2_.model_dof_ + 6;

    // std::cout << "contact force : " << std::endl;
    // std::cout << qpres.segment(idx, rd2_.contact_dof_).transpose() << std::endl;
    // idx += rd2_.contact_dof_;

    // std::cout << "torque : " << std::endl;
    // std::cout << qpres.segment(idx, rd2_.model_dof_).transpose() << std::endl;
    // idx += rd2_.model_dof_;

    // std::cout << "ineq constraint v : " << std::endl;
    // std::cout << qpres.segment(idx, slace_v_size).transpose() << std::endl;
    // idx += slace_v_size;

    // std::cout << "eq constraint w : " << std::endl;
    // std::cout << qpres.segment(idx, slace_w_size).transpose() << std::endl;
    // idx += slace_w_size;

    // x = acc + torque + contact force
    DWBC::HQP hqp_;

    int acc_size = rd2_.system_dof_;
    int torque_size = rd2_.model_dof_;
    int contact_size = rd2_.contact_dof_;

    hqp_.initialize(acc_size, torque_size, contact_size);

    int eq_constraint_size = 6;                     // newton euler
    int ineq_constraint_size = rd2_.model_dof_ * 2; // torque limit

    hqp_.addHierarchy(ineq_constraint_size, eq_constraint_size);
    hqp_.hqp_hs_[0].y_ans_.head(acc_size + torque_size + contact_size).setZero();

    MatrixXd A = MatrixXd::Zero(ineq_constraint_size, acc_size + torque_size + contact_size);
    VectorXd a = VectorXd::Zero(ineq_constraint_size);

    MatrixXd B = MatrixXd::Zero(eq_constraint_size, acc_size + torque_size + contact_size);
    VectorXd b = VectorXd::Zero(eq_constraint_size);

    B.block(0, 0, 6, acc_size) = rd2_.A_;
    B.block(0, acc_size + torque_size, 6, contact_size) = rd2_.J_C.transpose();
    b.head(6) = rd2_.B_.head(6);

    VectorXd tlim = VectorXd::Constant(torque_size, 200);

    A.block(0, acc_size, torque_size, torque_size) = MatrixXd::Identity(torque_size, torque_size);
    A.block(torque_size, acc_size, torque_size, torque_size) = -MatrixXd::Identity(torque_size, torque_size);
    a.head(torque_size) = -tlim;
    a.tail(torque_size) = -tlim;

    hqp_.hqp_hs_[0].updateConstraintMatrix(A, a, B, b);
    hqp_.hqp_hs_[0].v_ans_.setZero(ineq_constraint_size);
    hqp_.hqp_hs_[0].w_ans_.setZero(eq_constraint_size);
    hqp_.hqp_hs_[0].y_ans_.head(acc_size) = -rd2_.A_inv_ * rd2_.B_;

    int contact_constraint_size = 0;
    for (int i = 0; i < rd2_.cc_.size(); i++)
    {
        if (rd2_.cc_[i].contact)
        {
            contact_constraint_size += rd2_.cc_[i].constraint_number_;
        }
    }
    Eigen::MatrixXd A_const_a;
    A_const_a.setZero(contact_constraint_size, rd2_.contact_dof_);

    Eigen::MatrixXd A_rot;
    A_rot.setZero(rd2_.contact_dof_, rd2_.contact_dof_);
    int const_idx = 0;
    int contact_idx = 0;
    for (int i = 0; i < rd2_.cc_.size(); i++)
    {
        if (rd2_.cc_[i].contact)
        {
            A_rot.block(contact_idx, contact_idx, 3, 3) = rd2_.cc_[i].rotm.transpose();
            A_rot.block(contact_idx + 3, contact_idx + 3, 3, 3) = rd2_.cc_[i].rotm.transpose();

            A_const_a.block(const_idx, contact_idx, 4, 6) = rd2_.cc_[i].GetZMPConstMatrix4x6();
            A_const_a.block(const_idx + CONTACT_CONSTRAINT_ZMP, contact_idx, 6, 6) = rd2_.cc_[i].GetForceConstMatrix6x6();

            const_idx += rd2_.cc_[i].constraint_number_;
            contact_idx += rd2_.cc_[i].contact_dof_;
        }
    }

    eq_constraint_size = rd2_.contact_dof_;
    ineq_constraint_size = 10 * 2;
    hqp_.addHierarchy(ineq_constraint_size, eq_constraint_size);

    A.setZero(ineq_constraint_size, acc_size + torque_size + contact_size);
    a.setZero(ineq_constraint_size);

    A.block(0, acc_size + torque_size, contact_constraint_size, contact_size) = -A_const_a * A_rot;

    B.setZero(eq_constraint_size, acc_size + torque_size + contact_size);
    b.setZero(eq_constraint_size);

    B.block(0, 0, eq_constraint_size, acc_size) = rd2_.J_C;

    hqp_.hqp_hs_[1].updateConstraintMatrix(A, a, B, b);

    MatrixXd cost_h = MatrixXd::Zero(acc_size + torque_size + contact_size, acc_size + torque_size + contact_size);
    VectorXd cost_g = VectorXd::Zero(acc_size + torque_size + contact_size);

    cost_h.block(0, 0, acc_size, acc_size) = rd2_.A_;

    hqp_.hqp_hs_[1].updateCostMatrix(cost_h, cost_g);

    eq_constraint_size = 6;
    ineq_constraint_size = torque_size * 2;

    hqp_.addHierarchy(ineq_constraint_size, eq_constraint_size);

    A.setZero(ineq_constraint_size, acc_size + torque_size + contact_size);
    a.setZero(ineq_constraint_size);

    A.block(0, 6, torque_size, torque_size) = MatrixXd::Identity(torque_size, torque_size);            // torquelim
    A.block(torque_size, 6, torque_size, torque_size) = -MatrixXd::Identity(torque_size, torque_size); // torquelim

    VectorXd alim = VectorXd::Constant(torque_size, 10);
    a.head(torque_size) = -alim;
    a.tail(torque_size) = -alim;

    B.setZero(eq_constraint_size, acc_size + torque_size + contact_size);
    b.setZero(eq_constraint_size);

    B.block(0, 0, eq_constraint_size, acc_size) = rd2_.ts_[0].J_task_;
    b.head(6) = -rd2_.ts_[0].f_star_;

    hqp_.hqp_hs_[2].updateConstraintMatrix(A, a, B, b);

    hqp_.hqp_hs_[2].updateCostMatrix(cost_h, cost_g);

    // hqp_.addHierarchy(0, 6);
    // hqp_.hqp_hs_[3].updateConstraintMatrix(A, a, B, b);

    hqp_.prepare();

    auto t20 = std::chrono::high_resolution_clock::now();

    hqp_.solveSequential(true);
    for (int i = 0; i < repeat - 1; i++)
    {
        hqp_.solveSequential(false);
    }
    auto t21 = std::chrono::high_resolution_clock::now();

    double time_hqp_us = std::chrono::duration_cast<std::chrono::microseconds>(t21 - t20).count();

    std::cout << " Hierarchy QP  TOTAL CONSUMPTION : " << (float)(time_hqp_us / repeat) << " us" << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << " h0 res : " << hqp_.hqp_hs_[0].y_ans_.transpose() << std::endl;
    std::cout << " h1 res : " << hqp_.hqp_hs_[1].y_ans_.transpose() << std::endl;
    std::cout << " h2 res : " << hqp_.hqp_hs_[2].y_ans_.transpose() << std::endl;

    std::cout << "h2 jacc : " << hqp_.hqp_hs_[2].y_ans_.head(acc_size).transpose() << std::endl;
    std::cout << "h2 torque : " << hqp_.hqp_hs_[2].y_ans_.segment(acc_size, torque_size).transpose() << std::endl;
    std::cout << "h2 cf : " << hqp_.hqp_hs_[2].y_ans_.tail(contact_size).transpose() << std::endl;

    VectorXd jacc = hqp_.hqp_hs_[2].y_ans_.head(acc_size);
    VectorXd conf = hqp_.hqp_hs_[2].y_ans_.tail(contact_size);

    VectorXd calctorque = rd2_.A_.bottomRows(rd2_.model_dof_) * jacc + rd2_.B_.tail(rd2_.model_dof_) + rd2_.J_C.transpose().bottomRows(rd2_.model_dof_) * conf;

    std::cout << "Actuation Torque Calculated : " << calctorque.transpose() << std::endl;
    std::cout << "Fstar from accel :" << (rd2_.ts_[0].J_task_ * jacc).transpose() << std::endl;
    VectorXd act = rd2_.A_ * jacc + rd2_.B_ + rd2_.J_C.transpose() * conf;

    std::cout << "fstar from torque : " << (rd2_.ts_[0].J_task_ * rd2_.A_inv_ * rd2_.N_C * (act - rd2_.B_)).transpose() << std::endl;
    std::cout << "Contact Acceleration (0) : " << (rd2_.J_C * jacc).transpose() << std::endl;

    std::cout << "Actuation Torque(virtual incl.) : " << act.transpose() << std::endl;

    std::cout << "Base torque : " << (rd2_.A_.topRows(6) * jacc + rd2_.B_.head(6) + rd2_.J_C.transpose().topRows(6) * conf).transpose() << std::endl;

    // std::cout << "contact force validate" << std::endl;
    // std::cout << (-A_const_a * A_rot * conf).transpose() << std::endl;

    Vector3d zmp_pos = rd2_.getZMP(conf);

    std::cout << "lf cp pos : " << rd2_.cc_[0].zmp_pos.transpose() - rd2_.cc_[0].xc_pos.transpose() << std::endl;
    std::cout << "rf cp vel : " << rd2_.cc_[1].zmp_pos.transpose() - rd2_.cc_[1].xc_pos.transpose() << std::endl;
}
