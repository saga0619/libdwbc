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
    int repeat = 10000;

    bool use_hqp = false;
    std::cout << "use hqp? (y/n) : ";
    char input;
    std::cin >> input;
    if (input == 'y')
    {
        use_hqp = true;
        std::cout << "hqp enabled \n";
    }
    else
    {
        use_hqp = false;
        std::cout << "hqp disabled \n";
    }

    RobotData rd2_;

    std::string urdf2_file = std::string(URDF_DIR) + "/dyros_tocabi.urdf";

    std::string desired_control_target = "COM";
    std::string desired_control_target2 = "pelvis_link";
    std::string desired_control_target3 = "upperbody_link";
    std::string desired_control_target4 = "L_Wrist2_Link";

    VectorXd fstar_1;
    fstar_1.setZero(6);
    fstar_1 << 0.11, 0.5, 0.13, 0.12, -0.11, 0.05; // based on local frmae.

    // fstar_1
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

    q2 << 0, 0, 0.92983, qu.x(), qu.y(), qu.z(),
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
    rd2_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    rd2_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);

    rd2_.AddTaskSpace(TASK_LINK_6D, desired_control_target.c_str(), Vector3d::Zero(), verbose);
    rd2_.AddTaskSpace(TASK_LINK_ROTATION, desired_control_target2.c_str(), Vector3d::Zero(), verbose);
    rd2_.AddTaskSpace(TASK_LINK_ROTATION, desired_control_target3.c_str(), Vector3d::Zero(), verbose);
    // rd2_.AddTaskSpace(TASK_LINK_6D, desired_control_target4.c_str(), Vector3d::Zero(), verbose);
    // rd2_.AddTaskSpace(TASK_CUSTOM, 12);

    rd2_.SetContact(true, true);
    rd2_.CalcContactConstraint();

    fstar_1.segment(0, 3) = rd2_.link_[0].rotm * fstar_1.segment(0, 3);
    fstar_1.segment(3, 3) = rd2_.link_[0].rotm * fstar_1.segment(3, 3);

    MatrixXd J_task3;
    VectorXd f_star3;

    int lh_id = rd2_.getLinkID("L_Wrist2_Link");
    int rh_id = rd2_.getLinkID("R_Wrist2_Link");

    rd2_.link_[lh_id].UpdateJac(rd2_.model_, rd2_.q_system_);
    rd2_.link_[rh_id].UpdateJac(rd2_.model_, rd2_.q_system_);

    // J_task3.setZero(12, rd2_.system_dof_);
    // J_task3.topRows(6) = rd2_.link_[lh_id].jac_;
    // J_task3.bottomRows(6) = rd2_.link_[rh_id].jac_;

    // f_star3.setZero(12);
    // f_star3.segment(0, 6) = 0.2 * fstar_1;
    // f_star3.segment(6, 6) = 0.2 * fstar_1;

    rd2_.SetTaskSpace(0, fstar_1);
    rd2_.SetTaskSpace(1, fstar_1.segment(3, 3));
    rd2_.SetTaskSpace(2, fstar_1.segment(3, 3));
    // rd2_.SetTaskSpace(3, 0.5 * fstar_1);

    rd2_.CalcGravCompensation();
    rd2_.CalcTaskControlTorque(true, use_hqp);
    rd2_.CalcContactRedistribute(true);

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
        rd2_.CalcTaskControlTorque(false, use_hqp);
    }
    auto t12 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < repeat; i++)
    {
        rd2_.CalcContactRedistribute(false);
    }
    auto t13 = std::chrono::high_resolution_clock::now();

    // std::cout << " j task of ts[0] " << std::endl;
    // std::cout << rd2_.ts_[0].J_task_ << std::endl;

    // std::cout << rd_.torque_grav_.transpose() << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;
    // std::cout << "Original MODEL ::: task lambda of task 1 : \n";
    // std::cout << rd2_.ts_[0].Lambda_task_ << std::endl;

    std::cout << " QP Fstar modified : " << rd2_.ts_[0].f_star_.transpose() + rd2_.ts_[0].f_star_qp_.transpose() << std::endl;

    // MatrixXd original_J_task = rd2_.ts_[0].J_task_;
    // std::cout << "task jac of original model : " << std::endl;
    // std::cout << rd2_.ts_[0].J_task_ << std::endl;

    // Matrix3d Sample_rotm = Matrix3d::Zero();
    // Sample_rotm << 0, -1, 0,
    //     1, 0, 0,
    //     0, 0, 1;
    // Matrix6d Sample_tr = Matrix6d::Zero();
    // Sample_tr.block(0, 0, 3, 3) = Sample_rotm;
    // Sample_tr.block(3, 3, 3, 3) = Sample_rotm;

    std::cout << "Result Task Torque : " << std::endl;

    // std::cout << rd2_.torque_grav_.transpose() << std::endl;

    std::cout << rd2_.torque_task_.transpose() << std::endl;

    VectorXd original_torque_task = rd2_.torque_task_;

    std::cout << "Result grav Torque : " << std::endl;
    std::cout << rd2_.torque_grav_.transpose() << std::endl;

    std::cout << "contact torque : " << std::endl;
    std::cout << rd2_.torque_contact_.transpose() << std::endl;
    MatrixXd s_k = MatrixXd::Zero(rd2_.model_dof_, rd2_.model_dof_ + 6);
    s_k.block(0, 6, rd2_.model_dof_, rd2_.model_dof_) = MatrixXd::Identity(rd2_.model_dof_, rd2_.model_dof_);
    MatrixXd give_me_fstar = rd2_.ts_[0].J_task_ * rd2_.A_inv_ * rd2_.N_C * s_k.transpose();

    std::cout << "fstar : " << (give_me_fstar * rd2_.torque_task_).transpose() << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;

    double time_original_us = std::chrono::duration_cast<std::chrono::microseconds>(t13 - t10).count();
    double time_original1_us = std::chrono::duration_cast<std::chrono::microseconds>(t11 - t10).count();
    double time_original2_us = std::chrono::duration_cast<std::chrono::microseconds>(t12 - t11).count();
    double time_original3_us = std::chrono::duration_cast<std::chrono::microseconds>(t13 - t12).count();

    std::cout << "Original Dynamics Model  TOTAL CONSUMPTION : " << (int)(time_original_us / repeat) << " us" << std::endl;
    std::cout << "Original Dynamics Model 1 - Contact Constraint Calculation : " << (int)(time_original1_us / repeat) << " us" << std::endl;
    std::cout << "Original Dynamics Model 2 - Task Torque Calculation        : " << (int)(time_original2_us / repeat) << " us" << std::endl;
    std::cout << "Original Dynamics Model 3 - Contact Redistribution Calcula : " << (int)(time_original3_us / repeat) << " us" << std::endl;
    // std::cout << "rotm : " << std::endl;
    // std::cout << rd2_.link_[0].rotm << std::endl;

    // std::cout << "-----------------------------------------------------------------" << std::endl;
    // rd2_.DeleteLink("Waist1_Link", verbose);

    int lfoot = rd2_.getLinkID("l_ankleroll_link");
    int rfoot = rd2_.getLinkID("r_ankleroll_link");
    int pelvis = rd2_.getLinkID("pelvis_link");

    // std::cout << rd2_.ts_[0].torque_h_.transpose() << std::endl;
    // std::cout << rd2_.ts_[1].torque_h_.transpose() << std::endl;

    // std::cout << (rd2_.ts_[2].Lambda_task_) << std::endl;
    // std::cout << (rd2_.ts_[2].torque_h_).transpose() << std::endl;

    // VectorXd q3 = VectorXd::Zero(rd2_.model_.q_size);
    // VectorXd q3dot = VectorXd::Zero(rd2_.model_.qdot_size);
    // VectorXd q3ddot = VectorXd::Zero(rd2_.model_.qdot_size);

    // q3 << 0, 0, 0.92983, 0, 0, 0,
    //     0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
    //     0.0, 0.0, -0.24, 0.6, -0.36, 0.0;
    // rd2_.UpdateKinematics(q3, q3dot, q3ddot);
    // rd2_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    // rd2_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    // rd2_.SetContact(true, true);

    // jac of lfoot :
    // std::cout <<"lfoot jac : \n";
    // std::cout << -skew(rd2_.link_[lfoot].xipos- rd2_.link_[pelvis].xpos) << std::endl;
    // std::cout << rd2_.link_[lfoot].jac_com_ << std::endl;
    // std::cout <<"rfoot jac : \n";
    // std::cout << -skew(rd2_.link_[rfoot].xipos - rd2_.link_[pelvis].xpos) << std::endl;
    // std::cout << rd2_.link_[rfoot].jac_com_ << std::endl;
    // std::cout <<"pelvis jac : \n";
    // std::cout << -skew(rd2_.link_[pelvis].xipos - rd2_.link_[pelvis].xpos) << std::endl;
    // std::cout << rd2_.link_[pelvis].jac_com_ << std::endl;

    /*


    SET 1 configuration


    */
    // rd2_.SetTaskSpace(2, rd2_.link_[0].rotm.transpose() * fstar_1.segment(3, 3));

    // rd2_.SetTaskSpace(2, Vector3d::Zero());

    rd2_.SetContact(true, true);
    rd2_.ReducedDynamicsCalculate();
    rd2_.ReducedCalcContactConstraint();
    rd2_.ReducedCalcGravCompensation();
    rd2_.ReducedCalcTaskControlTorque(true, use_hqp);
    rd2_.ReducedCalcContactRedistribute(true);

    // std::cout << " j i nc  " << std::endl;
    // std::cout << rd2_.J_I_nc_ << std::endl;

    // start time
    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < repeat; i++)
    {
        rd2_.SetContact(true, true);
        rd2_.ReducedDynamicsCalculate();
    }
    auto t1 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < repeat; i++)
    {
        rd2_.ReducedCalcContactConstraint();
        rd2_.ReducedCalcGravCompensation();
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < repeat; i++)
    {
        rd2_.ReducedCalcTaskControlTorque(false, use_hqp);
    }
    auto t3 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < repeat; i++)
    {
        rd2_.ReducedCalcContactRedistribute(false);
    }
    auto t4 = std::chrono::high_resolution_clock::now();

    // std::cout << rd2_.ts_[0].torque_h_R_.transpose() << std::endl;
    // std::cout << rd2_.ts_[1].torque_h_R_.transpose() << std::endl;
    // std::cout << (rd2_.ts_[2].torque_h_R_).transpose() << std::endl;
    // std::cout << "AR " << std::endl;
    // std::cout << rd2_.A_R << std::endl;

    // MatrixXd J_task_;
    // J_task_.setZero(6, 24);
    // J_task_.block(0, 0, 6, 18) = original_J_task.leftCols(rd2_.vc_dof);
    // J_task_.block(0, 18, 6, 6) = rd2_.J_I_nc_inv_T * original_J_task.rightCols(21);

    // T = J' F
    // Tr = Jr' F

    // Matrix6d IM_ALL = InertiaMatrix(rd2_.link_.back().inertia, rd2_.link_.back().mass);

    // Vector3d local_com_nc_from_com = rd2_.com_pos_nc_ - rd2_.link_[0].rotm.transpose() * (rd2_.link_.back().xpos - rd2_.link_[0].xpos);

    // Matrix6d tr3;
    // Matrix3d rt_p = rd2_.link_[0].rotm.transpose();

    // tr3.block(0, 0, 3, 3) = rt_p;
    // tr3.block(3, 3, 3, 3) = rt_p;
    // tr3.block(0, 3, 3, 3) = skew(local_com_nc_from_com).transpose() * rt_p;

    // if (desired_control_target == "COM")
    // {
    //     J_task_.block(0, 18, 6, 6) = IM_ALL.inverse() * tr3.transpose() * rd2_.SI_nc_l_; // * tr3 * tr3.transpose();
    // }
    // else if (desired_control_target == "pelvis_link")
    // {
    //     // J_task_.setZero(6, 24);
    //     // J_task_.block(0, 0, 6, 18) = original_J_task.block(0, 0, 6, 18);
    // }

    // x_task_d = J_task * qdot
    // x_r_task_d = J_r_task * qdot_r
    // x_r_task_d is desired task velocity of reduced model, which the projection of original desired task velocity to reduced model

    // std::cout << "J_task original : " << std::endl;
    // std::cout << original_J_task << std::endl;
    // std::cout << "J_task reduced : " << std::endl;
    // std::cout << J_task_ << std::endl;
    // std::cout << "J_task reduced 2 : " << std::endl;
    // std::cout << original_J_task.rightCols(21) * widepsd << std::endl;
    // std::cout << "J_task reduced 2 : " << std::endl;
    // std::cout << original_J_task * widepsd2 << std::endl;
    // std::cout << "J_task reduced 3 : " << std::endl;
    // std::cout << original_J_task * JRINV_T.transpose() << std::endl;
    // std::cout << "J task non-contact : " << std::endl;
    // std::cout << rd2_.jac_inertial_nc_ << std::endl;

    // ReducedCalcTaskControlTorque
    // rd2_.SetTaskSpace(0, fstar_1, J_)
    // rd2_.ts_[0].CalcJKT_R(rd2_.A_R_inv, rd2_.N_CR, rd2_.W_R_inv, rd2_.J_I_nc_inv_T);
    // rd2_.torque_task_R_ = rd2_.ts_[0].J_kt_R_ * rd2_.ts_[0].Lambda_task_R_ * (rd2_.ts_[0].f_star_);
    // CalcSingleTaskTorqueWithQP_R()
    // std::cout << "JtaskR"<<std::endl;
    // std::cout << rd2_.ts_[0].J_task_R_ << std::endl;

    // MatrixXd J_kt_;
    // J_kt_.setZero(6, rd2_.reduced_model_dof_);
    // MatrixXd Lambda_task_;
    // Lambda_task_.setZero(6, 6);

    // CalculateJKT(J_task_, rd2_.A_R_inv, rd2_.N_CR, rd2_.W_R_inv, J_kt_, Lambda_task_);
    // CalculateJKT(J_task_, segA_inv, N_C, W_inv, J_kt_, Lambda_task_);

    // std::cout << rd2_.J_R.transpose() * rd2_.J_R_INV_T << std::endl;

    VectorXd torque_recalc = VectorXd::Zero(33);
    // torque_recalc.segment(0, 18) = rd2_.ts_[0].J_kt_R_ * rd2_.ts_[0].Lambda_task_R_ * fstar_1;

    // std::cout << rd2_.torque_task_R_.transpose() << std::endl;

    torque_recalc.segment(0, 18) = rd2_.torque_task_R_;

    // std::cout << "\n\n"
    //           << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "RESULT WITH Reduced Dynamics Model : \n";

    // std::cout << "J_temp2 " << std::endl;
    // std::cout << J_temp_2.block(0, 0, 3, 24) << std::endl;

    // std::cout << " link v " << std::endl;
    // std::cout << rd2_.link_v_.back().jac_com_ << std::endl;

    // std::cout << "J_task : \n";
    // std::cout << J_task_ << std::endl;

    // std::cout << "Aex : \n";
    // std::cout << segA << std::endl;

    // std::cout << "A_inv : \n";
    // std::cout << A_inv_2 << std::endl;
    // std::cout << "Jac contact : \n";
    // std::cout << J_C << std::endl;
    // std::cout << "N_C : \n"
    //           << N_C << std::endl;

    // std::cout << "Lambda task : \n";
    // std::cout << Lambda_task_ << std::endl;

    // std::cout << (J_kt_ * Lambda_task_ * fstar_1).transpose() << std::endl;
    // std::cout << (jin_nc.transpose() * torque_recalc.segment(12, 6)).transpose() << std::endl;

    // Vector6d fnc = torque_recalc.segment(12, 6);

    // torque_recalc.segment(12, 21) = rd2_.J_I_nc_.transpose() * fnc;
    std::cout << "task torque : \n";
    std::cout << rd2_.torque_task_.transpose() << std::endl;

    std::cout << "grav torque : " << std::endl;
    std::cout << rd2_.torque_grav_.transpose() << std::endl;

    std::cout << "contact torque : " << std::endl;
    std::cout << rd2_.torque_contact_.transpose() << std::endl;

    std::cout << "fstar 2 :";
    std::cout << (give_me_fstar * rd2_.torque_task_).transpose() << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;
    double time_reduced_us = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t0).count();
    double time_reduced1_us = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    double time_reduced2_us = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
    double time_reduced3_us = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count();
    std::cout << "Reduced Dynamics Model  TOTAL CONSUMPTION : " << (int)(time_reduced_us / repeat) << " us" << std::endl;
    std::cout << "Reduced Dynamics Model 1 - Reduced Dynamics Calculation   : " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / repeat << " us" << std::endl;
    std::cout << "Reduced Dynamics Model 2 - Contact Constraint Calculation : " << (int)(time_reduced1_us / repeat) << " us (" << (time_reduced1_us / time_original1_us) * 100 << "%)" << std::endl;
    std::cout << "Reduced Dynamics Model 3 - Task Torque Calculation        : " << (int)(time_reduced2_us / repeat) << " us (" << (time_reduced2_us / time_original2_us) * 100 << "%)" << std::endl;
    std::cout << "Reduced Dynamics Model 4 - Contact Redistribution Calcula : " << (int)(time_reduced3_us / repeat) << " us (" << (time_reduced3_us / time_original3_us) * 100 << "%)" << std::endl;

    // std::cout << "A_R : " << std::endl;

    std::cout << "-----------------------------------------------------------------" << std::endl;

    // Original vs reduced
    std::cout << "Original time vs Reduced time : " << (time_reduced_us / time_original_us) * 100 << "%" << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;

    // std::cout << rd2_.ts_[0].torque_h_R_.transpose() << std::endl;
    // std::cout << (rd2_.ts_[0].Null_task_R_ * rd2_.ts_[1].torque_h_R_).transpose() << std::endl;
    // std::cout << (rd2_.ts_[1].Null_task_R_ * rd2_.ts_[2].torque_h_R_).transpose() << std::endl;

    // std::cout << rd2_.ts_[0].J_task_ << std::endl;
    // std::cout << -skew(rd2_.link_.back().xpos - rd2_.link_[0].xpos) << std::endl;

    // MatrixXd S_K = MatrixXd::Zero(rd2_.model_dof_, rd2_.model_dof_ + 6);
    // S_K.block(0, 6, rd2_.model_dof_, rd2_.model_dof_) = MatrixXd::Identity(rd2_.model_dof_, rd2_.model_dof_);

    // std::cout << rd2_.ts_[0].J_kt_R_ * rd2_.ts_[0].Lambda_task_ * rd2_.ts_[0].f_star_ << std::endl;

    // std::cout << "Lambda task : \n";
    // std::cout << rd2_.ts_[0].Lambda_task_ << std::endl;

    // std::cout << "Lambda task R : \n";
    // std::cout << rd2_.ts_[0].Lambda_task_R_ << std::endl;
    // std::cout << J_R?

    // std::cout << " J_R_INV_T : \n";
    // std::cout << rd2_.J_R_INV_T << std::endl;

    // std::cout << "J_I_nc_inv_T : " << std::endl;
    // std::cout << rd2_.J_I_nc_inv_T << std::endl;

    //     Lambda_contact = (J_C * A_inv_ * J_C.transpose()).inverse();
    // J_C_INV_T = Lambda_contact * J_C * A_inv_;
    // N_C = MatrixXd::Identity(system_dof_, system_dof_) - J_C.transpose() * J_C_INV_T;
    // A_inv_N_C = A_inv_ * N_C;

    // std::cout << "J_C_INV_T comparison : " << std::endl;
    // std::cout << rd2_.J_C_INV_T << std::endl;
    // std::cout << "J_R comparison : " << std::endl;
    // std::cout << rd2_.J_R.transpose() << std::endl;
    // std::cout << "J_CR_INV_T comparison : " << std::endl;
    // std::cout << rd2_.J_CR_INV_T << std::endl;

    // std::cout << "N_C comparison : " << std::endl;
    // std::cout << rd2_.N_C << std::endl;
    // std::cout << "N_C comparison : " << std::endl;
    // std::cout << rd2_.N_CR << std::endl;

    // std::cout << "N_C comparison : " << std::endl;
    // std::cout << rd2_.A_inv_N_C << std::endl;
    // std::cout << "N_C comparison : " << std::endl;
    // std::cout << rd2_.J_R_INV_T.transpose() * rd2_.A_R_inv_N_CR * rd2_.J_R_INV_T << std::endl;

    // MatrixXd J_NC_ = MatrixXd::Zero(6, rd2_.system_dof_);

    // J_NC_.block(0, 0, 6, 6) = MatrixXd::Identity(6, 6);
    // J_NC_.block(0, 3, 3, 3) = -skew(rd2_.com_pos_nc_);
    // J_NC_.block(0, rd2_.vc_dof, 6, rd2_.nc_dof) = rd2_.J_I_nc_;

    // std::cout << rd2_.com_pos_nc_.transpose() << std::endl << std::endl;
    // std::cout << J_NC_ << std::endl << std::endl;
    // std::cout << (J_NC_ * rd2_.A_inv_ * J_NC_.transpose()).inverse() << std::endl << std::endl;

    // std::cout << (rd2_.J_I_nc_ * rd2_.A_inv_.block(rd2_.vc_dof, rd2_.vc_dof, rd2_.nc_dof, rd2_.nc_dof) * rd2_.J_I_nc_.transpose()).inverse() << std::endl << std::endl;

    // std::cout << (rd2_.A_R_inv.rightCols(6).bottomRows(6)).inverse() << std::endl;

    // std::cout << "lambda task : " <<std::endl;
    // std::cout << rd2_.ts_[0].Lambda_task_ << std::endl;
    // std::cout << "lambda task R : " <<std::endl;
    // std::cout << rd2_.ts_[0].Lambda_task_R_ << std::endl;

    // std::cout << "lambda task 3 : " <<std::endl;
    // std::cout << rd2_.ts_[3].Lambda_task_ << std::endl;
    // std::cout << "lambda task 3 R : " <<std::endl;
    // std::cout << rd2_.ts_[3].Lambda_task_R_ << std::endl;

    return 0;
}
