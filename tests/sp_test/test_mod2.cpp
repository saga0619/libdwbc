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
    double rot_z = M_PI_2;

    RobotData rd2_;

    std::string urdf2_file = std::string(URDF_DIR) + "/dyros_tocabi.urdf";

    // std::string desired_control_target = "pelvis_link";
    std::string desired_control_target = "COM";

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
    rd2_.SetTorqueLimit(tlim);
    // verbose = true;

    Vector3d euler_rotation(0, 0, rot_z);

    // get quaternion from euler angle in radian, euler_rotation
    Eigen::Quaterniond qu = Eigen::AngleAxisd(euler_rotation[0], Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(euler_rotation[1], Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(euler_rotation[2], Eigen::Vector3d::UnitZ());

    // Eigen::Quaterniond q2_()

    std::cout << "quaternion : " << qu.w() << " " << qu.x() << " " << qu.y() << " " << qu.z() << std::endl;

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
    // rd2_.AddTaskSpace(TASK_LINK_ROTATION, desired_control_target.c_str(), Vector3d::Zero(), verbose);

    MatrixXd origin_A = rd2_.A_;

    MatrixXd origin_A_66 = rd2_.A_.block(0, 0, 6, 6);

    rd2_.SetContact(true, true);
    rd2_.CalcContactConstraint();

    fstar_1.segment(0, 3) = rd2_.link_[0].rotm * fstar_1.segment(0, 3);
    fstar_1.segment(3, 3) = rd2_.link_[0].rotm * fstar_1.segment(3, 3);

    rd2_.SetTaskSpace(0, fstar_1);

    rd2_.CalcGravCompensation();
    rd2_.CalcTaskControlTorque(true);
    rd2_.CalcContactRedistribute(true);
    // std::cout << rd_.torque_grav_.transpose() << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;
    // std::cout << "Original MODEL ::: task lambda of task 1 : \n";
    // std::cout << rd2_.ts_[0].Lambda_task_ << std::endl;

    std::cout << " QP Fstar modified : " << rd2_.ts_[0].f_star_.transpose() + rd2_.ts_[0].f_star_qp_.transpose() << std::endl;

    MatrixXd original_J_task = rd2_.ts_[0].J_task_;
    // std::cout << "task jac of original model : " << std::endl;
    // std::cout << rd2_.ts_[0].J_task_ << std::endl;

    Matrix3d Sample_rotm = Matrix3d::Zero();
    Sample_rotm << 0, -1, 0,
        1, 0, 0,
        0, 0, 1;
    Matrix6d Sample_tr = Matrix6d::Zero();
    Sample_tr.block(0, 0, 3, 3) = Sample_rotm;
    Sample_tr.block(3, 3, 3, 3) = Sample_rotm;

    std::cout << "Result Task Torque : " << std::endl;

    // std::cout << rd2_.torque_grav_.transpose() << std::endl;

    std::cout << rd2_.torque_task_.transpose() << std::endl;

    std::cout << "Result grav Torque : " << std::endl;
    std::cout << rd2_.torque_grav_.transpose() << std::endl;

    MatrixXd s_k = MatrixXd::Zero(rd2_.model_dof_, rd2_.model_dof_ + 6);
    s_k.block(0, 6, rd2_.model_dof_, rd2_.model_dof_) = MatrixXd::Identity(rd2_.model_dof_, rd2_.model_dof_);
    MatrixXd give_me_fstar = rd2_.ts_[0].J_task_ * rd2_.A_inv_ * rd2_.N_C * s_k.transpose();

    std::cout << "fstar : " << (give_me_fstar * rd2_.torque_task_).transpose() << std::endl;
    // std::cout << "rotm : " << std::endl;
    // std::cout << rd2_.link_[0].rotm << std::endl;

    // std::cout << "-----------------------------------------------------------------" << std::endl;
    // rd2_.DeleteLink("Waist1_Link", verbose);

    int lfoot = rd2_.getLinkID("l_ankleroll_link");
    int rfoot = rd2_.getLinkID("r_ankleroll_link");
    int pelvis = rd2_.getLinkID("pelvis_link");

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

    rd2_.ReducedDynamicsCalculate();
    rd2_.ReducedCalcContactConstraint();
    rd2_.ReducedCalcGravCompensation();

    // std::cout << "AR " << std::endl;
    // std::cout << rd2_.A_R << std::endl;
  

    MatrixXd J_task_;
    J_task_.setZero(6, 24);
    J_task_.block(0, 0, 6, 18) = original_J_task.leftCols(rd2_.vc_dof);
    // J_task_.block(0, 18, 6, 6) = rd2_.J_I_nc_inv_T * original_J_task.rightCols(21);

    // T = J' F
    // Tr = Jr' F

    Matrix6d IM_ALL = InertiaMatrix(rd2_.link_.back().inertia, rd2_.link_.back().mass);

    Vector3d local_com_nc_from_com = rd2_.com_pos_nc_ - rd2_.link_[0].rotm.transpose() * (rd2_.link_.back().xpos - rd2_.link_[0].xpos);

    Matrix6d tr3;
    Matrix3d rt_p = rd2_.link_[0].rotm.transpose();

    tr3.block(0, 0, 3, 3) = rt_p;
    tr3.block(3, 3, 3, 3) = rt_p;
    tr3.block(0, 3, 3, 3) = skew(local_com_nc_from_com).transpose() * rt_p;

    if (desired_control_target == "COM")
    {
        J_task_.block(0, 18, 6, 6) = IM_ALL.inverse() * tr3.transpose() * rd2_.SI_nc_l_; // * tr3 * tr3.transpose();
    }
    else if (desired_control_target == "pelvis_link")
    {
        // J_task_.setZero(6, 24);
        // J_task_.block(0, 0, 6, 18) = original_J_task.block(0, 0, 6, 18);
    }

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

    MatrixXd J_kt_;
    J_kt_.setZero(6, rd2_.reduced_model_dof_);
    MatrixXd Lambda_task_;
    Lambda_task_.setZero(6, 6);

    CalculateJKT(J_task_, rd2_.A_R_inv, rd2_.N_CR, rd2_.W_R_inv, J_kt_, Lambda_task_);
    // CalculateJKT(J_task_, segA_inv, N_C, W_inv, J_kt_, Lambda_task_);

    VectorXd torque_recalc = VectorXd::Zero(33);
    torque_recalc.segment(0, 18) = J_kt_ * Lambda_task_ * fstar_1;

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

    std::cout << "task torque : \n";
    // std::cout << (J_kt_ * Lambda_task_ * fstar_1).transpose() << std::endl;
    // std::cout << (jin_nc.transpose() * torque_recalc.segment(12, 6)).transpose() << std::endl;

    Vector6d fnc = torque_recalc.segment(12, 6);

    torque_recalc.segment(12, 21) = rd2_.J_I_nc_.transpose() * fnc;
    std::cout << torque_recalc.transpose() << std::endl;

    std::cout << "fstar 2 :";
    std::cout << (give_me_fstar * torque_recalc).transpose() << std::endl;

    std::cout << "grav torque : " << std::endl;
    std::cout << rd2_.torque_grav_.transpose() << std::endl;
    // std::cout << "A_R : " << std::endl;


    // std::cout << " J_R_INV_T : \n";
    // std::cout << rd2_.J_R_INV_T << std::endl;

    // std::cout << "J_I_nc_inv_T : " << std::endl;
    // std::cout << rd2_.J_I_nc_inv_T << std::endl;

    return 0;
}
