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
    double rot_z = 0;
    std::string desired_control_target = "COM";
    std::string desired_control_target2 = "pelvis_link";
    std::string desired_control_target3 = "upperbody_link";
    std::string desired_control_target4 = "L_Wrist2_Link";
    std::string desired_control_target5 = "R_Wrist2_Link";
    std::string urdf2_file = std::string(URDF_DIR) + "/dyros_tocabi.urdf";
    VectorXd fstar_1;
    fstar_1.setZero(6);
    fstar_1 << 0.11, 0.5, 0.13, 0.12, -0.11, 0.05; // based on local frmae.

    RobotData rd2_;
    rd2_.LoadModelData(urdf2_file, true, false);

    VectorXd t2lim;
    VectorXd q2 = VectorXd::Zero(rd2_.model_.q_size);
    VectorXd q2dot = VectorXd::Zero(rd2_.model_.qdot_size);
    VectorXd q2ddot = VectorXd::Zero(rd2_.model_.qdot_size);

    VectorXd tlim;
    tlim.setConstant(rd2_.model_dof_, 500);
    // rd2_.SetTorqueLimit(tlim);
    // verbose = true;

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

    bool verbose = false;
    rd2_.UpdateKinematics(q2, q2dot, q2ddot);
    rd2_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    rd2_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);

    // rd2_.AddTaskSpace(0, TASK_LINK_6D, desired_control_target4.c_str(), Vector3d::Zero(), verbose);

    rd2_.AddTaskSpace(0, TASK_CUSTOM, 6);
    rd2_.AddTaskSpace(1, TASK_LINK_6D, desired_control_target4.c_str(), Vector3d::Zero(), true);

    Vector6d fstar;
    fstar << -2.525447768358226e-04, 0.009351865104458, 0.003798001886399, 0.032987134922079, 0.001145820825286, 0.009015168380664;
    std::cout << "Torque 0 : " << std::endl;
    VectorXd f_star1, f_star2, f_star3;
    f_star1 = 0.5 * fstar_1;
    f_star2 = 0.2 * fstar_1;
    f_star3.setZero(12);
    f_star3.segment(0, 6) = f_star1;
    f_star3.segment(6, 6) = f_star2;

    rd2_.ReducedDynamicsCalculate();

    MatrixXd j_task_nc;
    j_task_nc.setZero(6, rd2_.system_dof_);
    j_task_nc.block(0, 0, 6, 6).setIdentity();
    j_task_nc.block(0, 3, 3, 3) = -skew(rd2_.com_pos_nc_);
    j_task_nc.rightCols(rd2_.nc_dof) = rd2_.J_I_nc_;

    // int lh_id = rd2_.getLinkID(desired_control_target5);

    // rd2_.link_[lh_id].UpdateJac(rd2_.model_, rd2_.q_system_);

    // j_task_nc = rd2_.link_[lh_id].jac_;

    rd2_.SetTaskSpace(0, fstar, j_task_nc);
    rd2_.SetTaskSpace(1, f_star2);
    // rd2_.SetTaskSpace(1, f_star2);

    rd2_.SetContact(true, true);
    rd2_.CalcContactConstraint();
    rd2_.CalcGravCompensation();
    rd2_.CalcTaskControlTorque(true, false);
    rd2_.CalcContactRedistribute(true);

    std::cout << "Torque 0 : " << std::endl;
    std::cout << rd2_.ts_[0].torque_h_.transpose() << std::endl;

    std::cout << "Torque 1 : " << std::endl;
    std::cout << rd2_.ts_[1].torque_h_.transpose() << std::endl;

    std::cout << "null Torque 1 : " << std::endl;
    std::cout << (rd2_.ts_[0].Null_task_ * rd2_.ts_[1].torque_h_).transpose() << std::endl;

    // std::cout << "Torque Right : " << std::endl;
    // std::cout << rd2_.ts_[1].torque_h_.transpose() << std::endl;

    // std::cout << "Torque Total : " << std::endl;
    // std::cout << rd2_.ts_[0].torque_h_.transpose() + rd2_.ts_[1].torque_h_.transpose() << std::endl;
    // std::cout << "1::::" << std::endl;
    // std::cout << rd2_.J_R_INV_T * rd2_.ts_[1].J_task_.transpose() * rd2_.ts_[1].Lambda_task_ * rd2_.ts_[1].f_star_ << std::endl;
    // std::cout << "2::::" << std::endl;

    // make pseudoInverse rd2_.J_R_INV_T * rd2_.ts_[0].J_task_.transpose() * rd2_.ts_[0].Lambda_task_ * rd2_.ts_[0].f_star_

    MatrixXd oriJ = rd2_.J_R_INV_T * rd2_.ts_[1].J_task_.transpose() * rd2_.ts_[1].Lambda_task_ * rd2_.ts_[1].f_star_;
    MatrixXd pinvJ = (rd2_.J_R_INV_T * rd2_.ts_[0].J_task_.transpose() * rd2_.ts_[0].Lambda_task_).completeOrthogonalDecomposition().pseudoInverse();

    // R

    std::cout << oriJ.transpose() << std::endl;

    std::cout << (rd2_.J_R_INV_T * rd2_.ts_[0].J_task_.transpose() * rd2_.ts_[0].Lambda_task_ * pinvJ * oriJ).transpose() << std::endl;

    // make Vector6d fstar with [-2.525447768358226e-04;0.009351865104458;0.003798001886399;0.032987134922079;0.001145820825286;0.009015168380664]

    // Resultant force on the ub com from the left hand

    Vector6d force = rd2_.ts_[1].Lambda_task_ * rd2_.ts_[1].f_star_;

    std::cout << "Force on the left hand : " << force.transpose() << std::endl;

    Vector3d hand_pos = rd2_.link_[rd2_.getLinkID(desired_control_target4)].xpos;

    Matrix6d xtg = Matrix6d::Identity();
    xtg.block(3, 0, 3, 3) = skew(hand_pos - rd2_.com_pos_nc_);

    Vector6d force_on_ub = xtg * force;

    rd2_.SetTaskSpace(0, force_on_ub, j_task_nc);
    rd2_.SetTaskSpace(1, f_star2);
    // rd2_.SetTaskSpace(1, f_star2);

    rd2_.SetContact(true, true);
    rd2_.CalcContactConstraint();
    rd2_.CalcGravCompensation();
    rd2_.CalcTaskControlTorque(true, false);
    rd2_.CalcContactRedistribute(true);

    std::cout << "Torque 0 : " << std::endl;
    std::cout << rd2_.ts_[0].J_kt_ * force_on_ub << std::endl;
}
