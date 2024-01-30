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

    RobotData rd2_;

    std::string urdf2_file = std::string(URDF_DIR) + "/dyros_tocabi_ub.urdf";

    // std::string desired_control_target = "pelvis_link";
    std::string desired_control_target = "COM";

    VectorXd fstar_1;
    fstar_1.setZero(6);
    fstar_1 << 0.1, 0.5, 0.1, 0.1, -0.1, 0.1; // based on local frmae.

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
    std::cout << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;

    std::cout << "quaternion : " << qu.w() << " " << qu.x() << " " << qu.y() << " " << qu.z() << std::endl;

    q2 << 0, 0, 0, qu.x(), qu.y(), qu.z(),
        0, 0, 0,
        0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, qu.w();

    rd2_.UpdateKinematics(q2, q2dot, q2ddot);

    // rd2_.J_com_

    std::cout << " A_nc inv : " << std::endl;
    std::cout << rd2_.A_ << std::endl;

    std::cout << "Jac nc : " << std::endl;
    std::cout << rd2_.link_.back().jac_ << std::endl;

    std::cout << "Upperbody com : " << std::endl;
    std::cout << rd2_.link_.back().xpos.transpose() << std::endl;

    // q2 << 0, 0, 0.92983, 0, 0, 0,
    //     0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
    //     0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
    //     0, 0, 0,
    //     0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
    //     0, 0,
    //     -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;

    // bool verbose = false;
    // rd2_.UpdateKinematics(q2, q2dot, q2ddot);
    // rd2_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    // rd2_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);

    // rd2_.AddTaskSpace(0, TASK_LINK_6D, desired_control_target.c_str(), Vector3d::Zero(), verbose);
    // // rd2_.AddTaskSpace(TASK_LINK_ROTATION, desired_control_target.c_str(), Vector3d::Zero(), verbose);

    // MatrixXd origin_A = rd2_.A_;

    // MatrixXd origin_A_66 = rd2_.A_.block(0, 0, 6, 6);

    // rd2_.SetContact(true, true);

    // fstar_1.segment(0, 3) = rd2_.link_[0].rotm * fstar_1.segment(0, 3);
    // fstar_1.segment(3, 3) = rd2_.link_[0].rotm * fstar_1.segment(3, 3);

    // rd2_.SetTaskSpace(0, fstar_1);

    // rd2_.CalcGravCompensation();
    // rd2_.CalcTaskControlTorque(true);
    // rd2_.CalcContactRedistribute(true);
    // // std::cout << rd_.torque_grav_.transpose() << std::endl;
    // std::cout << "-----------------------------------------------------------------" << std::endl;
    // std::cout << "Original MODEL ::: task lambda of task 1 : \n";
    // std::cout << rd2_.ts_[0].Lambda_task_ << std::endl;

    // std::cout << " QP Fstar modified : " << rd2_.ts_[0].f_star_.transpose() + rd2_.ts_[0].f_star_qp_.transpose() << std::endl;

    // MatrixXd original_J_task = rd2_.ts_[0].J_task_;
    // // std::cout << "task jac of original model : " << std::endl;
    // // std::cout << rd2_.ts_[0].J_task_ << std::endl;

    // Matrix3d Sample_rotm = Matrix3d::Zero();
    // Sample_rotm << 0, -1, 0,
    //     1, 0, 0,
    //     0, 0, 1;
    // Matrix6d Sample_tr = Matrix6d::Zero();
    // Sample_tr.block(0, 0, 3, 3) = Sample_rotm;
    // Sample_tr.block(3, 3, 3, 3) = Sample_rotm;

    // std::cout << "Result Task Torque : " << std::endl;

    // // std::cout << rd2_.torque_grav_.transpose() << std::endl;

    // std::cout << rd2_.torque_task_.transpose() << std::endl;

    // MatrixXd s_k = MatrixXd::Zero(rd2_.model_dof_, rd2_.model_dof_ + 6);
    // s_k.block(0, 6, rd2_.model_dof_, rd2_.model_dof_) = MatrixXd::Identity(rd2_.model_dof_, rd2_.model_dof_);
    // MatrixXd give_me_fstar = rd2_.ts_[0].J_task_ * rd2_.A_inv_ * rd2_.N_C * s_k.transpose();

    // std::cout << "fstar : " << (give_me_fstar * rd2_.torque_task_).transpose() << std::endl;
    // std::cout << "rotm : " << std::endl;
    // std::cout << rd2_.link_[0].rotm << std::endl;

    // // std::cout << "-----------------------------------------------------------------" << std::endl;
    // // rd2_.DeleteLink("Waist1_Link", verbose);

    // int lfoot = rd2_.getLinkID("l_ankleroll_link");
    // int rfoot = rd2_.getLinkID("r_ankleroll_link");
    // int pelvis = rd2_.getLinkID("pelvis_link");

    // VectorXd q3 = VectorXd::Zero(rd2_.model_.q_size);
    // VectorXd q3dot = VectorXd::Zero(rd2_.model_.qdot_size);
    // VectorXd q3ddot = VectorXd::Zero(rd2_.model_.qdot_size);

    // rd2_.ReducedDynamicsCalculate(true);

    // euler_rotation(2) = 0.0;

    // // get quaternion from euler angle in radian, euler_rotation
    // qu = Eigen::AngleAxisd(euler_rotation[0], Eigen::Vector3d::UnitX()) *
    //      Eigen::AngleAxisd(euler_rotation[1], Eigen::Vector3d::UnitY()) *
    //      Eigen::AngleAxisd(euler_rotation[2], Eigen::Vector3d::UnitZ());

    // // Eigen::Quaterniond q2_()

    // std::cout << "quaternion : " << qu.w() << " " << qu.x() << " " << qu.y() << " " << qu.z() << std::endl;

    // q2 << 0, 0, 0.92983, qu.x(), qu.y(), qu.z(),
    //     0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
    //     0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
    //     0, 0, 0,
    //     0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
    //     0, 0,
    //     -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, qu.w();

    // rd2_.UpdateKinematics(q2, q2dot, q2ddot);
    // rd2_.SetContact(true, true);
    // rd2_.ReducedDynamicsCalculate(true);
}