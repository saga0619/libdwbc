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

    q = VectorXd::Zero(rd_.model_.q_size);
    qdot = VectorXd::Zero(rd_.model_.qdot_size);
    qddot = VectorXd::Zero(rd_.model_.qdot_size);
    tlim.setConstant(rd_.model_dof_, 500);

    fstar_1.setZero(6);
    fstar_1 << 0.1, 0.5, 0.1, 0.1, -0.1, 0.1;
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

    // 0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
    // 0, 0,
    // -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;

    rd_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    rd_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    // rd_.AddTaskSpace(TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero(), verbose);
    rd_.AddTaskSpace(TASK_LINK_6D, "pelvis_link", Vector3d::Zero(), verbose);

    rd_.UpdateKinematics(q, qdot, qddot);
    rd_.SetContact(true, true);
    rd_.SetTaskSpace(0, fstar_1);
    rd_.CalcGravCompensation();
    rd_.CalcTaskControlTorque(true);
    rd_.CalcContactRedistribute(true);

    std::cout << rd_.torque_task_.transpose() << std::endl;
    MatrixXd s_k = MatrixXd::Zero(rd_.model_dof_, rd_.model_dof_ + 6);
    s_k.block(0, 6, rd_.model_dof_, rd_.model_dof_) = MatrixXd::Identity(rd_.model_dof_, rd_.model_dof_);
    MatrixXd give_me_fstar = rd_.ts_[0].J_task_ * rd_.A_inv_ * rd_.N_C * s_k.transpose();

    VectorXd torque_result = VectorXd::Zero(rd_.model_dof_);
    torque_result.segment(0, rd_.model_dof_) = rd_.torque_task_;
    std::cout << "fstar : " << (give_me_fstar * torque_result).transpose() << std::endl;
    std::cout << " com pos : " << rd_.com_pos.transpose() << std::endl;

    std::cout.precision(10);

    // Link link;
    // Joint joint;
    // link = rd_.link_[rd_.getLinkID("head_link")];
    // joint = rd_.joint_[rd_.getLinkID("head_link")];
    // rd_.DeleteLink("head_link", verbose);
    // Link link2;
    // link2 = rd_.link_[rd_.getLinkID("neck_link")];
    // link2.AddLink(link, joint.parent_rotation_, joint.parent_translation_);
    // Matrix3d new_inertia;
    // Vector3d new_com;
    // double com_mass;

    // rd_.ChangeLinkInertia("neck_link", link2.inertia, link2.com_position_l_, link2.mass);
    // rd_.ChangeLinkToFixedJoint("head_link");
    // rd_.ChangeLinkToFixedJoint("neck_link");

    rd_.ChangeLinkToFixedJoint("R_Wrist2_Link");
    rd_.ChangeLinkToFixedJoint("R_Wrist1_Link");
    rd_.ChangeLinkToFixedJoint("R_Forearm_Link");
    rd_.ChangeLinkToFixedJoint("R_Elbow_Link");

    rd_.ChangeLinkToFixedJoint("L_Wrist2_Link");
    rd_.ChangeLinkToFixedJoint("L_Wrist1_Link");
    rd_.ChangeLinkToFixedJoint("L_Forearm_Link");
    rd_.ChangeLinkToFixedJoint("L_Elbow_Link");

    // rd_.ChangeLinkToFixedJoint("R_Armlink_Link");
    // rd_.ChangeLinkToFixedJoint("R_Shoulder3_Link");
    // rd_.ChangeLinkToFixedJoint("R_Shoulder2_Link");
    // rd_.ChangeLinkToFixedJoint("R_Shoulder1_Link");

    // rd_.ChangeLinkToFixedJoint("L_Wrist2_Link");
    // rd_.ChangeLinkToFixedJoint("L_Wrist1_Link");
    // rd_.ChangeLinkToFixedJoint("L_Forearm_Link");
    // rd_.ChangeLinkToFixedJoint("L_Elbow_Link");
    // rd_.ChangeLinkToFixedJoint("L_Armlink_Link");
    // rd_.ChangeLinkToFixedJoint("L_Shoulder3_Link");
    // rd_.ChangeLinkToFixedJoint("L_Shoulder2_Link");
    // rd_.ChangeLinkToFixedJoint("L_Shoulder1_Link");
    q.setZero(rd_.system_dof_ + 1);
    qdot.setZero(rd_.system_dof_);
    qddot.setZero(rd_.system_dof_);
    tlim.setConstant(rd_.model_dof_, 500);

    q << 0, 0, 0.92983, 0, 0, 0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0, 0, 0,
        0.3, 0.3, 1.5, -1.27,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 1;

    rd_.SetTorqueLimit(tlim);
    rd_.UpdateKinematics(q, qdot, qddot);
    rd_.SetContact(true, true);
    rd_.SetTaskSpace(0, fstar_1);
    rd_.CalcGravCompensation();
    rd_.CalcTaskControlTorque(true);
    rd_.CalcContactRedistribute(true);

    torque_result.setZero();
    torque_result.segment(0, rd_.model_dof_) = rd_.torque_task_;
    std::cout << "\n\nMODEL MODIFIED dof " << rd_.model_dof_ << "\n";
    std::cout << "rd_.torque_task size : " << rd_.torque_task_.size() << std::endl;
    std::cout << "Torque Answer : \n"
              << rd_.torque_task_.transpose() << std::endl;
    std::cout << "fstar1 : " << (give_me_fstar * torque_result).transpose() << std::endl;
    std::cout << " com pos : " << rd_.com_pos.transpose() << std::endl;
    // int neck_link_id = rd_.getLinkID("neck_link");
    // std::cout << "neck mass : " << rd_.link_[neck_link_id].mass << std::endl;
    // std::cout << "neck com pos : " << rd_.link_[neck_link_id].com_position_l_.transpose() << std::endl;
    // std::cout << "neck inertia :\n " << rd_.link_[neck_link_id].inertia << std::endl;

    RobotData rd2_;
    urdf_name = "/dyros_tocabi_no_right_arm.urdf";
    urdf_path = resource_path + urdf_name;
    rd2_.LoadModelData(urdf_path, true, false);

    std::cout << "\n\nURDF MODIFIED " << rd2_.model_dof_<<"\n";
    VectorXd q2, qdot2, qddot2, tlim2;

    q2 = VectorXd::Zero(rd2_.model_.q_size);
    qdot2 = VectorXd::Zero(rd2_.model_.qdot_size);
    qddot2 = VectorXd::Zero(rd2_.model_.qdot_size);
    tlim2.setConstant(rd2_.model_dof_, 500);
    fstar_1.setZero(6);
    fstar_1 << 0.1, 0.5, 0.1, 0.1, -0.1, 0.1;

    q2 << 0, 0, 0.92983, 0, 0, 0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,

    rd2_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    rd2_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    // rd_.AddTaskSpace(TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero(), verbose);
    rd2_.AddTaskSpace(TASK_LINK_6D, "pelvis_link", Vector3d::Zero(), verbose);
    rd2_.UpdateKinematics(q2, qdot2, qddot2);
    rd2_.SetContact(true, true);
    rd2_.SetTaskSpace(0, fstar_1);
    rd2_.CalcGravCompensation();
    rd2_.CalcTaskControlTorque(true);
    rd2_.CalcContactRedistribute(true);

    // cout decimal point to 10
    std::cout << rd2_.torque_task_.transpose() << std::endl;
    torque_result.setZero(rd_.model_dof_);
    torque_result.segment(0, rd2_.model_.qdot_size) = rd2_.torque_task_;
    std::cout << "fstar2 : " << (give_me_fstar * torque_result).transpose() << std::endl;
    std::cout << " com pos : " << rd2_.com_pos.transpose() << std::endl;

    // neck_link_id = rd2_.getLinkID("neck_link");

    // std::cout << "neck mass : " << rd2_.link_[neck_link_id].mass << std::endl;
    // std::cout << "neck com pos : " << rd2_.link_[neck_link_id].com_position_l_.transpose() << std::endl;
    // std::cout << "neck inertia :\n " << rd2_.link_[neck_link_id].inertia << std::endl;

    // // fstar_1 << 0.1, 2.0, 0.1, 0, 0, 0;
    // int head_id = rd_.getLinkID("head_link");
    // Link temp_link = rd_.link_[head_id];
    // Joint temp_joint = rd_.joint_[head_id];

    // rd_.DeleteLink("head_link", verbose);
    // // rd_.ChangeLinkToFixedJoint("head_link");
    // // rd_.ChangeLinkInertia("pelvis_link", Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0), verbose);

    // tlim.setConstant(rd_.model_.q_size, 500);
    // rd_.SetTorqueLimit(tlim);
    // // verbose = true;
    // q << 0, 0, 0.92983, 0, 0, 0,
    //     0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
    //     0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
    //     0, 0, 0,
    //     0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
    //     0, 0,
    //     -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;

    // rd_.UpdateKinematics(q, qdot, qddot);
    // rd_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    // rd_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    // // rd_.AddTaskSpace(TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero(), verbose);
    // rd_.AddTaskSpace(TASK_LINK_6D, "pelvis_link", Vector3d::Zero(), verbose);

    // rd_.SetContact(true, true);

    // std::cout << "A virtual : \n";
    // std::cout << rd_.A_.block(0, 0, 6, 6) << std::endl;

    // rd_.SetTaskSpace(0, fstar_1);
    // // rd_.SetTaskSpace(1, fstar_1.segment(3, 3));

    // rd_.CalcGravCompensation();
    // rd_.CalcTaskControlTorque(true);
    // rd_.CalcContactRedistribute(true);
    // // std::cout << rd_.torque_grav_.transpose() << std::endl;

    // std::cout << rd_.ts_[0].f_star_.transpose() + rd_.ts_[0].f_star_qp_.transpose() << std::endl;
    // std::cout << rd_.torque_task_.transpose() << std::endl;
    // // std::cout << rd_.torque_contact_.transpose() << std::endl;

    // std::cout << give_me_fstar * rd_.torque_task_ << std::endl;

    return 0;
}
