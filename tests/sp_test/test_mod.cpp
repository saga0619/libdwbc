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
    RobotData rd_;
    std::string resource_path = URDF_DIR;
    std::string urdf_name = "/dyros_tocabi.urdf";
    std::string urdf_path = resource_path + urdf_name;
    rd_.LoadModelData(urdf_path, true, false);
    VectorXd q;
    VectorXd qdot;
    VectorXd qddot;
    q.setZero(rd_.model_.q_size);
    qdot.setZero(rd_.model_.qdot_size);
    qddot.setZero(rd_.model_.qdot_size);

    q << 0, 0, 0.92983, 0, 0, 0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0, 0, 0,
        0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;

    rd_.UpdateKinematics(q, qdot, qddot);

    rd_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    rd_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    rd_.AddContactConstraint("l_wrist2_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);
    rd_.AddContactConstraint("r_wrist2_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);

    rd_.AddTaskSpace(TASK_LINK_6D, "pelvis_link", Vector3d::Zero());
    rd_.AddTaskSpace(TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero());

    // delete head

    VectorXd fstar_1;
    fstar_1.setZero(6);
    fstar_1(0) = 0.1;
    fstar_1(1) = 4.0;
    fstar_1(2) = 0.1;

    fstar_1(3) = 0.1;
    fstar_1(4) = -0.1;
    fstar_1(5) = 0.1;

    bool init = true;

    VectorXd tlim;
    tlim.setConstant(rd_.model_dof_, 300);

    rd_.SetTorqueLimit(tlim);

    bool verbose = false;

    RobotData rd2_;
    rd2_.LoadModelData(urdf_path, true, 0);

    rd2_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    rd2_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    // rd2_.AddContactConstraint("l_wrist2_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04, verbose);
    // rd2_.AddContactConstraint("r_wrist2_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04, verbose);

    rd2_.AddTaskSpace(TASK_LINK_6D, "pelvis_link", Vector3d::Zero(), verbose);
    rd2_.AddTaskSpace(TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero(), verbose);

    rd2_.UpdateKinematics(q, qdot, qddot);
    // rd_.save_mat_file_ = true;

    rd2_.SetContact(true, true);

    rd2_.SetTaskSpace(0, fstar_1);

    rd2_.SetTaskSpace(1, fstar_1.segment(3, 3));

    rd2_.CalcGravCompensation();

    rd2_.CalcTaskControlTorque(true);

    rd2_.CalcContactRedistribute(true);

    std::cout << "-------------------- Original Torque Calculation ---------------------" << std::endl;

    std::cout << "grav torque : " << std::endl;
    std::cout << rd2_.torque_grav_.transpose() << std::endl;
    std::cout << "task torque : " << std::endl;
    std::cout << rd2_.torque_task_.transpose() << std::endl;
    std::cout << "contact torque : " << std::endl;
    std::cout << rd2_.torque_contact_.transpose() << std::endl;
    // rd2_.printLinkInfo();

    // std::cout << "G : " << std::endl;
    // std::cout << rd2_.G_.transpose().segment(6, rd2_.model_dof_) << std::endl;

    // int link_id = rd2_.getLinkID("R_Wrist1_Link");
    // int body_id = rd2_.link_[link_id].body_id_;

    // std::cout << "X_T : "<< rd2_.model_.X_T[body_id]<<std::endl;
    // std::cout << "X_lambda : "<< rd2_.model_.X_lambda[body_id]<<std::endl;

    int link_idx = rd2_.getLinkID("head_link");
    Link link = rd2_.link_[link_idx];

    verbose = true;

    rd2_.DeleteLink("head_link", verbose);

    Vector3d joint_axis = Vector3d::Zero();
    joint_axis(1) = -1.0;

    rd2_.AddLink(link, JOINT_REVOLUTE, joint_axis, verbose);

    // rd2_.AddLink()

    // rd2_.ChangeLinkToFixedJoint("head_link", verbose);
    // rd2_.ChangeLinkToFixedJoint("neck_link", verbose);

    // rd2_.ChangeLinkToFixedJoint("R_Wrist2_Link", verbose);
    // rd2_.ChangeLinkToFixedJoint("L_Wrist2_Link", verbose);

    // rd2_.ChangeLinkToFixedJoint("R_Wrist1_Link", verbose);
    // rd2_.ChangeLinkToFixedJoint("L_Wrist1_Link", verbose);
    verbose = false;

    RigidBodyDynamics::Model v_model_;

    // v_model_ = rd2_.model_;

    // rd2_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    // rd2_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    // rd2_.AddContactConstraint("l_wrist2_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04, verbose);
    // rd2_.AddContactConstraint("r_wrist2_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04, verbose);

    // rd2_.AddTaskSpace(TASK_LINK_6D, "pelvis_link", Vector3d::Zero(), verbose);
    // rd2_.AddTaskSpace(TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero(), verbose);

    VectorXd q2;
    VectorXd qdot2;
    VectorXd qddot2;
    q2.setZero(rd2_.model_.q_size);
    qdot2.setZero(rd2_.model_.qdot_size);
    qddot2.setZero(rd2_.model_.qdot_size);

    q2 << 0, 0, 0.92983, 0, 0, 0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0, 0, 0,
        0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
        0,
        -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 0, 1;

    rd2_.UpdateKinematics(q2, qdot2, qddot2);

    // rd2_.printLinkInfo();
    std::cout << "UpdateKinematics complete" << std::endl;

    // rd2_.printLinkInfo();

    rd2_.SetContact(true, true);
    std::cout << "Set Contact Complete" << std::endl;

    rd2_.SetTaskSpace(0, fstar_1);

    rd2_.SetTaskSpace(1, fstar_1.segment(3, 3));

    rd2_.CalcGravCompensation();

    rd2_.CalcTaskControlTorque(true);

    rd2_.CalcContactRedistribute(true);





    std::cout << "-------------------- Modified Model Torque Calculation ---------------------" << std::endl;

    std::cout << "grav torque : " << std::endl;
    std::cout << rd2_.torque_grav_.transpose() << std::endl;
    std::cout << "task torque : " << std::endl;
    std::cout << rd2_.torque_task_.transpose() << std::endl;
    std::cout << "contact torque : " << std::endl;
    std::cout << rd2_.torque_contact_.transpose() << std::endl;

    RobotData rd3_;

    std::string urdf_name2 = "/dyros_tocabi_wo_head.urdf";
    std::string urdf_path2 = resource_path + urdf_name2;

    rd3_.LoadModelData(urdf_path2, true, false);
    rd3_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    rd3_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    rd3_.AddContactConstraint("l_wrist2_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04, verbose);
    rd3_.AddContactConstraint("r_wrist2_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04, verbose);

    rd3_.AddTaskSpace(TASK_LINK_6D, "pelvis_link", Vector3d::Zero(), verbose);
    rd3_.AddTaskSpace(TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero(), verbose);

    q2.setZero(rd3_.model_.q_size);
    qdot2.setZero(rd3_.model_.qdot_size);
    qddot2.setZero(rd3_.model_.qdot_size);

    q2 << 0, 0, 0.92983, 0, 0, 0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0, 0, 0,
        0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
        0,
        -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;

    rd3_.UpdateKinematics(q2, qdot2, qddot2);
    // rd_.save_mat_file_ = true;

    rd3_.SetContact(true, true);

    rd3_.SetTaskSpace(0, fstar_1);

    rd3_.SetTaskSpace(1, fstar_1.segment(3, 3));

    rd3_.CalcGravCompensation();

    rd3_.CalcTaskControlTorque(true);

    rd3_.CalcContactRedistribute(true);

    std::cout << "-------------------- Modified URDF Torque Calculation ---------------------" << std::endl;

    std::cout << "grav torque : " << std::endl;
    std::cout << rd3_.torque_grav_.transpose() << std::endl;
    std::cout << "task torque : " << std::endl;
    std::cout << rd3_.torque_task_.transpose() << std::endl;
    std::cout << "contact torque : " << std::endl;
    std::cout << rd3_.torque_contact_.transpose() << std::endl;

    // rd3_.printLinkInfo();

    // Extract link and joint information and construct virtual model from it.
}
