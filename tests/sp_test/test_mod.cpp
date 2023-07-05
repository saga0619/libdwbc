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
    bool verbose = false;
    int repeat_time = 1000;
    RobotData rd_;
    std::string resource_path = URDF_DIR;
    std::string urdf_name = "/dyros_tocabi.urdf";
    std::string urdf_path = resource_path + urdf_name;

    rd_.LoadModelData(urdf_path, true, false);
    VectorXd q = VectorXd::Zero(rd_.system_dof_ + 1);
    VectorXd qdot = VectorXd::Zero(rd_.system_dof_);
    VectorXd qddot = VectorXd::Zero(rd_.system_dof_);

    VectorXd fstar_1;
    fstar_1.setZero(6);
    fstar_1 << 0.1, 4.0, 0.1, 0.1, -0.1, 0.1;

    VectorXd tlim;
    tlim.setConstant(rd_.model_dof_, 500);

    rd_.SetTorqueLimit(tlim);
    q << 0, 0, 0.92983, 0, 0, 0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0, 0, 0,
        0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;

    rd_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    rd_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);

    rd_.AddTaskSpace(TASK_LINK_6D, "pelvis_link", Vector3d::Zero(), verbose);
    rd_.AddTaskSpace(TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero(), verbose);

    rd_.UpdateKinematics(q, qdot, qddot);
    rd_.SetContact(true, true);

    rd_.SetTaskSpace(0, fstar_1);
    rd_.SetTaskSpace(1, fstar_1.segment(3, 3));

    rd_.CalcGravCompensation();
    rd_.CalcTaskControlTorque(true);
    rd_.CalcContactRedistribute(true);
    std::cout << rd_.torque_grav_.transpose() << std::endl;

    // repeat test and measure time
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < repeat_time; i++)
    {
        rd_.UpdateKinematics(q, qdot, qddot);
        rd_.SetContact(true, true);

        rd_.SetTaskSpace(0, fstar_1);
        rd_.SetTaskSpace(1, fstar_1.segment(3, 3));

        rd_.CalcGravCompensation();
        rd_.CalcTaskControlTorque(false);
        rd_.CalcContactRedistribute(false);
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;

    double original_calc_us = elapsed.count() * 1000000 / repeat_time;

    std::cout << "Original Calculation elapsed time: " << original_calc_us << "us\n";

    std::vector<Link> vlink;
    std::vector<Joint> vjoint;

    int start_vlink_id = 0;

    // Add link and joint information to vector vlink and vjoint. Also, change the link id and parent id to start from 0.

    for (int i = 0; i < 21; i++)
    {
        vlink.push_back(rd_.link_[13 + i]);
        vjoint.push_back(rd_.joint_[13 + i]);

        if (i == 0)
            start_vlink_id = vlink[i].link_id_;

        vlink[i].link_id_ -= start_vlink_id;
        vlink[i].parent_id_ -= start_vlink_id;

        for (int j = 0; j < vlink[i].child_id_.size(); j++)
        {
            vlink[i].child_id_[j] -= start_vlink_id;
        }

        // print the link id, link name, parent id, child id
        // std::cout << "link id: " << vlink[i].link_id_ << " link name: " << vlink[i].name_ << " parent id: " << vlink[i].parent_id_ << " child id: ";
        // for (int j = 0; j < vlink[i].child_id_.size(); j++)
        // {
        //     std::cout << vlink[i].child_id_[j] << " ";
        // }
        // std::cout << std::endl;
    }

    RigidBodyDynamics::Model v_model;
    v_model.gravity = Vector3d(0, 0, -9.81);

    int parent_id = -1;
    int added_id = 0;
    RigidBodyDynamics::Math::SpatialTransform rbdl_joint_frame;

    int vlink_size = vlink.size();

    // create rbdl model with vlink and vjoint
    for (int i = 0; i < vlink_size; i++)
    {
        rbdl_joint_frame = RigidBodyDynamics::Math::SpatialTransform((RigidBodyDynamics::Math::Matrix3d)vjoint[i].joint_rotation_, vjoint[i].joint_translation_);

        // std::cout << i << " : rbdl joint frame : " << rbdl_joint_frame << std::endl;
        // std::cout << "lambda t : " << vlink[i].parent_rotm << std::endl;
        // std::cout << "lambda r : " << vlink[i].parent_trans.transpose() << std::endl;
        // std::cout << "rotm : " << vlink[i].rotm << std::endl;
        // std::cout << std::endl;

        if (vlink[i].parent_id_ < 0)
        {
            parent_id = 0;
        }
        else
        {
            parent_id = vlink[vlink[i].parent_id_].body_id_;
        }
        added_id = v_model.AddBody(parent_id, rbdl_joint_frame, vjoint[i].ToRBDLJoint(), vlink[i].ToRBDLBody(), vlink[i].name_);

        vlink[i].body_id_ = added_id;
    }

    Eigen::MatrixXd A_v_;

    VectorXd qv, qdotv, qddotv;

    // qv = q.segment(18, 21);
    qv.setZero(21);
    qv << 0, 0, 0,
        0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0;
    qdotv = qdot.segment(18, 21);
    qddotv = qddot.segment(18, 21);

    A_v_.setZero(v_model.qdot_size, v_model.qdot_size);

    // extracted link has body_id inside.
    //

    //

    rd_.UpdateVModel(v_model, qv, qdotv, qddotv, vlink, vjoint);

    Vector3d com_pos;
    Matrix3d com_inertia;
    double com_mass;

    rd_.CalcVirtualInertia(v_model, vlink, vjoint, com_inertia, com_pos, com_mass);

    rd_.DeleteLink("Waist1_Link", verbose);

    // Add virtual linkage to the original model, with 6dof joint
    Link virtual_link_ = vlink[2];
    Joint virtual_joint_ = vjoint[2];

    virtual_link_.parent_id_ = rd_.getLinkID("pelvis_link");
    virtual_joint_.joint_type_ = JOINT_6DOF;
    virtual_joint_.joint_rotation_.setIdentity();
    virtual_joint_.joint_translation_.setZero();

    // int origin_size = rd_.model_.mBodies.size();
    // std::cout << virtual_link_.mass << std::endl;
    // std::cout << com_mass << std::endl;
    // virtual_link_.mass = 30.5;
    // virtual_link_.com_position_l_ = com_pos;
    // virtual_link_.inertia = com_inertia;

    rd_.AddLink(virtual_joint_, virtual_link_, verbose); // UpperBody

    rd_.ChangeLinkInertia("upperbody_link", com_inertia, com_mass, verbose);

    rd_.ClearContactConstraint();
    rd_.ClearTaskSpace();

    tlim.resize(rd_.model_dof_);
    tlim.setConstant(rd_.model_dof_, 500);

    rd_.SetTorqueLimit(tlim);
    // q << 0, 0, 0.92983, 0, 0, 0,
    //     0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
    //     0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
    //     0, 0, 0,
    //     0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
    //     0, 0,
    //     -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;
    rd_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    rd_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    rd_.AddTaskSpace(TASK_LINK_6D, "pelvis_link", Vector3d::Zero(), verbose);
    // rd_.AddTaskSpace(TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero(), verbose);

    VectorXd q2;
    VectorXd qdot2;
    VectorXd qddot2;
    q2.setZero(rd_.model_.q_size);
    qdot2.setZero(rd_.model_.qdot_size);
    qddot2.setZero(rd_.model_.qdot_size);
    q2 << 0, 0, 0.92983, 0, 0, 0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0, 0, 0,
        0, 0, 0, 1;

    std::cout << com_pos.transpose() << std::endl;

    q2.segment(18, 3) = com_pos;

    // rd_.ChangeLinkInertia("upperbody_link", com_inertia, com_mass, verbose);
    // rd_.AddTaskSpace(TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero(), verbose);

    std::cout << com_pos.transpose() << std::endl;
    std::cout << com_mass << std::endl;

    rd_.UpdateKinematics(q2, qdot2, qddot2);
    rd_.SetContact(true, true);

    rd_.SetTaskSpace(0, fstar_1);
    // rd_.SetTaskSpace(1, fstar_1.segment(3, 3));

    rd_.CalcGravCompensation();
    rd_.CalcTaskControlTorque(true);
    rd_.CalcContactRedistribute(true);

    std::cout << rd_.torque_grav_.transpose() << std::endl;

    // repeat test and measure time
    start = std::chrono::high_resolution_clock::now();

    // repeat_time = 10;

    for (int i = 0; i < repeat_time; i++)
    {
        rd_.UpdateVModel(v_model, qv, qdotv, qddotv, vlink, vjoint);
        rd_.CalcVirtualInertia(v_model, vlink, vjoint, com_inertia, com_pos, com_mass);
        // rd_.ChangeLinkInertia("upperbody_link", com_inertia, com_mass, true);
        // rd_.AddTaskSpace(TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero(), verbose);

        // q2.segment(18, 3) = com_pos;

        rd_.UpdateKinematics(q2, qdot2, qddot2);
        rd_.SetContact(true, true);

        rd_.SetTaskSpace(0, fstar_1);
        // rd_.SetTaskSpace(1, fstar_1.segment(3, 3));

        rd_.CalcGravCompensation();
        rd_.CalcTaskControlTorque(false);
        rd_.CalcContactRedistribute(false);
    }
    end = std::chrono::high_resolution_clock::now();
    elapsed = end - start;

    std::cout << rd_.torque_grav_.transpose() << std::endl;
    double calc_time = elapsed.count() * 1000000 / repeat_time;

    std::cout << "elapsed time: " << calc_time << "us\n";

    std::cout << "compuatation performance improvement : " << 100 - calc_time / original_calc_us * 100 << "%\n";
}

// int main(void)
// {
//     RobotData rd_;
//     std::string resource_path = URDF_DIR;
//     std::string urdf_name = "/dyros_tocabi.urdf";
//     std::string urdf_path = resource_path + urdf_name;
//     rd_.LoadModelData(urdf_path, true, true);
//     VectorXd q;
//     VectorXd qdot;
//     VectorXd qddot;
//     q.setZero(rd_.model_.q_size);
//     qdot.setZero(rd_.model_.qdot_size);
//     qddot.setZero(rd_.model_.qdot_size);

//     q << 0, 0, 0.92983, 0, 0, 0,
//         0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
//         0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
//         0, 0, 0,
//         0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
//         0, 0,
//         -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;

//     rd_.UpdateKinematics(q, qdot, qddot);

//     rd_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
//     rd_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
//     rd_.AddContactConstraint("l_wrist2_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);
//     rd_.AddContactConstraint("r_wrist2_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);

//     rd_.AddTaskSpace(TASK_LINK_6D, "pelvis_link", Vector3d::Zero());
//     rd_.AddTaskSpace(TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero());

//     // delete head

//     VectorXd fstar_1;
//     fstar_1.setZero(6);
//     fstar_1(0) = 0.1;
//     fstar_1(1) = 4.0;
//     fstar_1(2) = 0.1;

//     fstar_1(3) = 0.1;
//     fstar_1(4) = -0.1;
//     fstar_1(5) = 0.1;

//     bool init = true;

//     VectorXd tlim;
//     tlim.setConstant(rd_.model_dof_, 300);

//     rd_.SetTorqueLimit(tlim);

//     bool verbose = false;

//     RobotData rd2_;
//     rd2_.LoadModelData(urdf_path, true, 0);

//     rd2_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
//     rd2_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
//     // rd2_.AddContactConstraint("l_wrist2_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04, verbose);
//     // rd2_.AddContactConstraint("r_wrist2_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04, verbose);

//     rd2_.AddTaskSpace(TASK_LINK_6D, "pelvis_link", Vector3d::Zero(), verbose);
//     rd2_.AddTaskSpace(TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero(), verbose);

//     rd2_.UpdateKinematics(q, qdot, qddot);
//     // rd_.save_mat_file_ = true;

//     rd2_.SetContact(true, true);

//     rd2_.SetTaskSpace(0, fstar_1);

//     rd2_.SetTaskSpace(1, fstar_1.segment(3, 3));

//     rd2_.CalcGravCompensation();

//     rd2_.CalcTaskControlTorque(true);

//     rd2_.CalcContactRedistribute(true);

//     std::cout << "-------------------- Original Torque Calculation ---------------------" << std::endl;

//     std::cout << "grav torque : " << std::endl;
//     std::cout << rd2_.torque_grav_.transpose() << std::endl;
//     std::cout << "task torque : " << std::endl;
//     std::cout << rd2_.torque_task_.transpose() << std::endl;
//     std::cout << "contact torque : " << std::endl;
//     std::cout << rd2_.torque_contact_.transpose() << std::endl;
//     // rd2_.printLinkInfo();

//     // std::cout << "G : " << std::endl;
//     // std::cout << rd2_.G_.transpose().segment(6, rd2_.model_dof_) << std::endl;

//     // int link_id = rd2_.getLinkID("R_Wrist1_Link");
//     // int body_id = rd2_.link_[link_id].body_id_;

//     // std::cout << "X_T : "<< rd2_.model_.X_T[body_id]<<std::endl;
//     // std::cout << "X_lambda : "<< rd2_.model_.X_lambda[body_id]<<std::endl;

//     std::vector<Link> Link_RArm;
//     std::vector<Joint> joint_RArm;

//     for (int i = 13; i <= 33; i++)
//     {
//         Link_RArm.push_back(rd2_.link_[i]);
//         joint_RArm.push_back(rd2_.joint_[i]);
//     }

//     verbose = true;

//     std::cout << "qsize : " << rd2_.model_.q_size << std::endl;
//     rd2_.DeleteLink("Waist1_Link", verbose);

//     // for(int i=0; i<Link_RArm.size(); i++)
//     // {
//     //     rd2_.AddLink(joint_RArm[i], Link_RArm[i], verbose);
//     // }

//     std::cout << "qsize : " << rd2_.model_.q_size << std::endl;

//     Link_RArm[2].parent_id_ = rd2_.getLinkID("pelvis_link");
//     joint_RArm[2].joint_type_ = JOINT_6DOF;
//     rd2_.AddLink(joint_RArm[2], Link_RArm[2], verbose); // UpperBody

//     std::cout << "qsize : " << rd2_.model_.q_size << std::endl;

//     std::cout << "mdof : " << rd2_.model_.mJoints.back().mDoFCount << std::endl;
//     // int link_idx = rd2_.getLinkID("head_link");
//     // Link link = rd2_.link_[link_idx];

//     // Joint joint_1 = rd2_.joint_[link_idx];

//     // rd2_.DeleteLink("head_link", verbose);

//     // Vector3d joint_axis = Vector3d::Zero();
//     // joint_axis(1) = -1.0;

//     // rd2_.AddLink(joint_1, link, verbose);

//     // rd2_.AddLink()

//     // rd2_.ChangeLinkToFixedJoint("head_link", verbose);
//     // rd2_.ChangeLinkToFixedJoint("neck_link", verbose);

//     // rd2_.ChangeLinkToFixedJoint("R_Wrist2_Link", verbose);
//     // rd2_.ChangeLinkToFixedJoint("L_Wrist2_Link", verbose);

//     // rd2_.ChangeLinkToFixedJoint("R_Wrist1_Link", verbose);
//     // rd2_.ChangeLinkToFixedJoint("L_Wrist1_Link", verbose);
//     verbose = false;

//     RigidBodyDynamics::Model v_model_;

//     // v_model_ = rd2_.model_;

//     // rd2_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
//     // rd2_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
//     // rd2_.AddContactConstraint("l_wrist2_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04, verbose);
//     // rd2_.AddContactConstraint("r_wrist2_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04, verbose);

//     // rd2_.AddTaskSpace(TASK_LINK_6D, "pelvis_link", Vector3d::Zero(), verbose);
//     rd2_.AddTaskSpace(TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero(), verbose);

//     VectorXd q2;
//     VectorXd qdot2;
//     VectorXd qddot2;
//     q2.setZero(rd2_.model_.q_size);
//     qdot2.setZero(rd2_.model_.qdot_size);
//     qddot2.setZero(rd2_.model_.qdot_size);

//     q2 << 0, 0, 0.92983, 0, 0, 0,
//         0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
//         0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
//         0, 0, 0,
//         0, 0, 0, 1;

//     // 0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
//     // 0,
//     // -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 0, 1;

//     rd2_.UpdateKinematics(q2, qdot2, qddot2);

//     // rd2_.printLinkInfo();
//     std::cout << "UpdateKinematics complete" << std::endl;

//     // rd2_.printLinkInfo();

//     rd2_.SetContact(true, true);
//     std::cout << "Set Contact Complete" << std::endl;

//     rd2_.SetTaskSpace(0, fstar_1);

//     rd2_.SetTaskSpace(1, fstar_1.segment(3, 3));

//     rd2_.CalcGravCompensation();

//     rd2_.CalcTaskControlTorque(true);

//     rd2_.CalcContactRedistribute(true);

//     std::cout << "-------------------- Modified Model Torque Calculation ---------------------" << std::endl;

//     std::cout << "grav torque : " << std::endl;
//     std::cout << rd2_.torque_grav_.transpose() << std::endl;
//     std::cout << "task torque : " << std::endl;
//     std::cout << rd2_.torque_task_.transpose() << std::endl;
//     std::cout << "contact torque : " << std::endl;
//     std::cout << rd2_.torque_contact_.transpose() << std::endl;

//     RobotData rd3_;

//     std::string urdf_name2 = "/dyros_tocabi_wo_head.urdf";
//     std::string urdf_path2 = resource_path + urdf_name2;

//     rd3_.LoadModelData(urdf_path2, true, false);
//     rd3_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
//     rd3_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
//     rd3_.AddContactConstraint("l_wrist2_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04, verbose);
//     rd3_.AddContactConstraint("r_wrist2_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04, verbose);

//     rd3_.AddTaskSpace(TASK_LINK_6D, "pelvis_link", Vector3d::Zero(), verbose);
//     rd3_.AddTaskSpace(TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero(), verbose);

//     q2.setZero(rd3_.model_.q_size);
//     qdot2.setZero(rd3_.model_.qdot_size);
//     qddot2.setZero(rd3_.model_.qdot_size);

//     q2 << 0, 0, 0.92983, 0, 0, 0,
//         0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
//         0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
//         0, 0, 0,
//         0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
//         0,
//         -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;

//     rd3_.UpdateKinematics(q2, qdot2, qddot2);
//     // rd_.save_mat_file_ = true;

//     rd3_.SetContact(true, true);

//     rd3_.SetTaskSpace(0, fstar_1);

//     rd3_.SetTaskSpace(1, fstar_1.segment(3, 3));

//     rd3_.CalcGravCompensation();

//     rd3_.CalcTaskControlTorque(true);

//     rd3_.CalcContactRedistribute(true);

//     std::cout << "-------------------- Modified URDF Torque Calculation ---------------------" << std::endl;

//     std::cout << "grav torque : " << std::endl;
//     std::cout << rd3_.torque_grav_.transpose() << std::endl;
//     std::cout << "task torque : " << std::endl;
//     std::cout << rd3_.torque_task_.transpose() << std::endl;
//     std::cout << "contact torque : " << std::endl;
//     std::cout << rd3_.torque_contact_.transpose() << std::endl;

//     // rd3_.printLinkInfo();

//     // Extract link and joint information and construct virtual model from it.
// }
