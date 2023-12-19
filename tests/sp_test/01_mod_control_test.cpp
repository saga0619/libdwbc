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

    // Desired Control Target

    std::string desired_control_target = "pelvis_link";
    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "Desired Control Target : " << desired_control_target << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;
    VectorXd fstar_1;
    fstar_1.setZero(6);
    fstar_1 << 0.1, 0.5, 0.1, 0.1, -0.1, 0.1;

    // fstar_1
    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "Desired Task FSTAR : " << fstar_1.transpose() << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;

    rd_.LoadModelData(urdf_path, true, false);

    VectorXd tlim;
    VectorXd q = VectorXd::Zero(rd_.model_.q_size);
    VectorXd qdot = VectorXd::Zero(rd_.model_.qdot_size);
    VectorXd qddot = VectorXd::Zero(rd_.model_.qdot_size);

    // rd_.ChangeLinkToFixedJoint("head_link", verbose);

    tlim.setConstant(rd_.model_dof_, 500);
    rd_.SetTorqueLimit(tlim);
    // verbose = true;
    q << 0, 0, 0.92983, 0, 0, 0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0, 0, 0,
        // 0, 0, 0, 0, 0, 0, 0, 0,
        // 0, 0,
        // 0, 0, 0, 0, 0, 0, 0, 0,
        // 0;
        0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;

    // rd_.UpdateKinematics(q, qdot, qddot);
    // rd_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    // rd_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    // // rd_.AddTaskSpace(TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero(), verbose);
    // rd_.AddTaskSpace(TASK_LINK_6D, "pelvis_link", Vector3d::Zero(), verbose);

    // rd_.SetContact(true, true);
    // rd_.SetTaskSpace(0, fstar_1);
    // rd_.CalcGravCompensation();
    // rd_.CalcTaskControlTorque(true);
    // rd_.CalcContactRedistribute(true);

    // std::cout << rd_.torque_task_.transpose() << std::endl;

    // fstar_1 << 0.1, 2.0, 0.1, 0, 0, 0;

    // int head_id = rd_.getLinkID("head_link");
    // Link temp_link = rd_.link_[head_id];
    // Joint temp_joint = rd_.joint_[head_id];

    // rd_.DeleteLink("head_link", verbose);
    // rd_.ChangeLinkToFixedJoint("head_link");
    // rd_.ChangeLinkInertia("pelvis_link", Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0), verbose);

    rd_.UpdateKinematics(q, qdot, qddot);
    rd_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    rd_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    // rd_.AddTaskSpace(TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero(), verbose);
    rd_.AddTaskSpace(TASK_LINK_6D, desired_control_target.c_str(), Vector3d::Zero(), verbose);

    rd_.SetContact(true, true);

    // std::cout << "A virtual : \n";
    // std::cout << rd_.A_.block(0, 0, 6, 6) << std::endl;

    rd_.SetTaskSpace(0, fstar_1);
    // rd_.SetTaskSpace(1, fstar_1.segment(3, 3));

    rd_.CalcGravCompensation();
    rd_.CalcTaskControlTorque(true);
    rd_.CalcContactRedistribute(true);
    // std::cout << rd_.torque_grav_.transpose() << std::endl;

    std::cout << " QP Fstar modified : " << rd_.ts_[0].f_star_.transpose() + rd_.ts_[0].f_star_qp_.transpose() << std::endl;
    std::cout << "Result Task Torque : " << std::endl;

    std::cout << rd_.torque_grav_.transpose() << std::endl;

    std::cout << rd_.torque_task_.transpose() << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;
    // std::cout << rd_.torque_contact_.transpose() << std::endl;

    MatrixXd s_k = MatrixXd::Zero(rd_.model_dof_, rd_.model_dof_ + 6);
    s_k.block(0, 6, rd_.model_dof_, rd_.model_dof_) = MatrixXd::Identity(rd_.model_dof_, rd_.model_dof_);

    MatrixXd give_me_fstar = rd_.ts_[0].J_task_ * rd_.A_inv_ * rd_.N_C * s_k.transpose();

    // std::cout << rd_.ts_[0].J_task_ << std::endl;

    // std::cout << rd_.ts_[0].Lambda_task_ << std::endl;

    // std::cout << rd_.A_.block(0, 0, 6, 6) << std::endl;
    // std::cout << rd_.J_C << std::endl;
    // std::cout << rd_.N_C << std::endl;
    std::cout << "fstar original recalc : " << (give_me_fstar * rd_.torque_task_).transpose() << std::endl;

    std::cout << "lambda matrix of original model " << std::endl;

    

    // return 0;

    // repeat test and measure time
    auto start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < repeat_time; i++)
    {
        rd_.UpdateKinematics(q, qdot, qddot);
        rd_.SetContact(true, true);

        rd_.SetTaskSpace(0, fstar_1);
        // rd_.SetTaskSpace(1, fstar_1.segment(3, 3));

        rd_.CalcGravCompensation();
        rd_.CalcTaskControlTorque(false);
        rd_.CalcContactRedistribute(false);
    }
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;

    int model_dof = rd_.model_dof_;

    VectorXd torque_vt = VectorXd::Zero(model_dof + 6);
    torque_vt.segment(6, model_dof) = rd_.torque_task_;
    s_k.block(0, 6, model_dof, model_dof).setIdentity();

    VectorXd fstar_result = rd_.ts_[0].J_task_ * rd_.A_inv_ * rd_.N_C * s_k.transpose() * rd_.torque_task_;
    // std::cout <<""
    // std::cout << "fstar_origin : " << rd_.ts_[0].f_star_.transpose() << std::endl;
    // std::cout << "fstar qp : " << rd_.ts_[0].f_star_qp_.transpose() << std::endl;
    // std::cout << "fstar_result : " << fstar_result.transpose() << std::endl;
    // std::cout << "lfstar : " << (rd_.ts_[0].Lambda_task_ * (rd_.ts_[0].f_star_ + rd_.ts_[0].f_star_qp_)).transpose() << std::endl;
    // std::cout << "my torque : " << (rd_.ts_[0].J_kt_ * rd_.ts_[0].Lambda_task_ * (rd_.ts_[0].f_star_ + rd_.ts_[0].f_star_qp_)).transpose() << std::endl;
    // std::cout << "lambda : \n"
    //           << rd_.ts_[0].Lambda_task_ << std::endl;
    // std::cout << "total mass : " << rd_.total_mass_ << std::endl;
    // std::cout << "com position : " << rd_.link_.back().xpos.transpose() << std::endl;

    double mass1 = rd_.total_mass_;
    Vector3d com_p1 = rd_.link_.back().xpos;

    double original_calc_us = elapsed.count() * 1000000 / repeat_time;

    std::cout << "Original Calculation elapsed time: " << original_calc_us << "us\n";

    std::cout << "-----------------------------------------------------------------" << std::endl
              << std::endl
              << std::endl;

    std::cout << "Model modified calculation result : " << std::endl;

    // std::cout << "j  ::" << std::endl;
    // std::cout << rd_.ts_[0].J_task_ * rd_.A_inv_ << std::endl;
    // // std::cout << temp_A_ << std::endl;
    // std::cout << " ------------------" << std::endl;


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

    VectorXd qv, qdotv, qddotv;

    // qv = q.segment(18, 21);
    qv.setZero(21);
    qv << 0, 0, 0,
        0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0;
    qdotv = qdot.segment(18, 21);
    qddotv = qddot.segment(18, 21);

    // extracted link has body_id inside.
    //

    MatrixXd A_virtual_;

    rd_.UpdateVModel(v_model, qv, qdotv, qddotv, A_virtual_, vlink, vjoint);

    Vector3d com_pos;
    Matrix3d com_inertia;
    double com_mass;

    rd_.CalcVirtualInertia(v_model, vlink, vjoint, com_inertia, com_pos, com_mass);

    Matrix3d v_com_inertia;
    v_com_inertia.setZero();

    for (int i = 0; i < vlink.size(); i++)
    {
        Vector3d tcom = com_pos - (vlink[i].xpos + vlink[i].rotm * vlink[i].com_position_l_);
        v_com_inertia += vlink[i].rotm * vlink[i].inertia * vlink[i].rotm.transpose() + vlink[i].mass * (skew(tcom) * skew(tcom).transpose());
    }

    // std::cout

    com_inertia = v_com_inertia;

    // rd_.CalcVJac(v_model, vlink, vjoint);

    rd_.DeleteLink("Waist1_Link", verbose);

    // Add virtual linkage to the original model, with 6dof joint
    Link virtual_link_ = vlink[2];
    Joint virtual_joint_ = vjoint[2];

    virtual_link_.parent_id_ = rd_.getLinkID("pelvis_link");
    virtual_joint_.joint_type_ = JOINT_6DOF;
    virtual_joint_.joint_rotation_.setIdentity();
    virtual_joint_.joint_translation_.setZero();

    RigidBodyDynamics::Model v_model2;
    // v_model2.AddBody(0, rbdl_joint_frame2, virtual_joint_.ToRBDLJoint(), virtual_link_.ToRBDLBody(), "body");

    RigidBodyDynamics::Math::Vector3d com_pos_rbdl = Vector3d::Zero();
    RigidBodyDynamics::Math::Matrix3d inertia_rbdl = com_inertia;

    RigidBodyDynamics::Body Body(com_mass, com_pos_rbdl, inertia_rbdl);

    RigidBodyDynamics::Joint joint = RigidBodyDynamics::Joint(
        RigidBodyDynamics::Math::SpatialVector(0., 0., 0., 1., 0., 0.),
        RigidBodyDynamics::Math::SpatialVector(0., 0., 0., 0., 1., 0.),
        RigidBodyDynamics::Math::SpatialVector(0., 0., 0., 0., 0., 1.),
        RigidBodyDynamics::Math::SpatialVector(1., 0., 0., 0., 0., 0.),
        RigidBodyDynamics::Math::SpatialVector(0., 1., 0., 0., 0., 0.),
        RigidBodyDynamics::Math::SpatialVector(0., 0., 1., 0., 0., 0.));

    // RigidBodyDynamics::Math::Matrix3d joint_rotm_rbdl = joint_rotm.transpose();
    RigidBodyDynamics::Math::SpatialTransform rbdl_joint_frame2 = RigidBodyDynamics::Math::SpatialTransform((RigidBodyDynamics::Math::Matrix3d)virtual_joint_.joint_rotation_, virtual_joint_.joint_translation_);

    v_model2.AddBody(0, rbdl_joint_frame2, joint, Body, "base");

    // RigidBodyDynamics::

    VectorXd q_v2(6);
    VectorXd qdot_v2(6);
    VectorXd qddot_v2(6);

    q_v2.setZero();
    qdot_v2.setZero();
    qddot_v2.setZero();

    q_v2.segment(0, 3) = com_pos;

    std::cout << " virtual 6dof joint link model test " << std::endl;

    std::cout << " model dof : " << v_model2.dof_count << "q dof : " << v_model2.q_size << std::endl;

    std::cout << "joint with : " << q_v2.transpose() << std::endl;

    RigidBodyDynamics::UpdateKinematicsCustom(v_model2, &q_v2, &qdot_v2, &qddot_v2);

    // Get body name of v_model2

    std::vector<std::string> body_name;
    body_name.resize(v_model2.mBodies.size());

    for (int i = 0; i < v_model2.mBodies.size(); i++)
    {
        body_name[i] = v_model2.GetBodyName(i);
        // std::cout << i << " : " << body_name[i] << std::endl;
    }

    // Get body id with "base"
    int sb_id = v_model2.GetBodyId("base");
    // std::cout << sb_id << std::endl;

    // Get body position, mass, inertia of added_id

    MatrixXd A_v2;
    A_v2.setZero(6, 6);
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(v_model2, q_v2, A_v2, false);

    std::cout << "Mass matrix : " << std::endl
              << A_v2 << std::endl;

    Vector3d xpos_v2;
    xpos_v2 = RigidBodyDynamics::CalcBodyToBaseCoordinates(v_model2, q_v2, sb_id, Vector3d::Zero(), true);

    MatrixXd jac_com;
    jac_com.resize(6, 6);

    RigidBodyDynamics::CalcPointJacobian6D(v_model2, q_v2, sb_id, Eigen::Vector3d::Zero(), jac_com, false);

    jac_com.topRows(3).swap(jac_com.bottomRows(3));

    std::cout << "jac_com : \n" << jac_com << std::endl;

    // std::cout << "lambda : "<< jac_com.inverse() << std::endl;

    MatrixXd lambda_v2;

    lambda_v2 = (jac_com * A_v2.inverse() * jac_com.transpose()).inverse();

    std::cout << "labmda_mat v2 : " << std::endl
              << lambda_v2 << std::endl;






    //////////////////////////////////////
    /* URDF UPPER BODY DATA : */

    std::cout << "URDF UPPER BODY DATA : " << std::endl;

    RigidBodyDynamics::Model v_model3;

    std::string urdf_name_ub = "/dyros_tocabi_ub.urdf";
    std::string urdf_path2 = resource_path + urdf_name_ub;

    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_path2.c_str(), &v_model3, false, false);

    VectorXd q_v3, qdot_v3, qddot_v3;

    q_v3.setZero(v_model3.q_size);
    qdot_v3.setZero(v_model3.qdot_size);
    qddot_v3.setZero(v_model3.qdot_size);

    q_v3 = qv.segment(6, v_model3.q_size);

    std::cout << " model dof : " << v_model3.dof_count << "  q dof : " << v_model3.q_size << std::endl;

    RigidBodyDynamics::UpdateKinematicsCustom(v_model3, &q_v3, &qdot_v3, &qddot_v3);
    MatrixXd A_v3;

    A_v3.setZero(v_model3.dof_count, v_model3.dof_count);
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(v_model3, q_v3, A_v3, false);

    std::cout << "Mass matrix : " << std::endl
              << A_v3 << std::endl;

    // // Calculate All jacobian and accumulate

    // MatrixXd jac_com_ub;
    // jac_com_ub.resize(6, v_model3.dof_count);

    // RigidBodyDynamics::CalcPointJacobian6D(v_model3, q_v3, 0, Vector3d::Zero(), jac_com_ub, false);






    ////





    // int origin_size = rd_.model_.mBodies.size();
    // std::cout << virtual_link_.mass << std::endl;
    // std::cout << com_mass << std::endl;
    // virtual_link_.mass = 30.5;
    // virtual_link_.com_position_l_ = com_pos;
    // virtual_link_.inertia = com_inertia;

    rd_.AddLink(virtual_joint_, virtual_link_, verbose); // UpperBody

    Vector3d zero = Vector3d::Zero();

    rd_.ChangeLinkInertia("upperbody_link", com_inertia, zero, com_mass, verbose);

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
    rd_.AddTaskSpace(TASK_LINK_6D, desired_control_target.c_str(), Vector3d::Zero(), verbose);
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

    // std::cout << com_pos.transpose() << std::endl;

    q2.segment(18, 3) = com_pos;

    // rd_.ChangeLinkInertia("upperbody_link", com_inertia, com_mass, verbose);
    // rd_.AddTaskSpace(TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero(), verbose);
    // std::cout << com_pos.transpose() << std::endl;
    // std::cout << com_mass << std::endl;

    rd_.UpdateKinematics(q2, qdot2, qddot2);
    rd_.SetContact(true, true);

    // std::cout << "A virtual : \n";
    // std::cout << rd_.A_.block(0, 0, 6, 6) << std::endl;

    rd_.SetTaskSpace(0, fstar_1);
    // rd_.SetTaskSpace(1, fstar_1.segment(3, 3));

    rd_.CalcGravCompensation();
    rd_.CalcTaskControlTorque(true);
    rd_.CalcContactRedistribute(true);

    // std::cout << rd_.torque_grav_.transpose() << std::endl;

    // repeat test and measure time
    start = std::chrono::high_resolution_clock::now();

    // repeat_time = 10;
    MatrixXd v_cmm;
    MatrixXd v_jac_r;
    MatrixXd v_jac_c;
    // MatrixXd A_virtual_;
    for (int i = 0; i < repeat_time; i++)
    {
        rd_.UpdateVModel(v_model, qv, qdotv, qddotv, A_virtual_, vlink, vjoint); // 17us

        // rd_.SetTaskSpace(1, fstar_1.segment(3, 3));
    }
    end = std::chrono::high_resolution_clock::now();

    elapsed = end - start;

    start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < repeat_time; i++)
    {
        rd_.CalcVirtualInertia(v_model, vlink, vjoint, com_inertia, com_pos, com_mass); // 3.8us

        rd_.CalcVirtualCMM(v_model, vlink, com_pos, v_cmm); // 10.7us
        v_com_inertia.setZero();

        for (int i = 0; i < vlink.size() - 1; i++)
        {

            Vector3d tcom = (vlink[i].xpos + vlink[i].rotm * vlink[i].com_position_l_) - com_pos;

            v_com_inertia += vlink[i].rotm * vlink[i].inertia * vlink[i].rotm.transpose() + vlink[i].mass * (skew(tcom) * skew(tcom).transpose());
        }

        rd_.ChangeLinkInertia("upperbody_link", v_com_inertia, zero, com_mass, false); // 0.07us

        vlink.back().inertia = v_com_inertia;
        vlink.back().jac_com_.bottomRows(3) = v_com_inertia.inverse() * v_cmm; // 0.05us

        rd_.UpdateKinematics(q2, qdot2, qddot2); // 21us
        rd_.SetContact(true, true);              // 22us
        rd_.SetTaskSpace(0, fstar_1);
        rd_.CalcGravCompensation();
        rd_.CalcTaskControlTorque(false); // 24us
    }

    std::cout << " model modified inertia : " << std::endl
              << com_inertia << std::endl;
    std::cout << " paralell axis inertia : " << std::endl
              << v_com_inertia << std::endl;

    // virtual lambda calculation :

    MatrixXd jac_com_virtual;
    MatrixXd lambda_virtual;

    jac_com_virtual = vlink.back().jac_com_;

    lambda_virtual = (jac_com_virtual * A_virtual_.inverse() * jac_com_virtual.transpose()).inverse();
    std::cout << "lambda virtual : \n";
    std::cout << lambda_virtual << std::endl;

    end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed2 = end - start;

    start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < repeat_time; i++)
    {

        rd_.CalcContactRedistribute(false); // 11us
    }
    end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed3 = end - start;
    VectorXd tg_;
    VectorXd tt_;

    tg_.setZero(33);
    tg_.segment(0, 12) = rd_.torque_grav_.segment(0, 12);
    tg_.segment(12, 21) = vlink.back().jac_com_.transpose() * rd_.torque_grav_.segment(12, 6);

    tt_.setZero(33);
    tt_.segment(0, 12) = rd_.torque_task_.segment(0, 12);
    tt_.segment(12, 21) = vlink.back().jac_com_.transpose() * rd_.torque_task_.segment(12, 6);
    // std::cout << "j  ::" << std::endl;
    // std::cout << rd_.ts_[0].J_task_ * rd_.A_inv_ << std::endl;
    // std::cout << " ------------------" << std::endl;

    // VectorXd ttv_ = VectorXd::Zero(24);
    // ttv_.segment(6, 18) = rd_.torque_task_;
    // VectorXd fstar_result2 = rd_.ts_[0].J_task_ * rd_.A_inv_ * rd_.N_C * ttv_;

    // std::cout << tg_.transpose() << std::endl;
    std::cout << tt_.transpose() << std::endl;
    // std::cout << rd_.torque_contact_.transpose() << std::endl;

    std::cout << "des fstar = " << (rd_.ts_[0].f_star_ + rd_.ts_[0].f_star_qp_).transpose() << std::endl;
    std::cout << "res fstar = " << (give_me_fstar * tt_).transpose() << std::endl;

    int link_uid = rd_.getLinkID("upperbody_link");
    std::cout << "upperbody jac : "<< std::endl;
    std::cout << rd_.link_[link_uid].jac_com_ << std::endl;
    std::cout << "up lambda :"<< std::endl;
    std::cout << (rd_.link_[link_uid].jac_com_ * rd_.A_inv_ * rd_.link_[link_uid].jac_com_.transpose()).inverse() << std::endl;

    std::cout << "rd_ j task : "<<std::endl;
    std::cout << rd_.ts_[0].J_task_ << std::endl;
    std::cout << "lambda : \n"
              << rd_.ts_[0].Lambda_task_ << std::endl;

    if (mass1 != rd_.total_mass_)
    {
        std::cout << "mass error : " << mass1 - rd_.total_mass_ << std::endl;
    }
    if (com_p1 != rd_.link_.back().xpos)
    {
        std::cout << "com error : " << com_p1 - rd_.link_.back().xpos << std::endl;
    }

    // std::cout << rd_.ts_[0].Lambda_task_ << std::endl;

    // std::cout << rd_.A_.block(0, 0, 6, 6) << std::endl;

    // std::cout << rd_.J_C << std::endl;
    // std::cout << rd_.N_C << std::endl;
    // std::cout << "total mass : " << rd_.total_mass_ << std::endl;
    // std::cout << "com position : " << rd_.link_.back().xpos.transpose() << std::endl;
    // rd_.torque_task_.
    double calc_time = elapsed.count() * 1000000 / repeat_time;
    double calc_time2 = elapsed2.count() * 1000000 / repeat_time;
    double calc_time3 = elapsed3.count() * 1000000 / repeat_time;

    std::cout << "elapsed time: " << calc_time << "us + " << calc_time2 << "us + " << calc_time3 << "us = " << calc_time + calc_time2 + calc_time3 << "us \n";

    std::cout << "compuatation performance improvement : " << original_calc_us / (calc_time + calc_time2 + calc_time3) * 100 << "%\n";

    // RobotData rd2_;
    // urdf_name = "/dyros_tocabi_no_waist.urdf";
    // urdf_path = resource_path + urdf_name;
    // rd2_.LoadModelData(urdf_path, true, false);

    // std::cout << "\n\nURDF MODIFIED " << rd2_.model_dof_ << "\n";
    // VectorXd q3, qdot3, qddot3, tlim3;

    // q3 = VectorXd::Zero(rd2_.model_.q_size);
    // qdot3 = VectorXd::Zero(rd2_.model_.qdot_size);
    // qddot3 = VectorXd::Zero(rd2_.model_.qdot_size);
    // tlim3.setConstant(rd2_.model_dof_, 500);
    // fstar_1.setZero(6);
    // fstar_1 << 0.1, 0.5, 0.1, 0.1, -0.1, 0.1;

    // q3 << 0, 0, 0.92983, 0, 0, 0,
    //     0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
    //     0.0, 0.0, -0.24, 0.6, -0.36, 0.0;

    // rd2_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    // rd2_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    // // rd_.AddTaskSpace(TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero(), verbose);
    // rd2_.AddTaskSpace(TASK_LINK_6D, "COM", Vector3d::Zero(), verbose);
    // rd2_.UpdateKinematics(q3, qdot3, qddot3);
    // rd2_.SetContact(true, true);
    // rd2_.SetTaskSpace(0, fstar_1);
    // rd2_.CalcGravCompensation();
    // rd2_.CalcTaskControlTorque(true);
    // rd2_.CalcContactRedistribute(true);

    // // cout decimal point to 10
    // std::cout << rd2_.torque_task_.transpose() << std::endl;
    // VectorXd torque_result;
    // torque_result.setZero(33);
    // torque_result.segment(0, rd2_.model_dof_) = rd2_.torque_task_;
    // // std::cout << give_me_fstar << std::endl;
    // // std::cout << torque_result.transpose() << std::endl;
    // std::cout << "fstar2 : " << (give_me_fstar * torque_result).transpose() << std::endl;
    // std::cout << " com pos : " << rd2_.com_pos.transpose() << std::endl;
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
