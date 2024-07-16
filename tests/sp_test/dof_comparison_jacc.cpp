#include <chrono>
#include "dwbc.h"
#include <unistd.h>
#include <iomanip>
#include <iostream>
#include <fstream>

#ifndef URDF_DIR
#define URDF_DIR ""
#endif

using namespace DWBC;
using namespace std;

// int main(void)

int main(int argc, char **argv)
{
    // retreive int arguments
    int option = 0;
    if (argc > 1)
    {
        option = std::stoi(argv[1]);
    }
    // std::cout << "option : " << option << std::endl;

    bool contact1 = true;
    bool contact2 = true;
    bool contact3 = false;

    double rot_z = 0;
    // double rot_z = M_PI_2;
    int repeat = 1000;

    bool use_hqp = true;

    std::string urdf2_file = std::string(URDF_DIR) + "/dyros_tocabi.urdf";

    std::string desired_control_target = "COM";
    std::string desired_control_target2 = "pelvis_link";
    std::string desired_control_target3 = "upperbody_link";
    std::string desired_control_target4 = "L_Wrist2_Link";
    std::string desired_control_target5 = "EE";

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

    q2 << 0, 0, 0, qu.x(), qu.y(), qu.z(),
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
    rd2_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.13, 0.06, verbose);
    rd2_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.13, 0.06, verbose);
    rd2_.AddContactConstraint(23, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);
    rd2_.AddContactConstraint(31, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);

    rd2_.AddTaskSpace(0, TASK_LINK_6D, desired_control_target.c_str(), Vector3d::Zero(), verbose);
    // rd2_.AddTaskSpace(1, TASK_LINK_ROTATION, desired_control_target2.c_str(), Vector3d::Zero(), verbose);
    // rd2_.AddTaskSpace(2, TASK_LINK_ROTATION, desired_control_target3.c_str(), Vector3d::Zero(), verbose);
    // rd2_.AddTaskSpace(3, TASK_LINK_6D, desired_control_target4.c_str(), Vector3d::Zero(), verbose);
    // rd2_.AddTaskSpace(3, TASK_LINK_6D, desired_control_target5.c_str(), Vector3d::Zero(), verbose);
    // rd2_.AddTaskSpace(TASK_CUSTOM, 12);

    rd2_.SetContact(contact1, contact2, contact3, false);
    rd2_.CalcContactConstraint();

    fstar_1.segment(0, 3) = rd2_.link_[0].rotm * fstar_1.segment(0, 3);
    fstar_1.segment(3, 3) = rd2_.link_[0].rotm * fstar_1.segment(3, 3);

    MatrixXd J_task3;
    VectorXd f_star3;

    int lh_id = rd2_.getLinkID("L_Wrist2_Link");
    int rh_id = rd2_.getLinkID("R_Wrist2_Link");

    f_star3.setZero(12);
    f_star3.segment(0, 6) = 0.5 * fstar_1;
    f_star3.segment(6, 6) = 0.2 * fstar_1;

    // f_star3(3) = 1.0;
    // f_star3(9) = 1.0;

    Vector6d f_star0;
    f_star0 << 0.4, 2, 0.2, 0, 0, 0;

    // rd2_.SetTaskSpace(0, f_star0);
    // rd2_.SetTaskSpace(1, fstar_1.segment(3, 3));
    // rd2_.SetTaskSpace(2, -fstar_1.segment(3, 3));
    // rd2_.SetTaskSpace(3, f_star3);

    // rd2_.CalcGravCompensation();
    // rd2_.CalcTaskControlTorque(use_hqp, true);
    // rd2_.CalcContactRedistribute(use_hqp, true);
    // auto t10 = std::chrono::high_resolution_clock::now();

    // for (int i = 0; i < repeat; i++)
    // {
    //     rd2_.SetContact(contact1, contact2, contact3);
    //     rd2_.CalcContactConstraint();
    //     rd2_.CalcGravCompensation();
    // }
    // auto t11 = std::chrono::high_resolution_clock::now();
    // for (int i = 0; i < repeat; i++)
    // {
    //     rd2_.CalcTaskSpace();
    // }
    // auto t12 = std::chrono::high_resolution_clock::now();
    // for (int i = 0; i < repeat; i++)
    // {
    //     rd2_.CalcTaskControlTorque(use_hqp, false, false);
    // }

    // auto t13 = std::chrono::high_resolution_clock::now();
    // for (int i = 0; i < repeat; i++)
    // {
    //     if (!rd2_.CalcContactRedistribute(use_hqp, false))
    //     {
    //         std::cout << "Contact Redistribution Failed at step : " << i << std::endl;
    //     }
    // }
    // auto t14 = std::chrono::high_resolution_clock::now();

    // // std::cout << "-----------------------------------------------------------------" << std::endl;

    // double time_original_us = std::chrono::duration_cast<std::chrono::microseconds>(t14 - t10).count();
    // double time_original1_us = std::chrono::duration_cast<std::chrono::microseconds>(t11 - t10).count();
    // double time_original2_us = std::chrono::duration_cast<std::chrono::microseconds>(t12 - t11).count();
    // double time_original3_us = std::chrono::duration_cast<std::chrono::microseconds>(t13 - t12).count();
    // double time_original4_us = std::chrono::duration_cast<std::chrono::microseconds>(t14 - t13).count();

    // std::cout << "Original Dynamics Model  TOTAL CONSUMPTION : " << (float)(time_original_us / repeat) << " us" << std::endl;
    // std::cout << "Original Dynamics Model 1 - Contact Constraint Calculation : " << (int)(time_original1_us / repeat) << " us" << std::endl;
    // std::cout << "Original Dynamics Model 2 - Task Space Calculation         : " << (int)(time_original2_us / repeat) << " us" << std::endl;
    // std::cout << "Original Dynamics Model 3 - Task Torque Calculation        : " << (int)(time_original3_us / repeat) << " us" << std::endl;
    // std::cout << "Original Dynamics Model 4 - Contact Redistribution Calcula : " << (int)(time_original4_us / repeat) << " us" << std::endl;

    std::cout << "-----------------------------------------------------------------" << std::endl;

    // for (int i = 0; i < rd2_.ts_.size(); i++)
    // {
    //     std::cout << "task " << i << " fstar qp  : " << rd2_.ts_[i].f_star_qp_.transpose() << std::endl;
    //     std::cout << "contact qp : " << rd2_.ts_[i].contact_qp_.transpose() << std::endl;
    // }
    // std::cout << "contact qp final : " << rd2_.cf_redis_qp_.transpose() << std::endl;

    // std::cout << "-----------------------------------------------------------------" << std::endl;

    int q_dof = rd2_.model_.q_size;
    int qdot_dof = rd2_.model_.qdot_size;
    int qddot_dof = qdot_dof;

    // for (int k = 0; k < 10; k++)
    // {
    MatrixXd give_me_fstar;

    VectorXd q3 = q2;
    VectorXd q3dot = q2dot;
    VectorXd q3ddot = q2ddot;

    // if (k > 0)
    // {

    int k = option;

    if (k > 0)
    {
        q_dof -= k;
        qdot_dof -= k;
        qddot_dof -= k;

        q3.setZero(q_dof);
        q3 = q2.segment(0, q_dof);
        q3(q_dof - 1) = q2(q_dof - 1);

        q3dot.setZero(qdot_dof);
        q3dot = q2dot.segment(0, qdot_dof);

        q3ddot.setZero(qddot_dof);
        q3ddot = q2ddot.segment(0, qddot_dof);
    }
    else if (k == 0)
    {
        q3 = q2;
        q3dot = q2dot;
        q3ddot = q2ddot;
    }
    else if (k < 0)
    {
        q_dof -= k;
        qdot_dof -= k;
        qddot_dof -= k;

        q3.setZero(q_dof);
        q3.segment(0, q_dof + k) = q2.segment(0, q_dof + k);

        q3.segment(q_dof + k, -k) = q2.segment(q_dof + k - 8, -k);

        // q3.segment(q_dof - 1, 1) = q2.segment(q_dof - 1, 1);

        q3(q_dof - 1) = q2(q_dof + k - 1);

        q3dot.setZero(qdot_dof);
        q3ddot.setZero(qddot_dof);
    }
    // }
    int dof = 33 - option;

    std::string urdf3_file = std::string(URDF_DIR) + "/dyros_tocabi_dof" + std::to_string(33 - option) + ".urdf";

    std::cout << "URDF : " << urdf3_file << std::endl;

    urdf2_file = std::string(URDF_DIR) + "/dyros_tocabi_dof31.urdf";
    bool t1_success = true;
    bool t2_success = true;

    double t1_time = 0;
    double t2_time = 0;
    double t2_dynamics_calc = 0;
    double t2_task_calc = 0;

    {
        RobotData rd_;
        rd_.LoadModelData(urdf3_file, true, false);

        rd_.UpdateKinematics(q3, q3dot, q3ddot);
        rd_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.13, 0.06, verbose);
        rd_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.13, 0.06, verbose);
        // rd_.AddContactConstraint(23, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);
        // rd_.AddContactConstraint(31, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);

        rd_.AddTaskSpace(0, TASK_LINK_6D, desired_control_target.c_str(), Vector3d::Zero(), verbose);
        // rd_.AddTaskSpace(1, TASK_LINK_ROTATION, desired_control_target2.c_str(), Vector3d::Zero(), verbose);
        rd_.AddTaskSpace(1, TASK_LINK_ROTATION, desired_control_target5.c_str(), Vector3d::Zero(), verbose);
        // rd_.AddTaskSpace(3, TASK_LINK_6D, desired_control_target4.c_str(), Vector3d::Zero(), verbose);
        // rd_.AddTaskSpace(1, TASK_LINK_6D, desired_control_target5.c_str(), Vector3d::Zero(), verbose);

        rd_.SetContact(contact1, contact2);
        // rd_.CalcContactConstraint();

        rd_.ReducedDynamicsCalculate();

        rd_.SetTaskSpace(0, f_star0);
        // rd_.SetTaskSpace(1, fstar_1.segment(3, 3));
        // rd_.SetTaskSpace(2, -fstar_1.segment(3, 3));
        rd_.SetTaskSpace(1, f_star3.head(3));

        auto t2_0 = std::chrono::high_resolution_clock::now();
        bool init_qp = true;
        for (int i = 0; i < repeat; i++)
        {
            rd_.SetContact(contact1, contact2);

            rd_.UpdateTaskSpace();

            for (int j = 0; j < rd_.ts_.size(); j++)
            {
                t1_success &= rd_.CalcSingleTaskTorqueWithJACC_QP(rd_.ts_[j], init_qp);
            }

            init_qp = false;
        }
        auto t2_1 = std::chrono::high_resolution_clock::now();

        double time_original_us2 = std::chrono::duration_cast<std::chrono::microseconds>(t2_1 - t2_0).count();

        t1_time = time_original_us2 / repeat;

        std::cout << "Original Dynamics Model COMP 2 TOTAL CONSUMPTION : " << (float)(time_original_us2 / repeat) << " us" << std::endl;
    }

    std::cout << "-----------------------------------------------------------------" << std::endl;
    {

        RobotData rd_;
        rd_.LoadModelData(urdf3_file, true, false);

        // if (k > 0)
        //     rd_.DeleteLink(link_id_last - k + 1, true);

        rd_.UpdateKinematics(q3, q3dot, q3ddot);
        rd_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.13, 0.06, verbose);
        rd_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.13, 0.06, verbose);
        // rd_.AddContactConstraint(23, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);
        // rd_.AddContactConstraint(31, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);

        rd_.AddTaskSpace(0, TASK_LINK_6D, desired_control_target.c_str(), Vector3d::Zero(), verbose);
        // rd_.AddTaskSpace(1, TASK_LINK_ROTATION, desired_control_target2.c_str(), Vector3d::Zero(), verbose);
        rd_.AddTaskSpace(1, TASK_LINK_ROTATION, desired_control_target5.c_str(), Vector3d::Zero(), verbose);
        // rd_.AddTaskSpace(1, TASK_LINK_6D, desired_control_target4.c_str(), Vector3d::Zero(), verbose);
        // rd_.AddTaskSpace(3, TASK_LINK_6D, desired_control_target5.c_str(), Vector3d::Zero(), verbose);

        // rd_.CalcContactConstraint();
        rd_.SetContact(contact1, contact2);

        rd_.SetTaskSpace(0, f_star0);

        // rd_.qp_task_.push_back(CQuadraticProgram());
        // rd_.SetTaskSpace(1, fstar_1.segment(3, 3));
        // rd_.SetTaskSpace(2, -fstar_1.segment(3, 3));
        rd_.SetTaskSpace(1, f_star3.head(3));

        auto t2_0 = std::chrono::high_resolution_clock::now();
        bool init_qp = true;
        for (int i = 0; i < repeat; i++)
        {
            rd_.SetContact(contact1, contact2);

            rd_.ReducedDynamicsCalculate();
        }
        auto t2_1 = std::chrono::high_resolution_clock::now();

        for (int i = 0; i < repeat; i++)
        {
            rd_.UpdateTaskSpace();

            t2_success &= rd_.CalcSingleTaskTorqueWithJACC_QP_R(rd_.ts_[0], init_qp);

            VectorXd desired_acc = rd_.ts_[0].acc_qp_.segment(rd_.vc_dof, 6);
            // int j=2;
            t2_success &= rd_.CalcSingleTaskTorqueWithJACC_QP_R_NC(rd_.ts_.back(), desired_acc, init_qp);

            init_qp = false;
        }
        auto t2_2 = std::chrono::high_resolution_clock::now();

        double time_original_us2 = std::chrono::duration_cast<std::chrono::microseconds>(t2_2 - t2_0).count();
        double time_original_us21 = std::chrono::duration_cast<std::chrono::microseconds>(t2_1 - t2_0).count();
        double time_original_us22 = std::chrono::duration_cast<std::chrono::microseconds>(t2_2 - t2_1).count();

        t2_time = time_original_us2 / repeat;
        t2_dynamics_calc = time_original_us21 / repeat;
        t2_task_calc = time_original_us22 / repeat;

        std::cout << "Reduced Dynamics Model COMP 2 TOTAL CONSUMPTION : " << (float)(time_original_us2 / repeat) << " us" << std::endl;
        std::cout << "Reduced Dynamics Model 1 - Reduced Dynamics calculation : " << (float)(time_original_us21 / repeat) << " us" << std::endl;
        std::cout << "Reduced Dynamics Model 2 - Task Space Calculation         : " << (float)(time_original_us22 / repeat) << " us" << std::endl;

        std::cout << "-----------------------------------------------------------------" << std::endl;

        std::cout << rd_.model_dof_ << "DOF : total comparison : " << t2_time / t1_time * 100 << " %" << std::endl;
    }
    // }

    if (t1_success && t2_success)
    {
        std::ofstream outputFile("/home/dyros/saga_ws/libdwbc/tests/sp_test/output.txt", std::ios::app);
        outputFile << std::fixed << std::setprecision(4);
        outputFile << dof << " " << t1_time << " " << t2_time << " " << t2_dynamics_calc << " " << t2_task_calc << " " << t2_time / t1_time << std::endl;
        outputFile.close();
    }
    else
    {
        std::cout << "Fail" << std::endl;
    }

    return 0;
}
