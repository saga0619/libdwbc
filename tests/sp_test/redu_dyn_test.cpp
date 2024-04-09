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
    bool contact1 = true;
    bool contact2 = true;
    bool contact3 = false;

    double rot_z = 0;
    int repeat = 10000;

    bool use_hqp = false;
    std::cout << "use hqp? (y/n) : ";
    char input;
    std::cin >> input;
    if (input == 'y')
    {
        use_hqp = true;
        std::cout << "HQP enabled \n";
    }
    else
    {
        use_hqp = false;
        std::cout << "HQP disabled \n";
    }

    std::string urdf2_file = std::string(URDF_DIR) + "/dyros_tocabi.urdf";

    std::string desired_control_target = "COM";
    std::string desired_control_target2 = "pelvis_link";
    std::string desired_control_target3 = "upperbody_link";
    std::string desired_control_target4 = "L_Wrist2_Link";
    std::string desired_control_target5 = "R_Wrist2_Link";

    VectorXd fstar_1;
    fstar_1.setZero(6);
    fstar_1 << 0.11, 0.5, 0.13, 0.12, -0.11, 0.05; // based on local frmae.

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
    rd2_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    rd2_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    rd2_.AddContactConstraint(23, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);
    rd2_.AddContactConstraint(31, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);

    rd2_.AddTaskSpace(0, TASK_LINK_POSITION, desired_control_target.c_str(), Vector3d::Zero(), verbose);
    rd2_.AddTaskSpace(1, TASK_LINK_ROTATION, desired_control_target2.c_str(), Vector3d::Zero(), verbose);
    rd2_.AddTaskSpace(2, TASK_LINK_ROTATION, desired_control_target3.c_str(), Vector3d::Zero(), verbose);
    // rd2_.AddTaskSpace(3, TASK_LINK_6D, desired_control_target4.c_str(), Vector3d::Zero(), verbose);
    rd2_.AddTaskSpace(3, TASK_LINK_6D, desired_control_target5.c_str(), Vector3d::Zero(), verbose);
    // rd2_.AddTaskSpace(TASK_CUSTOM, 12);

    rd2_.SetContact(contact1, contact2, contact3, false);
    rd2_.CalcContactConstraint();

    fstar_1.segment(0, 3) = rd2_.link_[0].rotm * fstar_1.segment(0, 3);
    fstar_1.segment(3, 3) = rd2_.link_[0].rotm * fstar_1.segment(3, 3);

    MatrixXd J_task3;
    VectorXd f_star3;

    int lh_id = rd2_.getLinkID("L_Wrist2_Link");
    int rh_id = rd2_.getLinkID("R_Wrist2_Link");

    f_star3.setZero(6);
    f_star3.segment(0, 6) = 0.5 * fstar_1;
    // f_star3.segment(6, 6) = 0.2 * fstar_1;

    // f_star3(3) = 1.0;
    // f_star3(9) = 1.0;

    rd2_.SetTaskSpace(0, 3 * fstar_1.segment(0, 3));
    rd2_.SetTaskSpace(1, fstar_1.segment(3, 3));
    rd2_.SetTaskSpace(2, -fstar_1.segment(3, 3));
    rd2_.SetTaskSpace(3, f_star3);

    rd2_.CalcGravCompensation();
    rd2_.CalcTaskControlTorque(use_hqp, true);
    rd2_.CalcContactRedistribute(use_hqp, true);
    auto t10 = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < repeat; i++)
    {
        rd2_.SetContact(contact1, contact2, contact3);
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
    std::cout << " QP Fstar modified : " << rd2_.ts_[0].f_star_.transpose() + rd2_.ts_[0].f_star_qp_.transpose() << std::endl;

    std::cout << "Result Task Torque : " << std::endl;

    VectorXd torque_task_original = rd2_.torque_task_;
    std::cout << rd2_.torque_task_.transpose() << std::endl;

    std::cout << "Result grav Torque : " << std::endl;
    std::cout << rd2_.torque_grav_.transpose() << std::endl;

    VectorXd torque_grav_original = rd2_.torque_grav_;

    std::cout << "contact torque : " << std::endl;
    std::cout << rd2_.torque_contact_.transpose() << std::endl;

    VectorXd torque_contact_original = rd2_.torque_contact_;

    MatrixXd s_k = MatrixXd::Zero(rd2_.model_dof_, rd2_.model_dof_ + 6);
    s_k.block(0, 6, rd2_.model_dof_, rd2_.model_dof_) = MatrixXd::Identity(rd2_.model_dof_, rd2_.model_dof_);
    MatrixXd give_me_fstar = rd2_.ts_[0].J_task_ * rd2_.A_inv_ * rd2_.N_C * s_k.transpose();

    std::cout << "fstar : " << (give_me_fstar * rd2_.torque_task_).transpose() << std::endl;

    if (use_hqp)
    {
        std::cout << "-----------------------------------------------------------------" << std::endl;

        for (int i = 0; i < rd2_.ts_.size(); i++)
        {
            std::cout << "task " << i << " fstar qp  : " << rd2_.ts_[i].f_star_qp_.transpose() << std::endl;
        }
    }
    std::vector<VectorXd> torque_task_original_vec;
    std::cout << "-----------------------------------------------------------------" << std::endl;
    for (int i = 0; i < rd2_.ts_.size(); i++)
    {
        std::cout << "task " << i << " torque : " << std::endl;
        std::cout << (rd2_.ts_[i].torque_h_).transpose() << std::endl;
        torque_task_original_vec.push_back(rd2_.ts_[i].torque_h_);
    }
    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::vector<VectorXd> torque_task_null_original_vec;
    std::cout << " NULL spaced torque : " << std::endl;
    for (int i = 1; i < rd2_.ts_.size(); i++)
    {
        std::cout << "task " << i << " torque : " << std::endl;
        std::cout << (rd2_.ts_[i - 1].Null_task_ * rd2_.ts_[i].J_kt_ * rd2_.ts_[i].Lambda_task_ * rd2_.ts_[i].f_star_).transpose() << std::endl;
        torque_task_null_original_vec.push_back(rd2_.ts_[i - 1].Null_task_ * rd2_.ts_[i].torque_h_);
        // std::cout << (torque_task_null_original_vec[i - 1]).transpose() << std::endl;
        // std::cout << (give_me_fstar * rd2_.ts_[i - 1].Null_task_ * rd2_.ts_[i].J_kt_ * rd2_.ts_[i].Lambda_task_ * rd2_.ts_[i].f_star_).transpose() << std::endl;
    }
    std::cout << "-----------------------------------------------------------------" << std::endl;

    /*


    SET 1 configuration


    */

    rd2_ = RobotData();
    rd2_.LoadModelData(urdf2_file, true, false);
    rd2_.UpdateKinematics(q2, q2dot, q2ddot);
    rd2_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    rd2_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    rd2_.AddContactConstraint(23, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);
    rd2_.AddContactConstraint(31, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);

    rd2_.AddTaskSpace(0, TASK_LINK_POSITION, desired_control_target.c_str(), Vector3d::Zero(), verbose);
    rd2_.AddTaskSpace(1, TASK_LINK_ROTATION, desired_control_target2.c_str(), Vector3d::Zero(), verbose);
    rd2_.AddTaskSpace(2, TASK_LINK_ROTATION, desired_control_target3.c_str(), Vector3d::Zero(), verbose);
    // rd2_.AddTaskSpace(3, TASK_LINK_6D, desired_control_target4.c_str(), Vector3d::Zero(), verbose);
    rd2_.AddTaskSpace(3, TASK_LINK_6D, desired_control_target5.c_str(), Vector3d::Zero(), verbose);

    rd2_.SetTaskSpace(0, 3 * fstar_1.segment(0, 3));
    rd2_.SetTaskSpace(1, fstar_1.segment(3, 3));
    rd2_.SetTaskSpace(2, -fstar_1.segment(3, 3));
    rd2_.SetTaskSpace(3, f_star3);

    rd2_.SetContact(contact1, contact2, contact3, false);

    rd2_.ReducedDynamicsCalculate();

    rd2_.ReducedCalcContactConstraint();
    rd2_.ReducedCalcGravCompensation();
    rd2_.ReducedCalcTaskSpace();
    rd2_.ReducedCalcTaskControlTorque(use_hqp, true, false);
    rd2_.ReducedCalcContactRedistribute(use_hqp, true);
    // start time
    auto t0 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < repeat; i++)
    {
        rd2_.SetContact(contact1, contact2, contact3, false);
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
        rd2_.ReducedCalcTaskSpace();
    }
    auto t3 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < repeat; i++)
    {
        rd2_.ReducedCalcTaskControlTorque(use_hqp, false, false);
    }
    auto t4 = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < repeat; i++)
    {
        rd2_.ReducedCalcContactRedistribute(use_hqp, false);
    }
    auto t5 = std::chrono::high_resolution_clock::now();

    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "RESULT WITH Reduced Dynamics Model : \n";

    double time_reduced_us = std::chrono::duration_cast<std::chrono::microseconds>(t5 - t0).count();
    double time_reduced1_us = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    double time_reduced2_us = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
    double time_reduced3_us = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count();
    double time_reduced4_us = std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count();
    std::cout << "Reduced Dynamics Model  TOTAL CONSUMPTION : " << (float)(time_reduced_us / repeat) << " us" << std::endl;
    std::cout << "Reduced Dynamics Model 1 - Reduced Dynamics Calculation   : " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / repeat << " us" << std::endl;
    std::cout << "Reduced Dynamics Model 2 - Contact Constraint Calculation : " << (int)(time_reduced1_us / repeat) << " us (" << (time_reduced1_us / time_original1_us) * 100 << "%)" << std::endl;
    std::cout << "Reduced Dynamics Model 3 - Task space Calculation         : " << (int)(time_reduced2_us / repeat) << " us (" << (time_reduced2_us / time_original2_us) * 100 << "%)" << std::endl;
    std::cout << "Reduced Dynamics Model 4 - Task Torque Calculation        : " << (int)(time_reduced3_us / repeat) << " us (" << (time_reduced3_us / time_original3_us) * 100 << "%)" << std::endl;
    std::cout << "Reduced Dynamics Model 5 - Contact Redistribution Calcula : " << (int)(time_reduced4_us / repeat) << " us (" << (time_reduced4_us / time_original4_us) * 100 << "%)" << std::endl;

    // std::cout << "A_R : " << std::endl;

    std::cout << "-----------------------------------------------------------------" << std::endl;

    // Original vs reduced
    std::cout << "Original time vs Reduced time : " << (time_reduced_us / time_original_us) * 100 << "%" << std::endl;
    std::cout << "Task Torque Similarity : " << (rd2_.torque_task_.transpose() - torque_task_original.transpose()).norm() << std::endl;
    std::cout << "Grav Torque Similarity : " << (rd2_.torque_grav_.transpose() - torque_grav_original.transpose()).norm() << std::endl;
    std::cout << "Contact Torque Similarity : " << (rd2_.torque_contact_.transpose() - torque_contact_original.transpose()).norm() << std::endl;

    std::cout << "-----------------------------------------------------------------" << std::endl;
    // std::cout << "task torque : \n";
    // std::cout << rd2_.torque_task_.transpose() << std::endl;
    // std::cout << "grav torque : " << std::endl;
    // std::cout << rd2_.torque_grav_.transpose() << std::endl;
    // std::cout << "contact torque : " << std::endl;
    // std::cout << rd2_.torque_contact_.transpose() << std::endl;

    // std::cout << "fstar 2 :";
    // std::cout << (give_me_fstar * rd2_.torque_task_).transpose() << std::endl;
    // std::cout << "-----------------------------------------------------------------" << std::endl;
    for (int i = 0; i < rd2_.ts_.size(); i++)
    {
        VectorXd h_torque = VectorXd::Zero(rd2_.model_dof_);
        h_torque.segment(0, rd2_.co_dof) = rd2_.ts_[i].torque_h_R_.segment(0, rd2_.co_dof);
        h_torque.segment(rd2_.co_dof, rd2_.nc_dof) = rd2_.J_I_nc_.transpose() * rd2_.ts_[i].torque_h_R_.segment(rd2_.co_dof, 6) + rd2_.N_I_nc_ * rd2_.ts_[i].torque_nc_;
        double similarity = (h_torque - torque_task_original_vec[i]).norm();
        std::cout << "task " << i << " torque | Similarity : " << similarity << std::endl;
        if (similarity > 1.0E-4)
            std::cout << (h_torque).transpose() << std::endl;
    }
    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << " NULL spaced torque : " << std::endl;
    for (int i = 1; i < rd2_.ts_.size(); i++)
    {
        VectorXd null_torque = VectorXd::Zero(rd2_.model_dof_);
        null_torque.segment(0, rd2_.co_dof) = rd2_.ts_[i].torque_null_h_R_.segment(0, rd2_.co_dof);
        null_torque.segment(rd2_.co_dof, rd2_.nc_dof) = rd2_.J_I_nc_.transpose() * rd2_.ts_[i].torque_null_h_R_.segment(rd2_.co_dof, 6) + rd2_.N_I_nc_ * rd2_.ts_[i].torque_null_h_nc_;
        double similarity = (null_torque - torque_task_null_original_vec[i - 1]).norm();
        std::cout << "task " << i << " torque : Similarity : " << similarity << std::endl;
        if (similarity > 1.0E-4)
            std::cout << (null_torque).transpose() << std::endl;
    }

    std::cout << "-----------------------------------------------------------------" << std::endl;

    // for (int i = 0; i < repeat; i++)
    // {

    vector<int> co_joint_idx_;
    vector<int> nc_joint_idx_;

    vector<int> co_link_idx_;
    vector<int> nc_link_idx_;

    co_link_idx_.push_back(0);

    for (int i = 0; i < rd2_.cc_.size(); i++)
    {
        if (rd2_.cc_[i].contact)
        {
            int link_idx = rd2_.cc_[i].link_number_;

            while (link_idx != 0)
            {
                co_link_idx_.push_back(link_idx);
                co_joint_idx_.push_back(rd2_.joint_[link_idx].joint_id_);
                link_idx = rd2_.link_[link_idx].parent_id_;
            }
        }
    }
    sort(co_link_idx_.begin(), co_link_idx_.end());

    for (int i = 0; i < rd2_.link_num_; i++)
    {
        if (std::find(co_link_idx_.begin(), co_link_idx_.end(), i) == co_link_idx_.end())
        {
            nc_link_idx_.push_back(i);
        }
    }

    sort(co_joint_idx_.begin(), co_joint_idx_.end());

    for (int i = 0; i < nc_link_idx_.size(); i++)
    {
        int link_idx = nc_link_idx_[i];
        while (link_idx != 0)
        {
            if (find(nc_joint_idx_.begin(), nc_joint_idx_.end(), rd2_.joint_[link_idx].joint_id_) == nc_joint_idx_.end())
            {
                nc_joint_idx_.push_back(rd2_.joint_[link_idx].joint_id_);
            }
            link_idx = rd2_.link_[link_idx].parent_id_;

            if (find(nc_link_idx_.begin(), nc_link_idx_.end(), link_idx) != nc_link_idx_.end())
            {
                break;
            }
        }
    }

    sort(nc_joint_idx_.begin(), nc_joint_idx_.end());

    MatrixXd ANC = MatrixXd::Zero(nc_joint_idx_.size(), nc_joint_idx_.size());

    for (int i = 0; i < nc_joint_idx_.size(); i++)
    {
        for (int j = 0; j < nc_joint_idx_.size(); j++)
        {
            ANC(i, j) = rd2_.A_(nc_joint_idx_[i], nc_joint_idx_[j]);
        }
    }

    // }
    // auto t32 = std::chrono::high_resolution_clock::now();

    // double time_contact_us = std::chrono::duration_cast<std::chrono::microseconds>(t32 - t31).count();

    // std::cout << "cd calc : " << (float)(time_contact_us / repeat) << " us" << std::endl;

    // print link_idx, joint_idx
    std::cout << "co_link_idx_ : ";
    for (int i = 0; i < co_link_idx_.size(); i++)
    {
        std::cout << co_link_idx_[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "nc_link_idx_ : ";
    for (int i = 0; i < nc_link_idx_.size(); i++)
    {
        std::cout << nc_link_idx_[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "co_joint_idx_ : ";
    for (int i = 0; i < co_joint_idx_.size(); i++)
    {
        std::cout << co_joint_idx_[i] << " ";
    }
    std::cout << std::endl;

    std::cout << "nc_joint_idx_ : ";
    for (int i = 0; i < nc_joint_idx_.size(); i++)
    {
        std::cout << nc_joint_idx_[i] << " ";
    }
    std::cout << std::endl;


    // reorder co_link_idx_

    // double f1, f2, f3;
    // for (int j = 2; j < 20; j++)
    // {
    //     auto tp1 = std::chrono::high_resolution_clock::now();

    //     MatrixXd A = MatrixXd::Random(10 * j, 10 * j);
    //     MatrixXd B = MatrixXd::Random(10 * j, 10 * j);
    //     MatrixXd C = MatrixXd::Random(10 * j, 10 * j);

    //     for (int i = 0; i < repeat; i++)
    //     {
    //         C = A * B;
    //     }

    //     auto tp2 = std::chrono::high_resolution_clock::now();

    //     for (int i = 0; i < repeat; i++)
    //     {
    //         C = A.inverse();
    //     }

    //     auto tp3 = std::chrono::high_resolution_clock::now();

    //         Eigen::CompleteOrthogonalDecomposition<MatrixXd> cod(A.rows(), A.cols());
    //     for (int i = 0; i < repeat; i++)
    //     {
    //         cod.compute(A);
    //         C = cod.pseudoInverse();
    //     }

    //     auto tp4 = std::chrono::high_resolution_clock::now();

    //     double time_matrix_us = std::chrono::duration_cast<std::chrono::nanoseconds>(tp2 - tp1).count();
    //     double time_matrix_us1 = std::chrono::duration_cast<std::chrono::nanoseconds>(tp3 - tp2).count();
    //     double time_matrix_us2 = std::chrono::duration_cast<std::chrono::nanoseconds>(tp4 - tp3).count();

    //     double first = 2;

    //     if(j==2)
    //     {
    //         f1 = time_matrix_us/repeat/1000.0;
    //         f2 = time_matrix_us1/repeat/1000.0;
    //         f3 = time_matrix_us2/repeat/1000.0;
    //     }

    //     std::cout << j*10<<" : "<<(j*j*j)/8.0 << " Matrix Mul : " << (time_matrix_us / repeat)/1000.0 << " us, "<<(time_matrix_us / repeat)/1000.0/f1 << " ratio : " << 1 << std::endl;
    //     std::cout << j*10<<" : "<<(j*j*j)/8.0 << " Matrix Inv : " << (time_matrix_us1 / repeat)/1000.0 << " us, " <<(time_matrix_us1 / repeat)/1000.0/f2 <<" ratio : " << time_matrix_us1/time_matrix_us <<  std::endl;
    //     std::cout << j*10<<" : "<<(j*j*j)/8.0 << " Matrix COD : " << (time_matrix_us2 / repeat)/1000.0 << " us, " <<(time_matrix_us2 / repeat)/1000.0/f3 << " ratio : " << time_matrix_us2/time_matrix_us << std::endl;
    // }
    // std::cout << rd2_.force_on_nc_R_qp_.transpose() << std::endl;

    // // std::cout <<

    // MatrixXd J_nc_;
    // J_nc_.setZero(6, rd2_.model_dof_ + 6);
    // J_nc_.block(0, 0, 6, 6) = rd2_.J_nc_R_.block(0, 0, 6, 6);
    // J_nc_.rightCols(rd2_.nc_dof) = rd2_.J_I_nc_;

    // MatrixXd Lambda_nc_;

    // Lambda_nc_ = (J_nc_ * rd2_.A_inv_N_C * J_nc_.transpose()).inverse();

    // MatrixXd J_nc_inv_T = Lambda_nc_ * J_nc_ * rd2_.A_inv_N_C;

    // VectorXd force_nc = J_nc_inv_T * rd2_.ts_[2].J_task_.transpose() * rd2_.ts_[2].Lambda_task_ * rd2_.ts_[2].f_star_;

    // std::cout << (force_nc).transpose() << std::endl;

    // std::cout << rd2_.ts_[2].force_on_nc_.transpose() << std::endl;
    // rd2_.ts_[2].torque_h_R_;

    // std::cout << (rd2_.J_nc_R_kt_ * force_nc).transpose() << std::endl;

    // std::cout << "original jac rotm : " << std::endl;
    // std::cout << rd2_.link_[lh_id].jac_.block(0, 3, 3, 3) << std::endl;
    // std::cout << "original xpos : " << std::endl;
    // std::cout << rd2_.link_[lh_id].xpos.transpose() << std::endl;

    // std::cout << "rotm? : " << std::endl;
    // std::cout << skew(-rd2_.link_[lh_id].xpos) * rd2_.link_[0].rotm << std::endl;

    // Vector6d Force_6d_local = Vector6d::Zero();
    // Vector6d force_local = Vector6d::Zero();
    // force_local << 1, 2, 3, 4, 5, 6;
    // Force_6d_local.segment(0, 3) = rd2_.link_[0].rotm * force_local.segment(0, 3);
    // Force_6d_local.segment(3, 3) = rd2_.link_[0].rotm * force_local.segment(3, 3);

    // std::cout << "Force 6d local : " << Force_6d_local.transpose() << std::endl;
    // std::cout << rd2_.J_nc_R_kt_ * Force_6d_local << std::endl;

    // std::cout << "-----------------------------------------------------------------" << std::endl;
    // std::cout << rd2_.ts_[2].force_on_nc_.transpose() << std::endl;

    // Vector6d force_on_b = rd2_.ts_[2].J_task_.transpose() * rd2_.ts_[2].Lambda_task_ * rd2_.ts_[2].f_star_;

    // std::cout << (rd2_.ts_[2].J_task_.transpose() * rd2_.ts_[2].Lambda_task_ * rd2_.ts_[2].f_star_).transpose() << std::endl;

    // MatrixXd J_task_base, J_task_kt;
    // J_task_base = rd2_.link_[0].jac_.leftCols(rd2_.reduced_system_dof_);
    // MatrixXd lambda_task;

    // auto t10_ = std::chrono::high_resolution_clock::now();
    // for (int i = 0; i < repeat; i++)
    //     CalculateJKT(J_b, rd2_.A_inv_, rd2_.N_C, rd2_.W_inv, J_b_kt, lambda_b);

    // auto t11_ = std::chrono::high_resolution_clock::now();

    // for (int i = 0; i < repeat; i++)
    //     CalculateJKT_R(J_task_base, rd2_.A_R_inv_N_CR, rd2_.W_R_inv, J_task_kt, lambda_task);

    // auto t12_ = std::chrono::high_resolution_clock::now();

    // double time_original_us_ = std::chrono::duration_cast<std::chrono::microseconds>(t11_ - t10_).count();
    // double time_reduced_us_ = std::chrono::duration_cast<std::chrono::microseconds>(t12_ - t11_).count();

    // std::cout << time_original_us_ / repeat << " us" << std::endl;
    // std::cout << time_reduced_us_ / repeat << " us" << std::endl;

    // VectorXd torque = J_task_kt * force_on_b;

    // std::cout << torque.transpose() << std::endl;
    // std::cout << rd2_.ts_[2].torque_h_R_.transpose() << std::endl;

    // std::cout << "-----------------------------------------------------------------" << std::endl;
    // std::cout << rd2_.ts_[3].force_on_nc_.transpose() << std::endl;
    // std::cout << (rd2_.ts_[3].J_task_.transpose() * rd2_.ts_[3].Lambda_task_ * rd2_.ts_[3].f_star_).transpose() << std::endl;
    // std::cout << "-----------------------------------------------------------------" << std::endl;

    // int task_num = 0;

    // MatrixXd J_tc, J_tc_kt, lambda_tc;

    // rd2_.link_[1].UpdateJac(rd2_.model_, rd2_.q_system_);

    // J_tc = rd2_.link_[1].jac_.bottomRows(3);;

    // rd2_.CalcContactConstraint();
    // CalculateJKT(J_tc, rd2_.A_inv_, rd2_.N_C, rd2_.W_inv, J_tc_kt, lambda_tc);

    // std::cout << "J_tc_kt : " << std::endl;
    // std::cout << J_tc_kt << std::endl;

    // VectorXd toruqe_v = J_tc.transpose() * lambda_tc * fstar_1.segment(0,3);

    // VectorXd force_v = toruqe_v.segment(0, 6);
    // std::cout << "J' * lambda * fstar :" << std::endl;
    // std::cout << toruqe_v.transpose() << std::endl;

    // VectorXd torque_l = (rd2_.J_base_R_kt_ * force_v).segment(0, 12);

    // std::cout << "J_base_R_kt_ * toruqe_v.segment(0, 6);" << std::endl;
    // std::cout << torque_l.transpose() << std::endl;

    // std::cout << "-----------------------------------------------------------------" << std::endl;
    // std::cout << (torque_l + toruqe_v.segment(6, 12)).transpose() - (J_tc_kt * lambda_tc * fstar_1).segment(0, 12).transpose() << std::endl;

    // std::cout << "-----------------------------------------------------------------" << std::endl;
    // std::cout << rd2_.J_base_R_ << std::endl;
    // std::cout << "-----------------------------------------------------------------" << std::endl;
    // std::cout << rd2_.J_base_R_kt_ << std::endl;
    // std::cout << "-----------------------------------------------------------------" << std::endl;

    // std::cout << rd2_.lambda_base_R_ << std::endl;
    // std::cout << "-----------------------------------------------------------------" << std::endl;
    // std::cout << rd2_.J_CR_INV_T << std::endl;

    return 0;
}
