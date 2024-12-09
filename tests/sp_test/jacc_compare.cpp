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
    // double rot_z = M_PI_2;
    int repeat = 1000;

    bool print_torque = false;
    bool print_vw = false;
    bool print_jacc = false;
    bool print_task_time = false;

    bool use_hqp = true;

    std::string urdf2_file = std::string(URDF_DIR) + "/dyros_tocabi.urdf";

    std::string desired_control_target = "COM";
    std::string desired_control_target2 = "pelvis_link";
    std::string desired_control_target3 = "upperbody_link";
    std::string desired_control_target4 = "L_Wrist2_Link";
    std::string desired_control_target5 = "R_Wrist2_Link";
    std::string desired_control_target_lf = "l_ankleroll_link";
    std::string desired_control_target_rf = "r_ankleroll_link";

    // VectorXd fstar_1;
    // fstar_1.setZero(6);
    // fstar_1 << 0.5, 0.3, 0.2, 0.12, -0.11, 0.05; // based on local frmae.

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
    bool verbose = false;

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

    // Add noise to joint coordinate

    // q2 << 0, 0, 0.92983, 0, 0, 0,
    //     0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
    //     0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
    //     0, 0, 0,
    //     0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
    //     0, 0,
    //     -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;

    // bool verbose = false;
    rd2_.UpdateKinematics(q2, q2dot, q2ddot);
    rd2_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.13, 0.06, verbose);
    rd2_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.13, 0.06, verbose);
    rd2_.AddContactConstraint(23, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);
    rd2_.AddContactConstraint(31, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);

    rd2_.AddTaskSpace(0, TASK_LINK_POSITION, desired_control_target.c_str(), Vector3d::Zero(), verbose);
    rd2_.AddTaskSpace(0, TASK_LINK_ROTATION, desired_control_target2.c_str(), Vector3d::Zero(), verbose);

    rd2_.AddTaskSpace(1, TASK_LINK_ROTATION, desired_control_target3.c_str(), Vector3d::Zero(), verbose);

    rd2_.AddTaskSpace(2, TASK_LINK_6D, desired_control_target4.c_str(), Vector3d::Zero(), verbose);
    rd2_.AddTaskSpace(2, TASK_LINK_6D, desired_control_target5.c_str(), Vector3d::Zero(), verbose);
    // rd2_.AddTaskSpace(TASK_CUSTOM, 12);

    bool contact0 = true;
    bool contact1 = true;
    bool contact2 = false;
    bool contact3 = false;

    VectorXd fstar_0(6);
    fstar_0 << 0.4, 2, 0.2, 0.1, -0.1, 0.2;

    VectorXd fstar_1(3);
    fstar_1 << 0.2, 0.1, 0.1;

    VectorXd fstar_2(12);
    fstar_2 << 0.4, 0.3, -0.4, 1, 0.3, 0.2, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;

    rd2_.SetTaskSpace(0, fstar_0);
    rd2_.SetTaskSpace(1, fstar_1);
    rd2_.SetTaskSpace(2, fstar_2);

    {
        RobotData rd_ = rd2_;

        rd_.osqp_contact_ = new OsqpEigen::Solver();

        rd_.SetContact(contact0, contact1, contact2, contact3);
        rd_.CalcContactConstraint();
        rd_.CalcGravCompensation();
        rd_.CalcTaskControlTorque(use_hqp, true);
        rd_.CalcContactRedistribute(use_hqp, true);
        rd_.CalcTaskControlTorque(use_hqp, false);
        rd_.CalcContactRedistribute(use_hqp, false);

        auto t10 = std::chrono::high_resolution_clock::now();

        for (int i = 0; i < repeat; i++)
        {
            rd_.SetContact(contact0, contact1, contact2, contact3);
            rd_.CalcContactConstraint();
            rd_.CalcGravCompensation();
        }
        auto t11 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < repeat; i++)
        {
            rd_.CalcTaskSpace();
        }
        auto t12 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < repeat; i++)
        {
            rd_.CalcTaskControlTorque(use_hqp, false, false);
        }

        auto t13 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < repeat; i++)
        {
            if (!rd_.CalcContactRedistribute(use_hqp, false))
            {
                std::cout << "Contact Redistribution Failed at step : " << i << std::endl;
            }
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

        std::cout << rd_.torque_task_.transpose() << std::endl;
        std::cout << rd_.torque_grav_.transpose() << std::endl;
        std::cout << rd_.torque_contact_.transpose() << std::endl;
        std::cout << "-----------------------------------------------------------------" << std::endl;
    }
    {
        RobotData rd_ = rd2_;

        rd_.SetContact(contact0, contact1, contact2, contact3);
        rd_.ReducedDynamicsCalculate();
        rd_.ReducedCalcContactConstraint();
        rd_.ReducedCalcGravCompensation();
        rd_.ReducedCalcTaskControlTorque(use_hqp, true);
        rd_.ReducedCalcContactRedistribute(use_hqp, true);

        auto t2_0 = std::chrono::high_resolution_clock::now();
        bool init_qp = false;

        for (int i = 0; i < repeat; i++)
        {
            rd_.SetContact(contact0, contact1, contact2, contact3);
            rd_.ReducedDynamicsCalculate();
        }
        auto t2_1 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < repeat; i++)
        {
            rd_.ReducedCalcContactConstraint();
            rd_.ReducedCalcGravCompensation();
        }

        auto t2_2 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < repeat; i++)
        {
            rd_.ReducedCalcTaskSpace();
        }
        auto t2_3 = std::chrono::high_resolution_clock::now();

        for (int i = 0; i < repeat; i++)
        {
            rd_.ReducedCalcTaskControlTorque(use_hqp, init_qp, false);
        }
        auto t2_4 = std::chrono::high_resolution_clock::now();

        for (int i = 0; i < repeat; i++)
        {
            rd_.ReducedCalcContactRedistribute(use_hqp, init_qp);

        }
        auto t2_5 = std::chrono::high_resolution_clock::now();

        double time_original_us2 = std::chrono::duration_cast<std::chrono::microseconds>(t2_5 - t2_0).count();
        double time_original_us21 = std::chrono::duration_cast<std::chrono::microseconds>(t2_1 - t2_0).count();
        double time_original_us22 = std::chrono::duration_cast<std::chrono::microseconds>(t2_2 - t2_1).count();
        double time_original_us23 = std::chrono::duration_cast<std::chrono::microseconds>(t2_3 - t2_2).count();
        double time_original_us24 = std::chrono::duration_cast<std::chrono::microseconds>(t2_4 - t2_3).count();
        double time_original_us25 = std::chrono::duration_cast<std::chrono::microseconds>(t2_5 - t2_4).count();

        std::cout << "Reduced Dynamics Model COMP 2 TOTAL CONSUMPTION : " << (float)(time_original_us2 / repeat) << " us" << std::endl;
        std::cout << "Reduced Dynamics Model 1 - Reduced Dynamics calculation : " << (int)(time_original_us21 / repeat) << " us" << std::endl;
        std::cout << "Reduced Dynamics Model 2 - Contact Constraint Calculation : " << (int)(time_original_us22 / repeat) << " us" << std::endl;
        std::cout << "Reduced Dynamics Model 3 - Task Space Calculation         : " << (int)(time_original_us23 / repeat) << " us" << std::endl;
        std::cout << "Reduced Dynamics Model 4 - Task Torque Calculation        : " << (int)(time_original_us24 / repeat) << " us" << std::endl;
        std::cout << "Reduced Dynamics Model 5 - Contact Redistribution Calcula : " << (int)(time_original_us25 / repeat) << " us" << std::endl;
        std::cout << rd_.torque_task_.transpose() << std::endl;
        std::cout << rd_.torque_grav_.transpose() << std::endl;
        std::cout << rd_.torque_contact_.transpose() << std::endl;
    }

    {
        RobotData rdo_ = rd2_;
        
        rdo_.SetContact(1, 1, 0, 0);
        rdo_.ReducedDynamicsCalculate();
        rdo_.UpdateTaskSpace();

        double time_origin = 0;
        double time_reduce = 0;
        std::cout << std::endl
                  << "--------------------------------------------------------------------------------------------------------" << std::endl;
        std::cout << "                       :: DOUBLE SUPPORT. ORIGINAL LQP :: " << std::endl;
        {
            bool init_qp = true;
            /**
             * hqp settings
             * priority 1 //
             *      inequality constraint : torque limit (2(n-6) dof)
             *
             *      equality constraint   : wbd (n dof)
             * Priority 2 //
             *      inequality constraint :
             *                              joint acceleration limit (2n dof)
             *                              CoP & ZMP limit (8 * c dof)
             *      equality constraint :
             *                              contact constraint (6 * c dof)
             *
             * priority 3 //
             *      equality constraint   : com space (3 dof)
             *                              pelv rotation (3 dof)
             *
             * priority 4 //
             *      equality constraint   : right arm (6 dof)
             */

            RobotData rd_ = rd2_;
            rd_.SetContact(1,1,0,0);
            rd_.UpdateTaskSpace();
            // rd_.ReducedDynamicsCalculate();


            DWBC::HQP hqp_;
            rd_.ConfigureLQP(hqp_);

            // hqp_.prepare();
            double time_step_max = 0;
            double time_step = 0;

            double time_cum = 0;
            double update_time_cum = 0;
            double solve_time_cum = 0;
            // double chrono_time_cum = 0;
            auto t0 = std::chrono::high_resolution_clock::now();
            rd_.CalcControlTorqueLQP(hqp_, true);
            for (int i = 0; i < repeat; i++)
            {
                rd_.ConfigureLQP(hqp_, false);
                rd_.CalcControlTorqueLQP(hqp_, false);

                time_cum += hqp_.total_time_step_;
                update_time_cum += hqp_.update_time_step_;
                solve_time_cum += hqp_.solve_time_step_;
            }
            auto t1 = std::chrono::high_resolution_clock::now();
            time_origin = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / repeat;
            std::cout << std::setw(30) << std::right << "TOTAL CONSUMPTION : " << std::fixed << std::setw(8) << std::setprecision(2) << time_origin << " us (" << hqp_.total_time_max_ << " us)" << std::endl;

            for (int i = 0; i < hqp_.hqp_hs_.size(); i++)
            {
                //     VectorXd jacc = hqp_.hqp_hs_[i].y_ans_.head(acc_size);
                //     VectorXd conf = hqp_.hqp_hs_[i].y_ans_.tail(contact_size);
                //     VectorXd torque = rd_.A_.bottomRows(torque_size) * jacc + rd_.J_C.transpose().bottomRows(torque_size) * conf + rd_.B_.tail(torque_size);
                //     std::cout << "jacc qp : " << jacc.transpose() << std::endl;
                //     std::cout << "torque qp : " << torque.transpose() << std::endl;
                //     std::cout << "contact qp : " << conf.transpose() << std::endl;

                // if (print_task_time)
                // {
                //     std::cout << "task " << i << " time : " << hqp_.hqp_hs_[i].qp_solve_time_ / repeat << " us" << std::endl;
                // }
                if (print_vw)
                {
                    std::cout << "v ans : " << hqp_.hqp_hs_[i].v_ans_.transpose() << std::endl;
                    std::cout << "w ans : " << hqp_.hqp_hs_[i].w_ans_.transpose() << std::endl;
                }
            }
            std::cout << std::setw(30) << std::right << " Internal   update time : " << std::fixed << std::setw(8) << std::setprecision(2) << (float)((update_time_cum) / repeat) << " us (" << hqp_.update_time_max_ << ")" << std::endl;
            std::cout << std::setw(30) << std::right << " Internal QP solve time : " << std::fixed << std::setw(8) << std::setprecision(2) << (float)((solve_time_cum) / repeat) << " us (" << hqp_.solve_time_max_ << ")" << std::endl;

            VectorXd jacc = hqp_.hqp_hs_.back().y_ans_.head(hqp_.acceleration_size_);
            VectorXd conf = hqp_.hqp_hs_.back().y_ans_.tail(hqp_.contact_size_);
            VectorXd torque = rd_.A_.bottomRows(rd_.model_dof_) * jacc + rd_.J_C.transpose().bottomRows(rd_.model_dof_) * conf + rd_.B_.tail(rd_.model_dof_);

            if (print_torque)
            {
                std::cout << "jacc qp : " << jacc.transpose() << std::endl;
                std::cout << "torque qp : " << torque.transpose() << std::endl;
                std::cout << "contact qp : " << conf.transpose() << std::endl;
            }
            // VectorXd jacc = hqp_.hqp_hs_[2].y_ans_.head(acc_size);
            // std::cout << "h2 task fstar : " << (J_task * hqp_.hqp_hs_[2].y_ans_.head(acc_size)).transpose() << " original : " << f_star.transpose() << std::endl;
            // std::cout << "h3 task fstar : " << (rd_.ts_[2].J_task_ * hqp_.hqp_hs_[3].y_ans_.head(acc_size)).transpose() << " original : " << rd_.ts_[2].f_star_.transpose() << std::endl;
        }
        std::cout << std::endl
                  << "--------------------------------------------------------------------------------------------------------" << std::endl;

        std::cout << "                       :: DOUBLE SUPPORT. REDUCED LQP W COST ::" << std::endl;
        {
            // RobotData rd_ = rdo_;
            RobotData rd_ = rd2_;
            bool init_qp = true;
            /**
             * hqp settings
             * priority 1 //
             *      inequality constraint : torque limit (2(n-6) dof)
             *
             *      equality constraint   : wbd (n dof)
             * Priority 2 //
             *      inequality constraint :
             *                              joint acceleration limit (2n dof)
             *                              CoP & ZMP limit (8 * c dof)
             *      equality constraint :
             *                              contact constraint (6 * c dof)
             *
             * priority 3 //
             *      equality constraint   : com space (3 dof)
             *                              pelv rotation (3 dof)
             *
             * priority 4 //
             *      equality constraint   : right arm (3 dof)
             */
            DWBC::HQP hqp_;
            rd_.ts_[1].noncont_task = true;
            rd_.ts_[2].noncont_task = true;

            rd_.SetContact(1, 1, 0, 0);

            rd_.UpdateTaskSpace();

            double time_redu_calc = 0;
            double time_redu_calc_max = 0;

            auto t0 = std::chrono::high_resolution_clock::now();

            for (int i = 0; i < repeat; i++)
            {
                auto t2_00 = std::chrono::high_resolution_clock::now();
                rd_.ReducedDynamicsCalculate();
                auto t2_0 = std::chrono::high_resolution_clock::now();

                double time_calc = std::chrono::duration_cast<std::chrono::microseconds>(t2_0 - t2_00).count();
                time_redu_calc += time_calc;
                time_redu_calc_max = std::max(time_redu_calc_max, time_calc);
            }

            rd_.ConfigureLQP_R(hqp_, true);
            rd_.CalcControlTorqueLQP_R(hqp_, true);

            HQP hqp_nc_;
            VectorXd jacc = hqp_.hqp_hs_.back().y_ans_.head(hqp_.acceleration_size_);
            rd_.ConfigureLQP_R_NC(hqp_nc_, jacc, true);
            rd_.CalcControlTorqueLQP_R_NC(hqp_nc_, true);

            double time_total = 0;
            double time_update = 0;
            double time_solve = 0;

            double time_total_nc = 0;
            double time_update_nc = 0;
            double time_solve_nc = 0;

            double total_max = 0;
            for (int i = 0; i < repeat; i++)
            {
                rd_.SetContact(1, 1);
                rd_.ReducedDynamicsCalculate();
                rd_.UpdateTaskSpace();

                rd_.ConfigureLQP_R(hqp_, false);
                rd_.CalcControlTorqueLQP_R(hqp_, false);
                time_total += hqp_.total_time_step_;
                time_update += hqp_.update_time_step_;
                time_solve += hqp_.solve_time_step_;

                rd_.ConfigureLQP_R_NC(hqp_nc_, jacc, false);
                rd_.CalcControlTorqueLQP_R_NC(hqp_nc_, false);
                time_total_nc += hqp_nc_.total_time_step_;
                time_update_nc += hqp_nc_.update_time_step_;
                time_solve_nc += hqp_nc_.solve_time_step_;

                total_max = std::max(total_max, hqp_nc_.total_time_step_ + hqp_.total_time_step_);
            }

            auto t1 = std::chrono::high_resolution_clock::now();

            // double time_original_us2 = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
            time_reduce = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / repeat;
            std::cout << std::setw(30) << std::right << "TOTAL CONSUMPTION : " << std::fixed << std::setw(8) << std::setprecision(2) << time_reduce << " us (" << total_max << " us)" << std::endl;

            std::cout << std::setw(30) << std::right << "Reduced calc : " << std::fixed << std::setw(8) << std::setprecision(2) << (float)((time_redu_calc) / repeat) << " us (" << time_redu_calc_max << " us)" << std::endl;
            std::cout << std::setw(30) << std::right << "       LQP 1 : " << std::fixed << std::setw(8) << std::setprecision(2) << (float)((time_total) / repeat) << " us (" << hqp_.total_time_max_ << " us)" << std::endl;
            std::cout << std::setw(30) << std::right << "       LQP 2 : " << std::fixed << std::setw(8) << std::setprecision(2) << (float)((time_total_nc) / repeat) << " us (" << hqp_nc_.total_time_max_ << " us)" << std::endl;
            // std::cout << "Reduced Dynamics Model HERZOG 2 NC : " << (float)((time_original_us4) / repeat) << " us" << std::endl;

            if (print_task_time || print_vw)
            {
                std::cout << "LQP 1 : ";
            }
            for (int i = 0; i < hqp_.hqp_hs_.size(); i++)
            {
                // if (print_task_time)
                // {
                //     std::cout << "task " << i << " time : " << hqp_.hqp_hs_[i].qp_solve_time_ / repeat << " us" << std::endl;
                // }
                if (print_vw)
                {
                    std::cout << "v ans : " << hqp_.hqp_hs_[i].v_ans_.transpose() << std::endl;
                    std::cout << "w ans : " << hqp_.hqp_hs_[i].w_ans_.transpose() << std::endl;
                }
            }
            if (print_task_time || print_vw)
                std::cout << std::endl;

            VectorXd conf = hqp_.hqp_hs_.back().y_ans_.tail(hqp_.contact_size_);
            VectorXd torque_lqp1 = rd_.A_R.bottomRows(rd_.reduced_model_dof_) * jacc + rd_.J_CR.transpose().bottomRows(rd_.reduced_model_dof_) * conf + rd_.G_R.tail(rd_.reduced_model_dof_);
            VectorXd torque_control = VectorXd::Zero(rd_.model_dof_);

            torque_control.head(rd_.co_dof) = torque_lqp1.head(rd_.co_dof);

            if (print_task_time || print_vw)
                std::cout << "LQP 2 : ";
            for (int i = 0; i < hqp_nc_.hqp_hs_.size(); i++)
            {
                // if (print_task_time)
                // {
                //     std::cout << " | task " << i << " time : " << hqp_.hqp_hs_[i].qp_solve_time_ / repeat;
                // }
                if (print_vw)
                {
                    std::cout << "v ans : " << hqp_nc_.hqp_hs_[i].v_ans_.transpose() << std::endl;
                    std::cout << "w ans : " << hqp_nc_.hqp_hs_[i].w_ans_.transpose() << std::endl;
                }
            }
            if (print_task_time || print_vw)
                std::cout << std::endl;

            VectorXd jacc_2 = hqp_nc_.hqp_hs_.back().y_ans_.head(hqp_nc_.acceleration_size_);
            VectorXd torque_lqp2 = rd_.A_NC.bottomRightCorner(hqp_nc_.acceleration_size_, hqp_nc_.acceleration_size_) * jacc_2 + rd_.G_NC;

            torque_control.tail(rd_.nc_dof) = torque_lqp2;

            std::cout << std::setw(30) << std::right << "  LQP 1 INT |   update time : " << std::fixed << std::setw(8) << std::setprecision(2) << (float)((time_update) / repeat) << " us (" << hqp_.update_time_max_ << " us)" << std::endl;
            std::cout << std::setw(30) << std::right << "         | QP solve time : " << std::fixed << std::setw(8) << std::setprecision(2) << (float)((time_solve) / repeat) << " us (" << hqp_.solve_time_max_ << " us)" << std::endl;
            std::cout << std::setw(30) << std::right << "  LQP 2 INT |   update time : " << std::fixed << std::setw(8) << std::setprecision(2) << (float)((time_update_nc) / repeat) << " us (" << hqp_nc_.update_time_max_ << " us)" << std::endl;
            std::cout << std::setw(30) << std::right << "         | QP solve time : " << std::fixed << std::setw(8) << std::setprecision(2) << (float)((time_update_nc) / repeat) << " us (" << hqp_nc_.solve_time_max_ << " us)" << std::endl;

            if (print_torque)
            {
                std::cout << "Torque control : " << torque_control.transpose() << std::endl;
                std::cout << "Contact force : " << conf.transpose() << std::endl;
            }
        }
        std::cout << " Efficieny : " << (float)(time_origin / time_reduce) << "X" << std::endl;
    }
    // {
    //     RobotData rdo_;
    //     rdo_.LoadModelData(urdf2_file, true, false);

    //     rdo_.UpdateKinematics(q2, q2dot, q2ddot);
    //     rdo_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.13, 0.06, verbose);
    //     rdo_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.13, 0.06, verbose);
    //     rdo_.AddContactConstraint(23, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);
    //     rdo_.AddContactConstraint(31, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);

    //     rdo_.AddTaskSpace(0, TASK_LINK_POSITION, desired_control_target.c_str(), Vector3d::Zero(), verbose);
    //     rdo_.AddTaskSpace(0, TASK_LINK_ROTATION, desired_control_target2.c_str(), Vector3d::Zero(), verbose);
    //     rdo_.AddTaskSpace(1, TASK_LINK_6D, desired_control_target_rf.c_str(), Vector3d::Zero(), verbose);
    //     rdo_.AddTaskSpace(1, TASK_LINK_6D, desired_control_target4.c_str(), Vector3d::Zero(), verbose);

    //     rdo_.SetContact(true, false, false, false);
    //     rdo_.CalcContactConstraint();

    //     rdo_.ReducedDynamicsCalculate();

    //     Vector6d fstar_01;
    //     fstar_01.topRows(3) = fstar_0;
    //     fstar_01.bottomRows(3) = fstar_1;

    //     Vector12d fstar_22;
    //     fstar_22.head(6) = fstar_2;
    //     fstar_22.tail(6) = fstar_2;

    //     rdo_.SetTaskSpace(0, fstar_01);
    //     rdo_.SetTaskSpace(1, fstar_22);

    //     rdo_.ts_[1].noncont_task = true;
    //     // rdo_.ts_[2].noncont_task = true;

    //     rdo_.UpdateTaskSpace();

    //     double time_origin = 0;
    //     double time_reduce = 0;

    //     std::cout << std::endl
    //               << "--------------------------------------------------------------------------------------------------------" << std::endl;
    //     std::cout << "                       :: SINGLE SUPPORT. ORIGINAL LQP ::" << std::endl;
    //     {
    //         bool init_qp = true;
    //         /**
    //          * hqp settings
    //          * priority 1 //
    //          *      inequality constraint : torque limit (2(n-6) dof)
    //          *
    //          *      equality constraint   : wbd (n dof)
    //          * Priority 2 //
    //          *      inequality constraint :
    //          *                              joint acceleration limit (2n dof)
    //          *                              CoP & ZMP limit (8 * c dof)
    //          *      equality constraint :
    //          *                              contact constraint (6 * c dof)
    //          *
    //          * priority 3 //
    //          *      equality constraint   : com space (3 dof)
    //          *                              pelv rotation (3 dof)
    //          *
    //          * priority 4 //
    //          *      equality constraint   : right arm (6 dof)
    //          */

    //         RobotData rd_ = rdo_;

    //         DWBC::HQP hqp_;
    //         rd_.ConfigureLQP(hqp_);

    //         double time_step_max = 0;
    //         double time_step = 0;

    //         double time_cum = 0;
    //         double update_time_cum = 0;
    //         double solve_time_cum = 0;
    //         // double chrono_time_cum = 0;
    //         auto t0 = std::chrono::high_resolution_clock::now();
    //         rd_.CalcControlTorqueLQP(hqp_, true);
    //         for (int i = 0; i < repeat; i++)
    //         {
    //             rd_.CalcControlTorqueLQP(hqp_, false);

    //             time_cum += hqp_.total_time_step_;
    //             update_time_cum += hqp_.update_time_step_;
    //             solve_time_cum += hqp_.solve_time_step_;
    //         }

    //         auto t1 = std::chrono::high_resolution_clock::now();

    //         time_origin = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / repeat;

    //         std::cout << std::setw(30) << std::right << "TOTAL CONSUMPTION : " << std::fixed << std::setw(8) << std::setprecision(2) << time_origin << " us (" << hqp_.total_time_max_ << " us)" << std::endl;

    //         for (int i = 0; i < hqp_.hqp_hs_.size(); i++)
    //         {
    //             // if (print_task_time)
    //             // std::cout << "task " << i << " time : " << hqp_.hqp_hs_[i].qp_solve_time_ / repeat << " us" << std::endl;
    //             //     VectorXd jacc = hqp_.hqp_hs_[i].y_ans_.head(acc_size);
    //             //     VectorXd conf = hqp_.hqp_hs_[i].y_ans_.tail(contact_size);
    //             //     VectorXd torque = rd_.A_.bottomRows(torque_size) * jacc + rd_.J_C.transpose().bottomRows(torque_size) * conf + rd_.B_.tail(torque_size);
    //             //     std::cout << "jacc qp : " << jacc.transpose() << std::endl;
    //             //     std::cout << "torque qp : " << torque.transpose() << std::endl;
    //             //     std::cout << "contact qp : " << conf.transpose() << std::endl;
    //             if (print_vw)
    //             {
    //                 std::cout << "v ans : " << hqp_.hqp_hs_[i].v_ans_.transpose() << std::endl;
    //                 std::cout << "w ans : " << hqp_.hqp_hs_[i].w_ans_.transpose() << std::endl;
    //             }
    //         }
    //         // std::cout << " total update time : " << total_update_time / repeat << " us" << std::endl;
    //         // std::cout << " Mat calc time : " << (total_comp_time + total_update_time) / repeat << " us" << std::endl;
    //         // std::cout << " QP solve time : " << (total_comp_time + total_update_time) / repeat << " us" << std::endl;
    //         std::cout << std::setw(30) << std::right << " Internal   update time : " << std::fixed << std::setw(8) << std::setprecision(2) << (float)((update_time_cum) / repeat) << " us (" << hqp_.update_time_max_ << ")" << std::endl;
    //         std::cout << std::setw(30) << std::right << " Internal QP solve time : " << std::fixed << std::setw(8) << std::setprecision(2) << (float)((solve_time_cum) / repeat) << " us (" << hqp_.solve_time_max_ << ")" << std::endl;

    //         VectorXd jacc = hqp_.hqp_hs_.back().y_ans_.head(hqp_.acceleration_size_);
    //         VectorXd conf = hqp_.hqp_hs_.back().y_ans_.tail(hqp_.contact_size_);
    //         VectorXd torque = rd_.A_.bottomRows(rd_.model_dof_) * jacc + rd_.J_C.transpose().bottomRows(rd_.model_dof_) * conf + rd_.B_.tail(rd_.model_dof_);
    //         if (print_torque)
    //         {
    //             std::cout << "jacc qp : " << jacc.transpose() << std::endl;
    //             std::cout << "torque qp : " << torque.transpose() << std::endl;
    //             std::cout << "contact qp : " << conf.transpose() << std::endl;
    //         }

    //         // VectorXd jacc = hqp_.hqp_hs_[2].y_ans_.head(acc_size);
    //         // std::cout << "h2 task fstar : " << (J_task * hqp_.hqp_hs_[2].y_ans_.head(acc_size)).transpose() << " original : " << f_star.transpose() << std::endl;
    //         // std::cout << "h3 task fstar : " << (rd_.ts_[2].J_task_ * hqp_.hqp_hs_[3].y_ans_.head(acc_size)).transpose() << " original : " << rd_.ts_[2].f_star_.transpose() << std::endl;
    //     }

    //     std::cout << std::endl
    //               << "--------------------------------------------------------------------------------------------------------" << std::endl;
    //     std::cout << "                       :: SINGLE SUPPORT. REDUCED LQP W COST :: " << std::endl;
    //     {

    //         RobotData rd_ = rdo_;
    //         bool init_qp = true;
    //         /**
    //          * hqp settings
    //          * priority 1 //
    //          *      inequality constraint : torque limit (2(n-6) dof)
    //          *
    //          *      equality constraint   : wbd (n dof)
    //          * Priority 2 //
    //          *      inequality constraint :
    //          *                              joint acceleration limit (2n dof)
    //          *                              CoP & ZMP limit (8 * c dof)
    //          *      equality constraint :
    //          *                              contact constraint (6 * c dof)
    //          *
    //          * priority 3 //
    //          *      equality constraint   : com space (3 dof)
    //          *                              pelv rotation (3 dof)
    //          *
    //          * priority 4 //
    //          *      equality constraint   : right arm (3 dof)
    //          */
    //         DWBC::HQP hqp_;
    //         rd_.ts_[1].noncont_task = true;

    //         rd_.ConfigureLQP_R(hqp_);

    //         rd_.SetContact(1, 1, 0, 0);

    //         rd_.UpdateTaskSpace();

    //         double time_redu_calc = 0;
    //         double time_redu_calc_max = 0;

    //         auto t0 = std::chrono::high_resolution_clock::now();

    //         for (int i = 0; i < repeat; i++)
    //         {
    //             auto t2_00 = std::chrono::high_resolution_clock::now();
    //             rd_.ReducedDynamicsCalculate();
    //             auto t2_0 = std::chrono::high_resolution_clock::now();

    //             double time_calc = std::chrono::duration_cast<std::chrono::microseconds>(t2_0 - t2_00).count();
    //             time_redu_calc += time_calc;
    //             time_redu_calc_max = std::max(time_redu_calc_max, time_calc);
    //         }

    //         rd_.CalcControlTorqueLQP_R(hqp_, true);

    //         HQP hqp_nc_;
    //         VectorXd jacc = hqp_.hqp_hs_.back().y_ans_.head(hqp_.acceleration_size_);
    //         rd_.ConfigureLQP_R_NC(hqp_nc_, jacc);
    //         rd_.CalcControlTorqueLQP_R_NC(hqp_nc_, true);

    //         double time_total = 0;
    //         double time_update = 0;
    //         double time_solve = 0;

    //         double time_total_nc = 0;
    //         double time_update_nc = 0;
    //         double time_solve_nc = 0;

    //         double total_max = 0;
    //         for (int i = 0; i < repeat; i++)
    //         {
    //             rd_.CalcControlTorqueLQP_R(hqp_, false);
    //             time_total += hqp_.total_time_step_;
    //             time_update += hqp_.update_time_step_;
    //             time_solve += hqp_.solve_time_step_;

    //             rd_.CalcControlTorqueLQP_R_NC(hqp_nc_, false);
    //             time_total_nc += hqp_nc_.total_time_step_;
    //             time_update_nc += hqp_nc_.update_time_step_;
    //             time_solve_nc += hqp_nc_.solve_time_step_;

    //             total_max = std::max(total_max, hqp_nc_.total_time_step_ + hqp_.total_time_step_);
    //         }

    //         auto t1 = std::chrono::high_resolution_clock::now();

    //         time_reduce = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() / repeat;

    //         // double time_original_us2 = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

    //         std::cout << std::setw(30) << std::right << "TOTAL CONSUMPTION : " << std::fixed << std::setw(8) << std::setprecision(2) << time_reduce << " us (" << total_max << " us)" << std::endl;

    //         std::cout << std::setw(30) << std::right << "Reduced calc : " << std::fixed << std::setw(8) << std::setprecision(2) << (float)((time_redu_calc) / repeat) << " us (" << time_redu_calc_max << " us)" << std::endl;
    //         std::cout << std::setw(30) << std::right << "       LQP 1 : " << std::fixed << std::setw(8) << std::setprecision(2) << (float)((time_total) / repeat) << " us (" << hqp_.total_time_max_ << " us)" << std::endl;
    //         std::cout << std::setw(30) << std::right << "       LQP 2 : " << std::fixed << std::setw(8) << std::setprecision(2) << (float)((time_total_nc) / repeat) << " us (" << hqp_nc_.total_time_max_ << " us)" << std::endl;
    //         // std::cout << "Reduced Dynamics Model HERZOG 2 NC : " << (float)((time_original_us4) / repeat) << " us" << std::endl;

    //         if (print_task_time || print_vw)
    //         {
    //             std::cout << "LQP 1 : ";
    //         }
    //         for (int i = 0; i < hqp_.hqp_hs_.size(); i++)
    //         {
    //             // if (print_task_time)
    //             // {
    //             //     std::cout << "task " << i << " time : " << hqp_.hqp_hs_[i].qp_solve_time_ / repeat << " us" << std::endl;
    //             // }
    //             if (print_vw)
    //             {
    //                 std::cout << "v ans : " << hqp_.hqp_hs_[i].v_ans_.transpose() << std::endl;
    //                 std::cout << "w ans : " << hqp_.hqp_hs_[i].w_ans_.transpose() << std::endl;
    //             }
    //         }
    //         if (print_task_time || print_vw)
    //             std::cout << std::endl;

    //         VectorXd conf = hqp_.hqp_hs_.back().y_ans_.tail(hqp_.contact_size_);
    //         VectorXd torque_lqp1 = rd_.A_R.bottomRows(rd_.reduced_model_dof_) * jacc + rd_.J_CR.transpose().bottomRows(rd_.reduced_model_dof_) * conf + rd_.G_R.tail(rd_.reduced_model_dof_);
    //         VectorXd torque_control = VectorXd::Zero(rd_.model_dof_);

    //         torque_control.head(rd_.co_dof) = torque_lqp1.head(rd_.co_dof);

    //         if (print_task_time || print_vw)
    //             std::cout << "LQP 2 : ";
    //         for (int i = 0; i < hqp_nc_.hqp_hs_.size(); i++)
    //         {
    //             // if (print_task_time)
    //             // {
    //             //     std::cout << " | task " << i << " time : " << hqp_.hqp_hs_[i].qp_solve_time_ / repeat;
    //             // }
    //             if (print_vw)
    //             {
    //                 std::cout << "v ans : " << hqp_nc_.hqp_hs_[i].v_ans_.transpose() << std::endl;
    //                 std::cout << "w ans : " << hqp_nc_.hqp_hs_[i].w_ans_.transpose() << std::endl;
    //             }
    //         }
    //         if (print_task_time || print_vw)
    //             std::cout << std::endl;

    //         VectorXd jacc_2 = hqp_nc_.hqp_hs_.back().y_ans_.head(hqp_nc_.acceleration_size_);
    //         VectorXd torque_lqp2 = rd_.A_NC.bottomRightCorner(hqp_nc_.acceleration_size_, hqp_nc_.acceleration_size_) * jacc_2 + rd_.G_NC;

    //         torque_control.tail(rd_.nc_dof) = torque_lqp2;

    //         std::cout << std::setw(30) << std::right << "  LQP 1 INT |   update time : " << std::fixed << std::setw(8) << std::setprecision(2) << (float)((time_update) / repeat) << " us (" << hqp_.update_time_max_ << " us)" << std::endl;
    //         std::cout << std::setw(30) << std::right << "         | QP solve time : " << std::fixed << std::setw(8) << std::setprecision(2) << (float)((time_solve) / repeat) << " us (" << hqp_.solve_time_max_ << " us)" << std::endl;
    //         std::cout << std::setw(30) << std::right << "  LQP 2 INT |   update time : " << std::fixed << std::setw(8) << std::setprecision(2) << (float)((time_update_nc) / repeat) << " us (" << hqp_nc_.update_time_max_ << " us)" << std::endl;
    //         std::cout << std::setw(30) << std::right << "         | QP solve time : " << std::fixed << std::setw(8) << std::setprecision(2) << (float)((time_update_nc) / repeat) << " us (" << hqp_nc_.solve_time_max_ << " us)" << std::endl;

    //         if (print_torque)
    //         {
    //             std::cout << "Torque control : " << torque_control.transpose() << std::endl;
    //             std::cout << "Contact force : " << conf.transpose() << std::endl;
    //         }
    //     }
    //     std::cout << " Efficieny : " << (float)(time_origin / time_reduce) << "X" << std::endl;
    // }
    return 0;
}
