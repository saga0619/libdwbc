#ifndef WBHQP_TASK_H
#define WBHQP_TASK_H

#include <Eigen/Dense>
#include "dwbc_wbd.h"
#include "dwbc_math.h"
#include "vector"

using namespace Eigen;

namespace DWBC
{
    enum TASK_TYPE
    {
        TASK_UNDEFINED,
        TASK_LINK,
        TASK_CUSTOM,
        TASK_NONCONTACT_CHAIN,  //Task from non-contact chain
        TASK_CONTACT_CHAIN,     // Task from contact chain
        TASK_CENTROIDAL,
    };

    enum LINK_MODE
    {
        TASK_LINK_6D,
        TASK_LINK_6D_COM_FRAME,
        TASK_LINK_6D_CUSTOM_FRAME,
        TASK_LINK_POSITION,
        TASK_LINK_POSITION_COM_FRAME,
        TASK_LINK_POSITION_CUSTOM_FRAME,
        TASK_LINK_ROTATION,
        TASK_LINK_ROTATION_CUSTOM_FRAME,
    };

    static const char *taskmode_str[] = {"TASK_LINK_6D",
                                         "TASK_LINK_6D_COM_FRAME",
                                         "TASK_LINK_6D_CUSTOM_FRAME",
                                         "TASK_LINK_POSITION",
                                         "TASK_LINK_POSITION_COM_FRAME",
                                         "TASK_LINK_POSITION_CUSTOM_FRAME",
                                         "TASK_LINK_ROTATION",
                                         "TASK_LINK_ROTATION_CUSTOM_FRAME",
                                         "TASK_CENTROIDAL_6D",
                                         "TASK_CENTROIDAL_LINEAR",
                                         "TASK_CENTROIDAL_ANGULAR",
                                         "TASK_CUSTOM",
                                         "TASK_NULL"};

    class TaskLink
    {
    public:
        TaskLink(int task_link_mode, int link_id, const Vector3d &task_point);

        Vector3d task_point_;

        unsigned int link_id_;
        unsigned int task_link_mode_;
        unsigned int t_dof_;

        bool traj_pos_set = false;
        bool traj_rot_set = false;
        bool traj_gain_set = false;

        Vector3d pos_init_;
        Matrix3d rot_init_;

        Vector3d pos_desired_;
        Vector3d vel_deisred_;

        Vector3d vel_init_;
        Vector3d w_init_;

        Matrix3d rot_desired_;
        Vector3d w_desired_;

        Vector3d rpy_traj;

        MatrixXd wr_mat;

        double traj_start_time_;
        double traj_act_time_;
        double traj_end_time_;

        Vector3d pos_p_gain_ = Vector3d::Zero();
        Vector3d pos_d_gain_ = Vector3d::Zero();
        Vector3d pos_a_gain_ = Vector3d::Zero();

        Vector3d rot_p_gain_ = Vector3d::Zero();
        Vector3d rot_d_gain_ = Vector3d::Zero();
        Vector3d rot_a_gain_ = Vector3d::Zero();

        Vector3d pos_traj_;
        Vector3d vel_traj_;
        Vector3d acc_traj_;

        void SetTrajectoryQuintic(double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d vel_init, Eigen::Vector3d pos_desired, Eigen::Vector3d vel_desired);

        void SetTrajectoryRotation(double start_time, double end_time, Eigen::Matrix3d rot_init, Eigen::Vector3d twist_init, Eigen::Matrix3d rot_desired, Eigen::Vector3d twist_desired);

        void SetTaskGain(Vector3d pos_p_gain, Vector3d pos_d_gain_, Vector3d pos_a_gain, Vector3d rot_p_gain_, Vector3d rot_d_gain_, Vector3d rot_a_gain_);

        Vector3d GetFstarPosPD(double current_time, Vector3d current_pos, Vector3d current_vel);

        Vector3d GetFstarRotPD(double current_time, Matrix3d current_vel, Vector3d current_twsit);
    };

    class TaskSpace
    {
    private:
        /* data */
    public:
        unsigned int task_dof_;
        unsigned int heirarchy_;
        unsigned int link_size_; 
        unsigned int task_type_;
        unsigned int nc_heirarchy_;

        std::vector<TaskLink> task_link_;

        MatrixXd J_task_;
        VectorXd f_star_;
        MatrixXd J_kt_;
        MatrixXd Null_task_;
        MatrixXd Lambda_task_;
        VectorXd torque_h_;
        VectorXd torque_null_h_;
        VectorXd torque_null_h_R_;
        VectorXd torque_null_h_nc_;

        bool reduced_task_ = false;
        bool noncont_task = false;
        bool cmm_task = false;

        MatrixXd J_task_R_;
        MatrixXd J_kt_R_;
        MatrixXd Lambda_task_R_;
        MatrixXd Null_task_R_;

        MatrixXd J_task_NC_;
        // MatrixXd J_kt_R_;
        MatrixXd Lambda_task_NC_;
        MatrixXd Null_task_NC_;
        VectorXd torque_h_R_;
        VectorXd torque_nc_;

        MatrixXd J_task_NC_inv_T;
        MatrixXd wr_mat;
        Vector6d force_on_nc_;

        MatrixXd Q_;
        MatrixXd Q_temp_;

        MatrixXd Q_t_inv;

        VectorXd f_star_qp_;
        VectorXd contact_qp_;

        TaskSpace();

        TaskSpace(int task_link_mode, int heirarchy, int task_dof);

        TaskSpace(int task_link_mode, int heirarchy, int link_number, const Vector3d &task_point);
        ~TaskSpace();

        void AddTaskLink(int task_link_mode, int link_number, const Vector3d &task_point);

        void Update(const VectorXd &f_star);

        void Update(const MatrixXd &J_task, const VectorXd &f_star);

        void CalcJKT(const MatrixXd &A_inv, const MatrixXd &N_C, const MatrixXd &W_inv);

        void CalcJKT_R(const MatrixXd &A_inv, const MatrixXd &A_inv_N_C, const MatrixXd &A_R_inv_N_CR, const MatrixXd &W_R_inv, const MatrixXd &J_I_NC_INV_T);

        void CalcNullMatrix(const MatrixXd &A_inv_N_C, const MatrixXd &prev_null);

        void CalcNullMatrix(const MatrixXd &A_inv_N_C);

        void CalcNullMatrix_R(const MatrixXd &A_R_inv_N_CR, const MatrixXd &prev_null);

        void CalcNullMatrix_R(const MatrixXd &A_R_inv_N_CR);

        void GetFstarPosPD(double current_time, Vector3d current_pos, Vector3d current_vel);

        void GetFstarRotPD(double current_time, Matrix3d current_vel, Vector3d current_twsit);

        // void GetFstarPositionalPDF(double current_time, Link);

        // void GetFstarRotationalPDF(double current_time, );
    };
}

#endif