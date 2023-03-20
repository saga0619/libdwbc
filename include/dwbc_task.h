#ifndef WBHQP_TASK_H
#define WBHQP_TASK_H

#include <Eigen/Dense>
#include "wbd.h"
#include "math.h"

using namespace Eigen;

namespace DWBC
{

    enum TASK_MODE
    {
        TASK_LINK_6D,
        TASK_LINK_POSITION,
        TASK_LINK_ROTATION,
        TASK_CUSTOM,
        TASK_COM_POSITION,
        TASK_NULL,
    };

    static const char *taskmode_str[] = {"TASK_LINK_6D", "TASK_LINK_POSITION", "TASK_LINK_ROTATION", "TASK_CUSTOM", "TASK_COM_POSITION"};

    class TaskSpace
    {
    private:
        /* data */
    public:
        unsigned int task_dof_;
        unsigned int heirarchy_;
        unsigned int link_number_;
        unsigned int body_id_;

        unsigned int task_mode_;

        unsigned int model_size_;

        Vector3d task_point_;

        bool traj_pos_set = false;
        bool traj_rot_set = false;

        Vector3d pos_init_;
        Matrix3d rot_init_;

        Vector3d pos_desired_;
        Vector3d vel_deisred_;

        Vector3d vel_init_;
        Vector3d w_init_;

        Matrix3d rot_desired_;
        Vector3d w_desired_;

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

        MatrixXd J_task_;
        VectorXd f_star_;
        MatrixXd J_kt_;
        MatrixXd Null_task_;
        MatrixXd Lambda_task_;
        VectorXd torque_h_;

        MatrixXd Q_;
        MatrixXd Q_temp_;

        MatrixXd Q_t_inv;

        VectorXd f_star_qp_;
        VectorXd contact_qp_;

        TaskSpace();

        TaskSpace(int task_mode, int heirarchy, int task_dof, int model_dof);

        TaskSpace(int task_mode, int heirarchy, int link_number, int link_id, const Vector3d &task_point, int model_dof);
        ~TaskSpace();

        void Update(const VectorXd &f_star);

        void Update(const MatrixXd &J_task, const VectorXd &f_star);

        void CalcJKT(const MatrixXd &A_inv, const MatrixXd &N_C, const MatrixXd &W_inv);

        void CalcNullMatrix(const MatrixXd &A_inv, const MatrixXd &N_C, const MatrixXd &prev_null);

        void CalcNullMatrix(const MatrixXd &A_inv, const MatrixXd &N_C);

        void SetTrajectoryQuintic(double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d vel_init, Eigen::Vector3d pos_desired, Eigen::Vector3d vel_desired);

        void SetTrajectoryRotation(double start_time, double end_time, Eigen::Matrix3d rot_init, Eigen::Vector3d twist_init, Eigen::Matrix3d rot_desired, Eigen::Vector3d twist_desired);

        void SetTaskGain(Vector3d pos_p_gain, Vector3d pos_d_gain_, Vector3d pos_a_gain, Vector3d rot_p_gain_, Vector3d rot_d_gain_, Vector3d rot_a_gain_);

        void GetFstarPosPD(double current_time, Vector3d current_pos, Vector3d current_vel);

        void GetFstarRotPD(double current_time, Matrix3d current_vel, Vector3d current_twsit);

        // void GetFstarPositionalPDF(double current_time, Link);

        // void GetFstarRotationalPDF(double current_time, );
    };
}

#endif