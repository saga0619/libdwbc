#include "dwbc_task.h"
namespace DWBC
{
    TaskSpace::TaskSpace()
    {
        task_mode_ = TASK_NULL;
    }

    TaskSpace::TaskSpace(int task_mode, int heirarchy, int task_dof)
    {
        task_dof_ = task_dof;

        task_mode_ = task_mode;

        heirarchy_ = heirarchy;
    }

    // Typical pos/rot task initialzer
    TaskSpace::TaskSpace(int task_mode, int heirarchy, int link_number, int body_id, const Vector3d &task_point)
    {
        task_mode_ = task_mode;

        heirarchy_ = heirarchy;

        link_id_ = link_number; // vector number

        // body_id_ = body_id; // rbdl body id

        if (task_mode == TASK_LINK_6D)
        {
            task_dof_ = 6;

            task_point_ = task_point;
        }
        else if (task_mode == TASK_LINK_POSITION || task_mode == TASK_LINK_ROTATION)
        {
            task_dof_ = 3;

            task_point_ = task_point;
        }
        else if (task_mode == TASK_COM_POSITION)
        {
            task_dof_ = 3;
            
            task_point_ = task_point;
        }
        f_star_.setZero(task_dof_);
    }
    TaskSpace::~TaskSpace() {}

    void TaskSpace::Update(const VectorXd &f_star)
    {
        f_star_ = f_star;
    }

    void TaskSpace::Update(const MatrixXd &J_task, const VectorXd &f_star)
    {
        J_task_ = J_task;
        f_star_ = f_star;
    }

    void TaskSpace::CalcJKT(const MatrixXd &A_inv, const MatrixXd &N_C, const MatrixXd &W_inv)
    {
        CalculateJKT(J_task_, A_inv, N_C, W_inv, J_kt_, Lambda_task_);
    }

    void TaskSpace::CalcNullMatrix(const MatrixXd &A_inv, const MatrixXd &N_C, const MatrixXd &prev_null)
    {
        CalculateTaskNullSpace(J_kt_, Lambda_task_, J_task_, A_inv, N_C, prev_null, Null_task_);
    }

    void TaskSpace::CalcNullMatrix(const MatrixXd &A_inv, const MatrixXd &N_C)
    {
        int model_size = J_task_.cols() - 6;
        CalculateTaskNullSpace(J_kt_, Lambda_task_, J_task_, A_inv, N_C, MatrixXd::Identity(model_size, model_size), Null_task_);
    }

    void TaskSpace::SetTrajectoryQuintic(double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d vel_init, Eigen::Vector3d pos_desired, Eigen::Vector3d vel_desired)
    {

        traj_start_time_ = start_time;
        traj_end_time_ = end_time;

        traj_act_time_ = end_time - start_time;

        pos_init_ = pos_init;
        vel_init_ = vel_init;

        pos_desired_ = pos_desired;
        vel_deisred_ = vel_desired;

        traj_pos_set = true;
    }
    void TaskSpace::SetTrajectoryRotation(double start_time, double end_time, Eigen::Matrix3d rot_init, Eigen::Vector3d twist_init, Eigen::Matrix3d rot_desired, Eigen::Vector3d twist_desired)
    {
        traj_start_time_ = start_time;
        traj_end_time_ = end_time;

        traj_act_time_ = end_time - start_time;

        rot_init_ = rot_init;
        w_init_ = twist_init;

        rot_desired_ = rot_desired;
        w_desired_ = twist_desired;

        traj_pos_set = true;
    }

    void TaskSpace::SetTaskGain(Vector3d pos_p_gain, Vector3d pos_d_gain, Vector3d pos_a_gain, Vector3d rot_p_gain, Vector3d rot_d_gain, Vector3d rot_a_gain)
    {
        pos_p_gain_ = pos_p_gain;
        pos_d_gain_ = pos_d_gain;
        pos_a_gain_ = pos_a_gain;

        rot_p_gain_ = rot_p_gain;
        rot_d_gain_ = rot_d_gain;
        rot_a_gain_ = rot_a_gain;

        traj_rot_set = true;
    }

    void TaskSpace::GetFstarPosPD(double current_time, Vector3d current_pos, Vector3d current_vel)
    {
        if (task_mode_ == TASK_LINK_6D || task_mode_ == TASK_LINK_POSITION || task_mode_ == TASK_COM_POSITION)
        {
            for (int j = 0; j < 3; j++)
            {
                Eigen::Vector3d quintic = QuinticSpline(current_time, traj_start_time_, traj_end_time_, pos_init_(j), vel_init_(j), 0, pos_desired_(j), vel_deisred_(j), 0);

                pos_traj_(j) = quintic(0);
                vel_traj_(j) = quintic(1);
                acc_traj_(j) = quintic(2);
            }
            f_star_ = pos_a_gain_.cwiseProduct(acc_traj_) + pos_p_gain_.cwiseProduct(pos_traj_ - current_pos) + pos_d_gain_.cwiseProduct(vel_traj_ - current_vel);
        }
    }

    void TaskSpace::GetFstarRotPD(double current_time, Matrix3d current_rot, Vector3d current_w)
    {
        int fstar_start_idx;
        if (task_mode_ == TASK_LINK_6D)
        {
            fstar_start_idx = 3;
        }
        else if (task_mode_ == TASK_LINK_ROTATION)
        {
            fstar_start_idx = 0;
        }

        MatrixXd rot_traj;
        Vector3d w_traj, a_traj, qs_;

        Eigen::Quaterniond rq_init, rq_desired, rq_traj;
        qs_ = QuinticSpline(current_time, traj_start_time_, traj_end_time_, 0, 0, 0, 1, 0, 0);

        rq_init = Quaterniond(rot_init_);
        rq_desired = Quaterniond(rot_desired_);
        rq_traj = rq_init.slerp(qs_(0), rq_desired);

        rot_traj = rq_traj.toRotationMatrix();

        Eigen::AngleAxisd axd(rq_desired * rq_init.inverse());

        w_traj = axd.angle() * qs_(1) * axd.axis();
        a_traj = axd.angle() * qs_(2) * axd.axis();

        f_star_.segment(fstar_start_idx, 3) = rot_p_gain_.cwiseProduct(GetPhi(current_rot, rot_traj)) + rot_d_gain_.cwiseProduct(w_traj - current_w);
    }
}