#include "dwbc_task.h"
#include "iostream"
namespace DWBC
{

    TaskLink::TaskLink(int task_link_mode, int link_id, const Vector3d &task_point)
    {
        task_link_mode_ = task_link_mode;
        link_id_ = link_id;
        task_point_ = task_point;

        switch (task_link_mode_)
        {
        case TASK_LINK_6D:
        case TASK_LINK_6D_COM_FRAME:
        case TASK_LINK_6D_CUSTOM_FRAME:
            t_dof_ = 6;
            break;
        case TASK_LINK_POSITION:
        case TASK_LINK_POSITION_COM_FRAME:
        case TASK_LINK_POSITION_CUSTOM_FRAME:
        case TASK_LINK_ROTATION:
        case TASK_LINK_ROTATION_CUSTOM_FRAME:
            t_dof_ = 3;
            break;
        default:
            break;
        }
    }

    TaskSpace::TaskSpace()
    {
        nc_heirarchy_ = 0;
        link_size_ = 0;
        task_type_ = TASK_UNDEFINED;
    }

    TaskSpace::TaskSpace(int task_type, int heirarchy, int task_dof)
    {
        task_type_ = task_type;
        task_dof_ = task_dof;
        link_size_ = 0;
        heirarchy_ = heirarchy;
        nc_heirarchy_ = 0;
        
    }

    // Typical pos/rot task initialzer
    // For a single Link
    TaskSpace::TaskSpace(int task_mode, int heirarchy, int link_number, const Vector3d &task_point)
    {
        task_type_ = TASK_UNDEFINED;
        heirarchy_ = heirarchy;
        nc_heirarchy_ = 0;
        task_link_.push_back(TaskLink(task_mode, link_number, task_point));

        link_size_ = 1;

        task_dof_ = task_link_[0].t_dof_;

        f_star_.setZero(task_dof_);
    }

    void TaskSpace::AddTaskLink(int task_mode, int link_number, const Vector3d &task_point)
    {
        task_link_.push_back(TaskLink(task_mode, link_number, task_point));

        task_dof_ = task_dof_ + task_link_.back().t_dof_;

        link_size_++;

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

    void TaskSpace::CalcJKT_R(const MatrixXd &A_inv, const MatrixXd &A_inv_N_C, const MatrixXd &A_R_inv_N_CR, const MatrixXd &W_R_inv, const MatrixXd &J_I_NC_INV_T)
    {
        int sys_dof = J_task_.cols();
        int r_sys_dof = A_R_inv_N_CR.rows();
        int nc_dof = J_I_NC_INV_T.cols();
        int vc_dof = sys_dof - nc_dof;
        J_task_R_.setZero(task_dof_, r_sys_dof);
        Lambda_task_R_.setZero(task_dof_, task_dof_);
        J_kt_R_.setZero(r_sys_dof - 6, task_dof_);
        force_on_nc_.setZero();

        if (cmm_task)
        {

            J_task_R_.block(0, 0, task_dof_, vc_dof) = J_task_.block(0, 0, task_dof_, vc_dof);
            J_task_R_.block(0, vc_dof, task_dof_, 6) = J_task_.block(0, vc_dof, task_dof_, nc_dof) * J_I_NC_INV_T.transpose();

            CalculateJKT_R(J_task_R_, A_R_inv_N_CR, W_R_inv, J_kt_R_, Lambda_task_R_);
            Lambda_task_ = Lambda_task_R_;
        }
        else
        {
            if (reduced_task_ && !noncont_task)
            {
                J_task_R_.block(0, 0, task_dof_, vc_dof) = J_task_.block(0, 0, task_dof_, vc_dof);

                CalculateJKT_R(J_task_R_, A_R_inv_N_CR, W_R_inv, J_kt_R_, Lambda_task_R_);
                Lambda_task_ = Lambda_task_R_;
            }
            else if (!reduced_task_ && noncont_task)
            {
                Lambda_task_ = (J_task_ * A_inv_N_C * J_task_.transpose()).inverse();


                J_task_NC_ = J_task_.block(0, vc_dof, task_dof_, nc_dof);


                J_task_R_.block(0, 0, task_dof_, vc_dof) = J_task_.block(0, 0, task_dof_, vc_dof);
                J_task_R_.block(0, vc_dof, task_dof_, 6) = J_task_.block(0, vc_dof, task_dof_, nc_dof) * J_I_NC_INV_T.transpose();
                // CalculateJKT_R(J_task_R_, A_R_inv_N_CR, W_R_inv, J_kt_R_, Lambda_task_R_);
            }
            else if (!reduced_task_ && !noncont_task)
            {
                std::cout << "UNDEFINED TASK TYPE" << std::endl;
            }
            else
            {
                std::cout << "UNDEFINED TASK TYPE" << std::endl;
            }
        }

        // if (task_type_ == TASK_NONCONTACT_CHAIN)
        // {
        //     Lambda_task_ = (J_task_ * A_inv_N_C * J_task_.transpose()).inverse();
        //     J_task_NC_ = J_task_.block(0, vc_dof, task_dof_, nc_dof);

        //     J_task_R_.block(0, 0, task_dof_, vc_dof) = J_task_.block(0, 0, task_dof_, vc_dof);
        //     J_task_R_.block(0, vc_dof, task_dof_, 6) = J_task_.block(0, vc_dof, task_dof_, nc_dof) * J_I_NC_INV_T.transpose();
        // }
        // else if (task_type_ == TASK_CONTACT_CHAIN)
        // {
        //     Lambda_task_ = Lambda_task_R_;

        //     J_task_R_.block(0, 0, task_dof_, vc_dof) = J_task_.block(0, 0, task_dof_, vc_dof);
        // }
        // else if (task_type_ == TASK_CENTROIDAL)
        // {
        //     Lambda_task_ = Lambda_task_R_;

        //     J_task_R_.block(0, 0, task_dof_, vc_dof) = J_task_.block(0, 0, task_dof_, vc_dof);
        //     J_task_R_.block(0, vc_dof, task_dof_, 6) = J_task_.block(0, vc_dof, task_dof_, nc_dof) * J_I_NC_INV_T.transpose();
        // }
        // else
        // {
        // }

        // if (J_task_.leftCols(vc_dof).norm() > 1.0E-4) // if contact task is not empty
        // {
        //     reduced_task_ = true;
        //     J_task_R_.block(0, 0, task_dof_, vc_dof) = J_task_.block(0, 0, task_dof_, vc_dof);
        //     J_task_R_.block(0, vc_dof, task_dof_, 6) = J_task_.block(0, vc_dof, task_dof_, nc_dof) * J_I_NC_INV_T.transpose();
        //     CalculateJKT_R(J_task_R_, A_R_inv_N_CR, W_R_inv, J_kt_R_, Lambda_task_R_);
        // }
        // else
        // {
        //     reduced_task_ = false;
        // }
        // if (!cmm_task)
        // {
        //     if (J_task_.rightCols(nc_dof).norm() > 1.0E-4) // if non-contact task is not empty
        //     {
        //         noncont_task = true;
        //         Lambda_task_ = (J_task_ * A_inv_N_C * J_task_.transpose()).inverse();
        //         J_task_NC_ = J_task_.block(0, vc_dof, task_dof_, nc_dof);
        //         // Lambda_task_NC_ = (J_task_NC_ * A_inv.block(vc_dof, vc_dof, nc_dof, nc_dof) * J_task_NC_.transpose()).inverse();
        //     }
        //     else
        //     {
        //         Lambda_task_ = Lambda_task_R_;
        //         noncont_task = false;
        //     }
        // }
        // else
        // {
        //     Lambda_task_ = Lambda_task_R_;
        //     noncont_task = false;
        // }
    }

    void TaskSpace::CalcNullMatrix(const MatrixXd &A_inv_N_C, const MatrixXd &prev_null)
    {
        CalculateTaskNullSpace(J_kt_, Lambda_task_, J_task_, A_inv_N_C, prev_null, Null_task_);
    }

    void TaskSpace::CalcNullMatrix(const MatrixXd &A_inv_N_C)
    {
        int model_size = J_task_.cols() - 6;
        CalculateTaskNullSpace(J_kt_, Lambda_task_, J_task_, A_inv_N_C, MatrixXd::Identity(model_size, model_size), Null_task_);
    }

    void TaskSpace::CalcNullMatrix_R(const MatrixXd &A_R_inv_N_CR, const MatrixXd &prev_null)
    {
        CalculateTaskNullSpace(J_kt_R_, Lambda_task_R_, J_task_R_, A_R_inv_N_CR, prev_null, Null_task_R_);
    }

    void TaskSpace::CalcNullMatrix_R(const MatrixXd &A_R_inv_N_CR)
    {
        int model_size = J_task_R_.cols() - 6;
        CalculateTaskNullSpace(J_kt_R_, Lambda_task_R_, J_task_R_, A_R_inv_N_CR, MatrixXd::Identity(model_size, model_size), Null_task_R_);
    }
    void TaskLink::SetTrajectoryQuintic(double start_time, double end_time, Eigen::Vector3d pos_init, Eigen::Vector3d vel_init, Eigen::Vector3d pos_desired, Eigen::Vector3d vel_desired)
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
    void TaskLink::SetTrajectoryRotation(double start_time, double end_time, Eigen::Matrix3d rot_init, Eigen::Vector3d twist_init, Eigen::Matrix3d rot_desired, Eigen::Vector3d twist_desired)
    {
        traj_start_time_ = start_time;
        traj_end_time_ = end_time;

        traj_act_time_ = end_time - start_time;

        rot_init_ = rot_init;
        w_init_ = twist_init;

        rot_desired_ = rot_desired;
        w_desired_ = twist_desired;

        traj_rot_set = true;
    }

    void TaskLink::SetTaskGain(Vector3d pos_p_gain, Vector3d pos_d_gain, Vector3d pos_a_gain, Vector3d rot_p_gain, Vector3d rot_d_gain, Vector3d rot_a_gain)
    {
        pos_p_gain_ = pos_p_gain;
        pos_d_gain_ = pos_d_gain;
        pos_a_gain_ = pos_a_gain;

        rot_p_gain_ = rot_p_gain;
        rot_d_gain_ = rot_d_gain;
        rot_a_gain_ = rot_a_gain;

        // traj_gain_set = true;
    }

    Vector3d TaskLink::GetFstarPosPD(double current_time, Vector3d current_pos, Vector3d current_vel)
    {
        switch (task_link_mode_)
        {
        case TASK_LINK_6D:
        case TASK_LINK_6D_COM_FRAME:
        case TASK_LINK_6D_CUSTOM_FRAME:
        case TASK_LINK_POSITION:
        case TASK_LINK_POSITION_COM_FRAME:
        case TASK_LINK_POSITION_CUSTOM_FRAME:
            for (int j = 0; j < 3; j++)
            {
                Eigen::Vector3d quintic = QuinticSpline(current_time, traj_start_time_, traj_end_time_, pos_init_(j), vel_init_(j), 0, pos_desired_(j), vel_deisred_(j), 0);

                pos_traj_(j) = quintic(0);
                vel_traj_(j) = quintic(1);
                acc_traj_(j) = quintic(2);

                // f_star_(j) = pos_a_gain_(j) * acc_traj_(j) + pos_p_gain_(j) * (pos_traj_(j) - current_pos(j)) + pos_d_gain_(j) * (vel_traj_(j) - current_vel(j));
            }
            return pos_a_gain_.cwiseProduct(acc_traj_) + pos_p_gain_.cwiseProduct(pos_traj_ - current_pos) + pos_d_gain_.cwiseProduct(vel_traj_ - current_vel);
        default:
            return Vector3d::Zero();
        }
    }

    Vector3d TaskLink::GetFstarRotPD(double current_time, Matrix3d current_rot, Vector3d current_w)
    {
        int fstar_start_idx;

        switch (task_link_mode_)
        {
        case TASK_LINK_6D:
        case TASK_LINK_6D_COM_FRAME:
        case TASK_LINK_6D_CUSTOM_FRAME:
            fstar_start_idx = 3;
            break;

        case TASK_LINK_ROTATION:
        case TASK_LINK_ROTATION_CUSTOM_FRAME:
            fstar_start_idx = 0;
            break;
        default:
            break;
        }

        Matrix3d rot_traj;
        Vector3d w_traj, a_traj, qs_;

        Eigen::Quaterniond rq_init, rq_desired, rq_traj;
        qs_ = QuinticSpline(current_time, traj_start_time_, traj_end_time_, 0, 0, 0, 1, 0, 0);

        rq_init = Quaterniond(rot_init_);
        rq_desired = Quaterniond(rot_desired_);
        rq_traj = rq_init.slerp(qs_(0), rq_desired);

        rot_traj = rq_traj.toRotationMatrix();
        // Get roll pitch yaw from rotation matrix
        rpy_traj = rot_traj.eulerAngles(2, 1, 0);

        Eigen::AngleAxisd axd(rq_desired * rq_init.inverse());

        w_traj = axd.angle() * qs_(1) * axd.axis();
        a_traj = axd.angle() * qs_(2) * axd.axis();

        return rot_p_gain_.cwiseProduct(GetPhi(current_rot, rot_traj)) + rot_d_gain_.cwiseProduct(w_traj - current_w);
    }
    // void TaskSpace::GetFstarPosPD(double current_time, Vector3d current_pos, Vector3d current_vel)
    // {
    //     for (int i = 0; i < link_size_; i++)
    //     {
    //         task_link_[i].GetFstarPosPD(current_time, current_pos, current_vel);
    //     }

    //     switch (task_mode_)
    //     {
    //     case TASK_LINK_6D:
    //     case TASK_LINK_6D_COM_FRAME:
    //     case TASK_LINK_6D_CUSTOM_FRAME:
    //     case TASK_LINK_POSITION:
    //     case TASK_LINK_POSITION_COM_FRAME:
    //     case TASK_LINK_POSITION_CUSTOM_FRAME:
    //         for (int j = 0; j < 3; j++)
    //         {
    //             Eigen::Vector3d quintic = QuinticSpline(current_time, traj_start_time_, traj_end_time_, pos_init_(j), vel_init_(j), 0, pos_desired_(j), vel_deisred_(j), 0);

    //             pos_traj_(j) = quintic(0);
    //             vel_traj_(j) = quintic(1);
    //             acc_traj_(j) = quintic(2);

    //             f_star_(j) = pos_a_gain_(j) * acc_traj_(j) + pos_p_gain_(j) * (pos_traj_(j) - current_pos(j)) + pos_d_gain_(j) * (vel_traj_(j) - current_vel(j));
    //         }
    //         // f_star_.segment(0,3) = pos_a_gain_.cwiseProduct(acc_traj_) + pos_p_gain_.cwiseProduct(pos_traj_ - current_pos) + pos_d_gain_.cwiseProduct(vel_traj_ - current_vel);
    //         break;
    //     default:
    //         break;
    //     }
    // }

    // void TaskSpace::GetFstarRotPD(double current_time, Matrix3d current_rot, Vector3d current_w)
    // {
    //     int fstar_start_idx;

    //     switch (task_mode_)
    //     {
    //     case TASK_LINK_6D:
    //     case TASK_LINK_6D_COM_FRAME:
    //     case TASK_LINK_6D_CUSTOM_FRAME:
    //         fstar_start_idx = 3;
    //         break;

    //     case TASK_LINK_ROTATION:
    //     case TASK_LINK_ROTATION_CUSTOM_FRAME:
    //         fstar_start_idx = 0;
    //         break;
    //     default:
    //         break;
    //     }

    //     MatrixXd rot_traj;
    //     Vector3d w_traj, a_traj, qs_;

    //     Eigen::Quaterniond rq_init, rq_desired, rq_traj;
    //     qs_ = QuinticSpline(current_time, traj_start_time_, traj_end_time_, 0, 0, 0, 1, 0, 0);

    //     rq_init = Quaterniond(rot_init_);
    //     rq_desired = Quaterniond(rot_desired_);
    //     rq_traj = rq_init.slerp(qs_(0), rq_desired);

    //     rot_traj = rq_traj.toRotationMatrix();

    //     Eigen::AngleAxisd axd(rq_desired * rq_init.inverse());

    //     w_traj = axd.angle() * qs_(1) * axd.axis();
    //     a_traj = axd.angle() * qs_(2) * axd.axis();

    //     f_star_.segment(fstar_start_idx, 3) = rot_p_gain_.cwiseProduct(GetPhi(current_rot, rot_traj)) + rot_d_gain_.cwiseProduct(w_traj - current_w);
    // }
}
