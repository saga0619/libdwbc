#include "task.h"
namespace DWBC
{
    TaskSpace::TaskSpace()
    {
        task_mode_ = TASK_NULL;
    }

    TaskSpace::TaskSpace(int task_mode, int heirarchy, int task_dof, int model_dof)
    {
        task_dof_ = task_dof;

        task_mode_ = task_mode;

        heirarchy_ = heirarchy;

        model_size_ = model_dof;
    }

    TaskSpace::TaskSpace(int task_mode, int heirarchy, int link_number, int link_id, const Vector3d &task_point, int model_dof)
    {
        task_mode_ = task_mode;

        heirarchy_ = heirarchy;

        link_number_ = link_number;

        link_id_ = link_id;

        model_size_ = model_dof;

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
        CalculateTaskNullSpace(J_kt_, Lambda_task_, J_task_, A_inv, N_C, MatrixXd::Identity(model_size_, model_size_), Null_task_);
    }
}