#ifndef WBHQP_TASK_H
#define WBHQP_TASK_H

#include <Eigen/Dense>
#include "qp_wrapper.h"
#include "wbd.h"

using namespace Eigen;

namespace DWBC
{

    enum
    {
        TASK_LINK_6D,
        TASK_LINK_POSITION,
        TASK_LINK_ROTATION,
        TASK_CUSTOM,
        TASK_NULL,
    };

    static const char *taskmode_str[] = {"TASK_LINK_6D", "TASK_LINK_POSITION", "TASK_LINK_ROTATION", "TASK_CUSTOM"};

    class TaskSpace
    {
    private:
        /* data */
    public:
        unsigned int task_dof_;
        unsigned int heirarchy_;
        unsigned int link_number_;
        unsigned int link_id_;

        unsigned int task_mode_;

        unsigned int model_size_;

        Vector3d task_point_;

        MatrixXd J_task_;
        VectorXd f_star_;
        MatrixXd J_kt_;
        MatrixXd Null_task_;
        MatrixXd Lambda_task_;
        VectorXd torque_h_;

        MatrixXd Q_;

        MatrixXd Q_t_inv;

        CQuadraticProgram qp_;

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
    };
}

#endif