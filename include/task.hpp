#include <Eigen/Dense>
#include "qp_wrapper.hpp"
#include <contact_constraint.hpp>

using namespace Eigen;

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
    TaskSpace()
    {
        task_mode_ = TASK_NULL;
    }

    TaskSpace(int task_mode, int heirarchy, int task_dof, int model_dof)
    {
        task_dof_ = task_dof;

        task_mode_ = task_mode;

        heirarchy_ = heirarchy;

        model_size_ = model_dof;
    }

    TaskSpace(int task_mode, int heirarchy, int link_number, int link_id, const Vector3d &task_point, int model_dof)
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
    ~TaskSpace() {}

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

    void Update(const VectorXd &f_star)
    {
        f_star_ = f_star;
    }

    void Update(const MatrixXd &J_task, const VectorXd &f_star)
    {
        J_task_ = J_task;

        f_star_ = f_star;
    }

    void CalcJKT(const MatrixXd &A_inv, const MatrixXd &N_C, const MatrixXd &W_inv)
    {
        Lambda_task_ = (J_task_ * A_inv * N_C * J_task_.transpose()).inverse();
        Q_ = (Lambda_task_ * J_task_ * A_inv * N_C).rightCols(model_size_);
        J_kt_ = W_inv * Q_.transpose() * PinvCOD(Q_ * W_inv * Q_.transpose());
    }
    void CalcNullMatrix(const MatrixXd &A_inv, const MatrixXd &N_C, const MatrixXd &prev_null)
    {
        Null_task_ = prev_null * (MatrixXd::Identity(model_size_, model_size_) - J_kt_ * Lambda_task_ * J_task_ * A_inv * N_C.rightCols(model_size_));
    }

    void CalcNullMatrix(const MatrixXd &A_inv, const MatrixXd &N_C)
    {
        Null_task_ = (MatrixXd::Identity(model_size_, model_size_) - J_kt_ * Lambda_task_ * J_task_ * A_inv * N_C.rightCols(model_size_));
    }

    void GetTaskTorque(std::vector<ContactConstraint> &cc_vector, const MatrixXd &task_null_matrix_, const VectorXd &torque_limit, const VectorXd &torque_prev, const MatrixXd &NwJw, const MatrixXd &J_C_INV_T, const MatrixXd &P_C)
    {
        // return fstar & contact force;
        // std::chrono::steady_clock::time_point t[11];
        // t[0] = std::chrono::steady_clock::now();
        int task_dof = f_star_.size();
        int contact_index = cc_vector.size();
        int contact_dof = -6;

        int total_contact_dof = 0;

        // int constraint_per_contact = 11;
        // int constraint_size = contact_index * constraint_per_contact + model_size_;

        int total_constraint_size = model_size_;

        int contact_constraint_size = 0;

        for (int i = 0; i < contact_index; i++)
        {
            total_contact_dof += cc_vector[i].contact_dof_;

            contact_constraint_size += cc_vector[i].constraint_number_;
        }
        contact_dof += total_contact_dof;
        int variable_size = task_dof + contact_dof;

        total_constraint_size += contact_constraint_size;

        // std::cout << "!!!" << std::endl;

        // std::cout << "NULL : " << task_null_matrix_ << std::endl;
        Eigen::MatrixXd A;
        Eigen::MatrixXd Ntorque_task = task_null_matrix_ * J_kt_ * Lambda_task_;

        A.setZero(total_constraint_size, variable_size);
        A.block(0, 0, model_size_, task_dof) = Ntorque_task;
        A.block(0, task_dof, model_size_, contact_dof) = NwJw;

        Eigen::VectorXd lbA, ubA;

        lbA.setZero(total_constraint_size);
        ubA.setZero(total_constraint_size);
        // std::cout << "!!!" << std::endl;

        lbA.segment(0, model_size_) = -torque_limit - torque_prev - Ntorque_task * f_star_;
        ubA.segment(0, model_size_) = torque_limit - torque_prev - Ntorque_task * f_star_;
        // t[1] = std::chrono::steady_clock::now();

        // std::cout << lbA.transpose() << std::endl;
        // std::cout << ubA.transpose() << std::endl;
        // std::cout << "!!!" << std::endl;
        Eigen::MatrixXd A_const_a;
        A_const_a.setZero(contact_constraint_size, total_contact_dof);

        Eigen::MatrixXd A_rot;
        A_rot.setZero(total_contact_dof, total_contact_dof);

        int constraint_idx = 0;
        int contact_idx = 0;

        for (int i = 0; i < cc_vector.size(); i++)
        {
            A_rot.block(contact_idx, contact_idx, 3, 3) = cc_vector[i].rotm.transpose();
            A_rot.block(contact_idx + 3, contact_idx + 3, 3, 3) = cc_vector[i].rotm.transpose();

            A_const_a.block(constraint_idx, contact_idx, 4, 6) = cc_vector[i].GetZMPConstMatrix4x6();
            A_const_a.block(constraint_idx + CONTACT_CONSTRAINT_ZMP, contact_idx, 6, 6) = cc_vector[i].GetForceConstMatrix6x6();

            constraint_idx += cc_vector[i].constraint_number_;
            contact_idx += cc_vector[i].contact_dof_;
        }

        Eigen::MatrixXd Atemp = A_const_a * A_rot * J_C_INV_T.rightCols(model_size_);
        // t[3] = std::chrono::steady_clock::now();
        A.block(model_size_, 0, contact_constraint_size, task_dof) = Atemp * Ntorque_task;
        A.block(model_size_, task_dof, contact_constraint_size, contact_dof) = Atemp * NwJw;
        // t[4] = std::chrono::steady_clock::now();

        Eigen::VectorXd bA = A_const_a * (A_rot * P_C) - Atemp * (torque_prev + Ntorque_task * f_star_);

        lbA.segment(model_size_, contact_constraint_size) = bA;

        Eigen::VectorXd ubA_contact;
        ubA_contact.setConstant(contact_constraint_size, 1E+6);

        ubA.segment(model_size_, contact_constraint_size) = bA + ubA_contact;

        MatrixXd H;
        VectorXd g;
        H.setZero(variable_size, variable_size);
        H.block(0, 0, task_dof, task_dof).setIdentity();
        
        g.setZero(variable_size);
        // t[7] = std::chrono::steady_clock::now();
        // if (init)
        if (true)
        {
            qp_.InitializeProblemSize(variable_size, total_constraint_size);
        }

        qp_.EnableEqualityCondition(0.0001);
        qp_.UpdateMinProblem(H, g);
        qp_.UpdateSubjectToAx(A, lbA, ubA);

        // std::cout << lbA.transpose() << std::endl;
        // std::cout << ubA.transpose() << std::endl;
        // t[8] = std::chrono::steady_clock::now();
        VectorXd qpres;
        // t[9] = std::chrono::steady_clock::now();
        if (qp_.SolveQPoases(100, qpres))
        {
            // std::cout << "task qp solve success" << std::endl;
            f_star_qp_ = qpres.segment(0, task_dof);
            // std::cout << f_star_qp_.transpose() << std::endl;

            // std::pair<VectorXd, VectorXd> ret(qpres.segment(0, task_dof), qpres.segment(task_dof, contact_dof));
            // t[10] = std::chrono::steady_clock::now();
            // std::cout << "hqp qp prepare : ";
            // for (int i = 0; i < 10; i++)
            // {
            //     std::cout << i << " : " << std::chrono::duration_cast<std::chrono::microseconds>(t[i + 1] - t[i]).count() << "\t";
            // }
            // std::cout << "total : " << std::chrono::duration_cast<std::chrono::microseconds>(t[10] - t[0]).count() << "\t";
            // std::cout << std::endl;
            // return ret;
        }
        else
        {
            std::cout << "task solve failed" << std::endl;
            f_star_qp_ = VectorXd::Zero(task_dof);
            // std::pair<VectorXd, VectorXd> ret(VectorXd::Zero(task_dof), VectorXd::Zero(contact_dof));
            // return ret;
        }
    }
};