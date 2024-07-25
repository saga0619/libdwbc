#include "dwbc_hqp.h"
#include <chrono>
using namespace DWBC;

constexpr double kTolerance = 1.0e-5;
constexpr double kInfinity = std::numeric_limits<double>::infinity();

HQP::HQP()
{
}

HQP::~HQP()
{
}

void HQP::initialize(const int &acceleration_size, const int &torque_size, const int &contact_size)
{
    acceleration_size_ = acceleration_size;
    torque_size_ = torque_size;
    contact_size_ = contact_size;
}

void HQP::prepare(bool verbose)
{
    getNullSpace(hqp_hs_[0].B_, hqp_hs_[0].Z_); // null space size

    hqp_hs_[0].null_space_size_ = hqp_hs_[0].Z_.cols();
    hqp_hs_[0].u_.setZero(hqp_hs_[0].null_space_size_);

    if (verbose)
    {
        std::cout << "Hierarchy : " << 0 << std::endl;
        std::cout << "QP variable size : " << hqp_hs_[0].acceleration_size_ + torque_size_ + contact_size_ + hqp_hs_[0].ineq_const_size_ << std::endl;
        std::cout << "inequality size : " << hqp_hs_[0].ineq_const_size_ << std::endl;
        std::cout << "equality size : " << hqp_hs_[0].eq_const_size_ << std::endl;
        std::cout << "null space size : " << hqp_hs_[0].null_space_size_ << std::endl;
    }

    for (int i = 1; i < hqp_hs_.size(); i++)
    {
        hqp_hs_[i].B_nulled_ = hqp_hs_[i].B_ * hqp_hs_[i - 1].Z_;

        MatrixXd nullB;
        getNullSpace(hqp_hs_[i].B_nulled_, nullB); // null space size

        hqp_hs_[i].Z_ = hqp_hs_[i - 1].Z_ * nullB;

        int null_space_size_ = hqp_hs_[i].Z_.cols();
        if (null_space_size_ == 0)
        {
            std::cout << "null space size is zero" << std::endl;
            null_space_size_ = 0;
        }
        // std::cout << i << " : nullspace size : " << null_space_size_ << std::endl;
        hqp_hs_[i].null_space_size_ = null_space_size_;

        hqp_hs_[i].u_.setZero(null_space_size_);
    }

    if (verbose)
    {
        for (int i = 1; i < hqp_hs_.size(); i++)
        {
            std::cout << "Hierarchy : " << i << std::endl;
            std::cout << "QP variable size : " << hqp_hs_[i - 1].null_space_size_ + hqp_hs_[i].ineq_const_size_ << std::endl;
            std::cout << "inequality size : " << hqp_hs_[i].ineq_const_size_ << std::endl;
            std::cout << "equality size : " << hqp_hs_[i].eq_const_size_ << std::endl;
        }
    }

    // for (int i = 0; i < hqp_hs_.size(); i++)
    // {
    //     // Constraint Analysis
    //     std::cout << "Hierarchy : " << i << std::endl;
    //     if (hqp_hs_[i].ineq_const_size_ > 0)
    //         std::cout << "A : " << hqp_hs_[i].A_.norm() << std::endl;

    //     std::cout << "B : " << hqp_hs_[i].B_.norm() << std::endl;
    //     if (hqp_hs_[i].enable_cost_)
    //     {
    //         std::cout << "H : " << hqp_hs_[i].H_.topLeftCorner(acceleration_size_, acceleration_size_).norm() << std::endl;
    //         // std::cout << hqp_hs_[i].H_ << std::endl;
    //     }
    // }
}

// void HQP::solveSequential(bool init, bool verbose)
// {
//     int total_inequality_constraint_size = 0;

//     int variable_size = acceleration_size_ + torque_size_ + contact_size_;

//     for (int i = 0; i < hqp_hs_.size(); i++)
//     {

//         hqp_hs_[i].qp_variable_size_ = variable_size + hqp_hs_[i].ineq_const_size_ + hqp_hs_[i].eq_const_size_;
//         int ineq_const_idx = variable_size;
//         int eq_const_idx = variable_size + hqp_hs_[i].ineq_const_size_;

//         hqp_hs_[i].qp_constraint_size_ = 2 * (hqp_hs_[i].ineq_const_size_ + hqp_hs_[i].eq_const_size_);

//         if (init && verbose)
//         {
//             std::cout << "current h : " << i << std::endl;
//             std::cout << "qpconstraint size : " << hqp_hs_[i].qp_constraint_size_ << std::endl;
//             std::cout << "ineq_const_size_ : " << hqp_hs_[i].ineq_const_size_ << std::endl;
//             std::cout << "eq_const_size_ : " << hqp_hs_[i].eq_const_size_ << std::endl;
//             std::cout << "qp_variable_size_ : " << hqp_hs_[i].qp_variable_size_ << std::endl;
//         }

//         for (int j = 0; j < i; j++)
//         {
//             hqp_hs_[i].qp_constraint_size_ += hqp_hs_[j].ineq_const_size_ + hqp_hs_[j].eq_const_size_;

//             if (init && verbose)
//             {
//                 std::cout << "prev h : " << j << std::endl;
//                 std::cout << "qp_constraint_size_ : " << hqp_hs_[i].qp_constraint_size_ << std::endl;
//                 std::cout << "ineq_const_size_ : " << hqp_hs_[j].ineq_const_size_ << std::endl;
//                 std::cout << "eq_const_size_ : " << hqp_hs_[j].eq_const_size_ << std::endl;
//             }
//         }

//         hqp_hs_[i].qp_H_.setZero(hqp_hs_[i].qp_variable_size_, hqp_hs_[i].qp_variable_size_);
//         hqp_hs_[i].qp_g_.setZero(hqp_hs_[i].qp_variable_size_);

//         hqp_hs_[i].qp_A_.setZero(hqp_hs_[i].qp_constraint_size_, hqp_hs_[i].qp_variable_size_);
//         hqp_hs_[i].qp_ubA_.setZero(hqp_hs_[i].qp_constraint_size_);
//         hqp_hs_[i].qp_lbA_.setZero(hqp_hs_[i].qp_constraint_size_);

//         // cost function setting :
//         hqp_hs_[i].qp_H_.block(ineq_const_idx, ineq_const_idx, hqp_hs_[i].ineq_const_size_, hqp_hs_[i].ineq_const_size_).setIdentity();
//         hqp_hs_[i].qp_H_.block(eq_const_idx, eq_const_idx, hqp_hs_[i].eq_const_size_, hqp_hs_[i].eq_const_size_).setIdentity();

//         // hqp_hs_[i].qp_

//         // if (init)
//         // {
//         //     std::cout << " qp H : " << std::endl;
//         //     std::cout << hqp_hs_[i].qp_H_ << std::endl;
//         // }

//         int A_idx = 0;

//         // inequality constraint setting
//         if (hqp_hs_[i].ineq_const_size_ > 0)
//         {
//             hqp_hs_[i].qp_A_.block(A_idx, 0, hqp_hs_[i].ineq_const_size_, variable_size) = hqp_hs_[i].V_ * hqp_hs_[i].A_;
//             hqp_hs_[i].qp_A_.block(A_idx, ineq_const_idx, hqp_hs_[i].ineq_const_size_, hqp_hs_[i].ineq_const_size_) = -MatrixXd::Identity(hqp_hs_[i].ineq_const_size_, hqp_hs_[i].ineq_const_size_);
//             hqp_hs_[i].qp_ubA_.segment(A_idx, hqp_hs_[i].ineq_const_size_) = -hqp_hs_[i].V_ * hqp_hs_[i].a_;
//             hqp_hs_[i].qp_lbA_.segment(A_idx, hqp_hs_[i].ineq_const_size_).setConstant(-INFTY);
//             A_idx += hqp_hs_[i].ineq_const_size_;

//             hqp_hs_[i].qp_A_.block(A_idx, ineq_const_idx, hqp_hs_[i].ineq_const_size_, hqp_hs_[i].ineq_const_size_).setIdentity();
//             hqp_hs_[i].qp_lbA_.segment(A_idx, hqp_hs_[i].ineq_const_size_).setZero();
//             hqp_hs_[i].qp_ubA_.segment(A_idx, hqp_hs_[i].ineq_const_size_).setConstant(INFTY);
//             A_idx += hqp_hs_[i].ineq_const_size_;
//         }

//         // equality constraint setting
//         hqp_hs_[i].qp_A_.block(A_idx, 0, hqp_hs_[i].eq_const_size_, variable_size) = hqp_hs_[i].W_ * hqp_hs_[i].B_;
//         hqp_hs_[i].qp_A_.block(A_idx, eq_const_idx, hqp_hs_[i].eq_const_size_, hqp_hs_[i].eq_const_size_) = -MatrixXd::Identity(hqp_hs_[i].eq_const_size_, hqp_hs_[i].eq_const_size_);
//         hqp_hs_[i].qp_ubA_.segment(A_idx, hqp_hs_[i].eq_const_size_) = -hqp_hs_[i].W_ * hqp_hs_[i].b_;
//         hqp_hs_[i].qp_lbA_.segment(A_idx, hqp_hs_[i].eq_const_size_) = -hqp_hs_[i].W_ * hqp_hs_[i].b_;
//         A_idx += hqp_hs_[i].eq_const_size_;

//         hqp_hs_[i].qp_A_.block(A_idx, eq_const_idx, hqp_hs_[i].eq_const_size_, hqp_hs_[i].eq_const_size_).setIdentity();
//         hqp_hs_[i].qp_lbA_.segment(A_idx, hqp_hs_[i].eq_const_size_).setZero();
//         hqp_hs_[i].qp_ubA_.segment(A_idx, hqp_hs_[i].eq_const_size_).setConstant(INFTY);
//         A_idx += hqp_hs_[i].eq_const_size_;

//         // prev inequality constraint setiing
//         for (int k = 0; k < i; k++)
//         {
//             if (hqp_hs_[k].ineq_const_size_ > 0)
//             {
//                 MatrixXd VA = hqp_hs_[k].V_ * hqp_hs_[k].A_;
//                 assert(VA.rows() == hqp_hs_[k].ineq_const_size_);
//                 assert(VA.cols() == variable_size);

//                 hqp_hs_[i].qp_A_.block(A_idx, 0, hqp_hs_[k].ineq_const_size_, variable_size) = VA;

//                 assert(hqp_hs_[k].v_ans_.size() == hqp_hs_[k].ineq_const_size_);

//                 MatrixXd Va = hqp_hs_[k].V_ * hqp_hs_[k].a_;

//                 assert(Va.rows() == hqp_hs_[k].ineq_const_size_);

//                 hqp_hs_[i].qp_ubA_.segment(A_idx, hqp_hs_[k].ineq_const_size_) = -hqp_hs_[k].V_ * hqp_hs_[k].a_ + hqp_hs_[k].v_ans_;
//                 hqp_hs_[i].qp_lbA_.segment(A_idx, hqp_hs_[k].ineq_const_size_).setConstant(-INFTY);
//                 A_idx += hqp_hs_[k].ineq_const_size_;
//             }

//             assert(hqp_hs_[k].eq_const_size_ == hqp_hs_[k].W_.rows());
//             assert(variable_size == hqp_hs_[k].B_.cols());

//             hqp_hs_[i].qp_A_.block(A_idx, 0, hqp_hs_[k].eq_const_size_, variable_size) = hqp_hs_[k].W_ * hqp_hs_[k].B_;
//             hqp_hs_[i].qp_ubA_.segment(A_idx, hqp_hs_[k].eq_const_size_) = -hqp_hs_[k].W_ * hqp_hs_[k].b_ + hqp_hs_[k].w_ans_;
//             hqp_hs_[i].qp_lbA_.segment(A_idx, hqp_hs_[k].eq_const_size_) = -hqp_hs_[k].W_ * hqp_hs_[k].b_ + hqp_hs_[k].w_ans_;
//             A_idx += hqp_hs_[k].eq_const_size_;
//         }

//         VectorXd qp_ans;
//         hqp_hs_[i].solve(qp_ans, init);
//         hqp_hs_[i].y_ans_ = qp_ans.head(variable_size);
//         hqp_hs_[i].w_ans_ = qp_ans.tail(hqp_hs_[i].eq_const_size_);

//         if (hqp_hs_[i].ineq_const_size_ > 0)
//             hqp_hs_[i].v_ans_ = qp_ans.segment(variable_size, hqp_hs_[i].ineq_const_size_);

//         if (init)
//         {
//             // std::cout << i << " qpans : " << hqp_hs_[i].y_ans_.transpose() << std::endl;
//         }
//     }
// }

void HQP::solveWeighted(bool init)
{
}

void HQP::solvefirst(bool init)
{
    auto t1 = std::chrono::high_resolution_clock::now();
    int total_inequality_constraint_size = 0;
    total_inequality_constraint_size += hqp_hs_[0].ineq_const_size_;
    hqp_hs_[0].qp_constraint_size_ = hqp_hs_[0].ineq_const_size_;
    int dec_var_size = acceleration_size_ + torque_size_ + contact_size_;
    hqp_hs_[0].qp_variable_size_ = dec_var_size + hqp_hs_[0].ineq_const_size_;

    hqp_hs_[0].qp_H_.setZero(hqp_hs_[0].qp_variable_size_, hqp_hs_[0].qp_variable_size_);
    hqp_hs_[0].qp_g_.setZero(hqp_hs_[0].qp_variable_size_);
    hqp_hs_[0].qp_A_.setZero(hqp_hs_[0].qp_constraint_size_, hqp_hs_[0].qp_variable_size_);
    hqp_hs_[0].qp_ubA_.setZero(hqp_hs_[0].qp_constraint_size_);
    hqp_hs_[0].qp_lbA_.setZero(hqp_hs_[0].qp_constraint_size_);

    hqp_hs_[0].qp_lb_.setConstant(hqp_hs_[0].qp_variable_size_, -INFTY);
    hqp_hs_[0].qp_ub_.setConstant(hqp_hs_[0].qp_variable_size_, INFTY);

    if (hqp_hs_[0].ineq_const_size_ > 0)
    {
        hqp_hs_[0].qp_H_.bottomRightCorner(hqp_hs_[0].ineq_const_size_, hqp_hs_[0].ineq_const_size_).setIdentity();
        hqp_hs_[0].qp_g_.tail(hqp_hs_[0].ineq_const_size_).setZero();

        hqp_hs_[0].qp_A_.topLeftCorner(hqp_hs_[0].ineq_const_size_, dec_var_size) = hqp_hs_[0].V_ * hqp_hs_[0].A_;
        hqp_hs_[0].qp_A_.topRightCorner(hqp_hs_[0].ineq_const_size_, hqp_hs_[0].ineq_const_size_) = -MatrixXd::Identity(hqp_hs_[0].ineq_const_size_, hqp_hs_[0].ineq_const_size_);
        hqp_hs_[0].qp_ubA_.head(hqp_hs_[0].ineq_const_size_) = -hqp_hs_[0].V_ * hqp_hs_[0].a_;
        hqp_hs_[0].qp_lbA_.head(hqp_hs_[0].ineq_const_size_).setConstant(-INFTY);

        hqp_hs_[0].qp_lb_.segment(dec_var_size, hqp_hs_[0].ineq_const_size_).setZero();
    }

    MatrixXd temp = (hqp_hs_[0].W_ * hqp_hs_[0].B_);
    VectorXd temp2 = hqp_hs_[0].W_ * hqp_hs_[0].b_;

    hqp_hs_[0].qp_H_.topLeftCorner(dec_var_size, dec_var_size) = temp.transpose() * temp;
    hqp_hs_[0].qp_g_.head(dec_var_size) = temp.transpose() * temp2;
    if (hqp_hs_[0].enable_cost_)
    {

        // double v1, v2;
        // v1 = 1;
        // v2 = 1;
        // MatrixXd manalysis = (hqp_hs_[i].B_);
        // MatrixXd manalysis2 = hqp_hs_[i].H_;
        // v1 = manalysis.norm();
        // v2 = manalysis2.norm();

        MatrixXd temp_A = hqp_hs_[0].H_ / 1.0; // hqp_hs_[i].H_.norm();

        double v3 = 1.0;

        hqp_hs_[0].qp_H_.topLeftCorner(dec_var_size, dec_var_size) += hqp_hs_[0].H_;
        // hqp_hs_[0].qp_g_.head(dec_var_size) += hqp_hs_[i].H_ * hqp_hs_[i - 1].y_ans_;
    }

    auto t2 = std::chrono::high_resolution_clock::now();

    hqp_hs_[0].qp_update_time_step_ = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    hqp_hs_[0].qp_update_time_max_ = std::max(hqp_hs_[0].qp_update_time_max_, hqp_hs_[0].qp_update_time_step_);

    VectorXd ans;
    hqp_hs_[0].solveOSQP(ans, init);

    hqp_hs_[0].y_ans_ = ans.head(dec_var_size);

    if (hqp_hs_[0].ineq_const_size_ > 0)
        hqp_hs_[0].v_ans_ = ans.tail(hqp_hs_[0].ineq_const_size_);
}

void HQP::solveSequentialSingle(int level, bool init)
{
    int i = level;
    auto t1 = std::chrono::high_resolution_clock::now();

    if (i == 1)
    {
        hqp_hs_[0].qp_constraint_size_ = hqp_hs_[0].ineq_const_size_;
    }
    hqp_hs_[i].qp_constraint_size_ = hqp_hs_[i - 1].qp_constraint_size_ + hqp_hs_[i].ineq_const_size_;
    hqp_hs_[i].qp_variable_size_ = hqp_hs_[i - 1].null_space_size_ + hqp_hs_[i].ineq_const_size_;

    if (init)
    {
        hqp_hs_[i].qp_H_.setZero(hqp_hs_[i].qp_variable_size_, hqp_hs_[i].qp_variable_size_);
        hqp_hs_[i].qp_g_.setZero(hqp_hs_[i].qp_variable_size_);
        hqp_hs_[i].qp_A_.setZero(hqp_hs_[i].qp_constraint_size_, hqp_hs_[i].qp_variable_size_);
        hqp_hs_[i].qp_ubA_.setZero(hqp_hs_[i].qp_constraint_size_);
        hqp_hs_[i].qp_lbA_.setZero(hqp_hs_[i].qp_constraint_size_);
    }

    hqp_hs_[i].qp_lb_.setConstant(hqp_hs_[i].qp_variable_size_, -INFTY);
    hqp_hs_[i].qp_ub_.setConstant(hqp_hs_[i].qp_variable_size_, INFTY);

    int const_idx = hqp_hs_[i].ineq_const_size_;
    // std::cout << "solve : " << i << std::endl;
    if (hqp_hs_[i].ineq_const_size_ > 0)
    {

        hqp_hs_[i].qp_H_.bottomRightCorner(hqp_hs_[i].ineq_const_size_, hqp_hs_[i].ineq_const_size_).setIdentity();
        hqp_hs_[i].qp_g_.tail(hqp_hs_[i].ineq_const_size_).setZero();

        // hqp_hs_[i].qp_A_.topLeftCorner(hqp_hs_[i].ineq_const_size_, hqp_hs_[i - 1].null_space_size_) = hqp_hs_[i].V_ * (hqp_hs_[i].A_ * hqp_hs_[i - 1].Z_);
        hqp_hs_[i].qp_A_.topLeftCorner(hqp_hs_[i].ineq_const_size_, hqp_hs_[i - 1].null_space_size_) = (hqp_hs_[i].A_ * hqp_hs_[i - 1].Z_);
        hqp_hs_[i].qp_A_.topRightCorner(hqp_hs_[i].ineq_const_size_, hqp_hs_[i].ineq_const_size_) = -MatrixXd::Identity(hqp_hs_[i].ineq_const_size_, hqp_hs_[i].ineq_const_size_);
        hqp_hs_[i].qp_ubA_.head(hqp_hs_[i].ineq_const_size_) = -(hqp_hs_[i].A_ * hqp_hs_[i - 1].y_ans_) - hqp_hs_[i].a_;
        // hqp_hs_[i].qp_ubA_.head(hqp_hs_[i].ineq_const_size_) = -hqp_hs_[i].V_ * (hqp_hs_[i].A_ * hqp_hs_[i - 1].y_ans_) - hqp_hs_[i].V_ * hqp_hs_[i].a_;

        hqp_hs_[i].qp_lbA_.head(hqp_hs_[i].ineq_const_size_).setConstant(-INFTY);

        hqp_hs_[i].qp_lb_.segment(hqp_hs_[i - 1].null_space_size_, hqp_hs_[i].ineq_const_size_).setZero();
    }

    // MatrixXd temp = hqp_hs_[i].W_ * (hqp_hs_[i].B_ * hqp_hs_[i - 1].Z_);
    // VectorXd temp2 = hqp_hs_[i].W_ * (hqp_hs_[i].B_ * hqp_hs_[i - 1].y_ans_) + hqp_hs_[i].W_ * hqp_hs_[i].b_;
    MatrixXd temp = (hqp_hs_[i].B_ * hqp_hs_[i - 1].Z_);
    VectorXd temp2 = (hqp_hs_[i].B_ * hqp_hs_[i - 1].y_ans_) + hqp_hs_[i].b_;

    hqp_hs_[i].qp_H_.topLeftCorner(hqp_hs_[i - 1].null_space_size_, hqp_hs_[i - 1].null_space_size_) = temp.transpose() * temp;
    hqp_hs_[i].qp_g_.head(hqp_hs_[i - 1].null_space_size_) = temp.transpose() * temp2;

    if (hqp_hs_[i].enable_cost_)
    {
        // double v1, v2;
        // v1 = 1;
        // v2 = 1;
        // MatrixXd manalysis = (hqp_hs_[i].B_);
        // MatrixXd manalysis2 = hqp_hs_[i].H_;
        // v1 = manalysis.norm();
        // v2 = manalysis2.norm();
        // MatrixXd temp_A = hqp_hs_[i].H_ / 1.0; // hqp_hs_[i].H_.norm();
        // double v3 = 1.0;

        hqp_hs_[i].qp_H_.topLeftCorner(hqp_hs_[i - 1].null_space_size_, hqp_hs_[i - 1].null_space_size_) += hqp_hs_[i - 1].Z_.transpose() * hqp_hs_[i].H_ * hqp_hs_[i - 1].Z_;
        hqp_hs_[i].qp_g_.head(hqp_hs_[i - 1].null_space_size_) += hqp_hs_[i - 1].Z_.transpose() * hqp_hs_[i].H_ * hqp_hs_[i - 1].y_ans_;
    }

    // temp = temp * (v1 + v2) * 0.5;
    // temp2 = temp2 * (v1 + v2) * 0.5;
    // MatrixXd temp_Am = ;
    // VectorXd temp_Av

    for (int j = 0; j < i; j++)
    {
        if (hqp_hs_[j].ineq_const_size_ > 0)
        {

            assert(hqp_hs_[j].V_.cols() == hqp_hs_[j].A_.rows());
            assert(hqp_hs_[j].A_.cols() == hqp_hs_[i - 1].Z_.rows());

            // hqp_hs_[i].qp_A_.block(const_idx, 0, hqp_hs_[j].ineq_const_size_, hqp_hs_[i - 1].null_space_size_) = hqp_hs_[j].V_ * hqp_hs_[j].A_ * hqp_hs_[i - 1].Z_;
            // hqp_hs_[i].qp_ubA_.segment(const_idx, hqp_hs_[j].ineq_const_size_) = -hqp_hs_[j].V_ * hqp_hs_[j].A_ * hqp_hs_[i - 1].y_ans_ + hqp_hs_[j].v_ans_ - hqp_hs_[j].V_ * hqp_hs_[j].a_;
            hqp_hs_[i].qp_A_.block(const_idx, 0, hqp_hs_[j].ineq_const_size_, hqp_hs_[i - 1].null_space_size_) = hqp_hs_[j].A_ * hqp_hs_[i - 1].Z_;
            hqp_hs_[i].qp_ubA_.segment(const_idx, hqp_hs_[j].ineq_const_size_) = -hqp_hs_[j].A_ * hqp_hs_[i - 1].y_ans_ + hqp_hs_[j].v_ans_ - hqp_hs_[j].a_;
            hqp_hs_[i].qp_lbA_.segment(const_idx, hqp_hs_[j].ineq_const_size_).setConstant(-INFTY);

            const_idx += hqp_hs_[j].ineq_const_size_;
        }
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    hqp_hs_[i].qp_update_time_step_ = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    hqp_hs_[i].qp_update_time_max_ = std::max(hqp_hs_[i].qp_update_time_max_, hqp_hs_[i].qp_update_time_step_);

    VectorXd ans;
    hqp_hs_[i].solveOSQP(ans, init);

    hqp_hs_[i].y_ans_ = hqp_hs_[i - 1].y_ans_ + hqp_hs_[i - 1].Z_ * ans.head(hqp_hs_[i - 1].null_space_size_);

    if (hqp_hs_[i].ineq_const_size_ > 0)
        hqp_hs_[i].v_ans_ = ans.tail(hqp_hs_[i].ineq_const_size_);
    hqp_hs_[i].w_ans_ = hqp_hs_[i].B_ * hqp_hs_[i].y_ans_ + hqp_hs_[i].b_;

    // hqp_hs_[i].w_ans_ = hqp_hs_[i].W_ * hqp_hs_[i].B_ * hqp_hs_[i].y_ans_ + hqp_hs_[i].W_ * hqp_hs_[i].b_;
}

void HQP::solveSequential(bool init, bool verbose)
{
    for (int i = 1; i < hqp_hs_.size(); i++)
    {
        solveSequentialSingle(i, init);
    }
}

HQP_Hierarch::HQP_Hierarch()
{
}

HQP_Hierarch::~HQP_Hierarch()
{
    if (solver_init)
    {
        delete osqp_solver_;
    }
}

void HQP_Hierarch::initOSQP()
{
    // solver_init = true;
    // solver = new osqp::OsqpSolver();

    // solver = new OsqpEigen::Solver();
}

void HQP::addHierarchy(const int &ineq_const_size, const int &eq_const_size)
{
    HQP_Hierarch hqp_h;
    // std::cout << "1declare hqp_h" << std::endl;
    hqp_h.initialize(hqp_hs_.size(), acceleration_size_, torque_size_, contact_size_, ineq_const_size, eq_const_size);

    // std::cout << "2declare hqp_h" << std::endl;
    hqp_hs_.push_back(hqp_h);
    // std::cout << "3declare hqp_h" << std::endl;
}

void HQP_Hierarch::initialize(const int &hierarchy_level, const int &acceleration_size, const int &torque_size, const int &contact_size, const int &ineq_const_size, const int &eq_const_size)
{
    hierarchy_level_ = hierarchy_level;
    acceleration_size_ = acceleration_size;
    torque_size_ = torque_size;
    contact_size_ = contact_size;

    variable_size_ = acceleration_size_ + torque_size_ + contact_size_;
    ineq_const_size_ = ineq_const_size;
    eq_const_size_ = eq_const_size;

    if (ineq_const_size > 0)
    {
        A_.setZero(ineq_const_size_, variable_size_);
        a_.setZero(ineq_const_size_);
        V_.setIdentity(ineq_const_size_, ineq_const_size_);
        v_.setZero(ineq_const_size_);
        v_ans_.setZero(ineq_const_size_);
    }

    y_.setZero(variable_size_);

    B_.setZero(eq_const_size_, variable_size_);
    b_.setZero(eq_const_size_);
    W_.setIdentity(eq_const_size_, eq_const_size_);
    w_.setZero(eq_const_size_);

    y_ans_.setZero(variable_size_);
    w_ans_.setZero(eq_const_size_);

    enable_cost_ = false;
    solver_init = false;

    qp_solve_time_step_ = 0;
    qp_update_time_step_ = 0;

    qp_solve_time_max_ = 0;
    qp_update_time_max_ = 0;

    osqp_solver_ = new OsqpEigen::Solver();
    // null space siize
    // Z_.setZero(variable_size_, null_space_size_);
    // u_.setZero(null_space_size_);

    // qp_solver_.InitializeProblemSize(variable_size_ + eq_const_size + ineq_const_size, eq_const_size_ + ineq_const_size_);
}

void HQP_Hierarch::updateCostMatrix(const MatrixXd &H, const VectorXd &g)
{
    enable_cost_ = true;

    assert(H.rows() == variable_size_);
    assert(H.cols() == variable_size_);
    assert(g.rows() == variable_size_);

    H_ = H;
    g_ = g;
}
void HQP_Hierarch::updateInequalityConstraintMatrix(const MatrixXd &A, const VectorXd &a)
{
    assert(A.rows() == ineq_const_size_);
    assert(A.cols() == variable_size_);
    assert(a.rows() == ineq_const_size_);

    A_ = A;
    a_ = a;
}
void HQP_Hierarch::updateInequalityCostWeight(const MatrixXd &V)
{
    V_ = V;
}
void HQP_Hierarch::updateInequalityCostWeight(const VectorXd &V)
{
    V_ = V.asDiagonal();
}

void HQP_Hierarch::updateEqualityConstraintMatrix(const MatrixXd &B, const VectorXd &b)
{
    assert(B.rows() == eq_const_size_);
    assert(B.cols() == variable_size_);
    assert(b.rows() == eq_const_size_);

    B_ = B;
    b_ = b;
}
void HQP_Hierarch::updateEqualityCostWeight(const MatrixXd &W)
{
    W_ = W;
}
void HQP_Hierarch::updateEqualityCostWeight(const VectorXd &W)
{
    W_ = W.asDiagonal();
}

void HQP_Hierarch::updateConstraintMatrix(const MatrixXd &A, const VectorXd &a, const MatrixXd &B, const VectorXd &b)
{
    if (ineq_const_size_ > 0)
    {
        assert(A.rows() == ineq_const_size_);
        assert(A.cols() == variable_size_);
        assert(a.rows() == ineq_const_size_);
        A_ = A;
        a_ = a;
    }

    assert(B.rows() == eq_const_size_);
    assert(B.rows() == b.rows());
    assert(B.cols() == variable_size_);

    B_ = B;
    b_ = b;
}

void HQP_Hierarch::updateConstraintWeight(const MatrixXd &V, const MatrixXd &W)
{
    V_ = V;
    W_ = W;
}

void HQP_Hierarch::normalizeConstraintMatrix()
{
    // Row normalize A
    if (ineq_const_size_ > 0)
    {
        for (int i = 0; i < ineq_const_size_; i++)
        {
            double norm = A_.row(i).norm();
            if (norm > 0)
            {
                A_.row(i) /= norm;
                a_(i) /= norm;
            }
        }
    }

    // Row normalize B
    for (int i = 0; i < eq_const_size_; i++)
    {
        double norm = B_.row(i).norm();
        if (norm > 0)
        {
            B_.row(i) /= norm;
            b_(i) /= norm;
        }
    }
}

void HQP_Hierarch::solveOSQP(VectorXd &qp_ans, bool init)
{

    sH_ = qp_H_.sparseView(1E-5);
    sA_ = qp_A_.sparseView(1E-5);

    qp_lbA_.setConstant(qp_lbA_.size(), -OsqpEigen::INFTY);

    if (init)
    {

        osqp_solver_->data()->setNumberOfVariables(qp_variable_size_);
        osqp_solver_->data()->setNumberOfConstraints(qp_constraint_size_);

        osqp_solver_->data()->setHessianMatrix(sH_);
        osqp_solver_->data()->setGradient(qp_g_);
        osqp_solver_->data()->setLinearConstraintsMatrix(sA_);
        osqp_solver_->data()->setUpperBound(qp_ubA_);
        osqp_solver_->data()->setLowerBound(qp_lbA_);

        osqp_solver_->settings()->setWarmStart(true);
        osqp_solver_->settings()->setVerbosity(false);
        osqp_solver_->settings()->setAlpha(1.0);

        osqp_solver_->initSolver();
        osqp_solver_->solveProblem();

        qp_ans = osqp_solver_->getSolution();
    }
    else
    {

        auto t1 = std::chrono::high_resolution_clock::now();
        osqp_solver_->updateGradient(qp_g_);
        osqp_solver_->updateUpperBound(qp_ubA_);
        osqp_solver_->updateHessianMatrix(sH_);
        osqp_solver_->updateLinearConstraintsMatrix(sA_);

        osqp_solver_->solveProblem();
        qp_ans = osqp_solver_->getSolution();

        auto t2 = std::chrono::high_resolution_clock::now();
        qp_solve_time_step_ = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
        qp_solve_time_max_ = std::max(qp_solve_time_max_, qp_solve_time_step_);
    }

    u_ans_ = qp_ans.head(null_space_size_);
    v_ans_ = qp_ans.tail(ineq_const_size_);
}

void HQP_Hierarch::solve(VectorXd &qp_ans, bool init)
{
    auto t1 = std::chrono::high_resolution_clock::now();
    if (init)
    {
        // std::cout << "init " << std::endl;
        qp_solver_.InitializeProblemSize(qp_variable_size_, qp_constraint_size_);
    }
    qp_solver_.UpdateMinProblem(qp_H_, qp_g_);
    // qp_solver_.UpdateSubjectToAxUb(qp_A_, qp_ubA_);
    qp_solver_.UpdateSubjectToAx(qp_A_, qp_lbA_, qp_ubA_);
    // qp_solver_.EnableEqualityCondition(0);
    qp_solver_.UpdateSubjectToX(qp_lb_, qp_ub_);
    // qp_solver_.DeleteSubjectToX();
    qp_ans.setZero(qp_variable_size_);
    // qp_solver_.PrintMinProb();
    // qp_solver_.DisableLowerBoundA();

    qp_solver_.SolveQPoases(600, qp_ans, true);
    auto t2 = std::chrono::high_resolution_clock::now();
    double micro_sec_solve = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    // qp_solve_time_ += micro_sec_solve;
    qp_solve_time_max_ = std::max(qp_solve_time_max_, micro_sec_solve);
}