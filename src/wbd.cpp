#include "dwbc_wbd.h"
#include "iostream"
namespace DWBC
{
    MatrixXd PinvCODWBt(const MatrixXd &A)
    {
        Eigen::CompleteOrthogonalDecomposition<MatrixXd> cod(A.rows(), A.cols());
        cod.setThreshold(COD_THRESHOLD);
        cod.compute(A);

        return cod.pseudoInverse();
    }

    MatrixXd PinvCODWB(const MatrixXd &A)
    {
        Eigen::CompleteOrthogonalDecomposition<MatrixXd> cod(A.rows(), A.cols());
        cod.setThreshold(COD_THRESHOLD);
        cod.compute(A);

        return cod.pseudoInverse();
    }

    void PinvCODWB(const MatrixXd &A, MatrixXd &ret)
    {
        Eigen::CompleteOrthogonalDecomposition<MatrixXd> cod(A.rows(), A.cols());
        cod.setThreshold(COD_THRESHOLD);
        cod.compute(A);

        ret = cod.pseudoInverse();
    }

    void PinvCODWB(const MatrixXd &A, MatrixXd &ret, MatrixXd &V2, int force_rank)
    {
        int rows = A.rows();
        int cols = A.cols();
        Eigen::CompleteOrthogonalDecomposition<MatrixXd> cod(rows, cols);
        cod.setThreshold(COD_THRESHOLD);
        cod.compute(A);
        int rank;
        if (force_rank > 0)
        {
            rank = force_rank;
        }
        else
        {
            rank = cod.rank();
        }

        MatrixXd Vtemp = cod.householderQ().transpose();
        V2 = (Vtemp).block(rank, 0, rows - rank, cols);

        ret = cod.pseudoInverse();
    }

    /*
     * Contact Constraint Computation
     *
     */
    MatrixXd GetZMPConstMatrix(double local_contact_plane_x, double local_contact_plane_y)
    {
        MatrixXd zmp_const_mat = MatrixXd::Zero(4, 6);

        zmp_const_mat(0, 2) = -local_contact_plane_x;
        zmp_const_mat(0, 4) = -1;

        zmp_const_mat(1, 2) = -local_contact_plane_x;
        zmp_const_mat(1, 4) = 1;

        zmp_const_mat(2, 2) = -local_contact_plane_y;
        zmp_const_mat(2, 3) = -1;

        zmp_const_mat(3, 2) = -local_contact_plane_y;
        zmp_const_mat(3, 3) = 1;

        return zmp_const_mat;
    }
    MatrixXd GetForceConstMatrix(double friction_ratio, double friction_ratio_z)
    {
        MatrixXd force_const_matrix = MatrixXd::Zero(6, 6);

        force_const_matrix(0, 0) = 1.0;
        force_const_matrix(0, 2) = -friction_ratio;
        force_const_matrix(1, 0) = -1.0;
        force_const_matrix(1, 2) = -friction_ratio;

        force_const_matrix(2, 1) = 1.0;
        force_const_matrix(2, 2) = -friction_ratio;
        force_const_matrix(3, 1) = -1.0;
        force_const_matrix(3, 2) = -friction_ratio;

        force_const_matrix(4, 5) = 1.0;
        force_const_matrix(4, 2) = -friction_ratio_z;
        force_const_matrix(5, 5) = -1.0;
        force_const_matrix(5, 2) = -friction_ratio_z;

        return force_const_matrix;
    }
    // MatrixXd GetContactToPointAxis()
    // {

    // }

    /*
     * With contact jacobian(j_contact) and inverse of mass matrix(A_inv), calculate contact constraint matrix
     * input  : j_c, A_inv
     * output : lambda_c, J_c_inv_T, N_c, W, W_inv, V2, NwJw
     */
    int CalculateContactConstraint(
        const MatrixXd &J_contact, const MatrixXd &A_inv,
        MatrixXd &lambda_contact, MatrixXd &J_C_INV_T, MatrixXd &N_C, MatrixXd &A_inv_N_C, MatrixXd &W, MatrixXd &NwJw, MatrixXd &Winv, MatrixXd &V2)
    {
        int rows = J_contact.rows(); // rows == contact_dof
        int cols = J_contact.cols(); // cols == system_dof

        lambda_contact = (J_contact * A_inv * J_contact.transpose()).inverse();
        J_C_INV_T = lambda_contact * J_contact * A_inv;
        N_C = MatrixXd::Identity(cols, cols) - J_contact.transpose() * J_C_INV_T;
        A_inv_N_C = A_inv * N_C;
        W = A_inv_N_C.block(6, 6, cols - 6, cols - 6);
        // A_inv.bottomRows(cols - 6) * N_C.rightCols(cols - 6);

        if (rows > 6)
        {
            PinvCODWB(W, Winv, V2, -1);

            if (rows - 6 == V2.rows())
            {
                NwJw = V2.transpose() * (J_C_INV_T.topRightCorner(rows - 6, cols - 6) * V2.transpose()).inverse();
                return 1;
            }
            else
            {
                NwJw = V2.transpose() * (J_C_INV_T.topRightCorner(rows - 6, cols - 6) * V2.transpose()).inverse();
                std::cout << "Contact Space Factorization Error : Required contact null dimension : " << J_contact.rows() - 6 << " factorization rank : " << V2.rows() << std::endl;
                return 0;
            }
        }
        else
        {
            PinvCODWB(W, Winv);
            return 1;
        }
    }

    // int CalculateContactConstraintR(
    //     const MatrixXd &J_CR, const MatrixXd &A_R_inv,
    //     MatrixXd &lambda_contact_R, MatrixXd &J_CR_INV_T, MatrixXd &N_CR, MatrixXd &A_R_inv_N_CR, MatrixXd &W_R, MatrixXd &NwJw_R, MatrixXd &W_R_inv, MatrixXd &V2_R)
    // {
    //     int rows = J_CR.rows(); // rows == contact_dof
    //     int cols = J_CR.cols(); // cols == system_dof

    //     lambda_contact_R = (J_CR * A_R_inv * J_CR.transpose()).inverse();
    //     J_CR_INV_T = lambda_contact_R * J_CR * A_R_inv;
    //     N_CR = MatrixXd::Identity(cols, cols) - J_CR.transpose() * J_CR_INV_T;
    //     A_R_inv_N_CR = A_R_inv * N_CR;
    //     W_R_inv = A_R_inv_N_CR.block(6, 6, cols - 6, cols - 6);
    //     // A_inv.bottomRows(cols - 6) * N_C.rightCols(cols - 6);

    //     if (rows > 6)
    //     {
    //         PinvCODWB(W_R, W_R_inv, V2_R, cols - (rows));

    //         if (rows - 6 == V2_R.rows())
    //         {
    //             NwJw_R = V2_R.transpose() * (J_CR_INV_T.rightCols(cols - 6).topRows(6) * V2_R.transpose()).inverse();
    //             return 1;
    //         }
    //         else
    //         {
    //             NwJw_R = V2_R.transpose() * (J_CR_INV_T.rightCols(cols - 6).topRows(6) * V2_R.transpose()).inverse();
    //             std::cout << "Contact Space Factorization Error : Required contact null dimension : " << J_CR.rows() - 6 << " factorization rank : " << V2_R.rows() << std::endl;
    //             return 0;
    //         }
    //     }
    //     else
    //     {
    //         PinvCODWB(W_R, W_R_inv);
    //         return 1;
    //     }
    // }
    /*
     * Gravity Compensation Torque Calculation
     * input  : A_inv, W_inv, N_C, J_C_INV_T, G
     * output : torque_grav,
     */
    void CalculateGravityCompensation(
        const MatrixXd &A_inv, const MatrixXd &W_inv, const MatrixXd &N_C, const MatrixXd &J_C_INV_T, const VectorXd &G,
        VectorXd &torque_grav, MatrixXd &P_C)
    {
        torque_grav = W_inv * (A_inv.bottomRows(W_inv.cols()) * (N_C * G));
        P_C = J_C_INV_T * G;
    }

    /*
     * Temporary
     *
     */
    void CalculateLQ(const MatrixXd &W_in)
    {
    }

    /*
     * Get Jkt
     * input  : J_task, A_inv, N_C, W_inv,
     * output : J_kt, lambda_task
     */
    void CalculateJKT(const MatrixXd &J_task, const MatrixXd &A_inv, const MatrixXd &N_C, const MatrixXd &W_inv,
                      MatrixXd &J_kt, MatrixXd &lambda_task)
    {
        lambda_task = (J_task * A_inv * N_C * J_task.transpose()).inverse();
        MatrixXd Q = (lambda_task * J_task * A_inv * N_C).rightCols(J_task.cols() - 6);
        J_kt = W_inv * Q.transpose() * PinvCODWB(Q * W_inv * Q.transpose());
    }

    /*
     * Get Jkt reduced
     * input  : J_task, A_inv, N_C, W_inv,
     * output : J_kt, lambda_task
     */
    void CalculateJKT_R(const MatrixXd &J_task_R, const MatrixXd &A_R_inv_N_CR, const MatrixXd &W_R_inv,
                        MatrixXd &J_kt_R, MatrixXd &lambda_task_R)
    {
        lambda_task_R = (J_task_R * A_R_inv_N_CR * J_task_R.transpose()).inverse();
        MatrixXd Q = (lambda_task_R * J_task_R * A_R_inv_N_CR).rightCols(J_task_R.cols() - 6);
        J_kt_R = W_R_inv * Q.transpose() * PinvCODWB(Q * W_R_inv * Q.transpose());
    }

    /*
     *
     *
     *
     */
    void CalculateJKTThreaded(const MatrixXd &J_task, const MatrixXd &A_inv, const MatrixXd &N_C, const MatrixXd &W_inv,
                              MatrixXd &Q, MatrixXd &Q_temp, MatrixXd &lambda_task)
    {
        lambda_task = (J_task * A_inv * N_C * J_task.transpose()).inverse();
        Q = (lambda_task * J_task * A_inv * N_C).rightCols(J_task.cols() - 6);
        Q_temp = Q * W_inv * Q.transpose();

        // J_kt = W_inv * Q.transpose() * PinvCODWB(Q * W_inv * Q.transpose());
    }

    /*
     *
     *
     */
    MatrixXd CalculateJKTonly(const MatrixXd &W_inv, const MatrixXd &Q)
    {
        return W_inv * Q.transpose() * PinvCODWB(Q * W_inv * Q.transpose());
    }

    /*
     * Calculate Taskspace Null matrix
     * input  : J_kt, lambda_task, J_task, A_inv, N_C, upper_task_null
     * output : Null_task
     */
    void CalculateTaskNullSpace(const MatrixXd &J_kt, const MatrixXd &Lambda_task, const MatrixXd &J_task, const MatrixXd &A_inv_N_C, const MatrixXd &prev_null, MatrixXd &Null_task)
    {
        int model_size = J_task.cols() - 6;
        Null_task = prev_null * (MatrixXd::Identity(model_size, model_size) - J_kt * Lambda_task * J_task * A_inv_N_C.rightCols(model_size));
    }

    /*
     * Calculate contact force from command torque
     * input  : command_torque, J_C_INV_T, P_C
     * output : ContactForce
     */
    void CalculateContactForce(const VectorXd &command_torque, const MatrixXd &J_C_INV_T, const MatrixXd &P_C, VectorXd &ContactForce)
    {
        ContactForce = J_C_INV_T.rightCols(command_torque.size()) * command_torque - P_C;
    }

    void ContactRedistributetwomod(double eta_cust, double footlength, double footwidth, double staticFrictionCoeff, double ratio_x, double ratio_y, Eigen::Vector3d P1, Eigen::Vector3d P2, Vector12d &F12, Vector6d &ResultantForce, Vector12d &ForceRedistribution, double &eta)
    {
        Eigen::Matrix<double, 6, 12> W;
        W.setZero();

        W.block(0, 0, 6, 6).setIdentity();
        W.block(0, 6, 6, 6).setIdentity();
        W.block(3, 0, 3, 3) = skew(P1);
        W.block(3, 6, 3, 3) = skew(P2);

        ResultantForce = W * F12; // F1F2;

        double eta_lb = 1.0 - eta_cust;
        double eta_ub = eta_cust;
        // printf("1 lb %f ub %f\n",eta_lb,eta_ub);
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////
        // boundary of eta Mx, A*eta + B < 0
        double A = (P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2);
        double B = ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2);
        double C = ratio_y * footwidth / 2.0 * abs(ResultantForce(2));
        double a = A * A;
        double b = 2.0 * A * B;
        double c = B * B - C * C;
        double sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        double sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        if (sol_eta1 > sol_eta2) // sol_eta1 ÀÌ upper boundary
        {
            if (sol_eta1 < eta_ub)
            {
                eta_ub = sol_eta1;
            }

            if (sol_eta2 > eta_lb)
            {
                eta_lb = sol_eta2;
            }
        }
        else // sol_eta2 ÀÌ upper boundary
        {
            if (sol_eta2 < eta_ub)
            {
                eta_ub = sol_eta2;
            }

            if (sol_eta1 > eta_lb)
            {
                eta_lb = sol_eta1;
            }
        }

        // printf("3 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////
        // boundary of eta My, A*eta + B < 0
        A = -(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2);
        B = ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2);
        C = ratio_x * footlength / 2.0 * abs(ResultantForce(2));
        a = A * A;
        b = 2.0 * A * B;
        c = B * B - C * C;
        sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        if (sol_eta1 > sol_eta2) // sol_eta1 ÀÌ upper boundary
        {
            if (sol_eta1 < eta_ub)
                eta_ub = sol_eta1;

            if (sol_eta2 > eta_lb)
                eta_lb = sol_eta2;
        }
        else // sol_eta2 ÀÌ upper boundary
        {
            if (sol_eta2 < eta_ub)
                eta_ub = sol_eta2;

            if (sol_eta1 > eta_lb)
                eta_lb = sol_eta1;
        }

        // printf("5 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);
        ///////////////////////////////////////////////////////////////////////////////////////////////////////////
        // boundary of eta Mz, (A^2-C^2)*eta^2 + 2*A*B*eta + B^2 < 0
        A = -(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0);
        B = ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1);
        C = staticFrictionCoeff * abs(ResultantForce(2));
        a = A * A;
        b = 2.0 * A * B;
        c = B * B - C * C;
        sol_eta1 = (-b + sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        sol_eta2 = (-b - sqrt(b * b - 4.0 * a * c)) / 2.0 / a;
        if (sol_eta1 > sol_eta2) // sol_eta1 ÀÌ upper boundary
        {
            if (sol_eta1 < eta_ub)
                eta_ub = sol_eta1;
            if (sol_eta2 > eta_lb)
                eta_lb = sol_eta2;
        }
        else // sol_eta2 ÀÌ upper boundary
        {
            if (sol_eta2 < eta_ub)
                eta_ub = sol_eta2;
            if (sol_eta1 > eta_lb)
                eta_lb = sol_eta1;
        }
        // printf("6 lb %f ub %f A %f B %f\n",eta_lb,eta_ub, sol_eta1, sol_eta2);

        double eta_s = (-ResultantForce(3) - P2(2) * ResultantForce(1) + P2(1) * ResultantForce(2)) / ((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2));

        eta = eta_s;
        if (eta_s > eta_ub)
            eta = eta_ub;
        else if (eta_s < eta_lb)
            eta = eta_lb;

        if ((eta > eta_cust) || (eta < 1.0 - eta_cust))
            eta = 0.5;

        ForceRedistribution(0) = eta * ResultantForce(0);
        ForceRedistribution(1) = eta * ResultantForce(1);
        ForceRedistribution(2) = eta * ResultantForce(2);
        ForceRedistribution(3) = ((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2)) * eta * eta + (ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2)) * eta;
        ForceRedistribution(4) = (-(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2)) * eta * eta + (ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2)) * eta;
        ForceRedistribution(5) = (-(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0)) * eta * eta + (ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1)) * eta;
        ForceRedistribution(6) = (1.0 - eta) * ResultantForce(0);
        ForceRedistribution(7) = (1.0 - eta) * ResultantForce(1);
        ForceRedistribution(8) = (1.0 - eta) * ResultantForce(2);
        ForceRedistribution(9) = (1.0 - eta) * (((P1(2) - P2(2)) * ResultantForce(1) - (P1(1) - P2(1)) * ResultantForce(2)) * eta + (ResultantForce(3) + P2(2) * ResultantForce(1) - P2(1) * ResultantForce(2)));
        ForceRedistribution(10) = (1.0 - eta) * ((-(P1(2) - P2(2)) * ResultantForce(0) + (P1(0) - P2(0)) * ResultantForce(2)) * eta + (ResultantForce(4) - P2(2) * ResultantForce(0) + P2(0) * ResultantForce(2)));
        ForceRedistribution(11) = (1.0 - eta) * ((-(P1(0) - P2(0)) * ResultantForce(1) + (P1(1) - P2(1)) * ResultantForce(0)) * eta + (ResultantForce(5) + P2(1) * ResultantForce(0) - P2(0) * ResultantForce(1)));
        // ForceRedistribution(9) = (1.0-eta)/eta*ForceRedistribution(3);
        // ForceRedistribution(10) = (1.0-eta)/eta*ForceRedistribution(4);
        // ForceRedistribution(11) = (1.0-eta)/eta*ForceRedistribution(5);
    }

}