#ifndef WBHQP_WBDYNAMICS_H
#define WBHQP_WBDYNAMICS_H

#include <Eigen/Dense>

#define CONTACT_CONSTRAINT_ZMP 4
#define CONTACT_CONSTRAINT_FORCE 6
#define CONTACT_CONSTRAINT_PRESS 3
#define COD_THRESHOLD 1.0E-6

using namespace Eigen;

namespace DWBC
{

    /*
     * pseudo inverse function
     */
    MatrixXd PinvCODWB(const MatrixXd &A);

    void PinvCODWB(const MatrixXd &A, MatrixXd &ret);

    void PinvCODWB(const MatrixXd &A, MatrixXd &ret, MatrixXd &V2);

    /*
     * Contact Constraint Computation
     *
     */
    MatrixXd GetZMPConstMatrix(double local_contact_plane_x, double local_contact_plane_y);

    MatrixXd GetForceConstMatrix(double friction_ratio_x, double friction_ratio_y, double friction_ratio_z);
    // MatrixXd GetContactToPointAxis()
    // {

    // }

    /*
     * With contact jacobian(j_contact) and inverse of mass matrix(A_inv), calculate contact constraint matrix
     * input  : j_c, A_inv
     * output : lambda_c, J_c_inv_T, N_c, W, W_inv, V2, NwJw
     */
    void CalculateContactConstraint(
        const MatrixXd &J_contact, const MatrixXd &A_inv,
        MatrixXd &lambda_contact, MatrixXd &J_C_INV_T, MatrixXd &N_C, MatrixXd &W, MatrixXd &NwJw, MatrixXd &Winv, MatrixXd &V2);

    /*
     * Gravity Compensation Torque Calculation
     * input  : A_inv, W_inv, N_C, J_C_INV_T, G
     * output : torque_grav,
     */
    void CalculateGravityCompensation(
        const MatrixXd &A_inv, const MatrixXd &W_inv, const MatrixXd &N_C, const MatrixXd &J_C_INV_T, const VectorXd &G,
        VectorXd &torque_grav, MatrixXd &P_C);

    /*
     * Temporary
     *
     */
    void CalculateLQ(const MatrixXd &W_in);

    /*
     * Get Jkt
     * input  : J_task, A_inv, N_C, W_inv,
     * output : J_kt, lambda_task
     */
    void CalculateJKT(const MatrixXd &J_task, const MatrixXd &A_inv, const MatrixXd &N_C, const MatrixXd &W_inv,
                      MatrixXd &J_kt, MatrixXd &lambda_task);

    /*
     * Calculate Taskspace Null matrix
     * input  : J_kt, lambda_task, J_task, A_inv, N_C, upper_task_null
     * output : Null_task
     */
    void CalculateTaskNullSpace(const MatrixXd &J_kt, const MatrixXd &Lambda_task, const MatrixXd &J_task, const MatrixXd &A_inv, const MatrixXd &N_C, const MatrixXd &prev_null, MatrixXd &Null_task);
    /*
     * Calculate contact force from command torque
     * input  : command_torque, J_C_INV_T, P_C
     * output : ContactForce
     */
    void CalculateContactForce(const VectorXd &command_torque, const MatrixXd &J_C_INV_T, const MatrixXd &P_C, VectorXd &ContactForce);

}
#endif