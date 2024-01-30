/*
 *  WBCL - WholeBody Control Library
 * Copyright (c) 2022 - Junewhee Ahn <june992@gmail.com>
 *
 * Licensed under the ...
 */

#ifndef WBHQP_MATH_H
#define WBHQP_MATH_H

#include <Eigen/Dense>

#define COD_THRESHOLD 1.0E-6

using namespace Eigen;

typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 12, 1> Vector12d;

namespace DWBC
{
    Matrix3d skew(const Vector3d &src);
    MatrixXd PinvCOD(const MatrixXd &A);
    void PinvCOD(const MatrixXd &A, MatrixXd &ret);
    void PinvCOD(const MatrixXd &A, MatrixXd &ret, MatrixXd &V2);
    Matrix3d rotateWithZ(double yaw_angle);
    Matrix3d rotateWithY(double pitch_angle);
    Matrix3d rotateWithX(double roll_angle);
    Matrix3d AxisTransform2V(const Vector3d &from, const Vector3d &to);
    const char *bool_cast(const bool b);
    Vector3d QuinticSpline(
        double time,      ///< Current time
        double time_0,    ///< Start time
        double time_f,    ///< End time
        double x_0,       ///< Start state
        double x_dot_0,   ///< Start state dot
        double x_ddot_0,  ///< Start state ddot
        double x_f,       ///< End state
        double x_dot_f,   ///< End state
        double x_ddot_f); ///< End state ddot
    Matrix3d rotationCubic(double time,
                           double time_0,
                           double time_f,
                           const Eigen::Matrix3d &rotation_0,
                           const Eigen::Matrix3d &rotation_f);

    double cubic(double time,    ///< Current time
                 double time_0,  ///< Start time
                 double time_f,  ///< End time
                 double x_0,     ///< Start state
                 double x_f,     ///< End state
                 double x_dot_0, ///< Start state dot
                 double x_dot_f  ///< End state dot
    );

    Vector3d GetPhi(Matrix3d current_rotation, Matrix3d desired_rotation);

    MatrixXd spatialTransformMatrix(const Vector3d &pos, const Matrix3d &rot);

    void InertiaMatrixSegment(const Matrix6d &inertia_matrix, Matrix3d &inertia, Vector3d &mass_center, double &mass);
    Matrix6d InertiaMatrix(const Matrix3d &inertia, const double &mass);

}

#endif