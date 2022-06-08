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
}

#endif