/*
 *  WBCL - WholeBody Control Library
 * Copyright (c) 2022 - Junewhee Ahn <june992@gmail.com>
 *
 * Licensed under the ...
 */

#ifndef WBHQP_MATH_H
#define WBHQP_MATH_H

#include <Eigen/Dense>
#include <iostream>

#define COD_THRESHOLD 1.0E-6

using namespace Eigen;
namespace DWBC{
    
Matrix3d skew(const Eigen::Vector3d &src);
MatrixXd PinvCOD(const MatrixXd &A);
void PinvCOD(const MatrixXd &A, MatrixXd &ret);
void PinvCOD(const MatrixXd &A, MatrixXd &ret, MatrixXd &V2);
Eigen::Matrix3d rotateWithZ(double yaw_angle);
Eigen::Matrix3d rotateWithY(double pitch_angle);
Eigen::Matrix3d rotateWithX(double roll_angle);
Eigen::Matrix3d AxisTransform2V(const Vector3d &from, const Vector3d &to);
const char *bool_cast(const bool b);
}

#endif