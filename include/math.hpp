/*
 *  WBCL - WholeBody Control Library
 * Copyright (c) 2022 - Junewhee Ahn <june992@gmail.com>
 *
 * Licensed under the ...
 */

#ifndef WBHQP_MATH_HPP
#define WBHQP_MATH_HPP

#include <Eigen/Dense>
#include <Eigen/QR>
#include <iostream>

using namespace Eigen;

Matrix3d skew(const Eigen::Vector3d &src)
{
    Eigen::Matrix3d skew;
    skew.setZero();
    skew(0, 1) = -src[2];
    skew(0, 2) = src[1];
    skew(1, 0) = src[2];
    skew(1, 2) = -src[0];
    skew(2, 0) = -src[1];
    skew(2, 1) = src[0];

    return skew;
}

MatrixXd PinvQR(const MatrixXd &A)
{
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(A);
    qr.setThreshold(1.0e-6);
    int rank = qr.rank();
    int cols = A.cols();
    int rows = A.rows();
    if (rank == 0)
    {
        std::cout << "WARN::Input Matrix seems to be zero matrix" << std::endl;
        return A;
    }
    else
    {
        if (cols > rows)
        {
            Eigen::MatrixXd Rpsinv2(rows, cols);
            Rpsinv2.setZero();
            Rpsinv2.topLeftCorner(rank, rank) = qr.matrixQR().topLeftCorner(rank, rank).template triangularView<Eigen::Upper>().solve(Eigen::MatrixXd::Identity(rank, rank));

            std::cout << "ERrrroooorrrrrr in pinv QR pair" << std::endl;
            return (qr.colsPermutation() * Rpsinv2 * qr.householderQ().transpose()).transpose();
        }
        else
        {
            Eigen::MatrixXd Rpsinv2(cols, rows);
            Rpsinv2.setZero();
            Rpsinv2.topLeftCorner(rank, rank) = qr.matrixQR().topLeftCorner(rank, rank).template triangularView<Eigen::Upper>().solve(Eigen::MatrixXd::Identity(rank, rank));

            Eigen::MatrixXd P;
            P = qr.householderQ().transpose();

            // V2 = P.block(rank, 0, P.rows() - rank, P.cols());

            // cols -> cols * cols
            // V2 = qr.colsPermutation().block(rank,0,cols - rank, rows - rank)

            return qr.colsPermutation() * Rpsinv2 * P;
        }
    }
}

Matrix<double, 33, 33> WinvCalc(const Matrix<double, 33, 33> &W, Eigen::MatrixXd &V2)
{
    Eigen::ColPivHouseholderQR<Eigen::Matrix<double, 33, 33>> qr(W);
    qr.setThreshold(1.0e-6);
    int cols = W.cols();
    int rows = W.rows();

    int rank = qr.rank();
    if ((rank == 33 - 18) || (rank == 33 - 12) || (rank == 33 - 6) || (rank == 33))
    {
        Eigen::Matrix<double, 33, 33> Rpsinv;
        Rpsinv.setZero();
        Rpsinv.topLeftCorner(rank, rank) = qr.matrixQR().topLeftCorner(rank, rank).template triangularView<Eigen::Upper>().solve(Eigen::MatrixXd::Identity(rank, rank));
        Eigen::Matrix<double, 33, 33> P;
        P = qr.householderQ().transpose();
        V2 = P.block(rank, 0, P.rows() - rank, P.cols());

        return qr.colsPermutation() * Rpsinv * P;
    }
    else
    {
        std::cout << "Winv Calc Error : rank = " << rank << std::endl;
        return W;
    }
}

MatrixXd PinvCOD(const MatrixXd &A)
{
    // int rows = ;
    // int cols = A.cols();

    Eigen::CompleteOrthogonalDecomposition<MatrixXd> cod(A.rows(), A.cols());
    cod.setThreshold(1.0e-6);
    cod.compute(A);
    // MatrixXd ret =

    return cod.pseudoInverse();
}

bool PinvCOD(const MatrixXd &A, MatrixXd &ret, MatrixXd &V2)
{
    int rows = A.rows();
    int cols = A.cols();
    Eigen::CompleteOrthogonalDecomposition<MatrixXd> cod(rows, cols);
    cod.setThreshold(1.0e-6);
    cod.compute(A);
    int rank = cod.rank();

    // V2 = cod.matrixZ().bottomRows(rows - rank);

    MatrixXd Vtemp = cod.householderQ().transpose();

    V2 = (Vtemp).block(rank, 0, rows - rank, cols);

    ret = cod.pseudoInverse();

    return true;
}

bool PinvQR(const MatrixXd &A, MatrixXd &ret, MatrixXd &V2)
{
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(A.rows(), A.cols());
    qr.setThreshold(1.0e-6);

    qr.compute(A);

    int rank = qr.rank();

    int cols, rows;

    cols = A.cols();
    rows = A.rows();

    std::cout << "rank: " << rank << std::endl;

    if (rank == 0)
    {
        std::cout << "WARN::Input Matrix seems to be zero matrix" << std::endl;
        ret = A;

        return false;
    }
    else
    {
        if (cols > rows)
        {
            Eigen::MatrixXd Rpsinv2(rows, cols);
            Rpsinv2.setZero();
            Rpsinv2.topLeftCorner(rank, rank) = qr.matrixQR().topLeftCorner(rank, rank).template triangularView<Eigen::Upper>().solve(Eigen::MatrixXd::Identity(rank, rank));

            std::cout << "ERrrroooorrrrrr in pinv QR pair" << std::endl;
            ret = (qr.colsPermutation() * Rpsinv2 * qr.householderQ().transpose()).transpose();

            return true;
        }
        else
        {
            Eigen::MatrixXd Rpsinv2(cols, rows);
            Rpsinv2.setZero();
            Rpsinv2.topLeftCorner(rank, rank) = qr.matrixQR().topLeftCorner(rank, rank).template triangularView<Eigen::Upper>().solve(Eigen::MatrixXd::Identity(rank, rank));

            Eigen::MatrixXd P_q(rows, rows);
            P_q.setZero();
            P_q = qr.householderQ().setLength(qr.nonzeroPivots()).transpose();

            V2 = P_q.block(rank, 0, P_q.rows() - rank, P_q.cols());

            ret = qr.colsPermutation() * Rpsinv2 * P_q;
            return true;
        }
    }
}
Eigen::Matrix3d rotateWithZ(double yaw_angle)
{
    Eigen::Matrix3d rotate_wth_z(3, 3);

    rotate_wth_z(0, 0) = cos(yaw_angle);
    rotate_wth_z(1, 0) = sin(yaw_angle);
    rotate_wth_z(2, 0) = 0.0;

    rotate_wth_z(0, 1) = -sin(yaw_angle);
    rotate_wth_z(1, 1) = cos(yaw_angle);
    rotate_wth_z(2, 1) = 0.0;

    rotate_wth_z(0, 2) = 0.0;
    rotate_wth_z(1, 2) = 0.0;
    rotate_wth_z(2, 2) = 1.0;

    return rotate_wth_z;
}
Eigen::Matrix3d rotateWithY(double pitch_angle)
{
    Eigen::Matrix3d rotate_wth_y(3, 3);

    rotate_wth_y(0, 0) = cos(pitch_angle);
    rotate_wth_y(1, 0) = 0.0;
    rotate_wth_y(2, 0) = -sin(pitch_angle);

    rotate_wth_y(0, 1) = 0.0;
    rotate_wth_y(1, 1) = 1.0;
    rotate_wth_y(2, 1) = 0.0;

    rotate_wth_y(0, 2) = sin(pitch_angle);
    rotate_wth_y(1, 2) = 0.0;
    rotate_wth_y(2, 2) = cos(pitch_angle);

    return rotate_wth_y;
}

Eigen::Matrix3d rotateWithX(double roll_angle)
{
    Eigen::Matrix3d rotate_wth_x(3, 3);

    rotate_wth_x(0, 0) = 1.0;
    rotate_wth_x(1, 0) = 0.0;
    rotate_wth_x(2, 0) = 0.0;

    rotate_wth_x(0, 1) = 0.0;
    rotate_wth_x(1, 1) = cos(roll_angle);
    rotate_wth_x(2, 1) = sin(roll_angle);

    rotate_wth_x(0, 2) = 0.0;
    rotate_wth_x(1, 2) = -sin(roll_angle);
    rotate_wth_x(2, 2) = cos(roll_angle);

    return rotate_wth_x;
}

const char *bool_cast(const bool b)
{
    return b ? "true" : "false";
}

#endif