#include "math.h"
namespace DWBC
{

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

    MatrixXd PinvCOD(const MatrixXd &A)
    {
        Eigen::CompleteOrthogonalDecomposition<MatrixXd> cod(A.rows(), A.cols());
        cod.setThreshold(COD_THRESHOLD);
        cod.compute(A);

        return cod.pseudoInverse();
    }

    void PinvCOD(const MatrixXd &A, MatrixXd &ret)
    {
        Eigen::CompleteOrthogonalDecomposition<MatrixXd> cod(A.rows(), A.cols());
        cod.setThreshold(COD_THRESHOLD);
        cod.compute(A);

        ret = cod.pseudoInverse();
    }

    void PinvCOD(const MatrixXd &A, MatrixXd &ret, MatrixXd &V2)
    {
        int rows = A.rows();
        int cols = A.cols();
        Eigen::CompleteOrthogonalDecomposition<MatrixXd> cod(rows, cols);
        cod.setThreshold(COD_THRESHOLD);
        cod.compute(A);
        int rank = cod.rank();
        MatrixXd Vtemp = cod.householderQ().transpose();
        V2 = (Vtemp).block(rank, 0, rows - rank, cols);

        ret = cod.pseudoInverse();
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

    Eigen::Matrix3d AxisTransform2V(const Vector3d &from, const Vector3d &to)
    {
        Eigen::Vector3d ori, tar;

        ori = from.normalized();

        tar = to.normalized();

        return AngleAxisd(acos(ori.dot(tar) / (ori.squaredNorm() * tar.squaredNorm())), ori.cross(tar).normalized()).matrix();
    }

    const char *bool_cast(const bool b)
    {
        return b ? "true" : "false";
    }
}