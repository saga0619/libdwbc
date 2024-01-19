#include "dwbc_math.h"

using namespace Eigen;



namespace DWBC
{
    Matrix3d skew(const Vector3d &src)
    {
        Matrix3d skm;
        skm.setZero();
        skm(0, 1) = -src[2];
        skm(0, 2) = src[1];
        skm(1, 0) = src[2];
        skm(1, 2) = -src[0];
        skm(2, 0) = -src[1];
        skm(2, 1) = src[0];

        return skm;
    }

    MatrixXd PinvCOD(const MatrixXd &A)
    {
        CompleteOrthogonalDecomposition<MatrixXd> cod(A.rows(), A.cols());
        cod.setThreshold(COD_THRESHOLD);
        cod.compute(A);

        return cod.pseudoInverse();
    }

    void PinvCOD(const MatrixXd &A, MatrixXd &ret)
    {
        CompleteOrthogonalDecomposition<MatrixXd> cod(A.rows(), A.cols());
        cod.setThreshold(COD_THRESHOLD);
        cod.compute(A);

        ret = cod.pseudoInverse();
    }

    void PinvCOD(const MatrixXd &A, MatrixXd &ret, MatrixXd &V2)
    {
        int rows = A.rows();
        int cols = A.cols();
        CompleteOrthogonalDecomposition<MatrixXd> cod(rows, cols);
        cod.setThreshold(COD_THRESHOLD);
        cod.compute(A);
        int rank = cod.rank();
        MatrixXd Vtemp = cod.householderQ().transpose();
        V2 = (Vtemp).block(rank, 0, rows - rank, cols);

        ret = cod.pseudoInverse();
    }

    Matrix3d rotateWithZ(double yaw_angle)
    {
        Matrix3d rotate_wth_z(3, 3);

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
    Matrix3d rotateWithY(double pitch_angle)
    {
        Matrix3d rotate_wth_y(3, 3);

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

    Matrix3d rotateWithX(double roll_angle)
    {
        Matrix3d rotate_wth_x(3, 3);

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

    Matrix3d AxisTransform2V(const Vector3d &from, const Vector3d &to)
    {
        Vector3d ori, tar;

        ori = from.normalized();

        tar = to.normalized();

        return AngleAxisd(acos(ori.dot(tar) / (ori.squaredNorm() * tar.squaredNorm())), ori.cross(tar).normalized()).matrix();
    }

    const char *bool_cast(const bool b)
    {
        return b ? "true" : "false";
    }

    Vector3d QuinticSpline(
        double time,     ///< Current time
        double time_0,   ///< Start time
        double time_f,   ///< End time
        double x_0,      ///< Start state
        double x_dot_0,  ///< Start state dot
        double x_ddot_0, ///< Start state ddot
        double x_f,      ///< End state
        double x_dot_f,  ///< End state
        double x_ddot_f) ///< End state ddot
    {
        double a1, a2, a3, a4, a5, a6;
        double time_s;

        Eigen::Vector3d result;

        if (time < time_0)
        {
            result << x_0, x_dot_0, x_ddot_0;
            return result;
        }
        else if (time > time_f)
        {
            result << x_f, x_dot_f, x_ddot_f;
            return result;
        }

        time_s = time_f - time_0;
        a1 = x_0;
        a2 = x_dot_0;
        a3 = x_ddot_0 / 2.0;

        Eigen::Matrix3d Temp;
        Temp << pow(time_s, 3), pow(time_s, 4), pow(time_s, 5),
            3.0 * pow(time_s, 2), 4.0 * pow(time_s, 3), 5.0 * pow(time_s, 4),
            6.0 * time_s, 12.0 * pow(time_s, 2), 20.0 * pow(time_s, 3);

        Eigen::Vector3d R_temp;
        R_temp << x_f - x_0 - x_dot_0 * time_s - x_ddot_0 * pow(time_s, 2) / 2.0,
            x_dot_f - x_dot_0 - x_ddot_0 * time_s,
            x_ddot_f - x_ddot_0;

        Eigen::Vector3d RES;

        RES = Temp.inverse() * R_temp;

        a4 = RES(0);
        a5 = RES(1);
        a6 = RES(2);

        double time_fs = time - time_0;

        double position = a1 + a2 * pow(time_fs, 1) + a3 * pow(time_fs, 2) + a4 * pow(time_fs, 3) + a5 * pow(time_fs, 4) + a6 * pow(time_fs, 5);
        double velocity = a2 + 2.0 * a3 * pow(time_fs, 1) + 3.0 * a4 * pow(time_fs, 2) + 4.0 * a5 * pow(time_fs, 3) + 5.0 * a6 * pow(time_fs, 4);
        double acceleration = 2.0 * a3 + 6.0 * a4 * pow(time_fs, 1) + 12.0 * a5 * pow(time_fs, 2) + 20.0 * a6 * pow(time_fs, 3);

        result << position, velocity, acceleration;

        return result;
    }
    double cubic(double time,    ///< Current time
                 double time_0,  ///< Start time
                 double time_f,  ///< End time
                 double x_0,     ///< Start state
                 double x_f,     ///< End state
                 double x_dot_0, ///< Start state dot
                 double x_dot_f  ///< End state dot
    )
    {
        double x_t;

        if (time < time_0)
        {
            x_t = x_0;
        }
        else if (time > time_f)
        {
            x_t = x_f;
        }
        else
        {
            double elapsed_time = time - time_0;
            double total_time = time_f - time_0;
            double total_time2 = total_time * total_time;  // pow(t,2)
            double total_time3 = total_time2 * total_time; // pow(t,3)
            double total_x = x_f - x_0;

            x_t = x_0 + x_dot_0 * elapsed_time

                  + (3 * total_x / total_time2 - 2 * x_dot_0 / total_time - x_dot_f / total_time) * elapsed_time * elapsed_time

                  + (-2 * total_x / total_time3 +
                     (x_dot_0 + x_dot_f) / total_time2) *
                        elapsed_time * elapsed_time * elapsed_time;
        }

        return x_t;
    }

    Matrix3d rotationCubic(double time,
                           double time_0,
                           double time_f,
                           const Eigen::Matrix3d &rotation_0,
                           const Eigen::Matrix3d &rotation_f)
    {
        if (time >= time_f)
        {
            return rotation_f;
        }
        else if (time < time_0)
        {
            return rotation_0;
        }
        double tau = cubic(time, time_0, time_f, 0, 1, 0, 0);
        // Eigen::Matrix3d rot_scaler_skew;
        // rot_scaler_skew = (rotation_0.transpose() * rotation_f).log();
        // rot_scaler_skew = rot_scaler_skew.log();
        /*
        Eigen::Matrix3d rotation_exp;
        Eigen::Vector3d a1, b1, c1, r1;
        r1(0) = rotation_temp(2,1);
        r1(1) = rotation_temp(0,2);
        r1(2) = rotation_temp(1,0);
        c1.setZero(); // angular velocity at t0 --> Zero
        b1.setZero(); // angular acceleration at t0 --> Zero
        a1 = r1 - b1 - c1;
        //double tau = (time - time_0) / (time_f-time_0);
        double tau2 = tau*tau;
        double tau3 = tau2*tau;
        //Eigen::Vector3d exp_vector = (a1*tau3+b1*tau2+c1*tau);
        Eigen::Vector3d exp_vector = (a1*tau);
        rotation_exp.setZero();
        rotation_exp(0,1) = -exp_vector(2);
        rotation_exp(0,2) =  exp_vector(1);
        rotation_exp(1,0) =  exp_vector(2);
        rotation_exp(1,2) = -exp_vector(0);
        rotation_exp(2,0) = -exp_vector(1);
        rotation_exp(2,1) =  exp_vector(0);
        */
        // Eigen::Matrix3d result = rotation_0 * rotation_exp.exp();
        // Eigen::Matrix3d result = rotation_0 * (rot_scaler_skew * tau).exp();
        Eigen::AngleAxisd ang_diff(rotation_f * rotation_0.transpose());
        Eigen::Matrix3d diff_m;
        diff_m = Eigen::AngleAxisd(ang_diff.angle() * tau, ang_diff.axis());
        Eigen::Matrix3d result = diff_m * rotation_0;

        return result;
    }

    Vector3d GetPhi(Matrix3d current_rotation, Matrix3d desired_rotation)
    {
        Eigen::Vector3d phi;
        Eigen::Vector3d s[3], v[3], w[3];

        for (int i = 0; i < 3; i++)
        {
            v[i] = current_rotation.block<3, 1>(0, i);
            w[i] = desired_rotation.block<3, 1>(0, i);
            s[i] = v[i].cross(w[i]);
        }
        phi = s[0] + s[1] + s[2];
        phi = -0.5 * phi;

        return -phi;
    }

    MatrixXd spatialTransformMatrix(const Vector3d &pos, const Matrix3d &rot)
    {
        Eigen::MatrixXd sp_mat = Eigen::MatrixXd::Zero(6, 6);

        sp_mat.block<3, 3>(0, 0) = rot;
        sp_mat.block<3, 3>(3, 3) = rot;
        sp_mat.block<3, 3>(0, 3) = skew(pos) * rot;

        return sp_mat;
    }

    void InertiaMatrixSegment(const Matrix6d &inertia_matrix, Matrix3d &inertia, Vector3d &mass_center, double &mass)
    {
        mass = inertia_matrix(0, 0);
        Matrix3d skm_temp = inertia_matrix.block(3, 0, 3, 3) / mass;
        mass_center << skm_temp(2, 1), skm_temp(0, 2), skm_temp(1, 0);
        inertia = inertia_matrix.block(3, 3, 3, 3) - mass * skew(mass_center) * skew(mass_center).transpose();
    }

    Matrix6d InertiaMatrix(const Matrix3d &inertia, const double &mass)
    {
        Matrix6d inertia_matrix;
        inertia_matrix.setZero();
        inertia_matrix.block(0, 0, 3, 3) = mass * Matrix3d::Identity();
        inertia_matrix.block(3, 3, 3, 3) = inertia;

        return inertia_matrix;
    }

    //Rotationmatrix to quaternion using Eigen
    Eigen::Quaterniond Rot2Quat(const Eigen::Matrix3d &Rot)
    {
        Eigen::Quaterniond quat;
        quat = Rot;
        return quat;
    }



}
