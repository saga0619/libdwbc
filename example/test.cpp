#include <chrono>
#include <dwbc/dwbc.h>
#include <unistd.h>
#include <iomanip>
#ifndef URDF_DIR
#define URDF_DIR ".."
#endif

class MyRobotData : public DWBC::RobotData
{
public:
    MyRobotData() : RobotData()
    {
    }

    int custum_data;
    double control_time;

    VectorXd torque_command_;

};

const int total_run = 10000;

using namespace DWBC;
int main()
{
    // running = true;

    MyRobotData rd_;

    std::string resource_path = URDF_DIR;

    std::string urdf_name = "/dyros_tocabi.urdf";

    std::string urdf_path = resource_path + urdf_name;

    rd_.InitModelData(urdf_path, true, false);

    VectorXd q;
    VectorXd qdot;
    VectorXd qddot;

    q.setZero(rd_.model_.q_size);
    qdot.setZero(rd_.model_.qdot_size);
    qddot.setZero(rd_.model_.qdot_size);

    MatrixXd j_tmp;

    int repeat_time = 10000;

    auto t_start = std::chrono::high_resolution_clock::now();

    int mid_consume = 0;

    VectorXd qmod;
    qmod.setZero(rd_.model_.q_size);

    for (int i = 0; i < repeat_time; i++)
    {
        q.setRandom();
        qdot.setRandom();
        rd_.UpdateKinematics(q, qdot, qddot);
        // std::cout << Vector3d(0, 0, -9.81) << std::endl;
        // auto t_midPoint = std::chrono::high_resolution_clock::now();
        // for (int i = 0; i < rd_.link_.size(); i++)
        // {
        //     j_tmp = rd_.link_[i].JacCOM();
        // }
        // mid_consume += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t_midPoint).count();
    }

    auto t_dur = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t_start);

    std::cout << "Update Kinematics Random q Repeat test : " << t_dur.count() / repeat_time << " us" << std::endl;

    q.setZero();
    q << 0, 0, 0.92983, 0, 0, 0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0, 0, 0,
        0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;

    // q << 0, 0, 0, 8.72949e-06, -0.000191618, -3.56927e-07, 7.21486e-07, -1.14801e-07, -0.233464, 0.581483, -0.348292, -1.31239e-06, -1.15455e-07, -4.6496e-07, -0.233484, 0.581517, -0.348293, -1.16079e-06, -3.29457e-07, -0.000329052, -1.22101e-07, 0.300002, 0.300095, 1.5, -1.27, -1.00115, 2.3016e-07, -1, -3.5045e-08, 1.36636e-06, -0.000406735, -0.300001, -0.300095, -1.5, 1.27, 1.00115, -4.30943e-08, 1, 5.41439e-09, 1;

    qdot.setZero();
    rd_.UpdateKinematics(q, qdot, qddot);

    int left_foot_id = 6;
    int right_foot_id = 12;

    rd_.AddContactConstraint(left_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    rd_.AddContactConstraint(right_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    rd_.AddContactConstraint(23, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);
    rd_.AddContactConstraint(31, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);

    rd_.AddTaskSpace(TASK_LINK_6D, 0, Vector3d::Zero());
    rd_.AddTaskSpace(TASK_LINK_ROTATION, 15, Vector3d::Zero());

    VectorXd fstar_1;
    fstar_1.setZero(6);
    fstar_1(0) = 0.1;
    fstar_1(1) = 4.0;
    fstar_1(2) = 0.1;

    fstar_1(3) = 0.1;
    fstar_1(4) = -0.1;
    fstar_1(5) = 0.1;

    bool init = true;

    long ns_uk = 0;
    long ns_sc = 0;
    long ns_gc = 0;
    long ns_ct = 0;
    long ns_cr = 0;

    // Quaterniond quat_random = Quaterniond()

    t_start = std::chrono::high_resolution_clock::now();
    VectorXd qr = q;

    VectorXd tlim;

    tlim.setConstant(rd_.model_dof_, 300);

    rd_.SetTorqueLimit(tlim);

    int calc_task_res = 0;
    int calc_cr_res = 0;

    int c_res = 0;
    int c_res2 = 0;
#ifdef CHECKDATA
    std::cout << " -----------------------------------------------" << std::endl;

    std::cout << " ---- DYNAMICS MATRIX VERIFICATION ---- " << std::endl;

    MyRobotData rd2_;
    rd2_.InitModelData(urdf_path, true, false);
    // Gen Matrix File
    rd2_.check_mat_file_ = true;
    rd_.UpdateKinematics(q, qdot, qddot);

    rd_.CopyKinematicsData(rd2_);
    // rd_.save_mat_file_ = true;

    rd2_.AddContactConstraint(left_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    rd2_.AddContactConstraint(right_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);

    rd2_.AddTaskSpace(TASK_LINK_6D, 0, Vector3d::Zero());
    rd2_.AddTaskSpace(TASK_LINK_ROTATION, 15, Vector3d::Zero());

    rd2_.SetContact(true, true);

    rd2_.SetTaskSpace(0, fstar_1);
    rd2_.SetTaskSpace(1, fstar_1.segment(3, 3));

    rd2_.CalcGravCompensation(); // Calulate Gravity Compensation
    rd2_.CalcTaskControlTorque(true);
    rd2_.CalcContactRedistribute(true);
    rd2_.check_mat_file_ = false;

    // rd_.save_mat_file_ = false;
    // int rows = rd_.W.rows();
    // int cols = rd_.W.cols();

    // Eigen::CompleteOrthogonalDecomposition<MatrixXd> cod(rows, cols);
    // cod.compute(rd_.W);

    // int rank = cod.rank();

    // MatrixXd mat_temp = cod.matrixT().topLeftCorner(rank, rank).template triangularView<Upper>();

    // std::cout << "rank : " << rank << std::endl
    //           << "cod z: " << std::endl;
    // std::cout << cod.matrixZ() << std::endl
    //         //   << "cod z: " << std::endl;
    // // std::cout << cod.matrixQ() << std::endl
    //           << "cod T: " << std::endl;
    // std::cout <<  mat_temp<< std::endl
    //           << "cod QTZ: " << std::endl;
    // std::cout << cod.matrixQTZ() << std::endl;
    // std::cout << "v2 : " << std::endl;
    // std::cout << rd_.V2 << std::endl;
#endif
    std::cout << " -----------------------------------------------" << std::endl;

    std::cout << " ----STARTING OSF REPEAT TEST---- " << std::endl;

    for (int i = 0; i < repeat_time; i++)
    {
        qmod.setRandom();

        qr = q + qmod * 0.01;
        qr.segment(3, 3).setZero();
        qr(rd_.system_dof_) = 1;

        auto t0 = std::chrono::high_resolution_clock::now();
        rd_.UpdateKinematics(qr, qdot, qddot);

        auto t1 = std::chrono::high_resolution_clock::now();
        rd_.SetContact(true, true);

        rd_.SetTaskSpace(0, fstar_1);
        rd_.SetTaskSpace(1, fstar_1.segment(3, 3));

        auto t2 = std::chrono::high_resolution_clock::now();
        rd_.CalcGravCompensation(); // Calulate Gravity Compensation
        // rd_.CalcTaskSpaceTorqueHQPWithThreaded(init); // Calculate Task Spaces...

        auto t3 = std::chrono::high_resolution_clock::now();
        c_res = rd_.CalcTaskControlTorque(init);
        calc_task_res += c_res;

        if (c_res == 0)
            break;

        auto t4 = std::chrono::high_resolution_clock::now();
        c_res2 = rd_.CalcContactRedistribute(init);
        calc_cr_res += c_res2;
        auto t5 = std::chrono::high_resolution_clock::now();

        if (c_res2 == 0)
            break;

        init = false;

        ns_uk += std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
        ns_sc += std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
        ns_gc += std::chrono::duration_cast<std::chrono::nanoseconds>(t3 - t2).count();
        ns_ct += std::chrono::duration_cast<std::chrono::nanoseconds>(t4 - t3).count();
        ns_cr += std::chrono::duration_cast<std::chrono::nanoseconds>(t5 - t4).count();
    }

    t_dur = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t_start);
    std::cout << " -----------------------------------------------" << std::endl;

    std::cout << " ----function repeat performance test result---- " << std::endl;

    int rp_ns = repeat_time * 1000;

    std::cout << std::setw(30) << std::right << "UpdateKinematics : " << ns_uk / rp_ns << " us" << std::endl;
    std::cout << std::setw(30) << std::right << "SetContact : " << ns_sc / rp_ns << " us" << std::endl;
    std::cout << std::setw(30) << std::right << "CalcGravCompensation : " << ns_gc / rp_ns << " us" << std::endl;
    std::cout << std::setw(30) << std::right << "CalcTaskTorque : " << ns_ct / rp_ns << " us" << std::endl;
    std::cout << std::setw(30) << std::right << "CalcContactRedistribute : " << ns_cr / rp_ns << " us" << std::endl;
    std::cout << std::setw(30) << std::right << "TOTAL : " << t_dur.count() / repeat_time << " us" << std::endl;

    std::cout << " -----------------------------------------------" << std::endl;

    std::cout << "qptask success : " << calc_task_res << std::endl;
    std::cout << "contact redis  qp : " << calc_cr_res << std::endl;

    std::cout << " -----------------------------------------------" << std::endl;

    return 0;
}
