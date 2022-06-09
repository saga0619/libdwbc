#include <chrono>
#include <dwbc.h>
#include <iomanip>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <atomic>
#include <unistd.h>

std::condition_variable cond_var;
std::mutex rd_lock;
std::atomic<bool> running;

std::condition_variable cond_var2;
std::mutex rd_lock2;

std::atomic_flag prod_complete;
std::atomic_flag calc_complete;

std::atomic<int> prod_count;
std::atomic<int> calc_count;

class tlocker
{
public:
    void producer_lock()
    {
        while (lock.test_and_set(std::memory_order_acquire))
        {
        }
    }

    void producer_ready()
    {
        counter++;
        lock.clear(std::memory_order_release);
    }

    void consumer_wait()
    {
        while (true)
        {
            if (counter > 0)
            {
                while (lock.test_and_set(std::memory_order_acquire))
                {
                }

                counter--;
                break;
            }
            else
            {
                asm("pause");
            }

            // if (!lock.test_and_set(std::memory_order_acquire))
            // {
            //     if (counter > 0)
            //     {
            //         counter--;

            //         break;
            //     }
            //     else
            //     {
            //         lock.clear(std::memory_order_release);
            //         std::this_thread::sleep_for(std::chrono::microseconds(20));
            //     }
            // }
        }
    }

    void consumer_done()
    {
        lock.clear(std::memory_order_release);
    }

private:
    std::atomic_flag lock = ATOMIC_FLAG_INIT;
    volatile int counter = 0;
};

tlocker tlock1;
tlocker tlock2;

class MyRobotData : public DWBC::RobotData
{
public:
    MyRobotData() : RobotData()
    {
    }

    int custum_data;
    double control_time;

    VectorXd torque_command_;

    // void CopyKinematicsData(MyRobotData &drd)
    // {
    //     RobotData::CopyKinematicsData(drd);
    // }
};

const int total_run = 10000;

void producer(MyRobotData &mrd, MyRobotData &global_rd)
{

    VectorXd q;
    VectorXd qdot;
    VectorXd qddot;

    q.setZero(mrd.model_.q_size);
    qdot.setZero(mrd.model_.qdot_size);
    qddot.setZero(mrd.model_.qdot_size);

    q << 0, 0, 0.92983, 0, 0, 0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0, 0, 0,
        0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;

    VectorXd qmod;
    qmod.setZero(mrd.model_.q_size);

    VectorXd qr;

    auto t_start = std::chrono::steady_clock::now();

    auto t500us = std::chrono::microseconds(500);
    int t_count = 0;

    long t_total = 0;

    prod_count = 0;
    while (t_count < total_run)
    {

        std::this_thread::sleep_until(t_start + t500us * t_count);

        qmod.setRandom();
        qr = q + qmod * 0.01;
        qr.segment(3, 3).setZero();
        qr(mrd.system_dof_) = 1;

        auto t_now = std::chrono::steady_clock::now();

        mrd.UpdateKinematics(qr, qdot, qddot);

        tlock1.producer_lock();
        mrd.CopyKinematicsData(global_rd);
        tlock1.producer_ready();

        tlock2.consumer_wait();
        mrd.torque_command_ = global_rd.torque_command_;
        tlock2.consumer_done();

        auto t_dur = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t_now).count();

        t_total += t_dur;

        t_count++;
    }

    running = false;

    std::cout << "producer end" << t_total / t_count << std::endl;
}

void consumer(MyRobotData &local_rd, MyRobotData &global_rd)
{
    int c_count = 0;

    bool init_ = true;

    while (running)
    {
        tlock1.consumer_wait();
        global_rd.CopyKinematicsData(local_rd);
        tlock1.consumer_done();

        // local_rd.UpdateKinematics(local_rd.q_system_, local_rd.q_dot_system_, local_rd.q_ddot_system_, false);
        local_rd.SetContact(true, true);

        local_rd.CalcGravCompensation(); // Calulate Gravity Compensation
        local_rd.CalcTaskControlTorque(init_);
        local_rd.CalcContactRedistribute(init_);

        local_rd.torque_command_ = local_rd.torque_contact_ + local_rd.torque_grav_ + local_rd.torque_task_;

        tlock2.producer_lock();
        global_rd.torque_command_ = local_rd.torque_command_;
        tlock2.producer_ready();

        c_count++;

        init_ = false;

        if (c_count == total_run)
            break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    std::cout << "cons end : " << c_count << std::endl;
}

void alone(MyRobotData &rd1, MyRobotData &rd2)
{
    auto ts = std::chrono::steady_clock::now();

    auto t500us = std::chrono::microseconds(500);
    int t_count = 0;

    long t_total = 0;

    int init2 = true;

    VectorXd q;
    VectorXd qdot;
    VectorXd qddot;

    q.setZero(rd1.model_.q_size);
    qdot.setZero(rd1.model_.qdot_size);
    qddot.setZero(rd1.model_.qdot_size);

    q << 0, 0, 0.92983, 0, 0, 0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0, 0, 0,
        0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;

    VectorXd qmod;
    qmod.setZero(rd1.model_.q_size);

    VectorXd qr;

    while (t_count < 10000)
    {

        std::this_thread::sleep_until(ts + t500us * t_count);

        qmod.setRandom();
        qr = q + qmod * 0.01;
        qr.segment(3, 3).setZero();
        qr(rd1.system_dof_) = 1;

        auto t_now = std::chrono::steady_clock::now();

        rd1.UpdateKinematics(qr, qdot, qddot);

        rd1.CopyKinematicsData(rd2);

        rd1.CopyKinematicsData(rd2);

        rd2.SetContact(true, true);

        rd2.CalcGravCompensation(); // Calulate Gravity Compensation
        rd2.CalcTaskControlTorque(init2);
        rd2.CalcContactRedistribute(init2);

        rd2.torque_command_ = rd2.torque_contact_ + rd2.torque_grav_ + rd2.torque_task_;

        auto t_dur = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t_now).count();

        t_total += t_dur;

        t_count++;

        init2 = false;
    }

    std::cout << "nopc : " << t_total / t_count << std::endl;
}

using namespace DWBC;

int main()
{
    running = true;

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

    int repeat_time = 1000;

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

    // Gen Matrix File
    rd_.check_mat_file_ = true;
    // rd_.save_mat_file_ = true;

    rd_.UpdateKinematics(q, qdot, qddot);
    rd_.SetContact(true, true);

    rd_.SetTaskSpace(0, fstar_1);
    rd_.SetTaskSpace(1, fstar_1.segment(3, 3));

    rd_.CalcGravCompensation(); // Calulate Gravity Compensation
    rd_.CalcTaskControlTorque(true);
    rd_.CalcContactRedistribute(true);
    rd_.check_mat_file_ = false;

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
        rd_.UpdateKinematics(qr, qdot, qddot, false);

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

        // if (c_res == 0)
        // {
        //     std::cout << "error at " << i << std::endl;
        //     std::cout << qr.transpose() << std::endl;
        // }

        // std::cout << std::endl
        //           << std::endl
        //           << i << "contact calc" << std::endl
        //           << std::endl;

        auto t4 = std::chrono::high_resolution_clock::now();
        c_res2 = rd_.CalcContactRedistribute(init);
        calc_cr_res += c_res2;
        auto t5 = std::chrono::high_resolution_clock::now();

        if (c_res2 == 0)
            break;

        // std::cout << std::endl
        //           << std::endl
        //           << i << "done" << std::endl
        //           << std::endl;
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

    // std::cout << " fstar qp : " << rd_.ts_[0].f_star_qp_.transpose() << std::endl;
    // std::cout << " Grav Torque : " << rd_.torque_grav_.transpose() << std::endl;
    // std::cout << " Task Torque : " << rd_.torque_task_.transpose() << std::endl;
    // std::cout << "contact Torque : " << rd_.torque_contact_.transpose() << std::endl;
    // std::cout << "contact force after : " << rd_.getContactForce(rd_.torque_grav_ + rd_.torque_task_ + rd_.torque_contact_).transpose() << std::endl;

    MyRobotData rd2_ = MyRobotData();

    rd_.CopyKinematicsData(rd2_);

    MyRobotData rd_global_;
    MyRobotData rd_producer_;
    MyRobotData rd_consumer_;

    rd_global_ = rd_;
    rd_producer_ = rd_;
    rd_consumer_ = rd_;

    rd2_ = rd_;

    MyRobotData rd3 = rd_;

    std::thread t1;
    std::thread t2;
    std::thread t3;

    t3 = std::thread(alone, std::ref(rd2_), std::ref(rd3));
    t3.join();

    t1 = std::thread(producer, std::ref(rd_producer_), std::ref(rd_global_));
    t2 = std::thread(consumer, std::ref(rd_consumer_), std::ref(rd_global_));

    t1.join();
    t2.join();

    // std::cout << rd2_.A_ - rd_.A_ << std::endl;
    // std::cout << std::endl;
    // std::cout << rd2_.A_inv_ - rd_.A_inv_ << std::endl;
    // std::cout << std::endl;
    // std::cout << "what?" << std::endl;

    //

    return 0;
}
