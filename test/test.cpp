#include <chrono>
#include <dwbc.h>

class MyRobotData : public DWBC::RobotData
{
    int custum_data;
    double control_time;
};

using namespace DWBC;

int main()
{
    MyRobotData rd_;

    std::string urdf_path = "../../test/dyros_tocabi.urdf";

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

    // std::cout << q.transpose() << std::endl;
    // std::cout << qdot.transpose() << std::endl;

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

    rd_.AddContactConstraint(left_foot_id, CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    rd_.AddContactConstraint(right_foot_id, CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);

    rd_.AddTaskSpace(TASK_LINK_6D, 0, Vector3d::Zero());
    rd_.AddTaskSpace(TASK_LINK_ROTATION, 15, Vector3d::Zero());
    VectorXd fstar_1;
    fstar_1.setZero(6);
    fstar_1(0) = 0.1;
    fstar_1(1) = 2.0;
    fstar_1(2) = 0.1;
    rd_.SetTaskSpace(0, fstar_1);
    rd_.SetTaskSpace(1, Vector3d::Zero());

    t_start = std::chrono::high_resolution_clock::now();

    bool init = true;

    for (int i = 0; i < repeat_time; i++)
    {
        rd_.SetContact(true, true);
        rd_.CalcTaskSpace();        // Calculate Task Spaces...
        rd_.CalcGravCompensation(); // Calulate Gravity Compensation
        rd_.CalcTaskTorque(true, init);
        rd_.CalcContactRedistribute(init);

        init = false;
    }
    t_dur = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t_start);

    std::cout << "Calc Task Repeat test : " << t_dur.count() / repeat_time << " us" << std::endl;

    std::cout << " Grav Torque : " << rd_.torque_grav_.transpose() << std::endl;
    std::cout << " Task Torque : " << rd_.torque_task_.transpose() << std::endl;
    std::cout << "contact Torque : " << rd_.torque_contact_.transpose() << std::endl;
    std::cout << "contact force after : " << rd_.getContactForce(rd_.torque_grav_ + rd_.torque_task_ + rd_.torque_contact_).transpose() << std::endl;




    Eigen::Matrix<qp_real, 3, 3> P;
    Eigen::Matrix<qp_real, 3, 1> c;
    Eigen::Matrix<qp_real, 2, 3> G;
    Eigen::Matrix<qp_real, 2, 1> h;
    Eigen::Matrix<qp_real, 1, 3> A;
    Eigen::Matrix<qp_real, 1, 1> b;
    P << 5.0, 1.0, 0.0,
        1.0, 2.0, 1.0,
        0.0, 1.0, 4.0;

    c << 1.0, 2.0, 1.0;

    G << -4.0, -4.0, 0.0,
        0.0, 0.0, -1.0;

    h << -1.0, -1.0;

    A << 1.0, -2.0, 1.0;

    b << 3.0;

    qp_int n = 3; /*! Number of Decision Variables */
    qp_int m = 2; /*! Number of Inequality Constraints */
    qp_int p = 1; /*! Number of equality Constraints */

    QP *myQP;

    myQP = QP_SETUP_dense(n, m, 0, P.data(), NULL, G.data(), c.data(), h.data(), NULL, NULL, COLUMN_MAJOR_ORDERING);

    qp_int ExitCode = QP_SOLVE(myQP);
    if (myQP != NULL)
        printf("Setup Time     : %f ms\n", myQP->stats->tsetup * 1000.0);
    if (ExitCode == QP_OPTIMAL)
    {
        printf("Solve Time     : %f ms\n", (myQP->stats->tsolve + myQP->stats->tsetup) * 1000.0);
        printf("KKT_Solve Time : %f ms\n", myQP->stats->kkt_time * 1000.0);
        printf("LDL Time       : %f ms\n", myQP->stats->ldl_numeric * 1000.0);
        printf("Diff	       : %f ms\n", (myQP->stats->kkt_time - myQP->stats->ldl_numeric) * 1000.0);
        printf("Iterations     : %ld\n", myQP->stats->IterationCount);
        printf("Optimal Solution Found\n");
    }
    if (ExitCode == QP_MAXIT)
    {
        printf("Solve Time     : %f ms\n", myQP->stats->tsolve * 1000.0);
        printf("KKT_Solve Time : %f ms\n", myQP->stats->kkt_time * 1000.0);
        printf("LDL Time       : %f ms\n", myQP->stats->ldl_numeric * 1000.0);
        printf("Diff	       : %f ms\n", (myQP->stats->kkt_time - myQP->stats->ldl_numeric) * 1000.0);
        printf("Iterations     : %ld\n", myQP->stats->IterationCount);
        printf("Maximum Iterations reached\n");
    }

    if (ExitCode == QP_FATAL)
    {
        printf("Unknown Error Detected\n");
    }

    if (ExitCode == QP_KKTFAIL)
    {
        printf("LDL Factorization fail\n");
    }

    /*! The Solution can be found as real pointer in myQP->x;It is an array of Dimension n*/
    std::cout << "Solution" << std::endl;

    for (int i = 0; i < 3; ++i)
    {
        std::cout << "x[" << i << "]: " << myQP->x[i] << std::endl;
    }

    QP_CLEANUP_dense(myQP);

    return 0;
}
