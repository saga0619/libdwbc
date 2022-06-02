#include "dwbc.h"
#include <iomanip>

using namespace DWBC;

#ifdef COMPILE_QPSWIFT
VectorXd qpSwiftSolve(QP *qpp, int var_size, int const_size, MatrixXd &H, VectorXd &G, MatrixXd &A, VectorXd &Ub, bool verbose)
{
    qp_int n = var_size;   /*! Number of Decision Variables */
    qp_int m = const_size; /*! Number of Inequality Constraints */
    qp_int p = 0;          /*! Number of equality Constraints */

    qpp = QP_SETUP_dense(n, m, 0, H.data(), NULL, A.data(), G.data(), Ub.data(), NULL, NULL, COLUMN_MAJOR_ORDERING);

    qp_int ExitCode = QP_SOLVE(qpp);

    if (verbose)
    {
        if (qpp != NULL)
            printf("Setup Time     : %f ms\n", qpp->stats->tsetup * 1000.0);
        if (ExitCode == QP_OPTIMAL)
        {
            printf("Solve Time     : %f ms\n", (qpp->stats->tsolve + qpp->stats->tsetup) * 1000.0);
            printf("KKT_Solve Time : %f ms\n", qpp->stats->kkt_time * 1000.0);
            printf("LDL Time       : %f ms\n", qpp->stats->ldl_numeric * 1000.0);
            printf("Diff	       : %f ms\n", (qpp->stats->kkt_time - qpp->stats->ldl_numeric) * 1000.0);
            printf("Iterations     : %ld\n", qpp->stats->IterationCount);
            printf("Optimal Solution Found\n");
        }
        if (ExitCode == QP_MAXIT)
        {
            printf("Solve Time     : %f ms\n", qpp->stats->tsolve * 1000.0);
            printf("KKT_Solve Time : %f ms\n", qpp->stats->kkt_time * 1000.0);
            printf("LDL Time       : %f ms\n", qpp->stats->ldl_numeric * 1000.0);
            printf("Diff	       : %f ms\n", (qpp->stats->kkt_time - qpp->stats->ldl_numeric) * 1000.0);
            printf("Iterations     : %ld\n", qpp->stats->IterationCount);
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
    }

    VectorXd ret;
    ret.setZero(n);
    for (int i = 0; i < n; i++)
    {
        ret(i) = qpp->x[i];
    }

    QP_CLEANUP_dense(qpp);

    return ret;
}

void RobotData::ClearQP()
{
    qp_task_.clear();
}

void RobotData::AddQP()
{
    QP *qp_ti;
    qp_task_.push_back(qp_ti);
}

#else
void RobotData::ClearQP()
{
    qp_task_.clear();
}

void RobotData::AddQP()
{
    qp_task_.push_back(CQuadraticProgram());
}
#endif

RobotData::RobotData(/* args */)
{
    torque_limit_set_ = false;

    save_mat_file_ = false;
    check_mat_file_ = false;
}

RobotData::~RobotData()
{
}

void RobotData::SetTorqueLimit(const VectorXd &torque_limit)
{
    torque_limit_set_ = true;
    torque_limit_ = torque_limit;
}

void RobotData::UpdateKinematics(const VectorXd q_virtual, const VectorXd q_dot_virtual, const VectorXd q_ddot_virtual)
{
    q_ = q_virtual;
    q_dot_ = q_dot_virtual;
    q_ddot_ = q_ddot_virtual;

    A_.setZero();
    RigidBodyDynamics::UpdateKinematicsCustom(model_, &q_virtual, &q_dot_virtual, &q_ddot_virtual);
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_, q_virtual, A_, false);
    // A_inv_ = A_.inverse();
    A_inv_ = A_.llt().solve(Eigen::MatrixXd::Identity(system_dof_, system_dof_)); // Faster than inverse()
    J_com_.setZero(3, system_dof_);
    G_.setZero(system_dof_);
    com_pos.setZero();
    for (int i = 0; i < link_.size(); i++)
    {
        link_[i].UpdateAll(model_, q_virtual, q_dot_virtual);
        J_com_ += link_[i].jac_com_.topRows(3) * link_[i].mass / total_mass_;
        com_pos += link_[i].xpos * link_[i].mass / total_mass_;
    }
    G_ = -J_com_.transpose() * total_mass_ * Vector3d(0, 0, -9.81);

    CMM_ = A_.block(3, 3, 3, 3) * (A_.block(3, 3, 3, 3) - (A_.block(3, 0, 3, 3) * A_.block(0, 3, 3, 3)) / total_mass_).inverse() * (A_.block(3, 6, 3, model_dof_) - (A_.block(3, 0, 3, 3) * A_.block(0, 6, 3, model_dof_) / total_mass_));

    B_.setZero(system_dof_);
    RigidBodyDynamics::NonlinearEffects(model_, q_virtual, q_dot_virtual, B_);
}

void RobotData::AddContactConstraint(int link_number, int contact_type, Vector3d contact_point, Vector3d contact_vector, double contact_x, double contact_y, bool verbose)
{

    cc_.push_back(ContactConstraint(model_, link_number, link_[link_number].link_id_, contact_type, contact_point, contact_vector, contact_x, contact_y));

    if (verbose)
    {
        std::cout << "#" << (cc_.size() - 1) << " Contact Constraint Added : " << link_[link_number].name_ << std::endl;
    }
}

void RobotData::ClearContactConstraint()
{
    cc_.clear();
}

void RobotData::UpdateContactConstraint()
{
    for (int i = 0; i < cc_.size(); i++)
    {
        cc_[i].Update(model_, q_);
    }
}

void RobotData::CalcContactConstraint(bool update)
{
    if (update)
        UpdateContactConstraint();
    // int contact_dof

    if (J_C.rows() != contact_dof_)
    {
        J_C.setZero(contact_dof_, system_dof_);
    }

    int dof_count = 0;
    for (int i = 0; i < cc_.size(); i++)
    {
        if (cc_[i].contact)
        {
            J_C.block(dof_count, 0, cc_[i].contact_dof_, system_dof_) = cc_[i].j_contact;
            dof_count += cc_[i].contact_dof_;
        }
    }

    CalculateContactConstraint(J_C, A_inv_, Lambda_contact, J_C_INV_T, N_C, W, NwJw, W_inv, V2);
#ifdef CHECKDATA
    if (save_mat_file_)
    {
        write_binary("/J_C", J_C);

        write_binary("/A_inv_", A_inv_);

        write_binary("/Lambda_contact", Lambda_contact);

        write_binary("/J_C_INV_T", J_C_INV_T);

        write_binary("/N_C", N_C);

        write_binary("/W", W);

        write_binary("/NwJw", NwJw);

        write_binary("/W_inv", W_inv);

        write_binary("/V2", V2);
    }

    if (check_mat_file_)
    {
        std::cout << std::setw(30) << std::right << "Matrix Check Mode !! Task Heirarchy : " << std::endl;

        std::cout << std::setw(30) << std::right << "J_C : " << check_binary("/J_C", J_C) << std::endl;

        std::cout << std::setw(30) << std::right << "A_inv_ : " << check_binary("/A_inv_", A_inv_) << std::endl;

        std::cout << std::setw(30) << std::right << "Lambda_contact : " << check_binary("/Lambda_contact", Lambda_contact) << std::endl;

        std::cout << std::setw(30) << std::right << "J_C_INV_T : " << check_binary("/J_C_INV_T", J_C_INV_T) << std::endl;

        std::cout << std::setw(30) << std::right << "N_C : " << check_binary("/N_C", N_C) << std::endl;

        std::cout << std::setw(30) << std::right << "W : " << check_binary("/W", W) << std::endl;

        std::cout << std::setw(30) << std::right << "NwJw : " << check_binary("/NwJw", NwJw) << std::endl;

        std::cout << std::setw(30) << std::right << "W_inv : " << check_binary("/W_inv", W_inv) << std::endl;

        std::cout << std::setw(30) << std::right << "V2 : " << check_binary("/V2", V2) << std::endl;
    }

#endif
}

void RobotData::ClearTaskSpace()
{
    ts_.clear();

    ClearQP();
}

void RobotData::AddTaskSpace(int task_mode, int task_dof, bool verbose)
{
    if (verbose)
        std::cout << "#" << ts_.size() << " Task Space Added with mode " << taskmode_str[task_mode] << std::endl;

    ts_.push_back(TaskSpace(task_mode, ts_.size(), task_dof, system_dof_));

    AddQP();
}
void RobotData::AddTaskSpace(int task_mode, int link_number, Vector3d task_point, bool verbose)
{
    if (verbose)
        std::cout << "#" << ts_.size() << " Task Space Added : " << link_[link_number].name_ << " " << taskmode_str[task_mode] << " at point : " << task_point.transpose() << std::endl;

    ts_.push_back(TaskSpace(task_mode, ts_.size(), link_number, link_[link_number].link_id_, task_point, model_dof_));

    AddQP();
}

void RobotData::SetTaskSpace(int heirarchy, const MatrixXd &f_star, const MatrixXd &J_task)
{
    if (heirarchy >= ts_.size())
    {
        std::cout << "ERROR : task space size overflow" << std::endl;
    }
    else
    {
        ts_[heirarchy].J_task_ = J_task;
        ts_[heirarchy].f_star_ = f_star;
    }
}

void RobotData::UpdateTaskSpace()
{
    for (int i = 0; i < ts_.size(); i++)
    {
        if (ts_[i].task_mode_ == TASK_LINK_6D)
        {
            ts_[i].J_task_ = link_[ts_[i].link_number_].GetPointJac(model_, q_dot_, ts_[i].task_point_);
        }
        else if (ts_[i].task_mode_ == TASK_LINK_POSITION)
        {
            ts_[i].J_task_ = link_[ts_[i].link_number_].GetPointJac(model_, q_dot_, ts_[i].task_point_).topRows(3);
        }
        else if (ts_[i].task_mode_ == TASK_LINK_ROTATION)
        {
            ts_[i].J_task_ = link_[ts_[i].link_number_].GetPointJac(model_, q_dot_, ts_[i].task_point_).bottomRows(3);
        }
    }
}

void Threadtester(const MatrixXd &A, std::promise<MatrixXd> &retVal)
{
    retVal.set_value(PinvCODWBt(A));
    // retVal.set_value(A);
}

void RobotData::CalcTaskSpaceTorqueHQPWithThreaded(bool init)
{
    UpdateTaskSpace();

    // std::vector<std::future<MatrixXd>> vt_jkt_;
    std::promise<MatrixXd> p1, p2;
    std::future<MatrixXd> f2 = p2.get_future();
    std::future<MatrixXd> f1 = p1.get_future();

    std::thread t1, t2;

    for (int i = 0; i < ts_.size(); i++)
    {
        CalculateJKTThreaded(ts_[i].J_task_, A_inv_, N_C, W_inv, ts_[i].Q_, ts_[i].Q_temp_, ts_[i].Lambda_task_);

        // vt_jkt_.push_back(
        //     std::async(std::launch::async, PinvCODWBt, cref(ts_[i].Q_temp_)));

        ts_[i].J_kt_ = W_inv * ts_[i].Q_.transpose() * PinvCODWBt(std::ref(ts_[i].Q_temp_));
    }

    // t1 = std::thread(Threadtester, std::ref(ts_[0].Q_temp_), std::ref(p1));
    // t1.join();
    // MatrixXd tt = f1.get();
    // t2 = std::thread(Threadtester, std::ref(ts_[1].Q_temp_), std::ref(p2));
    MatrixXd temp = PinvCODWBt(std::ref(ts_[0].Q_temp_));
    // t1.join();
    // t2.join();

    // ts_[0].J_kt_ = W_inv * ts_[0].Q_.transpose() * f1.get();
    // ts_[1].J_kt_ = W_inv * ts_[1].Q_.transpose() * f2.get();

    // ts_[0].CalcJKT(A_inv_, N_C, W_inv);
    // ts_[1].CalcJKT(A_inv_, N_C, W_inv);
    //  ts_[0].J_kt_ = W_inv * ts_[0].Q_.transpose() * PinvCODWBt(ts_[0].Q_temp_); // PinvCODWB(Q * W_inv * Q.transpose());

    for (int i = 0; i < ts_.size(); i++)
    {
        if (i == 0)
        {
            CalcTaskTorqueQP(ts_[i], MatrixXd::Identity(model_dof_, model_dof_), torque_grav_, NwJw, J_C_INV_T, P_C, init);

            torque_task_ = ts_[i].J_kt_ * ts_[i].Lambda_task_ * (ts_[i].f_star_ + ts_[i].f_star_qp_);
        }
        else
        {
            // ts_[i].J_kt_ = W_inv * ts_[i].Q_.transpose() * vt_jkt_[i - 1].get(); // PinvCODWB(Q * W_inv * Q.transpose());

            CalcTaskTorqueQP(ts_[i], ts_[i - 1].Null_task_, torque_grav_, NwJw, J_C_INV_T, P_C, init);

            torque_task_ += ts_[i - 1].Null_task_ * (ts_[i].J_kt_ * ts_[i].Lambda_task_ * (ts_[i].f_star_ + ts_[i].f_star_qp_));
        }

        if (i != ts_.size() - 1)
            ts_[i].CalcNullMatrix(A_inv_, N_C);
    }
}

void RobotData::CalcTaskSpace(bool update)
{
    if (update)
        UpdateTaskSpace();
    for (int i = 0; i < ts_.size(); i++)
    {
        ts_[i].CalcJKT(A_inv_, N_C, W_inv);
    }
}

int RobotData::CalcTaskTorque(bool init, bool hqp, bool update_task_space)
{
    if (update_task_space)
        CalcTaskSpace();

    torque_task_.setZero(model_dof_);

    int qp_res = 0;

    if (hqp)
    {
        for (int i = 0; i < ts_.size(); i++)
        {
            if (i == 0)
            {
                qp_res = CalcTaskTorqueQP(ts_[i], MatrixXd::Identity(model_dof_, model_dof_), torque_grav_, NwJw, J_C_INV_T, P_C, init);

                if (qp_res == 0)
                    return 0;

                torque_task_ = ts_[i].J_kt_ * ts_[i].Lambda_task_ * (ts_[i].f_star_ + ts_[i].f_star_qp_);
            }
            else
            {
                qp_res = CalcTaskTorqueQP(ts_[i], ts_[i - 1].Null_task_, torque_grav_ + torque_task_, NwJw, J_C_INV_T, P_C, init);
                if (qp_res == 0)
                    return 0;
                torque_task_ += ts_[i - 1].Null_task_ * (ts_[i].J_kt_ * ts_[i].Lambda_task_ * (ts_[i].f_star_ + ts_[i].f_star_qp_));
            }

            if (i != ts_.size() - 1)
                ts_[i].CalcNullMatrix(A_inv_, N_C);
        }
#ifdef CHECKDATA
        if (save_mat_file_)
        {

            write_binary("/torque_task_", torque_task_);
        }

        if (check_mat_file_)
        {

            std::cout << std::setw(30) << std::right << "torque task : " << check_binary("/torque_task_", torque_task_) << std::endl;
        }

#endif
        return 1;
    }
    else
    {
        for (int i = 0; i < ts_.size(); i++)
        {
            if (i == 0)
            {
                ts_[i].torque_h_ = ts_[i].J_kt_ * ts_[i].Lambda_task_ * ts_[i].f_star_;

                ts_[i].CalcNullMatrix(A_inv_, N_C);

                torque_task_ = ts_[i].torque_h_;
            }
            else
            {

                ts_[i].torque_h_ = ts_[i - 1].Null_task_ * ts_[i].J_kt_ * ts_[i].f_star_;

                if (i != (ts_.size() - 1))
                {
                    ts_[i].CalcNullMatrix(A_inv_, N_C, ts_[i - 1].Null_task_);
                }

                torque_task_ += ts_[i].torque_h_;
            }
        }

        return 1;
    }
}

/*
Init model data with rbdl
verbose 2 : Show all link Information
verbose 1 : Show link id information
verbose 0 : disable verbose
*/
void RobotData::InitModelData(std::string urdf_path, bool floating, int verbose)
{
    bool rbdl_v = false;
    if (verbose == 2)
    {
        rbdl_v = true;
    }

    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_path.c_str(), &model_, floating, rbdl_v);
    system_dof_ = model_.dof_count;
    model_dof_ = system_dof_ - 6;

    total_mass_ = 0;
    for (int i = 0; i < model_.mBodies.size(); i++)
    {
        if (model_.mBodies[i].mMass != 0) // if body has mass,
        {
            link_.push_back(Link(model_, i));
            total_mass_ += model_.mBodies[i].mMass;
        }
    }

    if (verbose == 1)
    {
        for (int i = 0; i < link_.size(); i++)
            std::cout << i << " : " << link_[i].name_ << std::endl;
    }
    q_.setZero(model_.q_size);
    q_dot_.setZero(model_.qdot_size);
    q_ddot_.setZero(model_.qdot_size);

    A_.setZero(system_dof_, system_dof_);
    A_inv_.setZero(system_dof_, system_dof_);
}

VectorXd RobotData::CalcGravCompensation()
{
    CalculateGravityCompensation(A_inv_, W_inv, N_C, J_C_INV_T, G_, torque_grav_, P_C);

#ifdef CHECKDATA
    if (save_mat_file_)
    {
        write_binary("/torque_grav_", torque_grav_);
    }

    if (check_mat_file_)
    {

        std::cout << std::setw(30) << std::right << "torque_grav_ : " << check_binary("/torque_grav_", torque_grav_) << std::endl;
    }

#endif
    return torque_grav_;
}

void RobotData::CalcGravCompensation(VectorXd &grav_torque)
{
    CalculateGravityCompensation(A_inv_, W_inv, N_C, J_C_INV_T, G_, grav_torque, P_C);

#ifdef CHECKDATA
    if (save_mat_file_)
    {
        write_binary("/torque_grav_", grav_torque);
    }

    if (check_mat_file_)
    {

        std::cout << std::setw(30) << std::right << "torque_grav_ : " << check_binary("/torque_grav_", grav_torque) << std::endl;
    }

#endif
}

VectorXd RobotData::getContactForce(const VectorXd &command_torque)
{
    VectorXd cf;
    CalculateContactForce(command_torque, J_C_INV_T, P_C, cf);
    return cf;
}

int RobotData::CalcTaskTorqueQP(TaskSpace &ts_, const MatrixXd &task_null_matrix_, const VectorXd &torque_prev, const MatrixXd &NwJw, const MatrixXd &J_C_INV_T, const MatrixXd &P_C, bool init_trigger)
{
    // return fstar & contact force;
    int task_dof = ts_.f_star_.size(); // size of task
    int contact_index = cc_.size();    // size of contact link
    int total_contact_dof = 0;         // total size of contact dof
    int contact_dof = -6;              // total_contact_dof - 6, (free contact space)
    int contact_constraint_size = 0;   // size of constraint by contact
    int model_size = model_dof_;       // size of joint

    int torque_limit_constraint_size = 2 * model_size;

    for (int i = 0; i < contact_index; i++)
    {
        total_contact_dof += cc_[i].contact_dof_;
        contact_constraint_size += cc_[i].constraint_number_;
    }
    contact_dof += total_contact_dof;

    int variable_size = task_dof + contact_dof;

    if (!torque_limit_set_)
        torque_limit_constraint_size = 0;
    int total_constraint_size = contact_constraint_size + torque_limit_constraint_size; // total contact constraint size

    MatrixXd H;
    VectorXd g;
    H.setZero(variable_size, variable_size);
    H.block(0, 0, task_dof, task_dof).setIdentity();
    g.setZero(variable_size);

    Eigen::MatrixXd A;
    Eigen::VectorXd lbA, ubA;
    A.setZero(total_constraint_size, variable_size);
    lbA.setZero(total_constraint_size);
    ubA.setZero(total_constraint_size);
    Eigen::MatrixXd Ntorque_task = task_null_matrix_ * ts_.J_kt_ * ts_.Lambda_task_;

    if (torque_limit_set_)
    {
        A.block(0, 0, model_size, task_dof) = Ntorque_task;
        A.block(0, task_dof, model_size, contact_dof) = NwJw;
        // lbA.segment(0, model_size) = -torque_limit_ - torque_prev - Ntorque_task * ts_.f_star_;
        ubA.segment(0, model_size) = torque_limit_ - torque_prev - Ntorque_task * ts_.f_star_;

        A.block(model_size, 0, model_size, task_dof) = -Ntorque_task;
        A.block(model_size, task_dof, model_size, contact_dof) = -NwJw;
        // lbA.segment(model_size, model_size) = -torque_limit_ - torque_prev - Ntorque_task * ts_.f_star_;
        ubA.segment(model_size, model_size) = torque_limit_ + torque_prev + Ntorque_task * ts_.f_star_;

        lbA.segment(0, torque_limit_constraint_size).setConstant(-INFTY);
    }

    Eigen::MatrixXd A_const_a;
    A_const_a.setZero(contact_constraint_size, total_contact_dof);

    Eigen::MatrixXd A_rot;
    A_rot.setZero(total_contact_dof, total_contact_dof);

    int const_idx = 0;
    int contact_idx = 0;
    for (int i = 0; i < cc_.size(); i++)
    {
        A_rot.block(contact_idx, contact_idx, 3, 3) = cc_[i].rotm.transpose();
        A_rot.block(contact_idx + 3, contact_idx + 3, 3, 3) = cc_[i].rotm.transpose();

        A_const_a.block(const_idx, contact_idx, 4, 6) = cc_[i].GetZMPConstMatrix4x6();
        A_const_a.block(const_idx + CONTACT_CONSTRAINT_ZMP, contact_idx, 6, 6) = cc_[i].GetForceConstMatrix6x6();

        const_idx += cc_[i].constraint_number_;
        contact_idx += cc_[i].contact_dof_;
    }

    Eigen::MatrixXd Atemp = A_const_a * A_rot * J_C_INV_T.rightCols(model_size);
    // t[3] = std::chrono::steady_clock::now();
    A.block(torque_limit_constraint_size, 0, contact_constraint_size, task_dof) = -Atemp * Ntorque_task;
    A.block(torque_limit_constraint_size, task_dof, contact_constraint_size, contact_dof) = -Atemp * NwJw;
    // t[4] = std::chrono::steady_clock::now();

    Eigen::VectorXd bA = A_const_a * (A_rot * P_C) - Atemp * (torque_prev + Ntorque_task * ts_.f_star_);
    // Eigen::VectorXd ubA_contact;
    lbA.segment(torque_limit_constraint_size, contact_constraint_size).setConstant(-INFTY);

    // lbA.segment(total_constraint_size) = -ubA_contact;
    ubA.segment(torque_limit_constraint_size, contact_constraint_size) = -bA;

    // qp_.EnableEqualityCondition(0.0001);

    VectorXd qpres;
#ifdef CHECKDATA
    if (save_mat_file_)
    {
        std::string fname = "/h" + std::to_string(ts_.heirarchy_) + "mat";

        write_binary(fname.c_str(), H);

        fname = "/g" + std::to_string(ts_.heirarchy_) + "mat";

        write_binary(fname.c_str(), g);

        fname = "/A" + std::to_string(ts_.heirarchy_) + "mat";

        write_binary(fname.c_str(), A);

        fname = "/ubA" + std::to_string(ts_.heirarchy_) + "mat";

        write_binary(fname.c_str(), ubA);
    }

    if (check_mat_file_)
    {
        std::cout << "Matrix Check Mode !! Task Heirarchy : " << ts_.heirarchy_ << std::endl;

        std::string hpath = "/h" + std::to_string(ts_.heirarchy_) + "mat";
        std::cout << std::setw(30) << std::right << "H : " << check_binary(hpath.c_str(), H) << std::endl;

        hpath = "/g" + std::to_string(ts_.heirarchy_) + "mat";
        std::cout << std::setw(30) << std::right << "g : " << check_binary(hpath.c_str(), g) << std::endl;

        hpath = "/A" + std::to_string(ts_.heirarchy_) + "mat";
        std::cout << std::setw(30) << std::right << "A : " << check_binary(hpath.c_str(), A) << std::endl;

        hpath = "/ubA" + std::to_string(ts_.heirarchy_) + "mat";
        std::cout << std::setw(30) << std::right << "ubA : " << check_binary(hpath.c_str(), ubA) << std::endl;
    }

#endif

#ifdef COMPILE_QPSWIFT
    qpres = qpSwiftSolve(qp_task_[ts_.heirarchy_], variable_size, total_constraint_size, H, g, A, ubA, false);
    ts_.f_star_qp_ = qpres.segment(0, task_dof);
    return 1;
#else
    if (init_trigger)
        qp_task_[ts_.heirarchy_].InitializeProblemSize(variable_size, total_constraint_size);

    qp_task_[ts_.heirarchy_].UpdateMinProblem(H, g);
    qp_task_[ts_.heirarchy_].UpdateSubjectToAx(A, lbA, ubA);
    qp_task_[ts_.heirarchy_].DeleteSubjectToX();

    if (qp_task_[ts_.heirarchy_].SolveQPoases(300, qpres))
    {
        ts_.f_star_qp_ = qpres.segment(0, task_dof);
        return 1;
    }
    else
    {
        std::cout << "task solve failed" << std::endl;
        ts_.f_star_qp_ = VectorXd::Zero(task_dof);

        // qp_task_[ts_.heirarchy_].PrintMinProb();

        // qp_task_[ts_.heirarchy_].PrintSubjectToAx();
        return 0;
    }
#endif
}

/*
modified axis contact force : mfc ( each contact point's z axis is piercing COM position)
minimize contact forces.

rotm * (rd_.J_C_INV_T.rightCols(MODEL_DOF) * control_torque + rd_.J_C_INV_T * rd_.G);
rd_.J_C_INV_T.rightCols(MODEL_DOF).transpose() * rotm.transpose() * rotm *rd_.J_C_INV_T.rightCols(MODEL_DOF) - 2 * rd_.J_C_INV_T * rd_.G

min mfc except z axis

s.t. zmp condition, friction condition.

H matrix :

contact condition : on/off
contact mode : plane, line, point

s. t. zmp condition :      (left_rotm.transpose(), 0; 0, right_rotm.transpose()) * rd_.J_C_INV_T.rightCols(MODEL_DOF) * (control_torue + contact_torque) + rd_.J_C_INV_T * rd_.G

contact_force = rd_.J_C_INV_T.rightCols(MODEL_DOF)*(torque_control + torque_contact) + rd_.J_C_INV_T * rd_.G;

torque_contact = Robot.qr_V2.transpose() * (Robot.J_C_INV_T.rightCols(MODEL_DOF).topRows(6) * Robot.qr_V2.transpose()).inverse() * desired_contact_force;

rd_.J_C_INV_T.rightCols(MODEL_DOF) * (torque + nwjw * des_Force) - rd_.

torque_contact = nwjw * des_Force
*/
int RobotData::CalcContactRedistribute(bool init)
{
    int contact_index = cc_.size();  // size of contact link
    int total_contact_dof = 0;       // size of contact dof
    int contact_dof = -6;            // total_contact_dof - 6, (free contact space)
    int contact_constraint_size = 0; // size of constraint by contact
    int model_size = model_dof_;     // size of joints
    int torque_limit_constraint_size = 2 * model_size;

    for (int i = 0; i < contact_index; i++)
    {
        total_contact_dof += cc_[i].contact_dof_;
        contact_constraint_size += cc_[i].constraint_number_;
    }
    contact_dof += total_contact_dof;

    if (!torque_limit_set_)
        torque_limit_constraint_size = 0;
    int variable_number = contact_dof;                                                  // total size of qp variable
    int total_constraint_size = contact_constraint_size + torque_limit_constraint_size; // total size of constraint

    VectorXd control_torque = torque_grav_ + torque_task_;

    if (contact_dof == 0)
    {
        torque_contact_ = VectorXd::Zero(model_size);

        return 1;
    }
    else
    {
        MatrixXd H, H_temp;
        VectorXd g;

        Eigen::MatrixXd crot_matrix = Eigen::MatrixXd::Zero(total_contact_dof, total_contact_dof);
        Eigen::MatrixXd RotW = Eigen::MatrixXd::Identity(total_contact_dof, total_contact_dof);
        int acc_cdof = 0;
        for (int i = 0; i < contact_index; i++)
        {
            Vector3d vec_origin, vec_target;
            vec_origin = cc_[i].rotm.rightCols(1);
            vec_target = (com_pos - cc_[i].xc_pos).normalized();
            Matrix3d cm = AxisTransform2V(vec_origin, vec_target);

            cm.setIdentity(); // comment this line to redistribute contact force with COM based vector

            if (cc_[i].contact_type_ == CONTACT_6D)
            {
                crot_matrix.block(acc_cdof, acc_cdof, 3, 3) = crot_matrix.block(acc_cdof + 3, acc_cdof + 3, 3, 3) = cm.transpose() * cc_[i].rotm.transpose();
                RotW(acc_cdof + 2, acc_cdof + 2) = 0;
                acc_cdof += cc_[i].contact_dof_;
            }
            else if (cc_[i].contact_type_ == CONTACT_POINT)
            {
                crot_matrix.block(acc_cdof, acc_cdof, 3, 3) = cm.transpose() * cc_[i].rotm.transpose();
                RotW(acc_cdof + 2, acc_cdof + 2) = 0;
                acc_cdof += cc_[i].contact_dof_;
            }
        }

        H_temp = RotW * crot_matrix * J_C_INV_T.rightCols(model_size) * NwJw;
        H = H_temp.transpose() * H_temp;
        g = (RotW * crot_matrix * (J_C_INV_T.rightCols(model_size) * control_torque - P_C)).transpose() * H_temp;

        MatrixXd A_qp;
        VectorXd lbA, ubA;
        A_qp.setZero(total_constraint_size, variable_number);
        lbA.setZero(total_constraint_size);
        ubA.setZero(total_constraint_size);

        if (torque_limit_set_)
        {
            A_qp.block(0, 0, model_size, contact_dof) = NwJw;
            // lbA.segment(0, model_size) = -torque_limit_ - control_torque;
            ubA.segment(0, model_size) = torque_limit_ - control_torque;

            A_qp.block(0, 0, model_size, contact_dof) = -NwJw;
            // lbA.segment(0, model_size) = -torque_limit_ - control_torque;
            ubA.segment(0, model_size) = torque_limit_ + control_torque;

            lbA.segment(0, torque_limit_constraint_size).setConstant(-INFTY);
        }

        MatrixXd A_const_a;
        A_const_a.setZero(contact_constraint_size, total_contact_dof);

        MatrixXd A_rot;
        A_rot.setZero(total_contact_dof, total_contact_dof);

        int const_idx = 0;
        int contact_idx = 0;
        for (int i = 0; i < contact_index; i++)
        {
            A_rot.block(contact_idx, contact_idx, 3, 3) = cc_[i].rotm.transpose(); // rd_.ee_[i].rotm.transpose();
            A_rot.block(contact_idx + 3, contact_idx + 3, 3, 3) = cc_[i].rotm.transpose();

            A_const_a.block(const_idx, contact_idx, 4, 6) = cc_[i].GetZMPConstMatrix4x6();
            A_const_a.block(const_idx + CONTACT_CONSTRAINT_ZMP, contact_idx, 6, 6) = cc_[i].GetForceConstMatrix6x6();

            const_idx += cc_[i].constraint_number_;
            contact_idx += cc_[i].contact_dof_;

            // specific vector on Global axis
            // [0 0 -1]T *
            // Force Constraint
            // Will be added
        }

        Eigen::MatrixXd Atemp = A_const_a * A_rot * J_C_INV_T.rightCols(model_size);
        Eigen::VectorXd bA = A_const_a * (A_rot * P_C) - Atemp * control_torque;

        A_qp.block(torque_limit_constraint_size, 0, contact_constraint_size, contact_dof) = -Atemp * NwJw;

        lbA.segment(torque_limit_constraint_size, contact_constraint_size).setConstant(-INFTY);
        ubA.segment(torque_limit_constraint_size, contact_constraint_size) = -bA;

#ifdef CHECKDATA

        if (save_mat_file_)
        {
            write_binary("/hcontact_mat", H);
            write_binary("/gcontact_mat", g);
            write_binary("/Acontact_mat", A_);
            write_binary("/ubAcontact_mat", ubA);
        }

        if (check_mat_file_)
        {
            std::cout << "Matrix Check Mode !! Contact Redistribution : " << std::endl;

            std::cout << std::setw(30) << std::right << "H : " << check_binary("/hcontact_mat", H) << std::endl;
            std::cout << std::setw(30) << std::right << "g : " << check_binary("/gcontact_mat", g) << std::endl;
            std::cout << std::setw(30) << std::right << "A : " << check_binary("/Acontact_mat", A_) << std::endl;
            std::cout << std::setw(30) << std::right << "ubA : " << check_binary("/ubAcontact_mat", ubA) << std::endl;
        }
#endif
        Eigen::VectorXd qpres;

#ifdef COMPILE_QPSWIFT
        qpres = qpSwiftSolve(qp_contact_, variable_number, total_constraint_size, H, g, A_, ubA, false);
        // ts_.f_star_qp_ = qpres.segment(0, task_dof);
        torque_contact_ = NwJw * qpres;
        return 1;

#else

        if (init)
            qp_contact_.InitializeProblemSize(variable_number, total_constraint_size);
        qp_contact_.UpdateMinProblem(H, g);
        qp_contact_.UpdateSubjectToAx(A_qp, lbA, ubA);
        if (qp_contact_.SolveQPoases(300, qpres))
        {
            torque_contact_ = NwJw * qpres;

#ifdef CHECKDATA
            if (save_mat_file_)
            {
                write_binary("/torque_contact_", torque_contact_);
            }

            if (check_mat_file_)
            {

                std::cout << std::setw(30) << std::right << "torque_contact_ Calculation : " << check_binary("/torque_contact_", torque_contact_) << std::endl;
            }

#endif

            return 1;
        }
        else
        {
            std::cout << "contact qp solve failed" << std::endl;
            torque_contact_.setZero(model_size);

            return 0;
        }
#endif
    }
}
