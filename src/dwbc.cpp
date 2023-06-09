#include "dwbc.h"
#include <future>
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

void RobotData::UpdateKinematics(const VectorXd q_virtual, const VectorXd q_dot_virtual, const VectorXd q_ddot_virtual, bool update_kinematics)
{
    // check q size and q_dot size
    if (model_.q_size != q_virtual.size())
    {
        std::cout << "q size is not matched : qsize : " << model_.q_size << " input size : " << q_virtual.size() << std::endl;
        return;
    }
    if (model_.qdot_size != q_dot_virtual.size())
    {
        std::cout << "q_dot size is not matched" << std::endl;
        return;
    }
    if (model_.qdot_size != q_ddot_virtual.size())
    {
        std::cout << "q_ddot size is not matched" << std::endl;
        return;
    }
    q_system_ = q_virtual;
    q_dot_system_ = q_dot_virtual;
    q_ddot_system_ = q_ddot_virtual;

    A_.setZero();
    if (update_kinematics)
    {
        RigidBodyDynamics::UpdateKinematicsCustom(model_, &q_virtual, &q_dot_virtual, &q_ddot_virtual);
        RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_, q_virtual, A_, false);
        // A_inv_ = A_.inverse();
        A_inv_ = A_.llt().solve(Eigen::MatrixXd::Identity(system_dof_, system_dof_)); // Faster than inverse()
    }
    J_com_.setZero(3, system_dof_);
    G_.setZero(system_dof_);
    com_pos.setZero();
    for (int i = 0; i < (link_.size() - 1); i++)
    {
        link_[i].UpdateAll(model_, q_virtual, q_dot_virtual);
        J_com_ += link_[i].jac_com_.topRows(3) * link_[i].mass / total_mass_;
        com_pos += link_[i].xipos * link_[i].mass / total_mass_;
    }

    link_.back().xpos = com_pos;
    link_.back().xipos = com_pos;

    link_.back().rotm = link_[0].rotm; // get info of pelvis rotation
    link_.back().w = link_[0].w;

    link_.back().jac_.setZero(6, system_dof_);
    link_.back().jac_.block(0, 0, 3, system_dof_) = J_com_;
    link_.back().jac_.block(3, 0, 3, system_dof_) = link_[0].jac_.block(3, 0, 3, system_dof_);
    link_.back().jac_com_ = J_com_;

    link_.back().v = J_com_ * q_dot_system_;

    G_ = -J_com_.transpose() * total_mass_ * Vector3d(0, 0, -9.81);

    CMM_ = A_.block(3, 3, 3, 3) * (A_.block(3, 3, 3, 3) - (A_.block(3, 0, 3, 3) * A_.block(0, 3, 3, 3)) / total_mass_).inverse() * (A_.block(3, 6, 3, model_dof_) - (A_.block(3, 0, 3, 3) * A_.block(0, 6, 3, model_dof_) / total_mass_));

    B_.setZero(system_dof_);
    RigidBodyDynamics::NonlinearEffects(model_, q_virtual, q_dot_virtual, B_);

    UpdateContactConstraint();
}

void RobotData::AddContactConstraint(int link_number, int contact_type, Vector3d contact_point, Vector3d contact_vector, double contact_x, double contact_y, bool verbose)
{
    for (int i = 0; i < cc_.size(); i++)
    {
        if (cc_[i].link_number_ == link_number)
        {
            std::cout << "Contact Constraint Already Exist for Link : " << link_[link_number].name_ << std::endl;
            return;
        }
    }
    cc_.push_back(ContactConstraint(model_, link_number, link_[link_number].body_id_, contact_type, contact_point, contact_vector, contact_x, contact_y));

    if (verbose)
    {
        std::cout << "#" << (cc_.size() - 1) << " Contact Constraint Added : " << link_[link_number].name_ << std::endl;
    }
}
void RobotData::AddContactConstraint(const char *link_name, int contact_type, Vector3d contact_point, Vector3d contact_vector, double contact_x, double contact_y, bool verbose)
{
    int link_number = -1;
    for (int i = 0; i < link_.size(); i++)
    {
        // compare the name of link and link_name string and return the index of link
        // ignore the case of character
        if (strcasecmp(link_[i].name_.c_str(), link_name) == 0)
        {
            link_number = i;
            break;
        }
    }
    if (link_number == -1)
    {
        std::cout << "Link Name is Wrong : " << link_name << std::endl;
        return;
    }

    for (int i = 0; i < cc_.size(); i++)
    {
        if (cc_[i].link_number_ == link_number)
        {
            std::cout << "Contact Constraint Already Exist for Link : " << link_[link_number].name_ << std::endl;
            return;
        }
    }
    cc_.push_back(ContactConstraint(model_, link_number, link_[link_number].body_id_, contact_type, contact_point, contact_vector, contact_x, contact_y));

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
        cc_[i].Update(model_, q_system_);
    }
}

int RobotData::CalcContactConstraint(bool update)
{
    if (update)
        UpdateContactConstraint();
    // int contact_dof
    if (J_C.rows() != contact_dof_ || J_C.cols() != system_dof_)
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

    Lambda_contact.setZero(contact_dof_, contact_dof_);
    J_C_INV_T.setZero(contact_dof_, system_dof_);
    N_C.setZero(system_dof_, system_dof_);

    W.setZero(model_dof_, model_dof_);
    W_inv.setZero(model_dof_, model_dof_);

    int contact_null_dof = contact_dof_ - 6;

    V2.setZero(contact_null_dof, model_dof_);
    NwJw.setZero(model_dof_, model_dof_);

    return CalculateContactConstraint(J_C, A_inv_, Lambda_contact, J_C_INV_T, N_C, W, NwJw, W_inv, V2);
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

    ts_.push_back(TaskSpace(task_mode, ts_.size(), task_dof));

    AddQP();
}
void RobotData::AddTaskSpace(int task_mode, int link_number, Vector3d task_point, bool verbose)
{
    if (verbose)
        std::cout << "#" << ts_.size() << " Task Space Added : " << link_[link_number].name_ << " " << taskmode_str[task_mode] << " at point : " << task_point.transpose() << std::endl;

    for (int i = 0; i < ts_.size(); i++)
    {
        if (ts_[i].link_id_ == link_number)
        {
            std::cout << "Task Space Already Exist for Link : " << link_[link_number].name_ << std::endl;
            return;
        }
    }

    ts_.push_back(TaskSpace(task_mode, ts_.size(), link_number, link_[link_number].body_id_, task_point));

    AddQP();
}

void RobotData::AddTaskSpace(int task_mode, const char *link_name, Vector3d task_point, bool verbose)
{
    int link_number = -1;

    for (int i = 0; i < link_.size(); i++)
    {
        // compare the name of link and link_name
        //  ignore the case of nmame
        if (strcasecmp(link_[i].name_.c_str(), link_name) == 0)
        {
            link_number = i;
            break;
        }
    }

    if (link_number == -1)
    {
        std::cout << "Link Name is not Correct" << std::endl;
        return;
    }

    if (verbose)
        std::cout << "#" << ts_.size() << " Task Space Added : " << link_[link_number].name_ << " " << taskmode_str[task_mode] << " at point : " << task_point.transpose() << std::endl;

    for (int i = 0; i < ts_.size(); i++)
    {
        if (ts_[i].link_id_ == link_number)
        {
            std::cout << "Task Space Already Exist for Link : " << link_[link_number].name_ << std::endl;
            return;
        }
    }

    ts_.push_back(TaskSpace(task_mode, ts_.size(), link_number, link_[link_number].body_id_, task_point));

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

// void RobotData::SetTaskTrajectory(int heirarchy, const MatrixXd &f_star, const MatrixXd &J_task)
// {

// }

void RobotData::UpdateTaskSpace()
{
    for (int i = 0; i < ts_.size(); i++)
    {
        if (ts_[i].task_mode_ == TASK_LINK_6D)
        {
            ts_[i].J_task_ = link_[ts_[i].link_id_].GetPointJac(model_, q_dot_system_, ts_[i].task_point_);

            if (ts_[i].traj_pos_set)
            {
                ts_[i].GetFstarPosPD(control_time_,
                                     link_[ts_[i].link_id_].xpos + link_[ts_[i].link_id_].rotm * ts_[i].task_point_,
                                     link_[ts_[i].link_id_].v + link_[ts_[i].link_id_].w.cross(ts_[i].task_point_));
            }

            if (ts_[i].traj_rot_set)
            {
                ts_[i].GetFstarRotPD(control_time_, link_[ts_[i].link_id_].rotm, link_[ts_[i].link_id_].w);
            }
        }
        else if (ts_[i].task_mode_ == TASK_LINK_POSITION)
        {
            ts_[i].J_task_ = link_[ts_[i].link_id_].GetPointJac(model_, q_dot_system_, ts_[i].task_point_).topRows(3);
            if (ts_[i].traj_pos_set)
            {
                ts_[i].GetFstarPosPD(control_time_,
                                     link_[ts_[i].link_id_].xpos + link_[ts_[i].link_id_].rotm * ts_[i].task_point_,
                                     link_[ts_[i].link_id_].v + link_[ts_[i].link_id_].w.cross(ts_[i].task_point_));
            }
        }
        else if (ts_[i].task_mode_ == TASK_LINK_ROTATION)
        {
            ts_[i].J_task_ = link_[ts_[i].link_id_].GetPointJac(model_, q_dot_system_, ts_[i].task_point_).bottomRows(3);
            if (ts_[i].traj_rot_set)
            {
                ts_[i].GetFstarRotPD(control_time_, link_[ts_[i].link_id_].rotm, link_[ts_[i].link_id_].w);
            }
        }
        else if (ts_[i].task_mode_ == TASK_COM_POSITION)
        {
            ts_[i].J_task_ = link_.back().jac_.topRows(3);
            if (ts_[i].traj_pos_set)
            {
                ts_[i].GetFstarPosPD(control_time_,
                                     link_[ts_[i].link_id_].xpos,
                                     link_[ts_[i].link_id_].v);
            }
        }
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

int RobotData::CalcTaskControlTorque(bool init, bool hqp, bool update_task_space)
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
                qp_res = CalcSingleTaskTorqueWithQP(ts_[i], MatrixXd::Identity(model_dof_, model_dof_), torque_grav_, NwJw, J_C_INV_T, P_C, init);

                if (qp_res == 0)
                    return 0;

                torque_task_ = ts_[i].J_kt_ * ts_[i].Lambda_task_ * (ts_[i].f_star_ + ts_[i].f_star_qp_);
            }
            else
            {
                qp_res = CalcSingleTaskTorqueWithQP(ts_[i], ts_[i - 1].Null_task_, torque_grav_ + torque_task_, NwJw, J_C_INV_T, P_C, init);
                if (qp_res == 0)
                    return 0;
                torque_task_ += ts_[i - 1].Null_task_ * (ts_[i].J_kt_ * ts_[i].Lambda_task_ * (ts_[i].f_star_ + ts_[i].f_star_qp_));
            }

            if (i != ts_.size() - 1)
                ts_[i].CalcNullMatrix(A_inv_, N_C);
        }

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
void RobotData::LoadModelData(std::string urdf_path, bool floating, int verbose)
{
    if (link_.size() > 0)
    {
        std::cout << "WARNING, VECTOR LINK IS NOT ZERO" << std::endl;
    }
    bool rbdl_v = false;
    if (verbose == 2)
    {
        rbdl_v = true;
    }
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_path.c_str(), &model_, floating, rbdl_v);

    InitModelData(verbose);
}

void RobotData::InitModelData(int verbose)
{
    ts_.clear();
    cc_.clear();
    link_.clear();
    qp_task_.clear();

    qp_contact_ = CQuadraticProgram();

    system_dof_ = model_.dof_count;
    model_dof_ = system_dof_ - 6;

    if (verbose)
    {
        std::cout << "System DOF : " << system_dof_ << std::endl;
        std::cout << "Model DOF : " << model_dof_ << std::endl;
        std::cout << "Model.dof : " << model_.dof_count << std::endl;
    }

    total_mass_ = 0;
    for (int i = 0; i < model_.mBodies.size(); i++)
    {
        if (model_.mBodies[i].mMass != 0) // if body has mass,
        {
            link_.push_back(Link(model_, i));
            total_mass_ += model_.mBodies[i].mMass;
        }
    }

    for (int i = 0; i < link_.size(); i++)
    {
        link_[i].parent_id_ = 0;
        link_[i].child_id_.clear();
        link_[i].link_id_ = i;
    }

    for (int i = 0; i < link_.size(); i++)
    {
        int temp_parent_body_id = model_.lambda[link_[i].body_id_];

        for (int j = 0; j < link_.size(); j++)
        {
            if (temp_parent_body_id == link_[j].body_id_)
            {

                link_[i].parent_id_ = j;
                link_[j].child_id_.push_back(i);
            }
        }
    }

    link_.push_back(Link()); // Add link for COM
    link_.back().name_ = "COM";

    if (verbose == 1)
    {
        int vlink = 0;
        for (int i = 0; i < link_.size() - 1; i++)
        {
            std::cout << vlink << " : " << link_[vlink].name_ << std::endl;
            std::cout << "mass : " << link_[vlink].mass << " body id : " << link_[vlink].body_id_ << " parent id : " << link_[vlink++].parent_id_ << std::endl;
        }

        std::cout << vlink << " : " << link_.back().name_ << std::endl;
    }
    q_system_.setZero(model_.q_size);
    q_dot_system_.setZero(model_.qdot_size);
    q_ddot_system_.setZero(model_.qdot_size);

    J_com_.setZero(6, system_dof_);

    A_.setZero(system_dof_, system_dof_);
    A_inv_.setZero(system_dof_, system_dof_);

    G_.setZero(system_dof_);
    torque_grav_.setZero(model_dof_);
    torque_task_.setZero(model_dof_);
    torque_contact_.setZero(model_dof_);

    torque_limit_.setZero(model_dof_);
}

VectorXd RobotData::CalcGravCompensation()
{
    CalculateGravityCompensation(A_inv_, W_inv, N_C, J_C_INV_T, G_, torque_grav_, P_C);

    return torque_grav_;
}

void RobotData::CalcGravCompensation(VectorXd &grav_torque)
{
    CalculateGravityCompensation(A_inv_, W_inv, N_C, J_C_INV_T, G_, grav_torque, P_C);
}

VectorXd RobotData::getContactForce(const VectorXd &command_torque)
{
    VectorXd cf;
    CalculateContactForce(command_torque, J_C_INV_T, P_C, cf);
    return cf;
}

int RobotData::CalcSingleTaskTorqueWithQP(TaskSpace &ts_, const MatrixXd &task_null_matrix_, const VectorXd &torque_prev, const MatrixXd &NwJw, const MatrixXd &J_C_INV_T, const MatrixXd &P_C, bool init_trigger)
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
        if (cc_[i].contact)
        {
            total_contact_dof += cc_[i].contact_dof_;
            contact_constraint_size += cc_[i].constraint_number_;
        }
    }
    contact_dof += total_contact_dof;

    if (contact_dof < 0)
    {
        contact_dof = 0;
    }

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
        if (contact_dof > 0)
            A.block(0, task_dof, model_size, contact_dof) = NwJw;
        // lbA.segment(0, model_size) = -torque_limit_ - torque_prev - Ntorque_task * ts_.f_star_;
        ubA.segment(0, model_size) = torque_limit_ - (torque_prev + Ntorque_task * ts_.f_star_);

        A.block(model_size, 0, model_size, task_dof) = -Ntorque_task;
        if (contact_dof > 0)
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
        if (cc_[i].contact)
        {
            A_rot.block(contact_idx, contact_idx, 3, 3) = cc_[i].rotm.transpose();
            A_rot.block(contact_idx + 3, contact_idx + 3, 3, 3) = cc_[i].rotm.transpose();

            A_const_a.block(const_idx, contact_idx, 4, 6) = cc_[i].GetZMPConstMatrix4x6();
            A_const_a.block(const_idx + CONTACT_CONSTRAINT_ZMP, contact_idx, 6, 6) = cc_[i].GetForceConstMatrix6x6();

            const_idx += cc_[i].constraint_number_;
            contact_idx += cc_[i].contact_dof_;
        }
    }

    Eigen::MatrixXd Atemp = A_const_a * A_rot * J_C_INV_T.rightCols(model_size);
    // t[3] = std::chrono::steady_clock::now();
    A.block(torque_limit_constraint_size, 0, contact_constraint_size, task_dof) = -Atemp * Ntorque_task;
    if (contact_dof > 0)
        A.block(torque_limit_constraint_size, task_dof, contact_constraint_size, contact_dof) = -Atemp * NwJw;
    // t[4] = std::chrono::steady_clock::now();

    Eigen::VectorXd bA = A_const_a * (A_rot * P_C) - Atemp * (torque_prev + Ntorque_task * ts_.f_star_);
    // Eigen::VectorXd ubA_contact;
    lbA.segment(torque_limit_constraint_size, contact_constraint_size).setConstant(-INFTY);

    // lbA.segment(total_constraint_size) = -ubA_contact;
    ubA.segment(torque_limit_constraint_size, contact_constraint_size) = -bA;

    // qp_.EnableEqualityCondition(0.0001);

    VectorXd qpres;

#ifdef COMPILE_QPSWIFT
    qpres = qpSwiftSolve(qp_task_[ts_.heirarchy_], variable_size, total_constraint_size, H, g, A, ubA, false);
    ts_.f_star_qp_ = qpres.segment(0, task_dof);
    return 1;
#else
    if (qp_task_[ts_.heirarchy_].CheckProblemSize(variable_size, total_constraint_size))
    {
        if (init_trigger)
        {
            qp_task_[ts_.heirarchy_].InitializeProblemSize(variable_size, total_constraint_size);
        }
    }
    else
    {
        qp_task_[ts_.heirarchy_].InitializeProblemSize(variable_size, total_constraint_size);
    }

    qp_task_[ts_.heirarchy_].UpdateMinProblem(H, g);
    qp_task_[ts_.heirarchy_].UpdateSubjectToAx(A, lbA, ubA);
    qp_task_[ts_.heirarchy_].DeleteSubjectToX();

    if (qp_task_[ts_.heirarchy_].SolveQPoases(300, qpres))
    {
        ts_.f_star_qp_ = qpres.segment(0, task_dof);

        ts_.contact_qp_ = qpres.segment(task_dof, contact_dof);
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
            CalcSingleTaskTorqueWithQP(ts_[i], MatrixXd::Identity(model_dof_, model_dof_), torque_grav_, NwJw, J_C_INV_T, P_C, init);

            torque_task_ = ts_[i].J_kt_ * ts_[i].Lambda_task_ * (ts_[i].f_star_ + ts_[i].f_star_qp_);
        }
        else
        {
            // ts_[i].J_kt_ = W_inv * ts_[i].Q_.transpose() * vt_jkt_[i - 1].get(); // PinvCODWB(Q * W_inv * Q.transpose());

            CalcSingleTaskTorqueWithQP(ts_[i], ts_[i - 1].Null_task_, torque_grav_, NwJw, J_C_INV_T, P_C, init);

            torque_task_ += ts_[i - 1].Null_task_ * (ts_[i].J_kt_ * ts_[i].Lambda_task_ * (ts_[i].f_star_ + ts_[i].f_star_qp_));
        }

        if (i != ts_.size() - 1)
            ts_[i].CalcNullMatrix(A_inv_, N_C);
    }
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
    return CalcContactRedistribute(torque_grav_ + torque_task_, init);
}

int RobotData::CalcContactRedistribute(VectorXd torque_input, bool init)
{
    int contact_index = cc_.size();  // size of contact link
    int total_contact_dof = 0;       // size of contact dof
    int contact_dof = -6;            // total_contact_dof - 6, (free contact space)
    int contact_constraint_size = 0; // size of constraint by contact
    int model_size = model_dof_;     // size of joints
    int torque_limit_constraint_size = 2 * model_size;

    for (int i = 0; i < contact_index; i++)
    {
        if (cc_[i].contact)
        {
            total_contact_dof += cc_[i].contact_dof_;
            contact_constraint_size += cc_[i].constraint_number_;
        }
    }
    contact_dof += total_contact_dof;

    if (!torque_limit_set_)
        torque_limit_constraint_size = 0;
    int variable_number = contact_dof;                                                  // total size of qp variable
    int total_constraint_size = contact_constraint_size + torque_limit_constraint_size; // total size of constraint

    if (contact_dof > 0)
    {
        MatrixXd H, H_temp;
        VectorXd g;

        Eigen::MatrixXd crot_matrix = Eigen::MatrixXd::Zero(total_contact_dof, total_contact_dof);
        Eigen::MatrixXd RotW = Eigen::MatrixXd::Identity(total_contact_dof, total_contact_dof);
        int acc_cdof = 0;
        for (int i = 0; i < contact_index; i++)
        {
            if (cc_[i].contact)
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
        }
        J_C_INV_T.rightCols(model_size) * NwJw;
        H_temp = RotW * crot_matrix * J_C_INV_T.rightCols(model_size) * NwJw;
        H = H_temp.transpose() * H_temp;
        g = (RotW * crot_matrix * (J_C_INV_T.rightCols(model_size) * torque_input - P_C)).transpose() * H_temp;

        MatrixXd A_qp;
        VectorXd lbA, ubA;
        A_qp.setZero(total_constraint_size, variable_number);
        lbA.setZero(total_constraint_size);
        ubA.setZero(total_constraint_size);

        if (torque_limit_set_)
        {
            A_qp.block(0, 0, model_size, contact_dof) = NwJw;
            // lbA.segment(0, model_size) = -torque_limit_ - control_torque;
            ubA.segment(0, model_size) = torque_limit_ - torque_input;

            A_qp.block(0, 0, model_size, contact_dof) = -NwJw;
            // lbA.segment(0, model_size) = -torque_limit_ - control_torque;
            ubA.segment(0, model_size) = torque_limit_ + torque_input;

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
            if (cc_[i].contact)
            {
                A_rot.block(contact_idx, contact_idx, 3, 3) = cc_[i].rotm.transpose(); // rd_.ee_[i].rotm.transpose();
                A_rot.block(contact_idx + 3, contact_idx + 3, 3, 3) = cc_[i].rotm.transpose();

                A_const_a.block(const_idx, contact_idx, 4, 6) = cc_[i].GetZMPConstMatrix4x6();
                A_const_a.block(const_idx + CONTACT_CONSTRAINT_ZMP, contact_idx, 6, 6) = cc_[i].GetForceConstMatrix6x6();

                const_idx += cc_[i].constraint_number_;
                contact_idx += cc_[i].contact_dof_;
            }

            // specific vector on Global axis
            // [0 0 -1]T *
            // Force Constraint
            // Will be added
        }

        Eigen::MatrixXd Atemp = A_const_a * A_rot * J_C_INV_T.rightCols(model_size);
        Eigen::VectorXd bA = A_const_a * (A_rot * P_C) - Atemp * torque_input;

        A_qp.block(torque_limit_constraint_size, 0, contact_constraint_size, contact_dof) = -Atemp * NwJw;

        lbA.segment(torque_limit_constraint_size, contact_constraint_size).setConstant(-INFTY);
        ubA.segment(torque_limit_constraint_size, contact_constraint_size) = -bA;

        Eigen::VectorXd qpres;

#ifdef COMPILE_QPSWIFT
        qpres = qpSwiftSolve(qp_contact_, variable_number, total_constraint_size, H, g, A_, ubA, false);
        // ts_.f_star_qp_ = qpres.segment(0, task_dof);
        torque_contact_ = NwJw * qpres;
        return 1;

#else
        if (qp_contact_.CheckProblemSize(variable_number, total_constraint_size))
        {
            if (init)
            {
                qp_contact_.InitializeProblemSize(variable_number, total_constraint_size);
            }
        }
        else
        {
            qp_contact_.InitializeProblemSize(variable_number, total_constraint_size);
        }

        qp_contact_.UpdateMinProblem(H, g);
        qp_contact_.UpdateSubjectToAx(A_qp, lbA, ubA);
        if (qp_contact_.SolveQPoases(600, qpres))
        {
            torque_contact_ = NwJw * qpres;

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
    else
    {
        torque_contact_ = VectorXd::Zero(model_size);

        return 1;
    }
}

VectorXd RobotData::GetControlTorque(bool task_control, bool init)
{
    if (init)
    {
    }

    VectorXd torque_control;

    return torque_control;
}

MatrixXd RobotData::CalcAngularMomentumMatrix()
{
    Eigen::MatrixXd H_C = MatrixXd::Zero(6, system_dof_);

    for (int i = 0; model_dof_; i++)
    {
        Eigen::MatrixXd rotm = MatrixXd::Identity(6, 6);
        rotm.block(0, 0, 3, 3) = link_[i].rotm;
        rotm.block(3, 3, 3, 3) = link_[i].rotm;

        link_[i].GetSpatialInertiaMatrix();

        Eigen::MatrixXd j_temp = MatrixXd::Zero(6, model_dof_);
        j_temp.topRows(3) = link_[i].jac_.bottomRows(3);
        j_temp.bottomRows(3) = link_[i].jac_.topRows(3);

        H_C += link_[i].GetSpatialTranform() * link_[i].GetSpatialInertiaMatrix() * rotm.transpose() * j_temp;
    }

    DWBC::Link com_;
    com_.rotm = Eigen::MatrixXd::Identity(3, 3);
    com_.xpos = link_[model_dof_].xpos;

    return com_.GetAdjointMatrix().transpose() * H_C;
}

void RobotData::CopyKinematicsData(RobotData &target_rd)
{
    target_rd.control_time_ = control_time_;
    target_rd.q_system_ = q_system_;
    target_rd.q_dot_system_ = q_dot_system_;
    target_rd.q_ddot_system_ = q_ddot_system_;

    target_rd.torque_limit_set_ = torque_limit_set_;

    target_rd.torque_limit_ = torque_limit_;

    // memcpy(&target_rd.model_, &model_, sizeof(model_));

    target_rd.model_ = model_;

    target_rd.A_ = A_;
    target_rd.A_inv_ = A_inv_;

    if (target_rd.link_.size() != link_.size())
    {
        target_rd.link_.resize(link_.size());
    }

    std::copy(link_.begin(), link_.end(), target_rd.link_.begin());

    if (target_rd.cc_.size() != cc_.size())
    {

        target_rd.cc_.resize(cc_.size());

        for (int i = 0; i < cc_.size(); i++)
        {
            cc_[i].contact = target_rd.cc_[i].contact;
        }
    }
    std::copy(cc_.begin(), cc_.end(), target_rd.cc_.begin());

    if (target_rd.ts_.size() != ts_.size())
    {
        target_rd.ts_.resize(ts_.size());
        target_rd.qp_task_.resize(qp_task_.size());
    }

    std::copy(ts_.begin(), ts_.end(), target_rd.ts_.begin());
    std::copy(qp_task_.begin(), qp_task_.end(), target_rd.qp_task_.begin());

    target_rd.G_ = G_;

    target_rd.CMM_ = CMM_;

    target_rd.B_ = B_;
}

void RobotData::DeleteLink(std::string link_name, bool verbose)
{
    // Edit rd link

    // find link index with strcasecmp
    int link_idx = -1;
    for (int i = 0; i < link_.size(); i++)
    {
        if (strcasecmp(link_[i].name_.c_str(), link_name.c_str()) == 0)
        {
            link_idx = i;
            break;
        }
    }

    if (link_idx == -1)
    {
        std::cout << "link name is not exist" << std::endl;
        return;
    }

    // find child link of link_idx and delete

    DeleteLink(link_idx, verbose);
}

void RobotData::DeleteLink(int link_idx, bool verbose)
{
    // if child link exist delete child link
    if (verbose)
        std::cout << "delete link : " << link_idx << link_[link_idx].name_ << std::endl;

    if (link_[link_idx].child_id_.size() > 0)
    {
        std::cout << "link : " << link_idx << "has child link : "  << link_[link_idx].child_id_.size() << " delete children link first. "<<std::endl;
    }

    // for (int i = 0; i < link_[link_idx].child_id_.size(); i++)
    // {

    //     DeleteLink(link_[link_idx].child_id_[i], verbose);
    // }

    // int rbdl_id = link_[link_idx].body_id_;

    int rbdl_id = model_.GetBodyId(link_[link_idx].name_.c_str());

    model_.hdotc.erase(model_.hdotc.begin() + rbdl_id);
    model_.hc.erase(model_.hc.begin() + rbdl_id);
    model_.Ic.erase(model_.Ic.begin() + rbdl_id);
    model_.I.erase(model_.I.begin() + rbdl_id);

    model_.f.erase(model_.f.begin() + rbdl_id);

    model_.U.erase(model_.U.begin() + rbdl_id);
    model_.pA.erase(model_.pA.begin() + rbdl_id);
    model_.IA.erase(model_.IA.begin() + rbdl_id);
    model_.c.erase(model_.c.begin() + rbdl_id);

    model_.X_T.erase(model_.X_T.begin() + rbdl_id);

    model_.multdof3_S.erase(model_.multdof3_S.begin() + rbdl_id);
    model_.multdof3_U.erase(model_.multdof3_U.begin() + rbdl_id);
    model_.multdof3_Dinv.erase(model_.multdof3_Dinv.begin() + rbdl_id);
    model_.multdof3_u.erase(model_.multdof3_u.begin() + rbdl_id);

    model_.S.erase(model_.S.begin() + rbdl_id); // joint motion subspace

    model_.X_J.erase(model_.X_J.begin() + rbdl_id); //
    model_.v_J.erase(model_.v_J.begin() + rbdl_id); //
    model_.c_J.erase(model_.c_J.begin() + rbdl_id); //

    model_.v.erase(model_.v.begin() + rbdl_id); // spatial velocity
    model_.a.erase(model_.a.begin() + rbdl_id); // spatial acceleration

    // std::cout << "model.mjoint dof : " << model_.mJoints[rbdl_id].mDoFCount << std::endl;
    // std::cout << "before model_.dof_count : " << model_.dof_count << std::endl;
    int joint_dof_count = model_.mJoints[rbdl_id].mDoFCount;
    model_.dof_count = model_.dof_count - joint_dof_count;

    // std::cout << "model.mjoint dof : " << model_.mJoints[rbdl_id].mDoFCount << std::endl;
    // std::cout << "after model_.dof_count : " << model_.dof_count << std::endl;

    int multdof3_joint_counter = 0;
    int mCustomJoint_joint_counter = 0;
    for (unsigned int i = 1; i < model_.mJoints.size(); i++)
    {
        if (model_.mJoints[i].mJointType == RigidBodyDynamics::JointTypeSpherical)
        {
            model_.multdof3_w_index[i] = model_.dof_count + multdof3_joint_counter;
            multdof3_joint_counter++;
        }
    }

    model_.q_size = model_.dof_count + multdof3_joint_counter;

    model_.qdot_size = model_.qdot_size - joint_dof_count;

    int parent_id = model_.lambda[rbdl_id];

    for (int i = 0; i < model_.mu[parent_id].size(); i++)
    {
        if (model_.mu[parent_id][i] == rbdl_id)
        {
            model_.mu[parent_id].erase(model_.mu[parent_id].begin() + i);
            break;
        }
    }

    model_.X_lambda.erase(model_.X_lambda.begin() + rbdl_id); // parent to child transform
    model_.X_base.erase(model_.X_base.begin() + rbdl_id);     // base to child transform
    model_.mBodies.erase(model_.mBodies.begin() + rbdl_id);   // body information

    for (int i = 0; i < model_.lambda.size(); i++)
    {
        if (model_.lambda[i] > rbdl_id)
        {
            model_.lambda[i]--;
        }
    }

    for (int i = 0; i < model_.mu.size(); i++)
    {
        for (int j = 0; j < model_.mu[i].size(); j++)
        {
            if (model_.mu[i][j] > rbdl_id)
            {
                model_.mu[i][j]--;
            }
        }
    }

    model_.mu.erase(model_.mu.begin() + rbdl_id); // children id of given body. in this case, mu is empty.

    model_.lambda.erase(model_.lambda.begin() + rbdl_id); // parent id

    // model_.lambda_q.erase(model_.lambda_q.begin() + rbdl_id); // parent id

    int joint_idx = model_.mJoints[rbdl_id].q_index;

    for (int i = 0; i < joint_dof_count; i++)
    {
        model_.lambda_q.erase(model_.lambda_q.begin() + joint_idx + i);
    }

    for (int i = 0; i < model_.lambda_q.size(); i++)
    {
        if (model_.lambda_q[i] > joint_idx)
        {
            model_.lambda_q[i] = model_.lambda_q[i] - joint_dof_count;
        }
    }

    for (int i = 0; i < model_.mJoints.size(); i++)
    {
        if (model_.mJoints[i].q_index > joint_idx)
        {
            model_.mJoints[i].q_index = model_.mJoints[i].q_index - joint_dof_count;
        }
    }

    model_.multdof3_w_index.erase(model_.multdof3_w_index.begin() + rbdl_id);
    model_.mJoints.erase(model_.mJoints.begin() + rbdl_id); // joint information

    model_.mJointUpdateOrder.clear();

    // update the joint order computation
    std::vector<std::pair<RigidBodyDynamics::JointType, unsigned int>> joint_types;
    for (unsigned int i = 0; i < model_.mJoints.size(); i++)
    {
        joint_types.push_back(
            std::pair<RigidBodyDynamics::JointType, unsigned int>(model_.mJoints[i].mJointType, i));
        model_.mJointUpdateOrder.push_back(model_.mJoints[i].mJointType);
    }

    model_.mJointUpdateOrder.clear();
    RigidBodyDynamics::JointType current_joint_type = RigidBodyDynamics::JointTypeUndefined;
    while (joint_types.size() != 0)
    {
        current_joint_type = joint_types[0].first;

        std::vector<std::pair<RigidBodyDynamics::JointType, unsigned int>>::iterator type_iter =
            joint_types.begin();

        while (type_iter != joint_types.end())
        {
            if (type_iter->first == current_joint_type)
            {
                model_.mJointUpdateOrder.push_back(type_iter->second);
                type_iter = joint_types.erase(type_iter);
            }
            else
            {
                ++type_iter;
            }
        }
    }

    // Edit fixed body

    for (int i = 0; i < model_.mFixedBodies.size(); i++)
    {
        if (model_.mFixedBodies[i].mMovableParent > rbdl_id)
        {
            model_.mFixedBodies[i].mMovableParent--;
        }
    }

    // std::cout << "model_.fixed_body_discriminator : " << model_.fixed_body_discriminator << std::endl;
    // std::cout << "model_.mFixedBodies.size() : " << model_.mFixedBodies.size() << std::endl;

    // std::cout << "model_.mFixedBodies[0].mMovableParent : " << model_.mFixedBodies[0].mMovableParent << std::endl;
    // std::cout << "model_.mFixedBodies[0].mParentTransform : " << model_.mFixedBodies[0].mParentTransform << std::endl;

    // std::cout << " Delete done." << std::endl;
    // // find parent link and delete child link info
    // int parent_id = link_[link_idx].parent_id_;

    // for (int i = 0; i < link_[parent_id].child_id_.size(); i++)
    // {
    //     // if child id is same with link_idx delete child id
    //     if (link_[parent_id].child_id_[i] == link_idx)
    //     {
    //         link_[parent_id].child_id_.erase(link_[parent_id].child_id_.begin() + i);
    //         break;
    //     }
    // }
    // // delete link
    // link_.erase(link_.begin() + link_idx);

    // print the data in mbodynamemap

    // for (auto it = model_.mBodyNameMap.begin(); it != model_.mBodyNameMap.end(); it++)
    // {
    //     std::cout << "key : " << it->first << " value : " << it->second << std::endl;
    // }

    model_.mBodyNameMap.erase(link_[link_idx].name_);

    // edit the value in mbodynamemap, if value is bigger than rbdl_id, value - 1
    for (auto it = model_.mBodyNameMap.begin(); it != model_.mBodyNameMap.end(); it++)
    {
        if (it->second > rbdl_id)
        {
            it->second--;
        }
    }
    // std::cout << "check change" << std::endl;
    // for (auto it = model_.mBodyNameMap.begin(); it != model_.mBodyNameMap.end(); it++)
    // {
    //     std::cout << "key : " << it->first << " value : " << it->second << std::endl;
    // }

    // Re update model data
    InitAfterModelMod(0, link_idx);

    if (verbose)
    {
        std::cout << "Delete Link done." << std::endl;
    }
}
/*
    joint_frame : translation, rotation (1x3, 3x3)
    Joint Axes
    Jointtype : // RigidBodyDynamics::{JointTypeRevoluteX .... }
    jointDofcount : //
        // How to create Joint ? RigidBodyDynamics::Joint()
        //
    q_index
    custom_joint_index(-1)

*/
void RobotData::AddLink(const char *parent_name, const char *link_name, const Matrix3d &joint_rotm, const Vector3d &joint_trans, double body_mass, const Vector3d &com_position, const Matrix3d &inertia, bool verbose)
{
    int link_id = getLinkID(parent_name);
    int parent_body_id = model_.GetBodyId(link_[link_id].name_.c_str());

    AddLink(parent_body_id, link_name, joint_rotm, joint_trans, body_mass, com_position, inertia, verbose);
}

void RobotData::AddLink(int parent_body_id, const char *link_name, const Matrix3d &joint_rotm, const Vector3d &joint_trans, double body_mass, const Vector3d &com_position, const Matrix3d &inertia, bool verbose)
{
    RigidBodyDynamics::Math::Vector3d com_pos_rbdl = com_position;
    RigidBodyDynamics::Math::Matrix3d inertia_rbdl = inertia;
    RigidBodyDynamics::Body Body(body_mass, com_pos_rbdl, inertia_rbdl);
    RigidBodyDynamics::Joint joint(RigidBodyDynamics::JointTypeFixed);
    RigidBodyDynamics::Math::Matrix3d joint_rotm_rbdl = joint_rotm;
    RigidBodyDynamics::Math::SpatialTransform rbdl_joint_frame = RigidBodyDynamics::Math::SpatialTransform(joint_rotm_rbdl, joint_trans);

    model_.AddBody(parent_body_id, rbdl_joint_frame, joint, Body, link_name);
    // if (init)
    //     InitAfterModelMod(false, );
    int parent_id = getLinkID(model_.GetBodyName(parent_body_id));

    // std::cout << parent_id << "pid" << std::endl;

    InitAfterModelMod(1, parent_id, verbose);
}

void RobotData::AddLink(Link &link, bool verbose)
{
    int parent_body_id = model_.GetBodyId(link_[link.parent_id_].name_.c_str());
    AddLink(parent_body_id, link.name_.c_str(), link.parent_rotm, link.parent_trans, link.mass, link.com_position_l_, link.inertia, verbose);
}

void RobotData::InitAfterModelMod(int mode, int link_id, bool verbose)
{
    int change_link_id = link_id;
    int corresponding_body_id = link_[change_link_id].body_id_;

    system_dof_ = model_.dof_count;
    model_dof_ = system_dof_ - 6;

    // delete link for link vector

    if (mode == 0)
    {
        int deleted_id = change_link_id;
        int deleted_body_id = corresponding_body_id;

        Link link_deleted = link_[deleted_id];
        link_.erase(link_.begin() + deleted_id);

        int parent_id = link_deleted.parent_id_;

        total_mass_ = total_mass_ - link_deleted.mass;
        link_.back().mass = total_mass_;

        for (int i = 0; i < link_.size() - 1; i++)
        {
            if (link_[i].body_id_ > deleted_body_id)
            {
                link_[i].body_id_--;
            }

            if (link_[i].link_id_ > deleted_id)
            {
                link_[i].link_id_--;
            }

            if (link_[i].parent_id_ > deleted_id)
            {
                link_[i].parent_id_--;
            }
        }

        // deleted link had parent link. that link has a children information of deleted link. delete that children information

        for (int i = 0; i < link_[parent_id].child_id_.size(); i++)
        {
            if (link_[parent_id].child_id_[i] == deleted_id)
            {
                link_[parent_id].child_id_.erase(link_[parent_id].child_id_.begin() + i);
                break;
            }
        }

        // since the link id has changed, the child link id of the parent link must be changed.
        for (int i = 0; i < link_.size(); i++)
        {
            for (int j = 0; j < link_[i].child_id_.size(); j++)
            {
                if (link_[i].child_id_[j] > deleted_id)
                {
                    link_[i].child_id_[j]--;
                }
            }
        }

        int deleted_task_id = -1;

        for (int i = 0; i < ts_.size(); i++)
        {
            if (ts_[i].link_id_ == deleted_id)
            {
                deleted_task_id = i;
                ts_.erase(ts_.begin() + i);
            }
        }

        if (deleted_task_id >= 0)
        {
            if (verbose)
            {
                std::cout << "Task " << deleted_task_id << " is deleted" << std::endl;
            }

            for (int i = 0; i < ts_.size(); i++)
            {
                if (ts_[i].heirarchy_ > deleted_task_id)
                {
                    ts_[i].heirarchy_--;
                }

                if (ts_[i].link_id_ > deleted_id)
                {
                    ts_[i].link_id_--;
                }
            }

            qp_task_.erase(qp_task_.begin() + deleted_task_id);
        }

        // for (int i = 0; i < cc_.size(); i++)
        // {
        //     if (cc_[i].link_number_ == deleted_id)
        //     {
        //         cc_.erase(cc_.begin() + i);
        //         i--;
        //     }
        //     else if (cc_[i].link_number_ > deleted_id)
        //     {
        //         cc_[i].link_number_--;
        //     }

        //     if (cc_[i].rbdl_body_id_ > deleted_body_id)
        //     {
        //         cc_[i].rbdl_body_id_--;
        //     }
        // }
    }
    else if (mode == 1)
    {
        // Link added with fixed joint.
        int parent_id = change_link_id;
        int added_parent_body_id = corresponding_body_id;

        // Change the inertial information of parent link
        link_[parent_id].com_position_l_ = model_.mBodies[added_parent_body_id].mCenterOfMass;
        link_[parent_id].mass = model_.mBodies[added_parent_body_id].mMass;
        link_[parent_id].inertia = model_.mBodies[added_parent_body_id].mInertia;
    }

    q_system_.setZero(model_.q_size);
    q_dot_system_.setZero(model_.qdot_size);
    q_ddot_system_.setZero(model_.qdot_size);

    J_com_.setZero(6, system_dof_);

    A_.setZero(system_dof_, system_dof_);
    A_inv_.setZero(system_dof_, system_dof_);

    G_.setZero(system_dof_);
    torque_grav_.setZero(model_dof_);
    torque_task_.setZero(model_dof_);
    torque_contact_.setZero(model_dof_);

    torque_limit_.setZero(model_dof_);

    // for (int i = 0; i < qp_task_.size(); i++)
    // {
    //     qp_task_[i] = CQuadraticProgram();
    // }

    // qp_contact_ = CQuadraticProgram();
}

void RobotData::ChangeLinkToFixedJoint(std::string link_name, bool verbose)
{
    // find corresponding link
    int link_idx = getLinkID(link_name);

    // save link and delete link
    Link link = link_[link_idx];
    if (verbose)
    {
        std::cout << "Delete link id : " << link_idx << std::endl;
        link.Print();
    }

    DeleteLink(link_name, verbose);

    // Add link
    AddLink(link, verbose);
    if (verbose)
        std::cout << "Deleted Link : " << link_name << " and Added Link : " << link_name << " as fixed joint." << std::endl;
}

int RobotData::getLinkID(std::string link_name)
{
    // get link id from link name, use strcasecmp
    for (int i = 0; i < link_.size(); i++)
    {
        if (strcasecmp(link_[i].name_.c_str(), link_name.c_str()) == 0)
        {
            return i;
        }
    }

    std::cout << "There is no link name : " << link_name << std::endl;
    return -1;
}

void RobotData::printLinkInfo()
{
    for (int i = 0; i < link_.size(); i++)
    {
        // print link id, link name, parent link id, child link list,
        // print link position, orientation, mass, inertia
        // print link jacobian
        std::cout << "------" << std::endl;
        std::cout << "link id : " << i << " rbdl id : " << link_[i].body_id_ << " link name : " << link_[i].name_ << " parent id : " << link_[i].parent_id_ << " child id : ";
        for (int j = 0; j < link_[i].child_id_.size(); j++)
        {
            std::cout << link_[i].child_id_[j] << " ";
        }
        std::cout << std::endl;
        std::cout << "link position : " << link_[i].xpos.transpose() << "link mass : " << link_[i].mass << std::endl
                  << "link orientation : " << std::endl
                  << link_[i].rotm << std::endl;
        // std::cout << " link inertia : " << link_[i].inertia << std::endl;
        // std::cout << "link jacobian : " << link_[i].jac_ << std::endl;
        std::cout << "joint frame E : " << model_.X_T[link_[i].body_id_].E << std::endl;
        std::cout << "joint frame r : " << model_.X_T[link_[i].body_id_].r.transpose() << std::endl;

        std::cout << std::endl;
    }
}