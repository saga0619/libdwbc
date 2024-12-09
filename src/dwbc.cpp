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
    osqp_task_.clear();
}

void RobotData::AddQP()
{
    qp_task_.push_back(CQuadraticProgram());

    OsqpEigen::Solver *osqp_ti = new OsqpEigen::Solver();
    osqp_task_.push_back(osqp_ti);
}
#endif

RobotData::RobotData(/* args */)
{
    torque_limit_set_ = false;
}

RobotData::~RobotData()
{
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
    is_floating_ = floating;

    bool rbdl_v = false;
    if (verbose == 2)
    {
        rbdl_v = true;
    }
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_path.c_str(), &model_, floating, rbdl_v);

    if (verbose)
    {
        std::cout << "rbdl urdf load success " << std::endl;
    }

    InitModelData(verbose);
}

void RobotData::InitModelData(int verbose)
{
    ts_.clear();
    cc_.clear();
    link_.clear();
    qp_task_.clear();

    qp_contact_ = CQuadraticProgram();

    osqp_contact_ = new OsqpEigen::Solver();

    system_dof_ = model_.dof_count;
    if (is_floating_)
    {
        model_dof_ = system_dof_ - 6;
    }
    else
    {
        model_dof_ = system_dof_;
    }

    if (verbose)
    {

        std::cout << "System DOF : " << system_dof_ << std::endl;
        std::cout << "Model DOF : " << model_dof_ << std::endl;
        std::cout << "Model.dof : " << model_.dof_count << std::endl;

        std::cout << "Model.mBodies.size() : " << model_.mBodies.size() << std::endl;
        std::cout << "Model.mJoints.size() : " << model_.mJoints.size() << std::endl;
    }

    total_mass_ = 0;

    int added_joint_dof = 0;
    for (int i = 0; i < model_.mBodies.size(); i++)
    {
        if (model_.mBodies[i].mMass != 0) // if body has mass,
        {
            link_.push_back(Link(model_, i));
            if (model_.mJoints[i].mJointType >= RigidBodyDynamics::JointTypeRevoluteX && model_.mJoints[i].mJointType <= RigidBodyDynamics::JointTypeRevoluteZ)
            {
                joint_.push_back(Joint(JOINT_REVOLUTE, model_.mJoints[i].mJointAxes[0].segment(0, 3)));
                joint_.back().joint_rotation_ = link_.back().joint_rotm;
                joint_.back().joint_translation_ = link_.back().joint_trans;
            }
            else if (model_.mJoints[i].mJointType == RigidBodyDynamics::JointTypeRevolute || model_.mJoints[i].mJointType == RigidBodyDynamics::JointTypeHelical)
            {

                joint_.push_back(Joint(JOINT_REVOLUTE, model_.mJoints[i].mJointAxes[0].segment(0, 3)));
                joint_.back().joint_rotation_ = link_.back().joint_rotm;
                joint_.back().joint_translation_ = link_.back().joint_trans;
            }
            else if (model_.mJoints[i].mJointType == RigidBodyDynamics::JointTypeFloatingBase || model_.mJoints[i].mJointType == RigidBodyDynamics::JointTypeSpherical)
            {
                joint_.push_back(Joint(JOINT_FLOATING_BASE));
            }
            else
            {
                if (verbose)
                {
                    std::cout << "JOINT TYPE at initializing Link & Joint vector : " << model_.mJoints[i].mJointType << " with dof : " << model_.mJoints[i].mDoFCount << " at link : " << link_.back().name_ << std::endl;
                    std::cout << model_.mJoints[i].mJointAxes[0] << std::endl;
                }
                joint_.push_back(Joint());
            }

            if (model_.mJoints[i].mDoFCount == 0)
            {
                joint_.back().joint_id_ = model_.mJoints[i].q_index;
            }
            else
            {
                joint_.back().joint_id_ = model_.mJoints[i].q_index;
            }

            // joint_.push_back(Joint())

            total_mass_ += model_.mBodies[i].mMass;
        }
    }

    link_num_ = link_.size();

    for (int i = 0; i < link_.size(); i++)
    {
        link_[i].parent_id_ = 0;
        link_[i].child_id_.clear();
        link_[i].link_id_ = i;
        link_[i].link_id_original_ = i;
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
        std::cout << "System DOF :" << system_dof_ << std::endl;
        std::cout << "Model DOF :" << model_dof_ << std::endl;

        std::cout << "Total Link with mass : " << link_num_ << std::endl;
        std::cout << "Total Mass : " << total_mass_ << std::endl;
        std::cout << "Link Information" << std::endl;

        int vlink = 0;
        for (int i = 0; i < link_.size() - 1; i++)
        {
            std::cout << vlink << " : " << link_[vlink].name_ << std::endl;
            std::cout << "mass : " << link_[vlink].mass << " body id : " << link_[vlink].body_id_ << " parent id : " << link_[vlink++].parent_id_ << std::endl;
        }

        std::cout << vlink << " : " << link_.back().name_ << std::endl;
    }
    InitializeMatrix();
}

void RobotData::SetTorqueLimit(const VectorXd &torque_limit)
{
    torque_limit_set_ = true;
    torque_limit_ = torque_limit;
}

void RobotData::InitializeMatrix()
{
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

    A_.setZero(system_dof_, system_dof_);
    if (update_kinematics)
    {
        RigidBodyDynamics::UpdateKinematicsCustom(model_, &q_virtual, &q_dot_virtual, &q_ddot_virtual);
        RigidBodyDynamics::CompositeRigidBodyAlgorithm(model_, q_virtual, A_, false);
        // A_inv_ = A_.inverse();
        A_inv_ = A_.llt().solve(Eigen::MatrixXd::Identity(system_dof_, system_dof_)); // Faster than inverse()
    }
    // J_com_.setZero(3, system_dof_);
    G_.setZero(system_dof_);
    com_pos.setZero();
    com_vel.setZero();
    for (int i = 0; i < (link_.size() - 1); i++)
    {
        link_[i].UpdateAll(model_, q_virtual, q_dot_virtual);
        joint_[i].parent_rotation_ = model_.X_lambda[link_[i].body_id_].E.transpose();
        joint_[i].parent_translation_ = model_.X_lambda[link_[i].body_id_].r;
    }

    Matrix3d skm_temp = link_[0].rotm * A_.block(3, 0, 3, 3) / total_mass_;

    Vector3d com_from_pelv(skm_temp(2, 1), skm_temp(0, 2), skm_temp(1, 0));

    com_pos = com_from_pelv + q_system_.segment(0, 3);

    link_.back().xpos = com_pos;
    link_.back().xipos = com_pos;

    // Centroidal momentum calculation !
    // inertial frame ::: position of com, rotation frame from global.
    Matrix6d cm_rot6 = Matrix6d::Identity(6, 6);
    cm_rot6.block(3, 3, 3, 3) = link_[0].rotm;                   // rotation matrix of com from global
    cm_rot6.block(3, 0, 3, 3) = skew(com_from_pelv).transpose(); // skew matrix of com position from global

    // Matrix6d cm_rot6 = Matrix6d::Identity(6, 6);
    // cm_rot6.block(3, 3, 3, 3) = link_[0].rotm;
    // Eigen::Vector3d com_pos_local = link_[0].rotm.transpose() * (com_from_pelv);
    // cm_rot6.block(3, 0, 3, 3) = link_[0].rotm * (skew(com_pos_local)).transpose() * link_[0].rotm.transpose();

    CMM_ = cm_rot6 * A_.topRows(6); // cmm calc, "Improved Computation of the Humanoid Centroidal Dynamics and Application for Whole-Body Control"
    // CMM matrix is from global frame

    B_.setZero(system_dof_);
    RigidBodyDynamics::NonlinearEffects(model_, q_virtual, q_dot_virtual, B_);

    link_.back().inertia = link_[0].rotm * A_.block(3, 3, 3, 3) * link_[0].rotm.transpose() - total_mass_ * skew(com_from_pelv) * skew(com_from_pelv).transpose(); // inertia matrix of com at global frame.

    SI_body_ = Matrix6d::Zero(6, 6);
    SI_body_.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity() * total_mass_;
    SI_body_.block(3, 3, 3, 3) = link_.back().inertia;

    link_.back().rotm = link_[0].rotm; // get info of pelvis rotation
    link_.back().mass = total_mass_;

    link_.back().jac_com_ = SI_body_.inverse() * CMM_;
    link_.back().jac_ = link_.back().jac_com_;

    G_ = -link_.back().jac_.topRows(3).transpose() * total_mass_ * Vector3d(0, 0, -9.81);

    Vector6d com_vel_temp = link_.back().jac_ * q_dot_system_;

    link_.back().v = com_vel_temp.segment(0, 3);
    // link_.back().w = link_[0].w;

    link_.back().vi = com_vel_temp.segment(0, 3);
    link_.back().w = com_vel_temp.segment(3, 3);

    // link_.back().vi = jac_com_

    // UpdateContactConstraint();
}
void RobotData::UpdateKinematics(bool update_kinematics)
{
    UpdateKinematics(q_system_, q_dot_system_, q_ddot_system_, update_kinematics);
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
}

int RobotData::CalcContactConstraint()
{
    // UpdateContactConstraint();
    // int contact_dof

    Lambda_contact.setZero(contact_dof_, contact_dof_);
    J_C_INV_T.setZero(contact_dof_, system_dof_);
    N_C.setZero(system_dof_, system_dof_);

    W.setZero(model_dof_, model_dof_);
    W_inv.setZero(model_dof_, model_dof_);

    int contact_null_dof = contact_dof_ - 6;

    V2.setZero(contact_null_dof, model_dof_);
    NwJw.setZero(model_dof_, model_dof_);

    P_C.setZero(contact_dof_, system_dof_);

    A_inv_N_C.setZero(system_dof_, system_dof_);

    return CalculateContactConstraint(J_C, A_inv_, Lambda_contact, J_C_INV_T, N_C, A_inv_N_C, W, NwJw, W_inv, V2);
}

MatrixXd RobotData::getContactConstraintMatrix()
{
    MatrixXd _C(contact_link_num_ * 10, contact_dof_);
    getContactConstraintMatrix(_C);
    return _C;
}
void RobotData::getContactConstraintMatrix(MatrixXd &C_)
{

    Eigen::MatrixXd A_const_a;
    A_const_a.setZero((CONTACT_CONSTRAINT_ZMP + CONTACT_CONSTRAINT_FORCE) * contact_link_num_, contact_dof_);

    Eigen::MatrixXd A_rot;
    A_rot.setZero(contact_dof_, contact_dof_);
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

    C_ = -A_const_a * A_rot;
}

void RobotData::ClearTaskSpace()
{
    ts_.clear();

    ClearQP();
}

void RobotData::AddTaskSpace(int heirarchy, int task_mode, int task_dof, bool verbose)
{
    if (verbose)
        std::cout << "#" << ts_.size() << " Task Space Added with mode " << taskmode_str[task_mode] << std::endl;

    ts_.push_back(TaskSpace(task_mode, ts_.size(), task_dof));

    AddQP();
}
void RobotData::AddTaskSpace(int heirarchy, int task_mode, int link_number, Vector3d task_point, bool verbose)
{
    if (verbose)
        std::cout << "#" << ts_.size() << " Task Space Added : " << link_[link_number].name_ << " " << taskmode_str[task_mode] << " at point : " << task_point.transpose() << std::endl;

    for (int i = 0; i < ts_.size(); i++)
    {
        for (int j = 0; j < ts_[i].link_size_; j++)
        {
            if (ts_[i].task_link_[j].link_id_ == link_number)
            {
                std::cout << "Task Space Already Exist for Link : " << link_[link_number].name_ << std::endl;
                return;
            }
        }
    }

    ts_.push_back(TaskSpace(task_mode, ts_.size(), link_number, task_point));

    AddQP();
}

void RobotData::AddTaskSpace(int heirarchy, int task_mode, const char *link_name, Vector3d task_point, bool verbose)
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

    // if link_name is "COM",

    for (int i = 0; i < ts_.size(); i++)
    {
        for (int j = 0; j < ts_[i].task_link_.size(); j++)
        {
            if (ts_[i].task_link_[j].link_id_ == link_number)
            {
                std::cout << "Task Space Already Exist for Link : " << link_[link_number].name_ << std::endl;
                return;
            }
        }
    }
    int ts_size = ts_.size();

    if (ts_size == heirarchy)
    {
        ts_.push_back(TaskSpace(task_mode, heirarchy, link_number, task_point));
        AddQP();
    }
    else if (ts_size > heirarchy)
    {
        ts_[heirarchy].AddTaskLink(task_mode, link_number, task_point);
    }
}

void RobotData::AddTaskLink(int heirarchy, int task_mode, int link_number, Vector3d task_point, bool verbose)
{
    if (verbose)
        std::cout << "#" << ts_.size() << " Task Space Added : " << link_[link_number].name_ << " " << taskmode_str[task_mode] << " at point : " << task_point.transpose() << std::endl;

    for (int i = 0; i < ts_.size(); i++)
    {
        for (int j = 0; j < ts_[i].link_size_; j++)
        {
            if (ts_[i].task_link_[j].link_id_ == link_number)
            {
                std::cout << "Task Space Already Exist for Link : " << link_[link_number].name_ << std::endl;
                return;
            }
        }
    }

    ts_[heirarchy].AddTaskLink(task_mode, link_number, task_point);
}

void RobotData::AddTaskLink(int heirarchy, int task_mode, const char *link_name, Vector3d task_point, bool verbose)
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
        for (int j = 0; j < ts_[i].link_size_; j++)
        {
            if (ts_[i].task_link_[j].link_id_ == link_number)
            {
                std::cout << "Task Space Already Exist for Link : " << link_[link_number].name_ << std::endl;
                return;
            }
        }
    }

    ts_[heirarchy].AddTaskLink(task_mode, link_number, task_point);
}

void RobotData::SetTaskSpace(int heirarchy, const VectorXd &f_star, const MatrixXd &J_task)
{
    if (ts_[heirarchy].task_dof_ != f_star.size())
    {
        std::cout << "ERROR : task dof not matching! heir : " << heirarchy << " fstar size : " << f_star.size() << " task_dof : " << ts_[heirarchy].task_dof_ << std::endl;
    }
    if (heirarchy >= ts_.size())
    {
        std::cout << "ERROR : task space size overflow" << std::endl;
    }
    else
    {
        ts_[heirarchy].J_task_ = J_task;
        ts_[heirarchy].f_star_ = f_star;
        ts_[heirarchy].f_star_qp_.setZero(f_star.size());
    }
}

// void RobotData::SetTaskTrajectory(int heirarchy, const MatrixXd &f_star, const MatrixXd &J_task)
// {

// }

void RobotData::UpdateTaskSpace()
{
    // int system_dof = model_.qdot_size;

    // Calculate Base Link By default
    link_[0].UpdateJac(model_, q_system_);

    for (int i = 0; i < ts_.size(); i++)
    {
        if (ts_[i].link_size_ > 0)
        {
            ts_[i].J_task_.setZero(ts_[i].task_dof_, system_dof_);
        }

        int task_cum_dof = 0;
        for (int j = 0; j < ts_[i].link_size_; j++)
        {
            if ((link_[ts_[i].task_link_[j].link_id_].name_ != "COM") && (ts_[i].task_link_[j].link_id_ != 0))
            {
                link_[ts_[i].task_link_[j].link_id_].UpdateJac(model_, q_system_);
            }

            if (ts_[i].task_type_ != TASK_CUSTOM)
            {
                switch (ts_[i].task_link_[j].task_link_mode_)
                {
                case TASK_LINK_6D:
                    ts_[i].J_task_.block(task_cum_dof, 0, 6, system_dof_) = link_[ts_[i].task_link_[j].link_id_].jac_;

                    if (ts_[i].task_link_[j].traj_pos_set)
                    {
                        ts_[i].f_star_.segment(task_cum_dof, 3) = ts_[i].task_link_[j].GetFstarPosPD(control_time_, link_[ts_[i].task_link_[j].link_id_].xpos, link_[ts_[i].task_link_[j].link_id_].v);
                    }
                    if (ts_[i].task_link_[j].traj_rot_set)
                    {
                        ts_[i].f_star_.segment(task_cum_dof + 3, 3) = ts_[i].task_link_[j].GetFstarRotPD(control_time_, link_[ts_[i].task_link_[j].link_id_].rotm, link_[ts_[i].task_link_[j].link_id_].w);
                    }
                    break;
                case TASK_LINK_6D_COM_FRAME:
                    ts_[i].J_task_.block(task_cum_dof, 0, 6, system_dof_) = link_[ts_[i].task_link_[j].link_id_].jac_com_;

                    if (ts_[i].task_link_[j].traj_pos_set)
                    {
                        ts_[i].f_star_.segment(task_cum_dof, 3) = ts_[i].task_link_[j].GetFstarPosPD(control_time_, link_[ts_[i].task_link_[j].link_id_].xipos, link_[ts_[i].task_link_[j].link_id_].vi);
                    }

                    if (ts_[i].task_link_[j].traj_rot_set)
                    {
                        ts_[i].f_star_.segment(task_cum_dof + 3, 3) = ts_[i].task_link_[j].GetFstarRotPD(control_time_, link_[ts_[i].task_link_[j].link_id_].rotm, link_[ts_[i].task_link_[j].link_id_].w);
                    }
                    break;
                case TASK_LINK_6D_CUSTOM_FRAME:
                    ts_[i].J_task_.block(task_cum_dof, 0, 6, system_dof_) = link_[ts_[i].task_link_[j].link_id_].GetPointJac(model_, q_dot_system_, ts_[i].task_link_[j].task_point_);
                    if (ts_[i].task_link_[j].traj_pos_set)
                    {
                        ts_[i].f_star_.segment(task_cum_dof, 3) = ts_[i].task_link_[j].GetFstarPosPD(control_time_,
                                                                                                     link_[ts_[i].task_link_[j].link_id_].xpos + link_[ts_[i].task_link_[j].link_id_].rotm * ts_[i].task_link_[j].task_point_,
                                                                                                     link_[ts_[i].task_link_[j].link_id_].v + link_[ts_[i].task_link_[j].link_id_].w.cross(ts_[i].task_link_[j].task_point_));
                    }
                    if (ts_[i].task_link_[j].traj_rot_set)
                    {
                        ts_[i].f_star_.segment(task_cum_dof + 3, 3) = ts_[i].task_link_[j].GetFstarRotPD(control_time_, link_[ts_[i].task_link_[j].link_id_].rotm, link_[ts_[i].task_link_[j].link_id_].w);
                    }
                    break;
                case TASK_LINK_POSITION:
                    ts_[i].J_task_.block(task_cum_dof, 0, 3, system_dof_) = link_[ts_[i].task_link_[j].link_id_].jac_.topRows(3);
                    if (ts_[i].task_link_[j].traj_pos_set)
                    {
                        ts_[i].f_star_.segment(task_cum_dof, 3) = ts_[i].task_link_[j].GetFstarPosPD(control_time_, link_[ts_[i].task_link_[j].link_id_].xpos, link_[ts_[i].task_link_[j].link_id_].v);
                    }
                    break;
                case TASK_LINK_POSITION_COM_FRAME:
                    ts_[i].J_task_.block(task_cum_dof, 0, 3, system_dof_) = link_[ts_[i].task_link_[j].link_id_].jac_com_.topRows(3);
                    if (ts_[i].task_link_[j].traj_pos_set)
                    {
                        ts_[i].f_star_.segment(task_cum_dof, 3) = ts_[i].task_link_[j].GetFstarPosPD(control_time_, link_[ts_[i].task_link_[j].link_id_].xipos, link_[ts_[i].task_link_[j].link_id_].vi);
                    }
                    break;
                case TASK_LINK_POSITION_CUSTOM_FRAME:
                    ts_[i].J_task_.block(task_cum_dof, 0, 3, system_dof_) = link_[ts_[i].task_link_[j].link_id_].GetPointJac(model_, q_dot_system_, ts_[i].task_link_[j].task_point_).topRows(3);
                    if (ts_[i].task_link_[j].traj_pos_set)
                    {
                        ts_[i].f_star_.segment(task_cum_dof, 3) = ts_[i].task_link_[j].GetFstarPosPD(control_time_,
                                                                                                     link_[ts_[i].task_link_[j].link_id_].xpos + link_[ts_[i].task_link_[j].link_id_].rotm * ts_[i].task_link_[j].task_point_,
                                                                                                     link_[ts_[i].task_link_[j].link_id_].v + link_[ts_[i].task_link_[j].link_id_].w.cross(ts_[i].task_link_[j].task_point_));
                    }
                    break;
                case TASK_LINK_ROTATION:
                    ts_[i].J_task_.block(task_cum_dof, 0, 3, system_dof_) = link_[ts_[i].task_link_[j].link_id_].jac_.bottomRows(3);
                    if (ts_[i].task_link_[j].traj_rot_set)
                    {
                        ts_[i].f_star_.segment(task_cum_dof, 3) = ts_[i].task_link_[j].GetFstarRotPD(control_time_, link_[ts_[i].task_link_[j].link_id_].rotm, link_[ts_[i].task_link_[j].link_id_].w);
                    }
                    break;
                case TASK_LINK_ROTATION_CUSTOM_FRAME:
                    ts_[i].J_task_.block(task_cum_dof, 0, 3, system_dof_) = link_[ts_[i].task_link_[j].link_id_].jac_.bottomRows(3);
                    if (ts_[i].task_link_[j].traj_rot_set)
                    {
                        ts_[i].f_star_.segment(task_cum_dof, 3) = ts_[i].task_link_[j].GetFstarRotPD(control_time_, link_[ts_[i].task_link_[j].link_id_].rotm, link_[ts_[i].task_link_[j].link_id_].w);
                    }
                    break;
                default:
                    break;
                }
                task_cum_dof += ts_[i].task_link_[j].t_dof_;
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

        if (i != ts_.size() - 1)
        {
            if (i == 0)
            {
                ts_[i].CalcNullMatrix(A_inv_N_C);
            }
            else
            {
                ts_[i].CalcNullMatrix(A_inv_N_C, ts_[i - 1].Null_task_);
            }
        }
    }
}

int RobotData::CalcTaskControlTorque(bool hqp, bool init, bool update_task_space)
{
    if (update_task_space)
        CalcTaskSpace();

    torque_task_.setZero(model_dof_);
    torque_contact_.setZero(model_dof_);

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

                ts_[i].torque_h_ = ts_[i].J_kt_ * ts_[i].Lambda_task_ * (ts_[i].f_star_ + ts_[i].f_star_qp_);
                torque_task_ = ts_[i].torque_h_;
            }
            else
            {
                qp_res = CalcSingleTaskTorqueWithQP(ts_[i], ts_[i - 1].Null_task_, torque_grav_ + torque_task_, NwJw, J_C_INV_T, P_C, init);
                if (qp_res == 0)
                    return 0;

                ts_[i].torque_h_ = ts_[i].J_kt_ * ts_[i].Lambda_task_ * (ts_[i].f_star_ + ts_[i].f_star_qp_);
                torque_task_ += ts_[i - 1].Null_task_ * ts_[i].torque_h_;
            }
            torque_contact_ = NwJw * ts_[i].contact_qp_;
        }

        return 1;
    }
    else
    {
        for (int i = 0; i < ts_.size(); i++)
        {
            ts_[i].torque_h_ = ts_[i].J_kt_ * ts_[i].Lambda_task_ * ts_[i].f_star_;
            if (i == 0)
            {
                torque_task_ = ts_[i].torque_h_;
            }
            else
            {
                torque_task_ += ts_[i - 1].Null_task_ * ts_[i].torque_h_;
            }
        }

        return 1;
    }
}

VectorXd RobotData::CalcGravCompensation()
{
    torque_task_.setZero(model_dof_);
    torque_contact_.setZero(model_dof_);
    torque_grav_.setZero(model_dof_);

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

Vector3d RobotData::getZMP(const VectorXd &contact_force)
{

    Vector3d zmp_pos;
    zmp_pos.setZero();

    double total_fz = 0;

    int contact_number = 0;
    for (int i = 0; i < cc_.size(); i++)
    {
        if (cc_[i].contact)
        {
            contact_number++;

            total_fz = total_fz + contact_force(2 + i * 6);
        }
    }

    for (int i = 0; i < cc_.size(); i++)
    {
        if (cc_[i].contact)
        {
            if (contact_force(i * 6 + 2) > -1.0E-3)
            {
                cc_[i].zmp_pos(0) = cc_[i].xc_pos(0);
                cc_[i].zmp_pos(1) = cc_[i].xc_pos(1);
                cc_[i].zmp_pos(2) = cc_[i].xc_pos(2);
            }
            else
            {
                cc_[i].zmp_pos(0) = cc_[i].xc_pos(0) + (-contact_force(i * 6 + 4) / contact_force(i * 6 + 2));
                cc_[i].zmp_pos(1) = cc_[i].xc_pos(1) + (contact_force(i * 6 + 3) / contact_force(i * 6 + 2));
                cc_[i].zmp_pos(2) = cc_[i].xc_pos(2);
            }

            zmp_pos = zmp_pos + cc_[i].zmp_pos * contact_force(i * 6 + 2) / total_fz;
        }
    }

    return zmp_pos;
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

    int torque_limit_constraint_size = 0;
    int fz_limit_const_size = 1;

    if (init_trigger)
    {
        if (torque_limit_set_)
        {
            if (model_size != torque_limit_.size())
            {
                cout << "Error : task qp torque limit size is not matched with model size! model dof :" << model_dof_ << "torque limit size : " << torque_limit_.size() << endl;
            }
        }

        // if(task_null_matrix_.)
    }

    for (int i = 0; i < contact_index; i++)
    {
        if (cc_[i].contact)
        {
            total_contact_dof += cc_[i].contact_dof_;
            if (cc_[i].fz_limiter_switch_)
            {
                contact_constraint_size += cc_[i].constraint_number_ + fz_limit_const_size;
            }
            else
            {
                contact_constraint_size += cc_[i].constraint_number_;
            }
        }
    }
    contact_dof += total_contact_dof;

    if (contact_dof < 0)
    {
        contact_dof = 0;
    }

    int variable_size = task_dof + contact_dof;

    if (torque_limit_set_)
        torque_limit_constraint_size = 2 * model_size;
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

    VectorXd Alim = VectorXd::Zero(contact_constraint_size);

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

            if (cc_[i].fz_limiter_switch_)
            {

                A_const_a(const_idx + CONTACT_CONSTRAINT_ZMP + 6, contact_idx + 2) = -1.0;
                Alim(const_idx + CONTACT_CONSTRAINT_ZMP + 6) = cc_[i].fz_limit_;
                const_idx += 1;
            }

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
    ubA.segment(torque_limit_constraint_size, contact_constraint_size) = -bA - Alim;

    // qp_.EnableEqualityCondition(0.0001);

    VectorXd qpres;

#ifdef COMPILE_QPSWIFT
    qpres = qpSwiftSolve(qp_task_[ts_.heirarchy_], variable_size, total_constraint_size, H, g, A, ubA, false);
    ts_.f_star_qp_ = qpres.segment(0, task_dof);
    return 1;
#else

    int solve_success = SolveOSQP(osqp_task_[ts_.heirarchy_], H, g, A, lbA, ubA, qpres, init_trigger);

    if (solve_success)
    {
        ts_.f_star_qp_ = qpres.segment(0, task_dof);
        ts_.contact_qp_ = qpres.segment(task_dof, contact_dof);
        return 1;
    }
    else
    {
        std::cout << "QP Solve Failed" << std::endl;
        return 0;
    }

    // if (qp_task_[ts_.heirarchy_].CheckProblemSize(variable_size, total_constraint_size))
    // {
    //     if (init_trigger)
    //     {
    //         qp_task_[ts_.heirarchy_].InitializeProblemSize(variable_size, total_constraint_size);
    //     }
    // }
    // else
    // {
    //     qp_task_[ts_.heirarchy_].InitializeProblemSize(variable_size, total_constraint_size);
    // }

    // qp_task_[ts_.heirarchy_].UpdateMinProblem(H, g);
    // qp_task_[ts_.heirarchy_].UpdateSubjectToAx(A, lbA, ubA);
    // qp_task_[ts_.heirarchy_].DeleteSubjectToX();

    // if (qp_task_[ts_.heirarchy_].SolveQPoases(1000, qpres))
    // {
    //     ts_.f_star_qp_ = qpres.segment(0, task_dof);
    //     ts_.contact_qp_ = qpres.segment(task_dof, contact_dof);
    //     ts_.qp_error = 0;

    // Check ZMP/force constraint result :

    // ts_.torque_h_ = torque_prev + Ntorque_task * (ts_.f_star_ + ts_.f_star_qp_) + NwJw * ts_.contact_qp_;
    // VectorXd contact_force = A_rot * (J_C_INV_T.rightCols(model_dof_) * ts_.torque_h_ - P_C);

    // for (int i = 0; i < cc_.size(); i++)
    // {
    //     if (cc_[i].contact)
    //     {
    //         if (abs(contact_force(i * 6 + 4) / contact_force(i * 6 + 2)) > cc_[i].contact_plane_x_ + 1.0E-6)
    //         {
    //             std::cout << " X ZMP Constraint Violated : " << i << " " << contact_force(i * 6 + 4) / contact_force(i * 6 + 2) << " " << cc_[i].contact_plane_x_ << std::endl;
    //         }
    //         if (abs(contact_force(i * 6 + 3) / contact_force(i * 6 + 2)) > cc_[i].contact_plane_y_ + 1.0E-6)
    //         {
    //             std::cout << " Y ZMP Constraint Violated : " << i << " " << contact_force(i * 6 + 3) / contact_force(i * 6 + 2) << " " << cc_[i].contact_plane_y_ << std::endl;
    //         }

    //         if (abs(contact_force(i * 6 + 0) / contact_force(i * 6 + 2)) > cc_[i].friction_ratio_ + 1.0E-6)
    //         {
    //             std::cout << " X Friction Constraint Violated : " << i << " " << contact_force(i * 6 + 0) / contact_force(i * 6 + 2) << " " << cc_[i].friction_ratio_ << std::endl;
    //         }
    //         if (abs(contact_force(i * 6 + 1) / contact_force(i * 6 + 2)) > cc_[i].friction_ratio_ + 1.0E-6)
    //         {
    //             std::cout << " Y Friction Constraint Violated : " << i << " " << contact_force(i * 6 + 1) / contact_force(i * 6 + 2) << " " << cc_[i].friction_ratio_ << std::endl;
    //         }
    //     }
    // }

    //     return 1;
    // }
    // else
    // {
    //     ts_.f_star_qp_ = VectorXd::Zero(task_dof);
    //     ts_.contact_qp_ = VectorXd::Zero(contact_dof);
    //     ts_.qp_error = 1;
    //     // qp_task_[ts_.heirarchy_].PrintMinProb();
    //     // qp_task_[ts_.heirarchy_].PrintSubjectToAx();
    //     return 0;
    // }
#endif
}

int RobotData::CalcSingleTaskTorqueWithQP2(TaskSpace &ts_, const MatrixXd &task_null_matrix_, const VectorXd &torque_prev, const MatrixXd &NwJw, const MatrixXd &J_C_INV_T, const MatrixXd &P_C, bool init_trigger)
{
    // Task Regulation with QP

    // return fstar & contact force;
    int task_dof = ts_.f_star_.size(); // size of task
    int contact_index = cc_.size();    // size of contact link
    int total_contact_dof = 0;         // total size of contact dof
    int contact_dof = -6;              // total_contact_dof - 6, (free contact space)
    int contact_constraint_size = 0;   // size of constraint by contact
    int model_size = model_dof_;       // size of joint

    int torque_limit_constraint_size = 2 * model_size;

    if (init_trigger)
    {
        if (torque_limit_set_)
        {
            if (model_size != torque_limit_.size())
            {
                cout << "Error : task qp torque limit size is not matched with model size! model dof :" << model_dof_ << "torque limit size : " << torque_limit_.size() << endl;
            }
        }

        // if(task_null_matrix_.)
    }

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
        ts_.qp_error = 0;
        return 1;
    }
    else
    {
        ts_.qp_error = 1;
        std::cout << "task solve failed" << std::endl;
        ts_.f_star_qp_ = VectorXd::Zero(task_dof);

        return 0;
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
            ts_[i].CalcNullMatrix(A_inv_N_C);
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
int RobotData::CalcContactRedistribute(bool hqp, bool init)
{
    return CalcContactRedistribute(torque_grav_ + torque_task_ + torque_contact_, hqp, init);
}

int RobotData::CalcContactRedistribute(VectorXd torque_input, bool hqp, bool init)
{
    int contact_index = cc_.size();  // size of contact link
    int total_contact_dof = 0;       // size of contact dof
    int contact_dof = -6;            // total_contact_dof - 6, (free contact space)
    int contact_constraint_size = 0; // size of constraint by contact
    int model_size = model_dof_;     // size of joints
    int torque_limit_constraint_size = 2 * model_size;
    int fz_limit_const_size = 1;

    if (init)
    {
        // Matrix validation
        if (torque_input.size() != model_size)
        {
            cout << "Contact Redistribution : torque input size is not matched with model size" << endl;
        }
    }

    if (hqp)
    {
        for (int i = 0; i < contact_index; i++)
        {
            if (cc_[i].contact)
            {
                total_contact_dof += cc_[i].contact_dof_;

                if (cc_[i].fz_limiter_switch_)
                {
                    contact_constraint_size += cc_[i].constraint_number_ + fz_limit_const_size;
                }
                else
                {
                    contact_constraint_size += cc_[i].constraint_number_;
                }
            }
        }
        contact_dof += total_contact_dof;

        if (!torque_limit_set_)
            torque_limit_constraint_size = 0;

        int variable_number = contact_dof;                                                                      // total size of qp variable
        int total_constraint_size = contact_link_num_ * contact_constraint_size + torque_limit_constraint_size; // total size of constraint

        // VectorXd contact_force = getContactForce(torque_input);

        if (contact_dof > 0)
        {
            MatrixXd H, H_temp;
            VectorXd g;

            // Eigen::MatrixXd crot_matrix = Eigen::MatrixXd::Zero(total_contact_dof, total_contact_dof);
            // Eigen::MatrixXd RotW = Eigen::MatrixXd::Identity(total_contact_dof, total_contact_dof);
            // int acc_cdof = 0;
            // for (int i = 0; i < contact_index; i++)
            // {
            //     if (cc_[i].contact)
            //     {
            //         Vector3d vec_origin, vec_target;
            //         vec_origin = cc_[i].rotm.rightCols(1);
            //         vec_target = (com_pos - cc_[i].xc_pos).normalized();
            //         Matrix3d cm = AxisTransform2V(vec_origin, vec_target);

            //         cm.setIdentity(); // comment this line to redistribute contact force with COM based vector

            //         if (cc_[i].contact_type_ == CONTACT_6D)
            //         {
            //             crot_matrix.block(acc_cdof, acc_cdof, 3, 3) = crot_matrix.block(acc_cdof + 3, acc_cdof + 3, 3, 3) = cm.transpose() * cc_[i].rotm.transpose();
            //             RotW(acc_cdof + 2, acc_cdof + 2) = 0;
            //             acc_cdof += cc_[i].contact_dof_;
            //         }
            //         else if (cc_[i].contact_type_ == CONTACT_POINT)
            //         {
            //             crot_matrix.block(acc_cdof, acc_cdof, 3, 3) = cm.transpose() * cc_[i].rotm.transpose();
            //             RotW(acc_cdof + 2, acc_cdof + 2) = 0;
            //             acc_cdof += cc_[i].contact_dof_;
            //         }
            //     }
            // }

            H.setZero(variable_number, variable_number);
            H.setIdentity();
            g.setZero(variable_number);

            // H_temp = RotW * crot_matrix * J_C_INV_T.rightCols(model_size) * NwJw;
            // H_temp = NwJw;
            // H = H_temp.transpose() * H_temp;
            // H = NwJw.transpose() * NwJw;
            // g = (RotW * crot_matrix * (J_C_INV_T.rightCols(model_size) * torque_input - P_C)).transpose() * H_temp;
            // g = NwJw.transpose() * (torque_input);

            MatrixXd A_qp;
            VectorXd lbA, ubA;
            A_qp.setZero(total_constraint_size, variable_number);
            lbA.setZero(total_constraint_size);
            ubA.setZero(total_constraint_size);

            if (torque_limit_set_)
            {
                A_qp.block(0, 0, model_size, contact_dof) = NwJw;
                ubA.segment(0, model_size) = torque_limit_ - torque_input;

                A_qp.block(model_size, 0, model_size, contact_dof) = -NwJw;
                ubA.segment(model_size, model_size) = torque_limit_ + torque_input;

                lbA.segment(0, 2 * model_size).setConstant(-INFTY);
            }

            MatrixXd A_const_a;
            A_const_a.setZero(contact_constraint_size, total_contact_dof);

            VectorXd Alim = VectorXd::Zero(contact_constraint_size);

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

                    if (cc_[i].fz_limiter_switch_)
                    {

                        A_const_a(const_idx + CONTACT_CONSTRAINT_ZMP + 6, contact_idx + 2) = -1.0;
                        Alim(const_idx + CONTACT_CONSTRAINT_ZMP + 6) = cc_[i].fz_limit_;
                        const_idx += 1;
                    }

                    const_idx += cc_[i].constraint_number_;
                    contact_idx += cc_[i].contact_dof_;
                }
            }

            Eigen::MatrixXd CM = -A_const_a * A_rot;

            // contact force = J_C_INV_T.rightCols(model_size) * (torque_input + NwJw * fw ) - P_C;

            // Eigen::MatrixXd Atemp = -CM * J_C_INV_T.rightCols(model_size);
            // Eigen::VectorXd bA = CM * P_C - CM * J_C_INV_T.rightCols(model_size) * torque_input;

            A_qp.block(torque_limit_constraint_size, 0, contact_constraint_size, contact_dof) = CM * J_C_INV_T.rightCols(model_dof_) * NwJw;

            lbA.segment(torque_limit_constraint_size, contact_constraint_size).setConstant(-INFTY);
            ubA.segment(torque_limit_constraint_size, contact_constraint_size) = CM * P_C - CM * J_C_INV_T.rightCols(model_size) * torque_input - Alim;

            const_idx = 0;
            contact_idx = 0;

            Eigen::VectorXd qpres;
            qpres.setZero(contact_dof);

#ifdef COMPILE_QPSWIFT
            qpres = qpSwiftSolve(qp_contact_, variable_number, total_constraint_size, H, g, A_, ubA, false);
            // ts_.f_star_qp_ = qpres.segment(0, task_dof);
            torque_contact_ = NwJw * qpres;
            return 1;

#else
            // if (qp_contact_.CheckProblemSize(variable_number, total_constraint_size))
            // {
            //     if (init)
            //     {
            //         qp_contact_.InitializeProblemSize(variable_number, total_constraint_size);
            //     }
            // }
            // else
            // {
            //     qp_contact_.InitializeProblemSize(variable_number, total_constraint_size);
            // }

            // qp_contact_.UpdateMinProblem(H, g);
            // qp_contact_.UpdateSubjectToAx(A_qp, lbA, ubA);
            // if (qp_contact_.SolveQPoases(300, qpres, false))
            // {
            //     cf_redis_qp_ = qpres;
            //     torque_contact_ += NwJw * qpres;

            //     return 1;
            // }
            // else
            // {
            //     std::cout << "Contact QP solve failed" << std::endl;
            //     torque_contact_.setZero(model_size);

            //     return 0;
            // }

            int solve_success = SolveOSQP(osqp_contact_, H, g, A_qp, lbA, ubA, qpres, init);

            if (solve_success)
            {
                cf_redis_qp_ = qpres;
                torque_contact_ += NwJw * qpres;

                return 1;
            }
            else
            {
                std::cout << "Contact QP solve failed" << std::endl;
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
    else
    { // IF hqp not,

        if (contact_dof_ == 12)
        {
            Vector12d ContactForce_ = J_C_INV_T.rightCols(model_dof_) * torque_input - P_C;

            Vector3d P1_ = cc_[0].xc_pos - com_pos;
            Vector3d P2_ = cc_[1].xc_pos - com_pos;

            Matrix3d Rotyaw = rotateWithZ(-link_[0].rpy(2));

            Eigen::Matrix<double, 12, 12> force_rot_yaw;
            force_rot_yaw.setZero();
            for (int i = 0; i < 4; i++)
            {
                force_rot_yaw.block(i * 3, i * 3, 3, 3) = Rotyaw;
            }

            Vector6d ResultantForce_;
            ResultantForce_.setZero();

            Vector12d ResultRedistribution_;
            ResultRedistribution_.setZero();

            Vector12d F12 = force_rot_yaw * ContactForce_;

            double eta_cust = 0.99;
            double foot_length = 0.26;
            double foot_width = 0.1;

            double eta = 0.95;

            ContactRedistributetwomod(0.99, foot_length, foot_width, 1.0, 0.9, 0.9, Rotyaw * P1_, Rotyaw * P2_, F12, ResultantForce_, ResultRedistribution_, eta);
            Vector12d fc_redist_ = force_rot_yaw.transpose() * ResultRedistribution_;

            Vector12d desired_force;
            desired_force.setZero();

            desired_force.segment(6, 6) = -ContactForce_.segment(6, 6) + fc_redist_.segment(6, 6);
            torque_contact_ = V2.transpose() * (J_C_INV_T.rightCols(model_dof_).bottomRows(6) * V2.transpose()).inverse() * desired_force.segment(6, 6);

            return 1;
        }
        else
        {
            torque_contact_.setZero();

            return 0;
        }
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

    for (int i = 0; i < link_.size() - 1; i++)
    {
        link_[i].UpdateJac(model_, q_system_);

        // Eigen::MatrixXd rotm = MatrixXd::Identity(6, 6);
        // rotm.block(0, 0, 3, 3) = link_[i].rotm;
        // rotm.block(3, 3, 3, 3) = link_[i].rotm;
        // Eigen::MatrixXd j_temp = MatrixXd::Zero(6, system_dof_);
        // j_temp.topRows(3) = link_[i].jac_.bottomRows(3);
        // j_temp.bottomRows(3) = link_[i].jac_.topRows(3);
        // H_C += (link_[i].GetSpatialTranform() * link_[i].GetSpatialInertiaMatrix() * rotm.transpose() * j_temp).topRows(3);

        Eigen::Matrix3d r_ = link_[i].rotm;
        Eigen::Matrix3d i_ = link_[i].inertia;
        Eigen::Vector3d c_ = link_[i].com_position_l_;
        Eigen::Vector3d x_ = link_[i].xpos;
        double m_ = link_[i].mass;

        H_C.topRows(3) += (r_ * (i_ + skew(c_) * skew(c_).transpose() * m_) * r_.transpose() + skew(x_) * r_ * skew(c_).transpose() * m_ * r_.transpose()) * link_[i].jac_.bottomRows(3) + (r_ * skew(c_) * m_ * r_.transpose() + m_ * skew(x_)) * link_[i].jac_.topRows(3);

        H_C.bottomRows(3) += m_ * r_ * skew(c_).transpose() * r_.transpose() * link_[i].jac_.bottomRows(3) + m_ * link_[i].jac_.topRows(3);
    }

    return H_C.topRows(3) - skew(link_.back().xpos) * H_C.bottomRows(3);
}

void RobotData::CalcAngularMomentumMatrix(MatrixXd &cmm)
{
    Eigen::MatrixXd H_C = MatrixXd::Zero(6, system_dof_);

    for (int i = 0; i < link_.size() - 1; i++)
    {
        Eigen::Matrix3d r_ = link_[i].rotm;
        Eigen::Matrix3d i_ = link_[i].inertia;
        Eigen::Vector3d c_ = link_[i].com_position_l_;
        Eigen::Vector3d x_ = link_[i].xpos;
        double m_ = link_[i].mass;

        H_C.topRows(3) += (r_ * (i_ + skew(c_) * skew(c_).transpose() * m_) * r_.transpose() + skew(x_) * r_ * skew(c_).transpose() * m_ * r_.transpose()) * link_[i].jac_.bottomRows(3) + (r_ * skew(c_) * m_ * r_.transpose() + m_ * skew(x_)) * link_[i].jac_.topRows(3);
        H_C.bottomRows(3) += m_ * r_ * skew(c_).transpose() * r_.transpose() * link_[i].jac_.bottomRows(3) + m_ * link_[i].jac_.topRows(3);
    }
    cmm = H_C;
    cmm.block(0, 0, 3, system_dof_) = H_C.topRows(3) - skew(link_.back().xpos) * H_C.bottomRows(3);
}

void RobotData::CalcVirtualCMM(RigidBodyDynamics::Model _v_model, std::vector<Link> &_link, Vector3d &_com_pos, MatrixXd &_cmm, bool verbose)
{
    Eigen::MatrixXd H_C = MatrixXd::Zero(6, _v_model.qdot_size);
    int link_size = _link.size();
    if (_link.back().name_ == "COM")
    {
        link_size -= 1;
    }

    for (int i = 0; i < link_size; i++)
    {
        Eigen::Matrix3d r_ = _link[i].rotm;
        Eigen::Matrix3d i_ = _link[i].inertia;
        Eigen::Vector3d c_ = _link[i].com_position_l_;
        Eigen::Vector3d x_ = _link[i].xpos;

        if (verbose)
        {
            std::cout << "link name : " << _link[i].name_ << " pos : " << x_.transpose() << std::endl;
        }
        double m_ = _link[i].mass;

        H_C.topRows(3) += (r_ * (i_ + skew(c_) * skew(c_).transpose() * m_) * r_.transpose() + skew(x_) * r_ * skew(c_).transpose() * m_ * r_.transpose()) * _link[i].jac_.bottomRows(3) + (r_ * skew(c_) * m_ * r_.transpose() + m_ * skew(x_)) * _link[i].jac_.topRows(3);

        H_C.bottomRows(3) += m_ * r_ * skew(c_).transpose() * r_.transpose() * _link[i].jac_.bottomRows(3) + m_ * _link[i].jac_.topRows(3);
    }
    _cmm = H_C.topRows(3) - skew(_com_pos) * H_C.bottomRows(3);
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
        int init_child_size = link_[link_idx].child_id_.size();
        int deleted_idx = 1;

        for (int i = 0; i < link_[link_idx].child_id_.size(); i++)
        {
            if (verbose)
            {
                std::cout << "delete child link : " << deleted_idx << "/" << init_child_size << " child id : " << link_[link_idx].child_id_[i] << " " << link_[link_[link_idx].child_id_[i]].name_ << std::endl;
            }
            DeleteLink(link_[link_idx].child_id_[i], verbose);
            i--;
            deleted_idx++;
        }
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
    InitAfterModelMod(DELETE_LINK, link_idx, verbose);

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

void RobotData::AddLink(const Joint &joint, const Link &link, bool verbose)
{
    if (verbose)
    {
        std::cout << "ADDLINK : " << link.name_ << " with mass : " << link.mass << std::endl;
        std::cout << "Attaching Link to " << link_[link.parent_id_].name_ << std::endl;
        std::cout << "Inertia : " << std::endl;
        std::cout << link.inertia.transpose() << std::endl;

        std::cout << "Joint type : ";
        if (joint.joint_type_ == JOINT_FIXED)
        {
            std::cout << " FIXED JOINT " << std::endl;
        }
        else if (joint.joint_type_ == JOINT_REVOLUTE)
        {
            std::cout << " REVOLUTE JOINT " << std::endl;
            std::cout << " With Axis : " << joint.joint_axis_.transpose() << std::endl;
        }
        else if (joint.joint_type_ == JOINT_FLOATING_BASE)
        {
            std::cout << " FLOATING JOINT " << std::endl;
            std::cout << " With Axis : " << joint.joint_axis_.transpose() << std::endl;
        }
        else if (joint.joint_type_ == JOINT_6DOF)
        {
            std::cout << " 6DOF JOINT " << std::endl;
        }
        std::cout << " Joint Axis position : " << joint.joint_translation_.transpose() << std::endl;
        std::cout << " Joint Axis rotation : " << std::endl
                  << joint.joint_rotation_ << std::endl;
        std::cout << " Joint Axis parent position : " << joint.parent_translation_.transpose() << std::endl;
        std::cout << " Joint Axis parent rotation : " << std::endl
                  << joint.parent_rotation_ << std::endl;
    }

    if (joint.joint_type_ == JOINT_FIXED)
    {
        AddLink(link.parent_id_, link.name_.c_str(), JOINT_FIXED, joint.joint_axis_, joint.parent_rotation_, joint.parent_translation_, link.mass, link.com_position_l_, link.inertia, verbose);
    }
    else
    {
        AddLink(link.parent_id_, link.name_.c_str(), joint.joint_type_, joint.joint_axis_, joint.joint_rotation_, joint.joint_translation_, link.mass, link.com_position_l_, link.inertia, verbose);
    }
}

void RobotData::AddLink(int parent_link_id, const char *link_name, int joint_type, const Vector3d &joint_axis, const Matrix3d &joint_rotm, const Vector3d &joint_trans, double body_mass, const Vector3d &com_position, const Matrix3d &inertia, bool verbose)
{
    int parent_body_id = link_[parent_link_id].body_id_;

    if (joint_type == JOINT_FIXED) // If joint type is fixed
    {
        RigidBodyDynamics::Math::Vector3d com_pos_rbdl = com_position;
        RigidBodyDynamics::Math::Matrix3d inertia_rbdl = inertia;
        RigidBodyDynamics::Body Body(body_mass, com_pos_rbdl, inertia_rbdl);
        RigidBodyDynamics::Joint joint(RigidBodyDynamics::JointTypeFixed);
        RigidBodyDynamics::Math::Matrix3d joint_rotm_rbdl = joint_rotm.transpose();
        RigidBodyDynamics::Math::SpatialTransform rbdl_joint_frame = RigidBodyDynamics::Math::SpatialTransform(joint_rotm_rbdl, joint_trans);

        model_.AddBody(parent_body_id, rbdl_joint_frame, joint, Body, link_name);

        InitAfterModelMod(ADD_LINK_WITH_FIXED_JOINT, parent_link_id, verbose);
    }
    else if (joint_type == JOINT_REVOLUTE) // If joint type is revolute
    {
        RigidBodyDynamics::Math::Vector3d com_pos_rbdl = com_position;
        RigidBodyDynamics::Math::Matrix3d inertia_rbdl = inertia;
        RigidBodyDynamics::Body Body(body_mass, com_pos_rbdl, inertia_rbdl);
        RigidBodyDynamics::Joint joint(RigidBodyDynamics::JointTypeRevolute, joint_axis);
        RigidBodyDynamics::Math::Matrix3d joint_rotm_rbdl = joint_rotm.transpose();
        RigidBodyDynamics::Math::SpatialTransform rbdl_joint_frame = RigidBodyDynamics::Math::SpatialTransform(joint_rotm_rbdl, joint_trans);

        model_.AddBody(parent_body_id, rbdl_joint_frame, joint, Body, link_name);

        InitAfterModelMod(ADD_LINK_WITH_REVOLUTE_JOINT, parent_link_id, verbose);
    }
    else if (joint_type == JOINT_6DOF) // if joint type is floating
    {
        RigidBodyDynamics::Math::Vector3d com_pos_rbdl = com_position;
        RigidBodyDynamics::Math::Matrix3d inertia_rbdl = inertia;

        RigidBodyDynamics::Body Body(body_mass, com_pos_rbdl, inertia_rbdl);

        RigidBodyDynamics::Joint joint = RigidBodyDynamics::Joint(
            RigidBodyDynamics::Math::SpatialVector(0., 0., 0., 1., 0., 0.),
            RigidBodyDynamics::Math::SpatialVector(0., 0., 0., 0., 1., 0.),
            RigidBodyDynamics::Math::SpatialVector(0., 0., 0., 0., 0., 1.),
            RigidBodyDynamics::Math::SpatialVector(1., 0., 0., 0., 0., 0.),
            RigidBodyDynamics::Math::SpatialVector(0., 1., 0., 0., 0., 0.),
            RigidBodyDynamics::Math::SpatialVector(0., 0., 1., 0., 0., 0.));

        RigidBodyDynamics::Math::Matrix3d joint_rotm_rbdl = joint_rotm.transpose();
        RigidBodyDynamics::Math::SpatialTransform rbdl_joint_frame = RigidBodyDynamics::Math::SpatialTransform(joint_rotm_rbdl, joint_trans);

        model_.AddBody(parent_body_id, rbdl_joint_frame, joint, Body, link_name);
        InitAfterModelMod(ADD_LINK_WITH_6DOF_JOINT, parent_link_id, verbose);
    }
    else
    {
        std::cout << "Error : Wrong joint type" << std::endl;
    }
}

void RobotData::InitAfterModelMod(int mode, int link_id, bool verbose)
{
    if (verbose)
    {
        std::cout << "Re initializing RobotData :: input link : " << link_[link_id].name_ << std::endl;
    }
    int change_link_id = link_id;
    int corresponding_body_id = link_[change_link_id].body_id_;

    system_dof_ = model_.dof_count;
    model_dof_ = system_dof_ - 6;

    // delete link for link vector

    if (mode == DELETE_LINK)
    {
        int deleted_id = change_link_id;
        int deleted_body_id = corresponding_body_id;

        Link link_deleted = link_[deleted_id];
        link_.erase(link_.begin() + deleted_id);
        joint_.erase(joint_.begin() + deleted_id);

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
            for (int j = 0; j < ts_[i].task_link_.size(); j++)
            {
                if (ts_[i].task_link_[j].link_id_ == deleted_id)
                {
                    deleted_task_id = i;
                    ts_.erase(ts_.begin() + i);
                }
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
                for (int j = 0; j < ts_[i].task_link_.size(); j++)
                {
                    if (ts_[i].task_link_[j].link_id_ > deleted_id)
                    {
                        ts_[i].task_link_[j].link_id_--;
                    }
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
    else if (mode == ADD_LINK_WITH_FIXED_JOINT)
    {
        // Link added with fixed joint.
        int parent_id = change_link_id;
        int added_parent_body_id = corresponding_body_id;

        // Change the inertial information of parent link
        link_[parent_id].com_position_l_ = model_.mBodies[added_parent_body_id].mCenterOfMass;
        link_[parent_id].mass = model_.mBodies[added_parent_body_id].mMass;
        link_[parent_id].inertia = model_.mBodies[added_parent_body_id].mInertia;
    }
    else if (mode == ADD_LINK_WITH_REVOLUTE_JOINT)
    {
        // Link added with revolute joint.

        int body_id = model_.previously_added_body_id;
        int parent_body_id = corresponding_body_id;
        int parent_link_id = change_link_id;

        // Insert Link() before the last element of link_
        link_.insert(link_.end() - 1, Link(model_, body_id));

        int added_link_id = link_.size() - 2;
        link_[added_link_id].link_id_ = added_link_id;

        // Add Children infromation to parent link
        link_[parent_link_id].child_id_.push_back(added_link_id);

        link_[added_link_id].parent_id_ = parent_link_id;

        // test
        // for (int i = 0; i < link_.size(); i++)
        // {
        //     std::cout << "link_[" << i << "].link_id_ : " << link_[i].link_id_ << "link name : " << link_[i].name_ << std::endl;
        // }

        if (verbose)
            link_[added_link_id].Print();
    }
    else if (mode == ADD_LINK_WITH_6DOF_JOINT)
    {
        int body_id = model_.previously_added_body_id;
        int parent_body_id = corresponding_body_id;
        int parent_link_id = change_link_id;

        // Insert Link() before the last element of link_
        link_.insert(link_.end() - 1, Link(model_, body_id));

        int added_link_id = link_.size() - 2;
        link_[added_link_id].link_id_ = added_link_id;

        // Add Children infromation to parent link
        link_[parent_link_id].child_id_.push_back(added_link_id);

        link_[added_link_id].parent_id_ = parent_link_id;
        // Link Added with floating joint
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
        std::cout << "Changing link to fixed joint link! id : " << link_idx << std::endl;
        // link.Print();
    }

    Joint joint = joint_[link_idx];
    joint.joint_type_ = JOINT_FIXED;

    DeleteLink(link_name, verbose);

    // Add link
    AddLink(joint, link, verbose);
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
    for (int i = 0; i < link_.size() - 1; i++)
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

void RobotData::InitModelWithLinkJoint(RigidBodyDynamics::Model &lmodel, std::vector<Link> &links, std::vector<Joint> &joints)
{
    lmodel = RigidBodyDynamics::Model();

    int parent_id = -1;
    int added_id = 0;
    RigidBodyDynamics::Math::SpatialTransform rbdl_joint_frame;

    int vlink_size = links.size();

    // create rbdl model with vlink and vjoint
    for (int i = 0; i < vlink_size; i++)
    {
        rbdl_joint_frame = RigidBodyDynamics::Math::SpatialTransform((RigidBodyDynamics::Math::Matrix3d)joints[i].joint_rotation_, joints[i].joint_translation_);

        // std::cout << i << " : rbdl joint frame : " << rbdl_joint_frame << std::endl;
        // std::cout << "lambda t : " << vlink[i].parent_rotm << std::endl;
        // std::cout << "lambda r : " << vlink[i].parent_trans.transpose() << std::endl;
        // std::cout << "rotm : " << vlink[i].rotm << std::endl;
        // std::cout << std::endl;

        if (links[i].parent_id_ < 0)
        {
            parent_id = 0;
        }
        else
        {
            parent_id = links[links[i].parent_id_].body_id_;
        }
        unsigned int null_parent = parent_id;

        if (joints[i].joint_type_ == JOINT_FLOATING_BASE)
        {
            // std::cout << "floating add " << std::endl;
            parent_id = 0;
            // std::cout << "parent id : " << parent_id << "links name : " << links[i].name_ << std::endl;
            added_id = lmodel.AddBody(parent_id, rbdl_joint_frame, joints[i].ToRBDLJoint(), links[i].ToRBDLBody(), links[i].name_);
            // std::cout << "floating add comp " << std::endl;
        }
        else
        {
            added_id = lmodel.AddBody(parent_id, rbdl_joint_frame, joints[i].ToRBDLJoint(), links[i].ToRBDLBody(), links[i].name_);
        }

        links[i].body_id_ = added_id;
    }
}
// {
//     std::cout << "vmodel.q_size != q_virtual.size()" << std::endl;
//     return;
// }

// if (vmodel.qdot_size != q_dot_virtual.size())
// {
//     std::cout << "vmodel.qdot_size != q_dot_virtual.size()" << std::endl;
//     return;
// }

// Eigen::MatrixXd A_v_;

// A_v_.setZero(vmodel.qdot_size, vmodel.qdot_size);

// RigidBodyDynamics::UpdateKinematicsCustom(vmodel, &q_virtual, &q_dot_virtual, &q_ddot_virtual);
// RigidBodyDynamics::CompositeRigidBodyAlgorithm(vmodel, q_virtual, A_v_, false);

// double total_mass = 0;

// // vmodel.mas

// if (links.back().name_ != "COM")
// {
//     std::cout << "Last link name is not COM" << std::endl;
//     links.push_back(Link());
//     links.back().name_ = "COM";
// }

// std::vector<std::future<void>> async;

// for (int i = 0; i < links.size() - 1; i++)
// {
void RobotData::CalcCOMInertia(std::vector<Link> &links, MatrixXd &com_inertia, VectorXd &com_momentum) // return inertia matrix, rotational first, translational last.
{

    // calculate total mass with link vector
    double total_mass = 0;

    int link_size = links.size();

    if (links.back().name_ == "COM")
    {
        link_size = links.size() - 1;
    }

    for (int i = 0; i < link_size; i++)
    {
        // std::cout << i << " : " << links[i].name_ << std::endl;
        total_mass += links[i].mass;
    }

    // calculate com position
    Vector3d com_pos__ = Vector3d::Zero();
    Vector3d com_vel__ = Vector3d::Zero();
    for (int i = 0; i < link_size; i++)
    {
        com_pos__ += links[i].mass * links[i].xipos / total_mass;
        com_vel__ += links[i].mass * links[i].vi / total_mass;
    }

    std::vector<MatrixXd> AdT(link_size);

    for (int i = 0; i < link_size; i++)
    {
        AdT[i].setZero(6, 6);
        AdT[i].block(0, 0, 3, 3) = links[i].rotm;
        AdT[i].block(3, 3, 3, 3) = links[i].rotm;
        AdT[i].block(3, 0, 3, 3) = skew(links[i].xpos - com_pos__) * links[i].rotm;
    }

    std::vector<MatrixXd> AdTinv(link_size);

    for (int i = 0; i < link_size; i++)
    {
        AdTinv[i].setZero(6, 6);
        AdTinv[i].block(0, 0, 3, 3) = links[i].rotm.transpose();
        AdTinv[i].block(3, 3, 3, 3) = links[i].rotm.transpose();

        AdTinv[i].block(3, 0, 3, 3) = skew(-links[i].rotm.transpose() * (links[i].xpos - com_pos__)) * links[i].rotm.transpose();

        // AdTinv[i].block(3, 0, 3, 3) = links[i].rotm.transpose() * skew(links[i].xpos - com_pos__).transpose();
    }

    // MatrixXd AdG2C;

    // AdG2C.setZero(6, 6);
    // AdG2C.setIdentity();
    // AdG2C.block(3, 0, 3, 3) = skew(-com_pos__);

    MatrixXd rotm_temp;
    rotm_temp.setZero(6, 6);

    // std::cout << AdG2C << std::endl;

    // std::vector<MatrixXd> link_inertia_matrix(link_size);
    std::vector<MatrixXd> baseframe_inertia_matrix(link_size);

    // calculate com inertia
    MatrixXd com_inertia_matrix;
    com_inertia_matrix.setZero(6, 6);

    VectorXd com_momentum_vector;
    com_momentum_vector.setZero(6);

    VectorXd link_vel_temp;
    link_vel_temp.setZero(6);
    for (int i = 0; i < link_size; i++)
    {
        // link_inertia_matrix[i].setZero(6, 6); // link inertia matrix from link base frame.
        // link_inertia_matrix[i].block(0, 0, 3, 3) = links[i].inertia + links[i].mass * skew(links[i].com_position_l_) * skew(links[i].com_position_l_).transpose();
        // link_inertia_matrix[i].block(0, 3, 3, 3) = links[i].mass * skew(links[i].com_position_l_);
        // link_inertia_matrix[i].block(3, 0, 3, 3) = links[i].mass * skew(links[i].com_position_l_).transpose();
        // link_inertia_matrix[i].block(3, 3, 3, 3) = links[i].mass * Matrix3d::Identity();

        baseframe_inertia_matrix[i] = AdTinv[i].transpose() * links[i].GetSpatialInertiaMatrix() * AdTinv[i];

        rotm_temp.block(0, 0, 3, 3) = links[i].rotm.transpose();
        rotm_temp.block(3, 3, 3, 3) = links[i].rotm.transpose();

        link_vel_temp.segment(0, 3) = links[i].w;
        link_vel_temp.segment(3, 3) = links[i].v;

        com_momentum_vector += baseframe_inertia_matrix[i] * AdT[i] * rotm_temp * link_vel_temp;

        com_inertia_matrix += baseframe_inertia_matrix[i];
    }

    // com_pos = com_pos__;
    // com_vel = com_vel__;
    // mass = total_mass;
    com_inertia = com_inertia_matrix; // Inertia matrix ->
    com_momentum = com_momentum_vector;
}

void RobotData::CalcVirtualInertia(RigidBodyDynamics::Model &vmodel, std::vector<Link> &links, std::vector<Joint> &joints, Matrix3d &new_inertia, Vector3d &new_com, double &new_mass)
{
    // With updated link infomation, assume that the every joint is fixed joint
    // Calculate the virtual inertia matrix

    // make copy of links

    std::vector<Link> links_copy = links;
    if (links_copy.back().name_ == "COM")
    {
        links_copy.pop_back();
    }
    // find end effector
    int child_number = -1;
    int link_num = 0;
    std::vector<int> end_effector_link_id;
    // print all links name, parent id, child id, mass, xpos, mass, inertia
    // std::cout << "All links info" << std::endl;
    // for (int i = 0; i < links_copy.size(); i++)
    // {
    //     std::cout << "link id : " << i << " link name : " << links_copy[i].name_ << " parent id : " << links_copy[i].parent_id_ << " child id : ";
    //     for (int j = 0; j < links_copy[i].child_id_.size(); j++)
    //     {
    //         std::cout << links_copy[i].child_id_[j] << " ";
    //     }
    //     std::cout << std::endl;
    //     std::cout << "link position : " << links_copy[i].xpos.transpose() << "link mass : " << links_copy[i].mass << std::endl
    //               << "link orientation : " << std::endl
    //               << links_copy[i].rotm << std::endl;
    //     std::cout << " link inertia : " << links_copy[i].inertia << std::endl;
    //     std::cout << std::endl;
    // }

    Link base_link;
    base_link.name_ = "base_link";

    base_link.mass = 0;
    base_link.com_position_l_.setZero();
    base_link.inertia.setZero();
    base_link.rotm.setIdentity();
    base_link.xpos.setZero();

    while (true) // repeat until the links_copy has only one link, which is the base link
    {
        if (links_copy.size() == 1)
        {
            break;
        }

        for (int i = links_copy.size() - 1; i >= 0; i--)
        {
            if (links_copy[i].child_id_.size() == 0)
            {
                // ee found!
                // std::cout << std::endl;
                // std::cout << i << "th link is end effector " << links_copy[i].name_ << " mass : " << links_copy[i].mass << std::endl;

                int parent_id_of_ee = links_copy[i].parent_id_;
                // std::cout << "Adding link " << links_copy[i].name_ << " to " << links_copy[parent_id_of_ee].name_ << std::endl;

                // find child id from parent link and delete the child id from the vector
                for (int j = 0; j < links_copy[parent_id_of_ee].child_id_.size(); j++)
                {
                    if (links_copy[parent_id_of_ee].child_id_[j] == i)
                    {
                        links_copy[parent_id_of_ee].child_id_.erase(links_copy[parent_id_of_ee].child_id_.begin() + j);
                        break;
                    }
                }

                // std::cout << "parent before : " << links_copy[parent_id_of_ee].com_position_l_.transpose() << std::endl;
                // std::cout << links_copy[parent_id_of_ee].mass << std::endl;

                links_copy[parent_id_of_ee].AddLink(links_copy[i], joints[i].parent_rotation_, joints[i].parent_translation_);

                // std::cout << "parent after : " << links_copy[parent_id_of_ee].com_position_l_.transpose() << std::endl;
                // std::cout << links_copy[parent_id_of_ee].mass << std::endl;

                links_copy.erase(links_copy.begin() + i);

                break;
            }
        }
    }

    // std::cout<<"end"<<std::endl;

    // std::cout << "" << links_copy[0].name_ << " mass : " << links_copy[0].mass << std::endl;
    // std::cout << "base pos : " << links_copy[0].xpos.transpose() << std::endl;
    // std::cout << "com pos : " << links_copy[0].com_position_l_.transpose() << std::endl;
    // std::cout << "rotm : " << std::endl
    //           << links_copy[0].rotm << std::endl;
    // std::cout << "inertia : " << std::endl
    //           << links_copy[0].inertia << std::endl;

    new_inertia = links_copy[0].inertia;
    new_com = links_copy[0].xpos + links_copy[0].rotm * links_copy[0].com_position_l_;
    new_mass = links_copy[0].mass;
}

void RobotData::ChangeLinkInertia(std::string link_name, Matrix3d &com_inertia, Vector3d &com_position, double com_mass, bool verbose)
{
    int link_id = getLinkID(link_name);

    int rbdl_id = link_[link_id].body_id_;
    if (verbose)
        std::cout << "Changing link " << link_name << " id : " << link_id << "rbdl id :" << rbdl_id << model_.GetBodyName(rbdl_id) << std::endl;

    link_[link_id].inertia = com_inertia;
    link_[link_id].com_position_l_ = com_position;
    link_[link_id].mass = com_mass;

    model_.mBodies[rbdl_id].mInertia = com_inertia;
    model_.mBodies[rbdl_id].mCenterOfMass = com_position;
    model_.mBodies[rbdl_id].mMass = com_mass;

    model_.I[rbdl_id] =
        RigidBodyDynamics::Math::SpatialRigidBodyInertia::createFromMassComInertiaC(
            model_.mBodies[rbdl_id].mMass,
            model_.mBodies[rbdl_id].mCenterOfMass,
            model_.mBodies[rbdl_id].mInertia);

    model_.Ic[rbdl_id] = model_.I[rbdl_id];

    // int link_idx = getLinkID(link_name);

    // // save link and delete link
    // Link link = link_[link_idx];
    // if (verbose)
    // {
    //     std::cout << "Delete link id : " << link_idx << std::endl;
    //     link.Print();
    // }

    // Joint joint = joint_[link_idx];
    // // joint.joint_type_ = JOINT_FIXED;

    // DeleteLink(link_name, verbose);

    // // Add link
    // AddLink(joint, link);
}

// void RobotData::

void RobotData::ReducedDynamicsCalculate(bool verbose)
{

    co_joint_idx_.clear();
    nc_joint_idx_.clear();

    co_link_idx_.clear();
    nc_link_idx_.clear();

    nc_rbdl_idx_.clear();

    co_link_idx_.push_back(0);

    for (int i = 0; i < cc_.size(); i++)
    {
        if (cc_[i].contact)
        {
            int link_idx = cc_[i].link_number_;

            while (link_idx != 0)
            {
                co_link_idx_.push_back(link_idx);
                co_joint_idx_.push_back(joint_[link_idx].joint_id_);
                link_idx = link_[link_idx].parent_id_;
            }
        }
    }
    sort(co_link_idx_.begin(), co_link_idx_.end());

    for (int i = 0; i < link_num_; i++)
    {
        if (std::find(co_link_idx_.begin(), co_link_idx_.end(), i) == co_link_idx_.end())
        {
            nc_link_idx_.push_back(i);
            nc_rbdl_idx_.push_back(link_[i].body_id_);
        }
    }

    sort(co_joint_idx_.begin(), co_joint_idx_.end());
    vc_joint_idx_.clear();
    for (int i = 0; i < 6; i++)
    {
        vc_joint_idx_.push_back(i);
    }
    vc_joint_idx_.insert(vc_joint_idx_.end(), co_joint_idx_.begin(), co_joint_idx_.end());

    for (int i = 0; i < nc_link_idx_.size(); i++)
    {
        int link_idx = nc_link_idx_[i];
        while (link_idx != 0)
        {
            if (find(nc_joint_idx_.begin(), nc_joint_idx_.end(), joint_[link_idx].joint_id_) == nc_joint_idx_.end())
            {
                nc_joint_idx_.push_back(joint_[link_idx].joint_id_);
            }
            link_idx = link_[link_idx].parent_id_;

            if (find(nc_link_idx_.begin(), nc_link_idx_.end(), link_idx) != nc_link_idx_.end())
            {
                break;
            }
        }
    }

    sort(nc_joint_idx_.begin(), nc_joint_idx_.end());

    nc_dof = nc_joint_idx_.size(); // nc_dof + co_dof + 6 = system_dof_
    co_dof = co_joint_idx_.size();
    vc_dof = co_dof + 6;

    reduced_model_dof_ = co_dof + 6;
    reduced_system_dof_ = reduced_model_dof_ + 6;

    A_NC_O.setZero(system_dof_, system_dof_);
    A_NC.setZero(nc_dof + 6, nc_dof + 6);

    for (unsigned int i = 1; i < model_.mBodies.size(); i++)
    {
        if (std::find(nc_rbdl_idx_.begin(), nc_rbdl_idx_.end(), i) != nc_rbdl_idx_.end())
        {
            model_.Ic[i] = model_.I[i];
        }
        else
        {
            model_.Ic[i] = RigidBodyDynamics::Math::SpatialRigidBodyInertia();
        }
    }

    for (unsigned int i = model_.mBodies.size() - 1; i > 0; i--)
    {
        if (std::find(nc_rbdl_idx_.begin(), nc_rbdl_idx_.end(), i) != nc_rbdl_idx_.end())
        {
            if (model_.lambda[i] != 0)
            {
                model_.Ic[model_.lambda[i]] = model_.Ic[model_.lambda[i]] + model_.X_lambda[i].applyTranspose(model_.Ic[i]);
            }

            unsigned int dof_index_i = model_.mJoints[i].q_index;

            if (model_.mJoints[i].mDoFCount == 1)
            {
                RigidBodyDynamics::Math::SpatialVector F = model_.Ic[i] * model_.S[i];
                A_NC_O(dof_index_i, dof_index_i) = model_.S[i].dot(F);

                unsigned int j = i;
                unsigned int dof_index_j = dof_index_i;

                while (model_.lambda[j] != 0)
                {
                    F = model_.X_lambda[j].applyTranspose(F);
                    j = model_.lambda[j];
                    dof_index_j = model_.mJoints[j].q_index;
                    if (model_.mJoints[j].mDoFCount == 1)
                    {
                        A_NC_O(dof_index_i, dof_index_j) = F.dot(model_.S[j]);
                        A_NC_O(dof_index_j, dof_index_i) = A_NC_O(dof_index_i, dof_index_j);
                    }
                    if (model_.mJoints[j].mDoFCount == 3)
                    {
                        Vector3d H_temp2 = (F.transpose() * model_.multdof3_S[j]).transpose();
                        A_NC_O.block<1, 3>(dof_index_i, dof_index_j) = H_temp2.transpose();
                        A_NC_O.block<3, 1>(dof_index_j, dof_index_i) = H_temp2;
                    }
                }
            }
        }
    }

    SI_nc_b_.setZero(6, 6);

    SI_nc_b_.block(0, 0, 3, 3) = model_.Ic[2].m * Matrix3d::Identity();
    SI_nc_b_.block(3, 3, 3, 3) = model_.Ic[2].toMatrix().block(0, 0, 3, 3);
    SI_nc_b_.block(3, 0, 3, 3) = model_.Ic[2].toMatrix().block(0, 3, 3, 3);
    SI_nc_b_.block(0, 3, 3, 3) = SI_nc_b_.block(3, 0, 3, 3).transpose();

    InertiaMatrixSegment(SI_nc_b_, inertia_nc_, com_pos_nc_, mass_nc_);
    SI_nc_l_ = InertiaMatrix(inertia_nc_, mass_nc_);

    A_NC.topLeftCorner(6, 6) = SI_nc_b_;

    for (int i = 0; i < nc_dof; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            A_NC(6 + i, j) = A_NC_O(nc_joint_idx_[i], j);
            A_NC(j, 6 + i) = A_NC_O(j, nc_joint_idx_[i]);
        }

        for (int j = 0; j < nc_dof; j++)
        {
            A_NC(6 + i, 6 + j) = A_NC_O(nc_joint_idx_[i], nc_joint_idx_[j]);
        }
    }

    A_NC.topRightCorner(3, nc_dof) = link_[0].rotm.transpose() * A_NC.topRightCorner(3, nc_dof);
    // A_NC.bottomLeftCorner(nc_dof, 3) = A_NC.topRightCorner(3, nc_dof).transpose();

    Matrix6d cm_rot6 = Matrix6d::Identity(6, 6);
    // Vector3d com_pos_nc_test = com_pos_nc_;
    // com_pos_nc_test(2) -= 0.1;
    cm_rot6.block(3, 0, 3, 3) = skew(com_pos_nc_).transpose(); // skew matrix of com position from pelvis
    cmm_nc_ = cm_rot6 * A_NC.topRightCorner(6, nc_dof);        // cmm calc, "Improved Computation of the Humanoid Centroidal Dynamics and Application for Whole-Body Control"
    // CMM matrix is from local frame (robot frame)
    J_I_nc_.setZero(6, nc_dof);
    J_I_nc_ = SI_nc_l_.inverse() * cmm_nc_;

    J_R.setZero(reduced_system_dof_, system_dof_);

    J_R.topLeftCorner(6, 6).setIdentity();
    for (int i = 0; i < vc_joint_idx_.size(); i++)
    {
        J_R(i, vc_joint_idx_[i]) = 1;
    }

    for (int i = 0; i < nc_joint_idx_.size(); i++)
    {
        J_R.bottomRows(6).col(nc_joint_idx_[i]) = J_I_nc_.col(i);
    }
    // J_R.bottomRightCorner(6, nc_dof) = J_I_nc_;

    /*
    Simplified Calculation of
    A_R_inv = J_R * A_inv_ * J_R.transpose();
    */

    A_R_inv.setZero(reduced_system_dof_, reduced_system_dof_);
    for (int i = 0; i < vc_dof; i++)
    {
        for (int j = 0; j < vc_dof; j++)
        {
            A_R_inv(i, j) = A_inv_(vc_joint_idx_[i], vc_joint_idx_[j]);
        }
    }
    MatrixXd A_tr_temp = MatrixXd::Zero(vc_dof, system_dof_);
    for (int i = 0; i < vc_dof; i++)
    {
        for (int j = 0; j < system_dof_; j++)
        {
            A_tr_temp(i, j) = A_inv_(vc_joint_idx_[i], j);
        }
    }

    A_R_inv.topRightCorner(vc_dof, 6) = A_tr_temp.rightCols(nc_dof) * J_I_nc_.transpose();
    A_R_inv.bottomLeftCorner(6, vc_dof) = A_R_inv.topRightCorner(vc_dof, 6).transpose();
    A_R_inv.bottomRightCorner(6, 6) = J_R.bottomRows(6) * A_inv_ * J_R.bottomRows(6).transpose();

    A_R = A_R_inv.llt().solve(Eigen::MatrixXd::Identity(reduced_system_dof_, reduced_system_dof_)); // Faster than inverse()

    // A_R.setZero(reduced_system_dof_, reduced_system_dof_);
    // A_R.block(0, 0, vc_dof, vc_dof) = A_.block(0, 0, vc_dof, vc_dof);
    // A_R.block(vc_dof, vc_dof, 6, 6) = (J_I_nc_ * A_NC.block(6, 6, nc_dof, nc_dof).inverse() * J_I_nc_.transpose()).inverse();
    // A_R.block(0, vc_dof, 3, 3) = link_[0].rotm * mass_nc_;
    // A_R.block(0, vc_dof + 3, 3, 3) = skew(com_pos_nc_) * mass_nc_;
    // A_R.block(3, vc_dof + 3, 3, 3) = inertia_nc_;
    // A_R.bottomLeftCorner(6,6) = A_R.topRightCorner(6,6).transpose();

    J_I_nc_inv_T.setZero(6, nc_dof);

    // J_I_nc_inv_T =
    J_I_nc_inv_T = A_R.block(vc_dof, 0, 6, vc_dof) * A_inv_.block(0, vc_dof, vc_dof, nc_dof) + A_R.block(vc_dof, vc_dof, 6, 6) * J_I_nc_ * A_inv_.block(vc_dof, vc_dof, nc_dof, nc_dof);

    // J_I_nc_inv_T = A_NC

    N_I_nc_ = MatrixXd::Identity(nc_dof, nc_dof) - J_I_nc_.transpose() * J_I_nc_inv_T;

    // J_R_INV_T = A_R * J_R * A_inv_;
    J_R_INV_T = MatrixXd::Zero(reduced_system_dof_, system_dof_);
    J_R_INV_T.block(0, 0, vc_dof, vc_dof).setIdentity();
    J_R_INV_T.block(vc_dof, vc_dof, 6, nc_dof) = J_I_nc_inv_T;

    G_R.setZero(reduced_system_dof_);
    G_R.segment(0, vc_dof) = G_.segment(0, vc_dof);
    G_R.segment(vc_dof, 6) = J_I_nc_inv_T * G_.segment(vc_dof, nc_dof);

    J_CR = MatrixXd::Zero(contact_dof_, reduced_system_dof_);
    J_CR.block(0, 0, contact_dof_, vc_dof) = J_C.block(0, 0, contact_dof_, vc_dof);

    G_NC = G_.tail(nc_dof);

    if (verbose)
    {
        Matrix6d A_mat_rot = Matrix6d::Identity(6, 6);
        A_mat_rot.block(0, 0, 3, 3) = link_[0].rotm.transpose();

        Matrix6d A_mat_rot2 = Matrix6d::Identity(6, 6);
        A_mat_rot2.block(3, 3, 3, 3) = link_[0].rotm;

        // std::cout << "I_NC from pelvis frame : " << std::endl;
        // std::cout << IM_NC << std::endl;

        // std::cout << "IM_NC + IM_C from pelvis frame : " << std::endl;
        // std::cout << IM_NC + IM_C << std::endl;

        // std::cout << "Mass matrix of Non-contact model from pelvis frame : " << std::endl;
        // std::cout << A_NC << std::endl;

        // std::cout << "Centroidal Momentum matrix from pelvis frame : " << std::endl;
        // std::cout << cmm_nc_ << std::endl;

        Matrix6d imo;
        imo.setZero();
        imo.block(0, 0, 3, 3) = total_mass_ * Matrix3d::Identity();
        imo.block(3, 3, 3, 3) = link_.back().inertia;

        Matrix6d tr1, tr2;

        tr1.setIdentity();
        tr2.setIdentity();

        Vector3d local_com = link_[0].rotm.transpose() * (link_.back().xpos - link_[0].xpos);

        Matrix3d rt_p = link_[0].rotm.transpose();

        tr1.block(0, 0, 3, 3) = rt_p;
        tr1.block(3, 3, 3, 3) = rt_p;

        tr2.block(0, 0, 3, 3) = rt_p;
        tr2.block(3, 3, 3, 3) = rt_p;

        tr1.block(0, 3, 3, 3) = skew(com_pos_co_ - local_com).transpose() * rt_p;
        tr2.block(0, 3, 3, 3) = skew(com_pos_nc_ - local_com).transpose() * rt_p;

        Matrix6d IXGT = Matrix6d::Identity(6, 6);
        IXGT.block(0, 0, 3, 3) = link_[0].rotm;
        IXGT.block(3, 3, 3, 3) = link_[0].rotm;

        IXGT.block(3, 0, 3, 3) = link_[0].rotm * skew(local_com).transpose();

        // std::cout << "Original floating mass Matrix from pelvis frame.  : " << std::endl;
        // std::cout << A_mat_rot * A_.block(0, 0, 6, 6) * A_mat_rot.transpose() << std::endl;

        // std::cout << "transform mass mat from pelv frame to global frame  : " << std::endl;
        // std::cout << IXGT * A_mat_rot * A_.block(0, 0, 6, 6) * A_mat_rot.transpose() * IXGT.transpose() << std::endl;

        // std::cout << "original mass mat from global frame.  : " << std::endl;
        // std::cout << A_mat_rot2 * A_.block(0, 0, 6, 6) * A_mat_rot2.transpose() << std::endl;

        std::cout << "TR 1 : " << std::endl;
        std::cout << tr1 << std::endl;

        std::cout << "TR 2 : " << std::endl;
        std::cout << tr2 << std::endl;

        std::cout << "IM 1 : " << std::endl;
        std::cout << tr1.transpose() * SI_co_l_ * tr1 << std::endl;

        std::cout << "IM 2 : " << std::endl;
        std::cout << tr2.transpose() * SI_nc_l_ * tr2 << std::endl;

        std::cout << "IM 1 local : " << std::endl;
        std::cout << SI_co_l_ << std::endl;

        std::cout << "IM 2 local : " << std::endl;
        std::cout << SI_nc_l_ << std::endl;

        std::cout << " Reconstruct A matrix from Origin COM, global rotation" << std::endl;
        std::cout << tr1.transpose() * SI_co_l_ * tr1 + tr2.transpose() * SI_nc_l_ * tr2 << std::endl;

        std::cout << "Inertial matrix with origin info. /global rotation. : " << std::endl;
        std::cout << imo << std::endl;

        std::cout << "Spatial Momentum : " << std::endl;
    }
}

int RobotData::ReducedCalcContactConstraint()
{
    const int cdof = contact_dof_;
    Lambda_CR.setZero(contact_dof_, contact_dof_);
    J_CR.setZero(contact_dof_, reduced_system_dof_);
    J_CR_INV_T.setZero(contact_dof_, reduced_system_dof_);
    N_CR.setZero(reduced_system_dof_, reduced_system_dof_);

    W_R.setZero(reduced_model_dof_, reduced_model_dof_);
    W_R_inv.setZero(reduced_model_dof_, reduced_model_dof_);

    V2_R.setZero(contact_dof_ - 6, reduced_model_dof_);
    NwJw_R.setZero(reduced_model_dof_, reduced_model_dof_);

    P_CR.setZero(contact_dof_, reduced_system_dof_);

    // Original Contact Constraint Calculation.
    // Lambda_contact = (J_C * A_inv_ * J_C.transpose()).inverse();
    // J_CR = J_C.block(0, 0, contact_dof_, reduced_system_dof_);

    J_CR = MatrixXd::Zero(contact_dof_, reduced_system_dof_);
    J_CR.block(0, 0, contact_dof_, vc_dof) = J_C.block(0, 0, contact_dof_, vc_dof);

    int rows = J_CR.rows(); // rows == contact_dof
    int cols = J_CR.cols(); // cols == system_dof

    Lambda_CR = (J_CR.leftCols(cdof + 6) * A_R_inv.topLeftCorner(cdof + 6, cdof + 6) * J_CR.leftCols(cdof + 6).transpose()).inverse();
    J_C_INV_T = (Lambda_CR * J_C) * A_inv_;

    // J_CR_INV_T = J_C_INV_T * J_R.transpose();
    J_CR_INV_T.block(0, 0, rows, vc_dof) = J_C_INV_T.block(0, 0, rows, vc_dof);
    J_CR_INV_T.block(0, vc_dof, rows, 6) = J_C_INV_T.block(0, vc_dof, rows, nc_dof) * J_I_nc_.transpose();

    N_C = MatrixXd::Identity(system_dof_, system_dof_) - J_C.transpose() * J_C_INV_T;
    N_CR = MatrixXd::Identity(cols, cols) - J_CR.transpose() * J_CR_INV_T;

    A_R_inv_N_CR = A_R_inv * N_CR;
    W_R = A_R_inv_N_CR.block(6, 6, cols - 6, cols - 6);
    // A_inv.bottomRows(cols - 6) * N_C.rightCols(cols - 6);
    int result;
    if (rows > 6)
    {
        PinvCODWB(W_R, W_R_inv, V2_R, cols - (rows));

        if (rows - 6 == V2_R.rows())
        {
            NwJw_R = V2_R.transpose() * (J_CR_INV_T.rightCols(cols - 6).topRows(contact_dof_ - 6) * V2_R.transpose()).inverse();
            result = 1;
        }
        else
        {
            NwJw_R = V2_R.transpose() * (J_CR_INV_T.rightCols(cols - 6).topRows(contact_dof_ - 6) * V2_R.transpose()).inverse();
            std::cout << "Contact Space Factorization Error : Required contact null dimension : " << J_CR.rows() - 6 << " factorization rank : " << V2_R.rows() << std::endl;
            result = 0;
        }
    }
    else
    {
        PinvCODWB(W_R, W_R_inv);
        result = 1;
    }

    // int result = CalculateContactConstraint(J_CR, A_R_inv, Lambda_CR, J_CR_INV_T, N_CR, A_R_inv_N_CR, W_R, NwJw_R, W_R_inv, V2_R);

    return result;
}

void RobotData::ReducedCalcGravCompensation()
{
    torque_grav_R_ = W_R_inv * (A_R_inv.bottomRows(reduced_model_dof_) * (N_CR * G_R));
    torque_grav_.segment(0, reduced_model_dof_) = torque_grav_R_;
    torque_grav_.segment(co_dof, nc_dof) = G_.segment(vc_dof, nc_dof);
    P_CR = J_CR_INV_T * G_R;
}

void RobotData::ReducedCalcTaskSpace(bool update_task_space)
{
    A_inv_N_C = A_inv_ * N_C;

    if (update_task_space)
        UpdateTaskSpace();

    J_base_R_ = link_[0].jac_.leftCols(reduced_system_dof_);
    CalculateJKT_R(J_base_R_, A_R_inv_N_CR, W_R_inv, J_base_R_kt_, lambda_base_R_);

    // Calc BaseLink Jkt
    //  J_base_R_

    int nc_heirarchy_ = 0;

    // task type classification :
    for (int i = 0; i < ts_.size(); i++)
    {
        if (ts_[i].task_type_ == TASK_CUSTOM)
        {
            ts_[i].noncont_task = false;
            ts_[i].reduced_task_ = false;
            ts_[i].cmm_task = false;

            if (ts_[i].J_task_.leftCols(vc_dof).norm() > 1.0E-4) // if contact task is not empty
            {
                ts_[i].reduced_task_ = true;
            }
            if (ts_[i].J_task_.rightCols(nc_dof).norm() > 1.0E-4) // if non-contact task is not empty
            {
                ts_[i].noncont_task = true;
                ts_[i].nc_heirarchy_ = nc_heirarchy_++;
            }
        }
        else
        {
            for (int j = 0; j < ts_[i].task_link_.size(); j++)
            {
                if (ts_[i].task_link_[j].link_id_ == link_num_)
                {
                    ts_[i].noncont_task = ts_[i].noncont_task || false;
                    ts_[i].reduced_task_ = ts_[i].reduced_task_ || false;
                    ts_[i].cmm_task = ts_[i].cmm_task || true;
                }
                else
                {
                    if (find(co_link_idx_.begin(), co_link_idx_.end(), ts_[i].task_link_[j].link_id_) != co_link_idx_.end())
                    {
                        ts_[i].noncont_task = ts_[i].noncont_task || false;
                        ts_[i].reduced_task_ = ts_[i].reduced_task_ || true;
                        ts_[i].cmm_task = ts_[i].cmm_task || false;
                    }
                    else
                    {
                        ts_[i].noncont_task = ts_[i].noncont_task || true;
                        ts_[i].reduced_task_ = ts_[i].reduced_task_ || false;
                        ts_[i].cmm_task = ts_[i].cmm_task || false;
                        ts_[i].nc_heirarchy_ = nc_heirarchy_++;
                    }

                    // if (contact_dependency_link_(ts_[i].task_link_[j].link_id_))
                    // {
                    //     ts_[i].noncont_task = ts_[i].noncont_task || false;
                    //     ts_[i].reduced_task_ = ts_[i].reduced_task_ || true;
                    //     ts_[i].cmm_task = ts_[i].cmm_task || false;
                    // }
                    // else
                    // {
                    //     ts_[i].noncont_task = ts_[i].noncont_task || true;
                    //     ts_[i].reduced_task_ = ts_[i].reduced_task_ || false;
                    //     ts_[i].cmm_task = ts_[i].cmm_task || false;
                    //     ts_[i].nc_heirarchy_ = nc_heirarchy_++;
                    // }
                }
            }
        }
    }

    for (int i = 0; i < ts_.size(); i++)
    {
        ts_[i].CalcJKT_R(A_inv_, A_inv_N_C, A_R_inv_N_CR, W_R_inv, J_I_nc_inv_T);

        if (i != (ts_.size() - 1)) // if task is not last
        {
            if (!ts_[i].noncont_task)
            {
                if (i == 0)
                {
                    ts_[i].CalcNullMatrix_R(A_R_inv_N_CR);
                }
                else
                {
                    ts_[i].CalcNullMatrix_R(A_R_inv_N_CR, ts_[i - 1].Null_task_R_);
                }
            }
            else
            {
                ts_[i].Null_task_R_ = ts_[i - 1].Null_task_R_;
            }
        }
    }
}

int RobotData::ReducedCalcTaskControlTorque(bool hqp, bool init, bool calc_task_space)
{
    if (calc_task_space)
    {
        ReducedCalcTaskSpace();
    }

    torque_task_R_.setZero(reduced_model_dof_);
    torque_task_.setZero(model_dof_);
    torque_task_NC_.setZero(nc_dof);

    int qp_res = 0;

    int first_heir = true;

    bool non_contact_calc_required = false;

    int non_con_task = 0;
    std::vector<int> non_con_task_idx;
    int prev_nc_idx = 0;
    VectorXd temp_torque_ = VectorXd::Zero(system_dof_);
    MatrixXd non_con_null = MatrixXd::Identity(nc_dof, nc_dof);
    VectorXd torque_task_R_qp = VectorXd::Zero(reduced_model_dof_);
    first_nc_TaskSpace_idx_ = -1;
    force_on_nc_r_.setZero();
    for (int i = 0; i < ts_.size(); i++)
    {

        ts_[i].torque_nc_.setZero(nc_dof);
        ts_[i].torque_h_.setZero(model_dof_);
        ts_[i].torque_nc_.setZero(nc_dof);
        ts_[i].torque_h_R_.setZero(reduced_model_dof_);
        ts_[i].torque_null_h_.setZero(model_dof_);
        ts_[i].torque_null_h_nc_.setZero(nc_dof);
        ts_[i].torque_null_h_R_.setZero(reduced_model_dof_);
        if (hqp)
        {
            if (ts_[i].noncont_task)
            {

                temp_torque_ = ts_[i].J_task_.transpose() * (ts_[i].Lambda_task_ * ts_[i].f_star_);
                ts_[i].torque_nc_ = temp_torque_.segment(vc_dof, nc_dof);
                ts_[i].force_on_nc_.segment(0, 3) = temp_torque_.segment(0, 3);
                ts_[i].force_on_nc_.segment(3, 3) = link_[0].rotm * temp_torque_.segment(3, 3);

                force_on_nc_r_ += ts_[i].force_on_nc_;
                ts_[i].torque_h_R_.segment(0, co_dof) = J_base_R_kt_.topRows(co_dof) * ts_[i].force_on_nc_;
                // ts_[i].torque_nc_ = (ts_[i].J_task_NC_.transpose() * (ts_[i].Lambda_task_ * ts_[i].f_star_));
                ts_[i].torque_h_R_.segment(co_dof, 6) = J_I_nc_inv_T * ts_[i].torque_nc_;

                if (ts_[i].nc_heirarchy_ == 0)
                {
                    first_nc_TaskSpace_idx_ = i;
                    ts_[i].torque_null_h_R_ = ts_[i - 1].Null_task_R_ * ts_[i].torque_h_R_;
                    ts_[i].torque_null_h_nc_ = ts_[i].torque_nc_;
                }
                else
                {
                    VectorXd null_force_ = ts_[i - 1].Lambda_task_ * (ts_[i - 1].J_task_ * (A_inv_N_C * (ts_[i].J_task_.transpose() * (ts_[i].Lambda_task_ * ts_[i].f_star_))));
                    VectorXd null_torque_h_r = VectorXd::Zero(reduced_model_dof_);

                    temp_torque_ = ts_[i - 1].J_task_.transpose() * null_force_;
                    ts_[i].torque_null_h_nc_ = ts_[i].torque_nc_ - temp_torque_.segment(vc_dof, nc_dof);
                    temp_torque_.segment(3, 3) = link_[0].rotm * temp_torque_.segment(3, 3);

                    null_torque_h_r.segment(0, co_dof) = ts_[i].torque_h_R_.segment(0, co_dof) - J_base_R_kt_.topRows(co_dof) * temp_torque_.segment(0, 6);
                    null_torque_h_r.segment(co_dof, 6) = J_I_nc_inv_T * (ts_[i].torque_nc_ - ts_[i - 1].J_task_NC_.transpose() * null_force_);

                    ts_[i].torque_null_h_R_ = ts_[i - 1].Null_task_R_ * null_torque_h_r;
                    force_on_nc_r_ -= temp_torque_.segment(0, 6);

                    // VectorXd null_force_ = ts_[i - 1].Lambda_task_ * (ts_[i - 1].J_task_ * (A_inv_N_C * (ts_[i].J_task_.transpose() * (ts_[i].Lambda_task_ * ts_[i].f_star_))));
                    // VectorXd null_torque_h_r = VectorXd::Zero(reduced_model_dof_);
                    // ts_[i].torque_null_h_nc_ = ts_[i].torque_nc_ - ts_[i - 1].J_task_NC_.transpose() * null_force_;
                    // null_torque_h_r.segment(0, co_dof) = ts_[i].torque_h_R_.segment(0, co_dof) - J_nc_R_kt_.topRows(co_dof) * ts_[i - 1].wr_mat * null_force_;
                    // null_torque_h_r.segment(co_dof, 6) = J_I_nc_inv_T * (ts_[i].torque_nc_ - ts_[i - 1].J_task_NC_.transpose() * null_force_);

                    // ts_[i].torque_null_h_R_ = ts_[i - 1].Null_task_R_ * null_torque_h_r;
                    // force_on_nc_r_ -= ts_[i - 1].wr_mat * null_force_;
                }
            }
            else
            {
                if (i == 0)
                {
                    qp_res = CalcSingleTaskTorqueWithQP_R(ts_[i], MatrixXd::Identity(reduced_model_dof_, reduced_model_dof_), torque_grav_R_, NwJw_R, J_CR_INV_T, P_CR, init);
                    if (qp_res == 0)
                    {
                        std::cout << "R-HQP solve error at " << i << "th task" << std::endl;
                        return 0;
                    }
                    ts_[i].torque_h_R_ = ts_[i].J_kt_R_ * ts_[i].Lambda_task_ * (ts_[i].f_star_ + ts_[i].f_star_qp_);
                    ts_[i].torque_null_h_R_ = ts_[i].torque_h_R_;
                    // first_heir = false;
                }
                else
                {
                    qp_res = CalcSingleTaskTorqueWithQP_R(ts_[i], ts_[i - 1].Null_task_R_, torque_grav_R_ + torque_task_R_, NwJw_R, J_CR_INV_T, P_CR, init);
                    if (qp_res == 0)
                    {
                        std::cout << "R-HQP solve error at " << i << "th task" << std::endl;
                        return 0;
                    }
                    ts_[i].torque_h_R_ = ts_[i].J_kt_R_ * ts_[i].Lambda_task_ * (ts_[i].f_star_ + ts_[i].f_star_qp_);
                    ts_[i].torque_null_h_R_ = ts_[i - 1].Null_task_R_ * ts_[i].torque_h_R_;
                }

                torque_task_R_ += ts_[i].torque_null_h_R_;
                torque_task_NC_ += ts_[i].torque_null_h_nc_;
            }
        }
        else
        {
            if (ts_[i].noncont_task)
            {
                temp_torque_ = ts_[i].J_task_.transpose() * (ts_[i].Lambda_task_ * ts_[i].f_star_);
                ts_[i].torque_nc_ = temp_torque_.segment(vc_dof, nc_dof);

                ts_[i].force_on_nc_.segment(0, 3) = temp_torque_.segment(0, 3);
                ts_[i].force_on_nc_.segment(3, 3) = link_[0].rotm * temp_torque_.segment(3, 3);

                ts_[i].torque_h_R_.segment(0, co_dof) = J_base_R_kt_.topRows(co_dof) * ts_[i].force_on_nc_;
                ts_[i].torque_h_R_.segment(co_dof, 6) = J_I_nc_inv_T * ts_[i].torque_nc_;

                if (ts_[i].nc_heirarchy_ == 0)
                {
                    first_nc_TaskSpace_idx_ = i;

                    ts_[i].torque_null_h_R_ = ts_[i - 1].Null_task_R_ * ts_[i].torque_h_R_;
                    ts_[i].torque_null_h_nc_ = ts_[i].torque_nc_;
                }
                else
                {
                    VectorXd null_force_ = ts_[i - 1].Lambda_task_ * (ts_[i - 1].J_task_ * (A_inv_N_C * (ts_[i].J_task_.transpose() * (ts_[i].Lambda_task_ * ts_[i].f_star_))));
                    VectorXd null_torque_h_r = VectorXd::Zero(reduced_model_dof_);

                    temp_torque_ = ts_[i - 1].J_task_.transpose() * null_force_;
                    ts_[i].torque_null_h_nc_ = ts_[i].torque_nc_ - temp_torque_.segment(vc_dof, nc_dof);
                    temp_torque_.segment(3, 3) = link_[0].rotm * temp_torque_.segment(3, 3);

                    null_torque_h_r.segment(0, co_dof) = ts_[i].torque_h_R_.segment(0, co_dof) - J_base_R_kt_.topRows(co_dof) * temp_torque_.segment(0, 6);
                    null_torque_h_r.segment(co_dof, 6) = J_I_nc_inv_T * (ts_[i].torque_nc_ - ts_[i - 1].J_task_NC_.transpose() * null_force_);

                    ts_[i].torque_null_h_R_ = ts_[i - 1].Null_task_R_ * null_torque_h_r;
                }
            }
            else
            {
                ts_[i].torque_h_R_ = ts_[i].J_kt_R_ * ts_[i].Lambda_task_ * ts_[i].f_star_;

                if (i == 0)
                {
                    ts_[i].torque_null_h_R_ = ts_[i].torque_h_R_;
                }
                else
                {
                    ts_[i].torque_null_h_R_ = ts_[i - 1].Null_task_R_ * ts_[i].torque_h_R_;
                }
            }
            torque_task_R_ += ts_[i].torque_null_h_R_;
            torque_task_NC_ += ts_[i].torque_null_h_nc_;
        }
    }

    if (hqp && (first_nc_TaskSpace_idx_ >= 0))
    {

        int qp_res = CalcSingleTaskTorqueWithQP_R_NC(ts_[first_nc_TaskSpace_idx_ - 1].Null_task_R_, torque_grav_R_ + torque_task_R_, NwJw_R, J_CR_INV_T, P_CR, init);

        if (qp_res == 0)
        {
            std::cout << "R-HQP solve error at non-contact task" << std::endl;
            return 0;
        }

        for (int i = first_nc_TaskSpace_idx_; i < ts_.size(); i++)
        {
            if (ts_[i].noncont_task)
            {
                torque_task_R_ += ts_[i].torque_null_h_R_;
                torque_task_NC_ += ts_[i].torque_null_h_nc_;
            }
        }

        torque_task_R_qp = J_base_R_kt_.topRows(co_dof) * force_on_nc_R_qp_;
    }

    torque_task_.segment(0, co_dof) = torque_task_R_.segment(0, co_dof) + torque_task_R_qp.segment(0, co_dof);
    torque_task_.segment(co_dof, nc_dof) = J_I_nc_.transpose() * torque_task_R_.segment(co_dof, 6) + N_I_nc_ * torque_task_NC_;

    return 1;
}

int RobotData::CalcSingleTaskTorqueWithQP_R(TaskSpace &ts_, const MatrixXd &task_null_matrix_, const VectorXd &torque_prev, const MatrixXd &NwJw, const MatrixXd &J_C_INV_T, const MatrixXd &P_C, bool init_trigger)
{
    // return fstar & contact force;
    int task_dof = ts_.f_star_.size();   // size of task
    int contact_index = cc_.size();      // size of contact link
    int total_contact_dof = 0;           // total size of contact dof
    int contact_dof = -6;                // total_contact_dof - 6, (free contact space)
    int contact_constraint_size = 0;     // size of constraint by contact
    int model_size = reduced_model_dof_; // size of joint

    int torque_limit_constraint_size = 2 * model_size;

    if (init_trigger)
    {
        if (torque_limit_set_)
        {
            if (model_size != torque_limit_.size())
            {
                cout << "Error : task qp torque limit size is not matched with model size! model dof :" << model_dof_ << "torque limit size : " << torque_limit_.size() << endl;
            }
        }

        // if(task_null_matrix_.)
    }

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
    Eigen::MatrixXd Ntorque_task = task_null_matrix_ * (ts_.J_kt_R_ * ts_.Lambda_task_);

    if (torque_limit_set_)
    {
        A.block(0, 0, model_size, task_dof) = Ntorque_task;
        if (contact_dof > 0)
            A.block(0, task_dof, model_size, contact_dof) = NwJw_R;
        // lbA.segment(0, model_size) = -torque_limit_ - torque_prev - Ntorque_task * ts_.f_star_;
        ubA.segment(0, model_size) = torque_limit_ - (torque_prev + Ntorque_task * ts_.f_star_);

        A.block(model_size, 0, model_size, task_dof) = -Ntorque_task;
        if (contact_dof > 0)
            A.block(model_size, task_dof, model_size, contact_dof) = -NwJw_R;
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

    Eigen::MatrixXd Atemp = A_const_a * A_rot * J_CR_INV_T.rightCols(model_size);
    // t[3] = std::chrono::steady_clock::now();
    A.block(torque_limit_constraint_size, 0, contact_constraint_size, task_dof) = -Atemp * Ntorque_task;
    if (contact_dof > 0)
        A.block(torque_limit_constraint_size, task_dof, contact_constraint_size, contact_dof) = -Atemp * NwJw_R;
    // t[4] = std::chrono::steady_clock::now();

    Eigen::VectorXd bA = A_const_a * (A_rot * P_CR) - Atemp * (torque_prev + Ntorque_task * ts_.f_star_);
    // Eigen::VectorXd ubA_contact;
    lbA.segment(torque_limit_constraint_size, contact_constraint_size).setConstant(-INFTY);

    // lbA.segment(total_constraint_size) = -ubA_contact;
    ubA.segment(torque_limit_constraint_size, contact_constraint_size) = -bA;

    // qp_.EnableEqualityCondition(0.0001);

    VectorXd qpres;
    int solve_success = SolveOSQP(osqp_task_[ts_.heirarchy_], H, g, A, lbA, ubA, qpres, init_trigger);

    if (solve_success)
    {
        ts_.f_star_qp_ = qpres.segment(0, task_dof);
        ts_.contact_qp_ = qpres.segment(task_dof, contact_dof);
        ts_.qp_error = 0;
        return 1;
    }
    else
    {
        std::cout << "task solve failed" << std::endl;
        ts_.f_star_qp_ = VectorXd::Zero(task_dof);
        ts_.qp_error = 1;
        return 0;
    }
    // if (qp_task_[ts_.heirarchy_].CheckProblemSize(variable_size, total_constraint_size))
    // {
    //     if (init_trigger)
    //     {
    //         qp_task_[ts_.heirarchy_].InitializeProblemSize(variable_size, total_constraint_size);
    //     }
    // }
    // else
    // {
    //     qp_task_[ts_.heirarchy_].InitializeProblemSize(variable_size, total_constraint_size);
    // }

    // qp_task_[ts_.heirarchy_].UpdateMinProblem(H, g);
    // qp_task_[ts_.heirarchy_].UpdateSubjectToAx(A, lbA, ubA);
    // qp_task_[ts_.heirarchy_].DeleteSubjectToX();

    // if (qp_task_[ts_.heirarchy_].SolveQPoases(300, qpres))
    // {
    //     ts_.f_star_qp_ = qpres.segment(0, task_dof);
    //     ts_.contact_qp_ = qpres.segment(task_dof, contact_dof);
    //     ts_.qp_error = 0;
    //     return 1;
    // }
    // else
    // {
    //     std::cout << "task solve failed" << std::endl;
    //     ts_.f_star_qp_ = VectorXd::Zero(task_dof);
    //     ts_.qp_error = 1;

    //     // qp_task_[ts_.heirarchy_].PrintMinProb();

    //     // qp_task_[ts_.heirarchy_].PrintSubjectToAx();
    //     return 0;
    // }
}

int RobotData::CalcSingleTaskTorqueWithQP_R_NC(const MatrixXd &task_null_matrix_, const VectorXd &torque_prev, const MatrixXd &NwJw, const MatrixXd &J_C_INV_T, const MatrixXd &P_C, bool init_trigger)
{

    Vector6d nc_control_force_r;

    nc_control_force_r = force_on_nc_r_;

    // return fstar & contact force;
    int task_dof = 6;                    // size of task
    int contact_index = cc_.size();      // size of contact link
    int total_contact_dof = 0;           // total size of contact dof
    int contact_dof = -6;                // total_contact_dof - 6, (free contact space)
    int contact_constraint_size = 0;     // size of constraint by contact
    int model_size = reduced_model_dof_; // size of joint

    int torque_limit_constraint_size = 2 * model_size;

    if (init_trigger)
    {
        if (torque_limit_set_)
        {
            if (model_size != torque_limit_.size())
            {
                cout << "Error : task qp torque limit size is not matched with model size! model dof :" << model_dof_ << "torque limit size : " << torque_limit_.size() << endl;
            }
        }

        // if(task_null_matrix_.)
    }

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
    Eigen::MatrixXd Ntorque_task = task_null_matrix_ * (J_base_R_kt_);

    if (torque_limit_set_)
    {
        A.block(0, 0, model_size, task_dof) = Ntorque_task;
        if (contact_dof > 0)
            A.block(0, task_dof, model_size, contact_dof) = NwJw_R;
        // lbA.segment(0, model_size) = -torque_limit_ - torque_prev - Ntorque_task * ts_.f_star_;
        ubA.segment(0, model_size) = torque_limit_ - (torque_prev + Ntorque_task * nc_control_force_r);

        A.block(model_size, 0, model_size, task_dof) = -Ntorque_task;
        if (contact_dof > 0)
            A.block(model_size, task_dof, model_size, contact_dof) = -NwJw_R;
        // lbA.segment(model_size, model_size) = -torque_limit_ - torque_prev - Ntorque_task * ts_.f_star_;
        ubA.segment(model_size, model_size) = torque_limit_ + torque_prev + Ntorque_task * nc_control_force_r;

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

    Eigen::MatrixXd Atemp = A_const_a * A_rot * J_CR_INV_T.rightCols(model_size);
    // t[3] = std::chrono::steady_clock::now();
    A.block(torque_limit_constraint_size, 0, contact_constraint_size, task_dof) = -Atemp * Ntorque_task;
    if (contact_dof > 0)
        A.block(torque_limit_constraint_size, task_dof, contact_constraint_size, contact_dof) = -Atemp * NwJw_R;
    // t[4] = std::chrono::steady_clock::now();

    Eigen::VectorXd bA = A_const_a * (A_rot * P_CR) - Atemp * (torque_prev + Ntorque_task * nc_control_force_r);
    // Eigen::VectorXd ubA_contact;
    lbA.segment(torque_limit_constraint_size, contact_constraint_size).setConstant(-INFTY);

    // lbA.segment(total_constraint_size) = -ubA_contact;
    ubA.segment(torque_limit_constraint_size, contact_constraint_size) = -bA;

    // qp_.EnableEqualityCondition(0.0001);

    VectorXd qpres;
    int solve_success = SolveOSQP(osqp_task_[first_nc_TaskSpace_idx_], H, g, A, lbA, ubA, qpres, init_trigger);
    if (solve_success)
    {
        force_on_nc_R_qp_ = qpres.segment(0, task_dof);

        nc_qp_contact_ = qpres.segment(task_dof, contact_dof);
        return 1;
    }
    else
    {
        std::cout << "task solve failed" << std::endl;
        force_on_nc_R_qp_ = VectorXd::Zero(task_dof);
        return 0;
    }

    // if (qp_task_[first_nc_TaskSpace_idx_].CheckProblemSize(variable_size, total_constraint_size))
    // {
    //     if (init_trigger)
    //     {
    //         qp_task_[first_nc_TaskSpace_idx_].InitializeProblemSize(variable_size, total_constraint_size);
    //     }
    // }
    // else
    // {
    //     qp_task_[first_nc_TaskSpace_idx_].InitializeProblemSize(variable_size, total_constraint_size);
    // }

    // qp_task_[first_nc_TaskSpace_idx_].UpdateMinProblem(H, g);
    // qp_task_[first_nc_TaskSpace_idx_].UpdateSubjectToAx(A, lbA, ubA);
    // qp_task_[first_nc_TaskSpace_idx_].DeleteSubjectToX();

    // if (qp_task_[first_nc_TaskSpace_idx_].SolveQPoases(300, qpres))
    // {
    //     force_on_nc_R_qp_ = qpres.segment(0, task_dof);

    //     nc_qp_contact_ = qpres.segment(task_dof, contact_dof);
    //     return 1;
    // }
    // else
    // {
    //     std::cout << "task solve failed" << std::endl;
    //     force_on_nc_R_qp_ = VectorXd::Zero(task_dof);

    //     // qp_task_[ts_.heirarchy_].PrintMinProb();

    //     // qp_task_[ts_.heirarchy_].PrintSubjectToAx();
    //     return 0;
    // }
}

int RobotData::ReducedCalcContactRedistribute(bool hqp, bool init)
{
    int ret = 0;
    if (contact_dof_ > 6)
    {
        ret = CalcContactRedistributeR(torque_grav_R_ + torque_task_R_, hqp, init);

        torque_contact_.setZero(model_dof_);
        torque_contact_.segment(0, contact_dof_) = torque_contact_R_.segment(0, contact_dof_);
    }

    return ret;
}

int RobotData::CalcSingleTaskTorqueWithJACC_QP(TaskSpace &lts, bool init)
{
    int _task_dof = lts.task_dof_;
    int _acc_dof = system_dof_;
    int _contact_dof = contact_dof_;
    int _torque_dof = model_dof_;

    // optimization variable : system dof + active joint dof + contact dof + task dof
    int variable_size = _acc_dof + _torque_dof + _contact_dof + _task_dof;
    int act_idx_ = _acc_dof;
    int contact_idx_ = act_idx_ + _torque_dof;
    int task_idx_ = contact_idx_ + _contact_dof;

    int current_hierarchy = lts.heirarchy_;

    int contact_constraint_size = 0;
    for (int i = 0; i < cc_.size(); i++)
    {
        if (cc_[i].contact)
        {
            contact_constraint_size += cc_[i].constraint_number_;
        }
    }

    int prev_task_constraint_size_ = 0;
    for (int i = 0; i < lts.heirarchy_; i++)
    {
        prev_task_constraint_size_ += ts_[i].task_dof_;
    }

    int equality_constraint_size = _acc_dof + _contact_dof + prev_task_constraint_size_ + _task_dof;

    // variable for variable optimization problem
    MatrixXd H;
    VectorXd g;
    H.setZero(variable_size, variable_size);
    g.setZero(variable_size);
    H.block(0, 0, _acc_dof, _acc_dof) = A_;
    H.block(task_idx_, task_idx_, _task_dof, _task_dof) = 100 * MatrixXd::Identity(_task_dof, _task_dof);

    VectorXd acc_lim = VectorXd::Constant(_acc_dof, 10.0);
    // torque limit
    if (torque_limit_set_)
    {
    }

    // constraint matrix
    MatrixXd A;
    MatrixXd Aeq, Aieq;
    VectorXd ubAeq, lbAeq, ubAieq, lbAieq;
    // equality constraint : rigid body dynamics, contact constraint, task constraint

    Aeq.setZero(equality_constraint_size, variable_size);
    lbAeq.setZero(equality_constraint_size);
    ubAeq.setZero(equality_constraint_size);

    MatrixXd ST_ = MatrixXd::Zero(_acc_dof, _torque_dof);
    ST_.block(6, 0, _torque_dof, _torque_dof).setIdentity();

    Aeq.block(0, 0, _acc_dof, _acc_dof) = A_;
    Aeq.block(0, act_idx_, _acc_dof, _torque_dof) = -ST_;
    Aeq.block(0, contact_idx_, _acc_dof, _contact_dof) = J_C.transpose();
    lbAeq.segment(0, _acc_dof) = ubAeq.segment(0, _acc_dof) = -G_;

    Aeq.block(_acc_dof, 0, _contact_dof, _acc_dof) = J_C;

    int prev_task_idx = _acc_dof + _contact_dof;
    // handle previous task constraint.
    for (int i = 0; i < lts.heirarchy_; i++)
    {
        int p_task_dof = ts_[i].task_dof_;
        Aeq.block(prev_task_idx, 0, p_task_dof, _acc_dof) = ts_[i].J_task_;
        lbAeq.segment(prev_task_idx, p_task_dof) = ubAeq.segment(prev_task_idx, p_task_dof) = ts_[i].f_star_ + ts_[i].f_star_qp_;

        prev_task_idx += p_task_dof;
    }

    // handle current task constraint

    Aeq.block(prev_task_idx, 0, _task_dof, _acc_dof) = lts.J_task_;
    Aeq.block(prev_task_idx, task_idx_, _task_dof, _task_dof) = -MatrixXd::Identity(_task_dof, _task_dof);
    lbAeq.segment(prev_task_idx, _task_dof) = ubAeq.segment(prev_task_idx, _task_dof) = lts.f_star_;

    // inequality constraint : contact force limit

    Eigen::MatrixXd A_const_a;
    A_const_a.setZero(contact_constraint_size, _contact_dof);

    Eigen::MatrixXd A_rot;
    A_rot.setZero(_contact_dof, _contact_dof);

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

    Aieq.setZero(contact_constraint_size, variable_size);
    Aieq.block(0, contact_idx_, contact_constraint_size, _contact_dof) = getContactConstraintMatrix();
    ubAieq.setZero(contact_constraint_size);
    lbAieq.setZero(contact_constraint_size);
    lbAieq.setConstant(-INFTY);

    A.setZero(equality_constraint_size + contact_constraint_size, variable_size);
    A.block(0, 0, equality_constraint_size, variable_size) = Aeq;
    A.block(equality_constraint_size, 0, contact_constraint_size, variable_size) = Aieq;

    VectorXd lbA, ubA;
    lbA.setZero(equality_constraint_size + contact_constraint_size);
    ubA.setZero(equality_constraint_size + contact_constraint_size);
    lbA.segment(0, equality_constraint_size) = ubA.segment(0, equality_constraint_size) = lbAeq;
    lbA.segment(equality_constraint_size, contact_constraint_size) = lbAieq;
    ubA.segment(equality_constraint_size, contact_constraint_size) = ubAieq;

    int total_constraint_size = equality_constraint_size + contact_constraint_size;

    VectorXd lb, ub;

    lb.setConstant(variable_size, -INFTY);
    ub.setConstant(variable_size, INFTY);

    // lb.segment(0,6).setCons
    lb.segment(6, _torque_dof).setConstant(-10);
    ub.segment(6, _torque_dof).setConstant(10);
    lb.segment(_acc_dof, _torque_dof).setConstant(-200);
    ub.segment(_acc_dof, _torque_dof).setConstant(200);

    VectorXd qpres;

    qpres.setZero(variable_size);

    if (qp_task_[lts.heirarchy_].CheckProblemSize(variable_size, total_constraint_size))
    {
        if (init)
        {
            qp_task_[lts.heirarchy_].InitializeProblemSize(variable_size, total_constraint_size);
        }
    }
    else
    {
        qp_task_[lts.heirarchy_].InitializeProblemSize(variable_size, total_constraint_size);
    }

    qp_task_[lts.heirarchy_].UpdateMinProblem(H, g);
    qp_task_[lts.heirarchy_].UpdateSubjectToAx(A, lbA, ubA);
    qp_task_[lts.heirarchy_].UpdateSubjectToX(lb, ub);
    // qp_task_[lts.heirarchy_].DeleteSubjectToX();
    qp_task_[lts.heirarchy_].EnableEqualityCondition(0.0001);
    if (!qp_task_[lts.heirarchy_].SolveQPoases(300, qpres))
        return 0;
    lts.f_star_qp_.setZero(_task_dof);
    lts.acc_qp_.setZero(_acc_dof);
    lts.torque_qp_.setZero(_torque_dof);
    lts.contact_qp_.setZero(_contact_dof);
    // lts.torque_h_ = qpres.segment(_acc_dof, _torque_dof);
    lts.f_star_qp_ = qpres.segment(task_idx_, _task_dof);
    lts.acc_qp_ = qpres.segment(0, _acc_dof);
    lts.torque_qp_ = qpres.segment(_acc_dof, _torque_dof);
    lts.contact_qp_ = qpres.segment(contact_idx_, _contact_dof);

    return 1;
}

int RobotData::CalcSingleTaskTorqueWithJACC_QP_R(TaskSpace &lts, bool init)
{
    int _task_dof = lts.task_dof_;
    int _acc_dof = reduced_system_dof_;
    int _contact_dof = contact_dof_;
    int _torque_dof = reduced_model_dof_;

    // optimization variable : system dof + active joint dof + contact dof + task dof
    int variable_size = _acc_dof + _torque_dof + _contact_dof + _task_dof;
    int act_idx_ = _acc_dof;
    int contact_idx_ = act_idx_ + _torque_dof;
    int task_idx_ = contact_idx_ + _contact_dof;

    int current_hierarchy = lts.heirarchy_;

    int contact_constraint_size = 0;
    for (int i = 0; i < cc_.size(); i++)
    {
        if (cc_[i].contact)
        {
            contact_constraint_size += cc_[i].constraint_number_;
        }
    }

    int prev_task_constraint_size_ = 0;
    for (int i = 0; i < lts.heirarchy_; i++)
    {
        prev_task_constraint_size_ += ts_[i].task_dof_;
    }

    int equality_constraint_size = _acc_dof + _contact_dof + prev_task_constraint_size_ + _task_dof;

    // variable for variable optimization problem
    MatrixXd H;
    VectorXd g;
    H.setZero(variable_size, variable_size);
    g.setZero(variable_size);
    H.block(0, 0, _acc_dof, _acc_dof) = A_R;
    H.block(task_idx_, task_idx_, _task_dof, _task_dof) = 100 * MatrixXd::Identity(_task_dof, _task_dof);

    VectorXd acc_lim = VectorXd::Constant(_acc_dof, 10.0);
    // torque limit
    if (torque_limit_set_)
    {
    }

    // constraint matrix
    MatrixXd A;
    MatrixXd Aeq, Aieq;
    VectorXd ubAeq, lbAeq, ubAieq, lbAieq;
    // equality constraint : rigid body dynamics, contact constraint, task constraint

    Aeq.setZero(equality_constraint_size, variable_size);
    lbAeq.setZero(equality_constraint_size);
    ubAeq.setZero(equality_constraint_size);

    MatrixXd ST_ = MatrixXd::Zero(_acc_dof, _torque_dof);
    ST_.block(6, 0, _torque_dof, _torque_dof).setIdentity();

    Aeq.block(0, 0, _acc_dof, _acc_dof) = A_R;
    Aeq.block(0, act_idx_, _acc_dof, _torque_dof) = -ST_;
    Aeq.block(0, contact_idx_, _acc_dof, _contact_dof) = J_CR.transpose();
    lbAeq.segment(0, _acc_dof) = ubAeq.segment(0, _acc_dof) = -G_R;

    Aeq.block(_acc_dof, 0, _contact_dof, _acc_dof) = J_CR;

    int prev_task_idx = _acc_dof + _contact_dof;
    // handle previous task constraint.
    for (int i = 0; i < lts.heirarchy_; i++)
    {
        int p_task_dof = ts_[i].task_dof_;
        Aeq.block(prev_task_idx, 0, p_task_dof, _acc_dof) = ts_[i].J_task_ * J_R_INV_T.transpose();
        lbAeq.segment(prev_task_idx, p_task_dof) = ubAeq.segment(prev_task_idx, p_task_dof) = ts_[i].f_star_ + ts_[i].f_star_qp_;

        prev_task_idx += p_task_dof;
    }

    // handle current task constraint

    Aeq.block(prev_task_idx, 0, _task_dof, _acc_dof) = lts.J_task_ * J_R_INV_T.transpose();
    Aeq.block(prev_task_idx, task_idx_, _task_dof, _task_dof) = -MatrixXd::Identity(_task_dof, _task_dof);
    lbAeq.segment(prev_task_idx, _task_dof) = ubAeq.segment(prev_task_idx, _task_dof) = lts.f_star_;

    // inequality constraint : contact force limit

    Eigen::MatrixXd A_const_a;
    A_const_a.setZero(contact_constraint_size, _contact_dof);

    Eigen::MatrixXd A_rot;
    A_rot.setZero(_contact_dof, _contact_dof);

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

    Aieq.setZero(contact_constraint_size, variable_size);
    Aieq.block(0, contact_idx_, contact_constraint_size, _contact_dof) = -A_const_a * A_rot;
    ubAieq.setZero(contact_constraint_size);
    lbAieq.setZero(contact_constraint_size);
    lbAieq.setConstant(-INFTY);

    A.setZero(equality_constraint_size + contact_constraint_size, variable_size);
    A.block(0, 0, equality_constraint_size, variable_size) = Aeq;
    A.block(equality_constraint_size, 0, contact_constraint_size, variable_size) = Aieq;

    VectorXd lbA, ubA;
    lbA.setZero(equality_constraint_size + contact_constraint_size);
    ubA.setZero(equality_constraint_size + contact_constraint_size);
    lbA.segment(0, equality_constraint_size) = ubA.segment(0, equality_constraint_size) = lbAeq;
    lbA.segment(equality_constraint_size, contact_constraint_size) = lbAieq;
    ubA.segment(equality_constraint_size, contact_constraint_size) = ubAieq;

    int total_constraint_size = equality_constraint_size + contact_constraint_size;

    VectorXd lb, ub;

    lb.setConstant(variable_size, -INFTY);
    ub.setConstant(variable_size, INFTY);

    // lb.segment(0,6).setCons
    lb.segment(6, _torque_dof).setConstant(-10);
    ub.segment(6, _torque_dof).setConstant(10);
    lb.segment(_acc_dof, _torque_dof - 6).setConstant(-200);
    ub.segment(_acc_dof, _torque_dof - 6).setConstant(200);

    VectorXd qpres;

    qpres.setZero(variable_size);

    if (qp_task_[lts.heirarchy_].CheckProblemSize(variable_size, total_constraint_size))
    {
        if (init)
        {
            qp_task_[lts.heirarchy_].InitializeProblemSize(variable_size, total_constraint_size);
        }
    }
    else
    {
        qp_task_[lts.heirarchy_].InitializeProblemSize(variable_size, total_constraint_size);
    }

    qp_task_[lts.heirarchy_].UpdateMinProblem(H, g);
    qp_task_[lts.heirarchy_].UpdateSubjectToAx(A, lbA, ubA);
    qp_task_[lts.heirarchy_].UpdateSubjectToX(lb, ub);
    // qp_task_[lts.heirarchy_].DeleteSubjectToX();
    qp_task_[lts.heirarchy_].EnableEqualityCondition(0.0001);
    if (!qp_task_[lts.heirarchy_].SolveQPoases(300, qpres))
        return 0;
    // lts.torque_h_R_ = qpres.segment(_acc_dof, _torque_dof);
    // lts.f_star_qp_ = qpres.segment(task_idx_, _task_dof);
    // lts.contact_qp_ = qpres.segment(contact_idx_, _contact_dof);

    lts.f_star_qp_.setZero(_task_dof);
    lts.acc_qp_.setZero(_acc_dof);
    lts.torque_qp_.setZero(_torque_dof);
    lts.contact_qp_.setZero(_contact_dof);

    lts.f_star_qp_ = qpres.segment(task_idx_, _task_dof);
    lts.acc_qp_ = qpres.segment(0, _acc_dof);
    lts.torque_qp_ = qpres.segment(_acc_dof, _torque_dof);
    lts.contact_qp_ = qpres.segment(contact_idx_, _contact_dof);

    return 1;
}

int RobotData::CalcSingleTaskTorqueWithJACC_QP_R_NC(TaskSpace &lts, VectorXd &prev_acc, bool init)
{
    // int task_dof = lts.task_dof_;
    // int task_dof = 0;
    int _acc_dof = nc_dof;
    int _torque_dof = nc_dof;
    int _gacc_dof = 6;
    int _task_dof = lts.task_dof_;

    // optimization variable : nc jacc + nc c torque + com_acc_dof + task dof
    int variable_size = _acc_dof + _torque_dof + _gacc_dof + _task_dof;

    int acc_idx = 0;
    int torque_idx = _acc_dof;
    int gacc_idx_ = torque_idx + _torque_dof;
    int task_idx_ = gacc_idx_ + _gacc_dof;

    int current_hierarchy = lts.heirarchy_;
    if (prev_acc.size() != reduced_system_dof_)
    {
        cout << "Error : prev_acc size is not matched with nc_dof" << endl;
        return 0;
    }

    Vector6d prev_hier_gacc = prev_acc.tail(6);
    Vector6d prev_hier_pelv_acc = prev_acc.head(6);

    MatrixXd Ja = MatrixXd::Identity(6, 6);
    Ja.block(0, 3, 3, 3) = skew(link_[lts.task_link_[0].link_id_].xpos - link_[0].xpos);

    Vector6d fstar_local = Ja * (lts.f_star_ - prev_hier_pelv_acc);

    // int prev_task_constraint_size_ = 0;
    // for (int i = 0; i < lts.heirarchy_; i++)
    // {
    //     prev_task_constraint_size_ += ts_[i].task_dof_;
    // }

    int equality_constraint_size = _acc_dof + _gacc_dof + _task_dof;

    // variable for variable optimization problem
    MatrixXd H;
    VectorXd g;
    H.setZero(variable_size, variable_size);
    g.setZero(variable_size);
    // H.block(task_idx_, task_idx_, task_dof, task_dof).setIdentity();
    H.block(gacc_idx_, gacc_idx_, 6, 6).setIdentity();                                                  // min W_G;
    H.block(task_idx_, task_idx_, _task_dof, _task_dof) = 5 * MatrixXd::Identity(_task_dof, _task_dof); // min W_task;

    // constraint matrix
    MatrixXd A;
    MatrixXd Aeq, Aieq;
    VectorXd ubAeq, lbAeq, ubAieq, lbAieq;
    // equality constraint : rigid body dynamics, contact constraint, task constraint

    Aeq.setZero(equality_constraint_size, variable_size);
    lbAeq.setZero(equality_constraint_size);
    ubAeq.setZero(equality_constraint_size);

    // Equality Constraint 1 : Equation of Motion
    Aeq.block(0, 0, _acc_dof, _acc_dof) = A_NC.bottomRightCorner(nc_dof, nc_dof);
    Aeq.block(0, torque_idx, _torque_dof, _torque_dof) = -MatrixXd::Identity(nc_dof, nc_dof);
    lbAeq.segment(0, nc_dof) = ubAeq.segment(0, nc_dof) = -G_.segment(vc_dof, nc_dof);

    // Equality Constraint 2 : COM acceleration
    Aeq.block(_acc_dof, 0, 6, nc_dof) = J_I_nc_;
    Aeq.block(_acc_dof, gacc_idx_, 6, 6) = -MatrixXd::Identity(6, 6);
    lbAeq.segment(_acc_dof, 6) = ubAeq.segment(_acc_dof, 6) = prev_hier_gacc;

    // Equality Constraint 3 : Task Constraint
    Aeq.block(_acc_dof + _gacc_dof, 0, _task_dof, nc_dof) = lts.J_task_.rightCols(nc_dof);
    Aeq.block(_acc_dof + _gacc_dof, task_idx_, _task_dof, _task_dof) = -MatrixXd::Identity(_task_dof, _task_dof);
    lbAeq.segment(_acc_dof + _gacc_dof, _task_dof) = ubAeq.segment(_acc_dof + _gacc_dof, _task_dof) = fstar_local;

    // int prev_task_idx = reduced_system_dof_ + contact_dof_;
    // // handle previous task constraint.
    // for (int i = 0; i < lts.heirarchy_; i++)
    // {
    //     int p_task_dof = ts_[i].task_dof_;
    //     Aeq.block(prev_task_idx, 0, p_task_dof, reduced_system_dof_) = ts_[i].J_task_ * J_R_INV_T.transpose();
    //     lbAeq.segment(prev_task_idx, p_task_dof) = ubAeq.segment(prev_task_idx, p_task_dof) = ts_[i].f_star_ + ts_[i].f_star_qp_;

    //     prev_task_idx += p_task_dof;
    // }

    // handle current task constraint

    // Aeq.block(prev_task_idx, 0, task_dof, reduced_system_dof_) = lts.J_task_ * J_R_INV_T.transpose();
    // Aeq.block(prev_task_idx, task_idx_, task_dof, task_dof) = -MatrixXd::Identity(task_dof, task_dof);
    // lbAeq.segment(prev_task_idx, task_dof) = ubAeq.segment(prev_task_idx, task_dof) = lts.f_star_;

    // inequality constraint : contact force limit

    // Eigen::MatrixXd A_const_a;
    // A_const_a.setZero(contact_constraint_size, contact_dof_);

    // Eigen::MatrixXd A_rot;
    // A_rot.setZero(contact_dof_, contact_dof_);

    // int const_idx = 0;
    // int contact_idx = 0;
    // for (int i = 0; i < cc_.size(); i++)
    // {
    //     if (cc_[i].contact)
    //     {
    //         A_rot.block(contact_idx, contact_idx, 3, 3) = cc_[i].rotm.transpose();
    //         A_rot.block(contact_idx + 3, contact_idx + 3, 3, 3) = cc_[i].rotm.transpose();

    //         A_const_a.block(const_idx, contact_idx, 4, 6) = cc_[i].GetZMPConstMatrix4x6();
    //         A_const_a.block(const_idx + CONTACT_CONSTRAINT_ZMP, contact_idx, 6, 6) = cc_[i].GetForceConstMatrix6x6();

    //         const_idx += cc_[i].constraint_number_;
    //         contact_idx += cc_[i].contact_dof_;
    //     }
    // }

    // Aieq.setZero(contact_constraint_size, variable_size);
    // Aieq.block(0, contact_idx_, contact_constraint_size, contact_dof_) = -A_const_a * A_rot;
    // ubAieq.setZero(contact_constraint_size);
    // lbAieq.setZero(contact_constraint_size);
    // lbAieq.setConstant(-INFTY);

    A.setZero(equality_constraint_size, variable_size);
    A.block(0, 0, equality_constraint_size, variable_size) = Aeq;
    // A.block(equality_constraint_size, 0, 0, variable_size) = Aieq;

    VectorXd lbA, ubA;
    lbA.setZero(equality_constraint_size + 0);
    ubA.setZero(equality_constraint_size + 0);
    lbA.segment(0, equality_constraint_size) = ubA.segment(0, equality_constraint_size) = lbAeq;

    VectorXd lb, ub;

    lb.setConstant(variable_size, -INFTY);
    ub.setConstant(variable_size, INFTY);

    lb.segment(0, nc_dof).setConstant(-10.0);
    ub.segment(0, nc_dof).setConstant(10.0);

    lb.segment(nc_dof, nc_dof).setConstant(-100.0);
    ub.segment(nc_dof, nc_dof).setConstant(100.0);

    int total_constraint_size = equality_constraint_size;

    VectorXd qpres;

    qpres.setZero(variable_size);

    if (qp_task_[lts.heirarchy_].CheckProblemSize(variable_size, total_constraint_size))
    {
        if (init)
        {
            qp_task_[lts.heirarchy_].InitializeProblemSize(variable_size, total_constraint_size);
        }
    }
    else
    {
        qp_task_[lts.heirarchy_].InitializeProblemSize(variable_size, total_constraint_size);
    }

    qp_task_[lts.heirarchy_].UpdateMinProblem(H, g);
    qp_task_[lts.heirarchy_].UpdateSubjectToAx(A, lbA, ubA);
    // qp_task_[lts.heirarchy_].UpdateSubjectToX(lb, ub);
    qp_task_[lts.heirarchy_].DeleteSubjectToX();

    qp_task_[lts.heirarchy_].EnableEqualityCondition(0.0001);
    int solve_res = qp_task_[lts.heirarchy_].SolveQPoases(300, qpres);

    if (solve_res == 0)
        return 0;

    lts.acc_qp_ = qpres.segment(0, nc_dof);
    lts.torque_qp_ = qpres.segment(nc_dof, nc_dof);
    lts.gacc_qp_ = qpres.segment(gacc_idx_, 6);
    lts.f_star_qp_ = qpres.segment(task_idx_, _task_dof);
    // lts.f_sar_qp_ = qpres.segment(task_idx_, task_dof);

    return 1;
}

int RobotData::ConfigureLQP(HQP &hqp, bool init)
{
    /**
     * hqp settings
     * priority 1 //
     *      inequality constraint : torque limit (2(n-6) dof)
     *
     *      equality constraint   : wbd (n dof)
     * Priority 2 //
     *      inequality constraint :
     *                              joint acceleration limit (2n dof)
     *                              CoP & ZMP limit (8 * c dof)
     *      equality constraint :
     *                              contact constraint (6 * c dof)
     *
     * priority 3 //
     *      equality constraint   : com space (3 dof)
     *                              pelv rotation (3 dof)
     *
     * priority 4 //
     *      equality constraint   : right arm (6 dof)
     */

    int ret = 0;

    int acc_size = system_dof_;
    int torque_size = model_dof_;
    int contact_size = contact_dof_;
    int variable_size = acc_size + contact_size;

    int acc_idx = 0;
    int contact_idx = acc_size + 0;

    if (init)
    {
        hqp.initialize(acc_size, 0, contact_size);
    }

    MatrixXd cost_h = MatrixXd::Zero(acc_size + contact_size, acc_size + contact_size);
    VectorXd cost_g = VectorXd::Zero(acc_size + contact_size);

    cost_h.block(0, 0, acc_size, acc_size) = A_ / A_.topLeftCorner(6, 6).norm() * hqp.jacc_cost;

    // Priority 1
    int ineq_constraint_size = torque_size * 2; // torque limit
    int eq_constraint_size = 6;                 // newton euler

    if (init)
    {
        hqp.addHierarchy(ineq_constraint_size, eq_constraint_size);
    }

    // hqp_.hqp_hs_[0].y_ans_.head(variable_size).setZero();

    MatrixXd A = MatrixXd::Zero(ineq_constraint_size, variable_size);
    VectorXd a = VectorXd::Zero(ineq_constraint_size);

    MatrixXd B = MatrixXd::Zero(eq_constraint_size, variable_size);
    VectorXd b = VectorXd::Zero(eq_constraint_size);

    B.block(0, 0, eq_constraint_size, acc_size) = A_.topRows(eq_constraint_size);
    B.block(0, acc_size, eq_constraint_size, contact_size) = J_C.transpose().topRows(eq_constraint_size);
    b.head(eq_constraint_size) = B_.head(eq_constraint_size);

    VectorXd tlim = VectorXd::Constant(torque_size, hqp.tlim_def);

    A.block(0, 0, torque_size, acc_size) = A_.bottomRows(torque_size);
    A.block(0, acc_size, torque_size, contact_size) = J_C.transpose().bottomRows(torque_size);
    A.block(torque_size, 0, torque_size, acc_size) = -A_.bottomRows(torque_size);
    A.block(torque_size, acc_size, torque_size, contact_size) = -J_C.transpose().bottomRows(torque_size);

    // A.block(0, acc_size, torque_size, torque_size) = MatrixXd::Identity(torque_size, torque_size);            // torque limit
    // A.block(torque_size, acc_size, torque_size, torque_size) = -MatrixXd::Identity(torque_size, torque_size); // torque limit
    a.head(torque_size) = -tlim + B_.tail(torque_size);
    a.tail(torque_size) = -tlim - B_.tail(torque_size);

    // std::cout << "test0 " << hqp_.hqp_hs_[0].ineq_const_size_ << std::endl;
    // std::cout << "A :rows : " << A.rows() << " cols : " << A.cols() << std::endl;

    hqp.hqp_hs_[0].updateConstraintMatrix(A, a, B, b);
    hqp.hqp_hs_[0].normalizeConstraintMatrix();
    // Solve
    hqp.hqp_hs_[0].v_ans_.setZero(ineq_constraint_size);
    hqp.hqp_hs_[0].w_ans_.setZero(eq_constraint_size);
    hqp.hqp_hs_[0].y_ans_.head(acc_size) = -A_inv_ * B_;

    // Priority 2
    int contact_constraint_size = contact_link_num_ * 10;
    ineq_constraint_size = contact_constraint_size + torque_size * 2;
    eq_constraint_size = contact_size;
    if (init)
        hqp.addHierarchy(ineq_constraint_size, eq_constraint_size);

    A.setZero(ineq_constraint_size, variable_size);
    a.setZero(ineq_constraint_size);
    A.block(0, acc_size, contact_constraint_size, contact_size) = getContactConstraintMatrix();
    A.block(contact_constraint_size, 6, torque_size, torque_size) = MatrixXd::Identity(torque_size, torque_size);
    A.block(contact_constraint_size + torque_size, 6, torque_size, torque_size) = -MatrixXd::Identity(torque_size, torque_size);

    VectorXd alim = VectorXd::Constant(torque_size, hqp.alim_def);
    a.segment(contact_constraint_size, torque_size) = -alim;
    a.segment(contact_constraint_size + torque_size, torque_size) = -alim;

    B.setZero(eq_constraint_size, variable_size);
    b.setZero(eq_constraint_size);
    B.block(0, 0, contact_size, acc_size) = J_C;

    // std::cout << "test1" << std::endl;
    hqp.hqp_hs_[1].updateConstraintMatrix(A, a, B, b);
    hqp.hqp_hs_[1].updateCostMatrix(cost_h, cost_g);
    hqp.hqp_hs_[1].normalizeConstraintMatrix();

    for (int i = 0; i < ts_.size(); i++)
    {
        ineq_constraint_size = 0;
        eq_constraint_size = ts_[i].task_dof_;
        if (init)
            hqp.addHierarchy(ineq_constraint_size, eq_constraint_size);

        B.setZero(eq_constraint_size, variable_size);
        b.setZero(eq_constraint_size);

        B.block(0, 0, eq_constraint_size, acc_size) = ts_[i].J_task_;
        b.head(eq_constraint_size) = -ts_[i].f_star_;

        hqp.hqp_hs_[2 + i].updateConstraintMatrix(A, a, B, b);
        hqp.hqp_hs_[2 + i].updateCostMatrix(cost_h, cost_g);
        hqp.hqp_hs_[2 + i].normalizeConstraintMatrix();
    }

    hqp.prepare();

    return ret;
}

int RobotData::CalcControlTorqueLQP(HQP &hqp, bool init)
{
    int ret = 0;
    hqp.solveSequential(init);
    double solve_time = 0;
    double update_time = 0;
    for (int i = 0; i < hqp.hqp_hs_.size(); i++)
    {
        if (!init)
        {
            solve_time += hqp.hqp_hs_[i].qp_solve_time_step_;
            update_time += hqp.hqp_hs_[i].qp_update_time_step_;
        }
    }
    hqp.update_time_step_ = update_time;
    hqp.solve_time_step_ = solve_time;
    hqp.total_time_step_ = update_time + solve_time;

    hqp.update_time_max_ = std::max(hqp.update_time_max_, update_time);
    hqp.solve_time_max_ = std::max(hqp.solve_time_max_, solve_time);
    hqp.total_time_max_ = std::max(hqp.total_time_max_, hqp.total_time_step_);

    hqp.jacc = hqp.hqp_hs_.back().y_ans_.head(system_dof_);
    hqp.contact_force = hqp.hqp_hs_.back().y_ans_.tail(contact_dof_);
    return ret;
}
int RobotData::CalcControlTorqueLQP_R(HQP &hqp_r, bool init)
{
    int ret = 0;
    hqp_r.solveSequential(init);
    double solve_time = 0;
    double update_time = 0;
    for (int i = 0; i < hqp_r.hqp_hs_.size(); i++)
    {
        if (!init)
        {
            solve_time += hqp_r.hqp_hs_[i].qp_solve_time_step_;
            update_time += hqp_r.hqp_hs_[i].qp_update_time_step_;
        }
    }
    hqp_r.update_time_step_ = update_time;
    hqp_r.solve_time_step_ = solve_time;
    hqp_r.total_time_step_ = update_time + solve_time;

    hqp_r.update_time_max_ = std::max(hqp_r.update_time_max_, update_time);
    hqp_r.solve_time_max_ = std::max(hqp_r.solve_time_max_, solve_time);
    hqp_r.total_time_max_ = std::max(hqp_r.total_time_max_, hqp_r.total_time_step_);
    return ret;
}

int RobotData::CalcControlTorqueLQP_R_NC(HQP &hqp_nc, bool init)
{
    int ret = 0;
    hqp_nc.solvefirst(init);
    hqp_nc.solveSequential(init);
    double solve_time = 0;
    double update_time = 0;
    for (int i = 0; i < hqp_nc.hqp_hs_.size(); i++)
    {
        if (!init)
        {
            solve_time += hqp_nc.hqp_hs_[i].qp_solve_time_step_;
            update_time += hqp_nc.hqp_hs_[i].qp_update_time_step_;
        }
    }
    hqp_nc.update_time_step_ = update_time;
    hqp_nc.solve_time_step_ = solve_time;
    hqp_nc.total_time_step_ = update_time + solve_time;

    hqp_nc.update_time_max_ = std::max(hqp_nc.update_time_max_, update_time);
    hqp_nc.solve_time_max_ = std::max(hqp_nc.solve_time_max_, solve_time);
    hqp_nc.total_time_max_ = std::max(hqp_nc.total_time_max_, hqp_nc.total_time_step_);
    return ret;
}

int RobotData::ConfigureLQP_R(HQP &hqp_, bool init)
{
    int ret = 0;

    // classify the task space
    // for (int i = 0; i < ts_.size(); i++)
    // {
    //     bool cc_link = false;
    //     for(int j=0;j<ts_[i].task_link_.size();j++)
    //     {
    //         auto it = std::find(co_link_idx_.begin(), co_link_idx_.end(), ts_[i].task_link_[j].link_id_);
    //         if(it != co_link_idx_.end())
    //         { // if the link is int the cc link
    //             cc_link |= true;
    //         }
    //         else{ // if the link is not in the cc link
    //             cc_link |= false;
    //         }
    //     }
    //     ts_[i].noncont_task_ = !co_link;
    // }
    int acc_size = reduced_system_dof_;
    int torque_size = reduced_model_dof_;
    int contact_size = contact_dof_;
    int variable_size = acc_size + contact_size;

    int acc_idx = 0;
    // int toruqe_idx = acc_size;
    int contact_idx = acc_size + 0;
    hqp_.initialize(acc_size, 0, contact_size);

    MatrixXd cost_h = MatrixXd::Zero(acc_size + contact_size, acc_size + contact_size);
    VectorXd cost_g = VectorXd::Zero(acc_size + contact_size);

    cost_h.block(0, 0, acc_size, acc_size) = A_R / A_.topLeftCorner(6, 6).norm() * hqp_.jacc_cost;

    // Priority 1
    int ineq_constraint_size = torque_size * 2; // torque limit
    int eq_constraint_size = 6;                 // newton euler
    if (init)
        hqp_.addHierarchy(ineq_constraint_size, eq_constraint_size);
    // hqp_.hqp_hs_[0].y_ans_.head(variable_size).setZero();

    MatrixXd A = MatrixXd::Zero(ineq_constraint_size, variable_size);
    VectorXd a = VectorXd::Zero(ineq_constraint_size);

    MatrixXd B = MatrixXd::Zero(eq_constraint_size, variable_size);
    VectorXd b = VectorXd::Zero(eq_constraint_size);

    B.block(0, 0, eq_constraint_size, acc_size) = A_R.topRows(eq_constraint_size);
    B.block(0, acc_size, eq_constraint_size, contact_size) = (J_CR.transpose()).topRows(eq_constraint_size);
    b.head(eq_constraint_size) = G_R.head(eq_constraint_size);

    VectorXd tlim = VectorXd::Constant(torque_size, hqp_.tlim_def);
    int tlim_size = tlim.size();

    tlim(tlim_size - 4) = 600;

    A.block(0, 0, torque_size, acc_size) = A_R.bottomRows(torque_size);
    A.block(0, acc_size, torque_size, contact_size) = J_CR.transpose().bottomRows(torque_size);
    A.block(torque_size, 0, torque_size, acc_size) = -A_R.bottomRows(torque_size);
    A.block(torque_size, acc_size, torque_size, contact_size) = -J_CR.transpose().bottomRows(torque_size);

    // A.block(0, acc_size, torque_size, torque_size) = MatrixXd::Identity(torque_size, torque_size);            // torque limit
    // A.block(torque_size, acc_size, torque_size, torque_size) = -MatrixXd::Identity(torque_size, torque_size); // torque limit
    a.head(torque_size) = -tlim + G_R.tail(torque_size);
    a.tail(torque_size) = -tlim - G_R.tail(torque_size);

    // std::cout << "test0 " << hqp_.hqp_hs_[0].ineq_const_size_ << std::endl;
    // std::cout << "A :rows : " << A.rows() << " cols : " << A.cols() << std::endl;

    hqp_.hqp_hs_[0].updateConstraintMatrix(A, a, B, b);
    hqp_.hqp_hs_[0].normalizeConstraintMatrix();
    // Solve
    hqp_.hqp_hs_[0].v_ans_.setZero(ineq_constraint_size);
    hqp_.hqp_hs_[0].w_ans_.setZero(eq_constraint_size);
    hqp_.hqp_hs_[0].y_ans_.head(acc_size) = -A_R_inv * G_R;

    // Priority 2
    int contact_constraint_size = contact_link_num_ * 10;
    ineq_constraint_size = contact_constraint_size + torque_size * 2;
    eq_constraint_size = contact_size;
    if (init)
        hqp_.addHierarchy(ineq_constraint_size, eq_constraint_size);
    A.setZero(ineq_constraint_size, variable_size);
    a.setZero(ineq_constraint_size);
    A.block(0, acc_size, contact_constraint_size, contact_size) = getContactConstraintMatrix();
    A.block(contact_constraint_size, 6, torque_size, torque_size) = MatrixXd::Identity(torque_size, torque_size);
    A.block(contact_constraint_size + torque_size, 6, torque_size, torque_size) = -MatrixXd::Identity(torque_size, torque_size);

    VectorXd alim = VectorXd::Constant(torque_size, hqp_.alim_def);
    a.segment(contact_constraint_size, torque_size) = -alim;
    a.segment(contact_constraint_size + torque_size, torque_size) = -alim;

    B.setZero(eq_constraint_size, variable_size);
    b.setZero(eq_constraint_size);
    B.block(0, 0, contact_size, acc_size) = J_CR;

    // std::cout << "test1" << std::endl;
    hqp_.hqp_hs_[1].updateConstraintMatrix(A, a, B, b);
    hqp_.hqp_hs_[1].updateCostMatrix(cost_h, cost_g);
    hqp_.hqp_hs_[1].normalizeConstraintMatrix();

    // Priority 3

    for (int i = 0; i < ts_.size(); i++)
    {
        if (!ts_[i].noncont_task)
        {
            ineq_constraint_size = 0;
            eq_constraint_size = ts_[i].task_dof_;
            if (init)
                hqp_.addHierarchy(ineq_constraint_size, eq_constraint_size);

            B.setZero(eq_constraint_size, variable_size);
            b.setZero(eq_constraint_size);

            B.block(0, 0, eq_constraint_size, acc_size) = ts_[i].J_task_ * J_R_INV_T.transpose();
            b.head(eq_constraint_size) = -ts_[i].f_star_;

            hqp_.hqp_hs_[2 + i].updateConstraintMatrix(A, a, B, b);
            hqp_.hqp_hs_[2 + i].updateCostMatrix(cost_h, cost_g);
            hqp_.hqp_hs_[2 + i].normalizeConstraintMatrix();
        }
    }
    hqp_.prepare();

    return ret;
}

int RobotData::ConfigureLQP_R_NC(HQP &hqp_nc_, VectorXd &q_acc, bool init)
{
    int ret = 0;
    int acc_size_nc = nc_dof;
    int torque_size_nc = nc_dof;
    int variable_size = acc_size_nc;

    Vector6d fstar_gnc = q_acc.tail(6);
    Vector6d fstar_base = q_acc.head(6);

    MatrixXd cost_h_nc = MatrixXd::Zero(variable_size, variable_size);
    VectorXd cost_g_nc = VectorXd::Zero(variable_size);

    cost_h_nc = A_NC.bottomRightCorner(acc_size_nc, acc_size_nc) / A_NC.topLeftCorner(6, 6).norm() * hqp_nc_.jacc_cost;

    hqp_nc_.initialize(acc_size_nc, 0, 0);

    int eq_constraint_size = 6;
    int ineq_constraint_size = 2 * acc_size_nc;
    if (init)
        hqp_nc_.addHierarchy(ineq_constraint_size, eq_constraint_size);

    MatrixXd A_nc = MatrixXd::Zero(ineq_constraint_size, variable_size);
    VectorXd a_nc = VectorXd::Zero(ineq_constraint_size);

    MatrixXd B_nc = MatrixXd::Zero(eq_constraint_size, variable_size);
    VectorXd b_nc = VectorXd::Zero(eq_constraint_size);

    // gcom const
    B_nc.block(0, 0, eq_constraint_size, acc_size_nc) = J_I_nc_;
    b_nc.head(eq_constraint_size) = -fstar_gnc;

    VectorXd tlim_nc = VectorXd::Constant(torque_size_nc, hqp_nc_.tlim_def);
    A_nc.block(0, 0, acc_size_nc, acc_size_nc) = A_NC.bottomRightCorner(acc_size_nc, acc_size_nc);
    a_nc.segment(0, acc_size_nc) = -tlim_nc + G_NC;
    A_nc.block(1 * acc_size_nc, 0, acc_size_nc, acc_size_nc) = -A_NC.bottomRightCorner(acc_size_nc, acc_size_nc);
    a_nc.segment(1 * acc_size_nc, acc_size_nc) = -tlim_nc - G_NC;

    hqp_nc_.hqp_hs_[0].updateConstraintMatrix(A_nc, a_nc, B_nc, b_nc);
    hqp_nc_.hqp_hs_[0].updateCostMatrix(cost_h_nc, cost_g_nc);

    bool first = true;

    int nc_task_idx = 0;

    // for (int i = 0; i < ts_.size(); i++)
    // {
    //     if (ts_[i].noncont_task)
    //     {
    //         if (first)
    //         {
    //             first = false;
    //             eq_constraint_size = ts_[i].task_dof_;
    //             ineq_constraint_size = 2 * acc_size_nc;

    //             hqp_nc_.addHierarchy(ineq_constraint_size, eq_constraint_size);

    //             MatrixXd Ja = MatrixXd::Identity(6, 6);
    //             Ja.block(0, 3, 3, 3) = skew(link_[ts_[i].task_link_[0].link_id_].xpos - link_[0].xpos);

    //             VectorXd fstar_local = Ja * (ts_[i].f_star_ - fstar_base);

    //             B_nc.setZero(eq_constraint_size, variable_size);
    //             b_nc.setZero(eq_constraint_size);

    //             B_nc.block(0, 0, 6, acc_size_nc) = ts_[i].J_task_.rightCols(nc_dof);
    //             b_nc.head(6) = -fstar_local;

    //             // acc lim
    //             A_nc.block(0, 0, acc_size_nc, acc_size_nc) = MatrixXd::Identity(acc_size_nc, acc_size_nc);
    //             a_nc.segment(0, acc_size_nc) = -VectorXd::Constant(acc_size_nc, 5);
    //             A_nc.block(acc_size_nc, 0, acc_size_nc, acc_size_nc) = -MatrixXd::Identity(acc_size_nc, acc_size_nc);
    //             a_nc.segment(acc_size_nc, acc_size_nc) = -VectorXd::Constant(acc_size_nc, 5);

    //             hqp_nc_.hqp_hs_[1].updateConstraintMatrix(A_nc, a_nc, B_nc, b_nc);
    //             hqp_nc_.hqp_hs_[1].updateCostMatrix(cost_h_nc, cost_g_nc);
    //         }
    //         else
    //         {
    //             eq_constraint_size = ts_[i].task_dof_;
    //             ineq_constraint_size = 0;

    //             hqp_nc_.addHierarchy(ineq_constraint_size, eq_constraint_size);

    //             B_nc.setZero(eq_constraint_size, variable_size);
    //             b_nc.setZero(eq_constraint_size);

    //             B_nc.block(0, 0, eq_constraint_size, acc_size_nc) = ts_[i].J_task_.rightCols(acc_size_nc);
    //             b_nc.head(eq_constraint_size) = -ts_[i].f_star_;

    //             hqp_nc_.hqp_hs_[1 + i].updateEqualityConstraintMatrix(B_nc, b_nc);
    //             hqp_nc_.hqp_hs_[1 + i].updateCostMatrix(cost_h_nc, cost_g_nc);
    //         }

    //         nc_task_idx++;
    //     }
    // }

    eq_constraint_size = 6;
    ineq_constraint_size = 2 * acc_size_nc;
    if (init)
        hqp_nc_.addHierarchy(ineq_constraint_size, eq_constraint_size);

    MatrixXd Ja = MatrixXd::Identity(6, 6);
    Ja.block(0, 3, 3, 3) = skew(link_[ts_[1].task_link_[0].link_id_].xpos - link_[0].xpos);

    VectorXd fstar_local = Ja * (ts_[1].f_star_ - fstar_base);

    B_nc.setZero(eq_constraint_size, variable_size);
    b_nc.setZero(eq_constraint_size);

    B_nc.block(0, 0, 6, acc_size_nc) = ts_[1].J_task_.rightCols(nc_dof);
    b_nc.head(6) = -fstar_local;

    // acc lim
    A_nc.block(0, 0, acc_size_nc, acc_size_nc) = MatrixXd::Identity(acc_size_nc, acc_size_nc);
    a_nc.segment(0, acc_size_nc) = -VectorXd::Constant(acc_size_nc, hqp_nc_.alim_def);

    A_nc.block(acc_size_nc, 0, acc_size_nc, acc_size_nc) = -MatrixXd::Identity(acc_size_nc, acc_size_nc);
    a_nc.segment(acc_size_nc, acc_size_nc) = -VectorXd::Constant(acc_size_nc, hqp_nc_.alim_def);

    hqp_nc_.hqp_hs_[1].updateConstraintMatrix(A_nc, a_nc, B_nc, b_nc);
    hqp_nc_.hqp_hs_[1].updateCostMatrix(cost_h_nc, cost_g_nc);

    hqp_nc_.prepare();
    return ret;
}

int RobotData::CalcContactRedistributeR(bool hqp, bool init)
{
    int ret = 0;
    if (contact_dof_ > 6)
    {
        ret = CalcContactRedistributeR(torque_grav_R_ + torque_task_R_, hqp, init);

        torque_contact_.setZero(model_dof_);
        torque_contact_.segment(0, contact_dof_) = torque_contact_R_.segment(0, contact_dof_);
    }

    return ret;
}

int RobotData::CalcContactRedistributeR(VectorXd torque_input, bool hqp, bool init)
{
    int contact_index = cc_.size();      // size of contact link
    int total_contact_dof = 0;           // size of contact dof
    int contact_dof = -6;                // total_contact_dof - 6, (free contact space)
    int contact_constraint_size = 0;     // size of constraint by contact
    int model_size = reduced_model_dof_; // size of joints
    int torque_limit_constraint_size = 2 * model_size;

    if (init)
    {
        // Matrix validation
        if (torque_input.size() != model_size)
        {
            cout << "Contact Redistribution : torque input size is not matched with model size" << endl;
        }
    }

    for (int i = 0; i < contact_index; i++)
    {
        if (cc_[i].contact)
        {
            total_contact_dof += cc_[i].contact_dof_;
            contact_constraint_size += cc_[i].constraint_number_;
        }
    }
    contact_dof += total_contact_dof;

    if (hqp)
    {

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
            // for (int i = 0; i < contact_index; i++)
            // {
            //     if (cc_[i].contact)
            //     {
            //         Vector3d vec_origin, vec_target;
            //         vec_origin = cc_[i].rotm.rightCols(1);
            //         vec_target = (com_pos - cc_[i].xc_pos).normalized();
            //         Matrix3d cm = AxisTransform2V(vec_origin, vec_target);

            //         cm.setIdentity(); // comment this line to redistribute contact force with COM based vector

            //         if (cc_[i].contact_type_ == CONTACT_6D)
            //         {
            //             crot_matrix.block(acc_cdof, acc_cdof, 3, 3) = crot_matrix.block(acc_cdof + 3, acc_cdof + 3, 3, 3) = cm.transpose() * cc_[i].rotm.transpose();
            //             RotW(acc_cdof + 2, acc_cdof + 2) = 0;
            //             acc_cdof += cc_[i].contact_dof_;
            //         }
            //         else if (cc_[i].contact_type_ == CONTACT_POINT)
            //         {
            //             crot_matrix.block(acc_cdof, acc_cdof, 3, 3) = cm.transpose() * cc_[i].rotm.transpose();
            //             RotW(acc_cdof + 2, acc_cdof + 2) = 0;
            //             acc_cdof += cc_[i].contact_dof_;
            //         }
            //     }
            // }
            // J_CR_INV_T.rightCols(model_size) * NwJw_R;
            // H_temp = RotW * crot_matrix * J_CR_INV_T.rightCols(model_size) * NwJw_R;
            // H = H_temp.transpose() * H_temp;
            // g = (RotW * crot_matrix * (J_CR_INV_T.rightCols(model_size) * torque_input - P_CR)).transpose() * H_temp;

            H.setZero(variable_number, variable_number);
            H.setIdentity();
            g.setZero(variable_number);

            MatrixXd A_qp;
            VectorXd lbA, ubA;
            A_qp.setZero(total_constraint_size, variable_number);
            lbA.setZero(total_constraint_size);
            ubA.setZero(total_constraint_size);

            if (torque_limit_set_)
            {
                A_qp.block(0, 0, model_size, contact_dof) = NwJw_R;
                // lbA.segment(0, model_size) = -torque_limit_ - control_torque;
                ubA.segment(0, model_size) = torque_limit_ - torque_input;

                A_qp.block(0, 0, model_size, contact_dof) = -NwJw_R;
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
            Eigen::MatrixXd CM = -A_const_a * A_rot;

            // Eigen::MatrixXd Atemp = A_const_a * A_rot * J_CR_INV_T.rightCols(model_size);
            // Eigen::VectorXd bA = A_const_a * (A_rot * P_CR) - Atemp * torque_input;

            A_qp.block(torque_limit_constraint_size, 0, contact_constraint_size, contact_dof) = CM * J_CR_INV_T.rightCols(model_size) * NwJw_R;

            lbA.segment(torque_limit_constraint_size, contact_constraint_size).setConstant(-INFTY);
            ubA.segment(torque_limit_constraint_size, contact_constraint_size) = CM * P_CR - CM * J_CR_INV_T.rightCols(model_size) * torque_input;

            Eigen::VectorXd qpres;
            int solve_success = SolveOSQP(osqp_contact_, H, g, A_qp, lbA, ubA, qpres, init);
            // if (qp_contact_.CheckProblemSize(variable_number, total_constraint_size))
            // {
            //     if (init)
            //     {
            //         qp_contact_.InitializeProblemSize(variable_number, total_constraint_size);
            //     }
            // }
            // else
            // {
            //     qp_contact_.InitializeProblemSize(variable_number, total_constraint_size);
            // }

            // qp_contact_.UpdateMinProblem(H, g);
            // qp_contact_.UpdateSubjectToAx(A_qp, lbA, ubA);
            // if (qp_contact_.SolveQPoases(600, qpres))
            // {
            //     torque_contact_R_ = NwJw_R * qpres;

            //     return 1;
            // }
            // else
            // {
            //     std::cout << "Contact Reduced qp solve failed" << std::endl;
            //     torque_contact_R_.setZero(model_size);

            //     return 0;
            // }
            if (solve_success)
            {
                torque_contact_R_ = NwJw_R * qpres;

                return 1;
            }
            else
            {
                std::cout << "Contact Reduced qp solve failed" << std::endl;
                torque_contact_R_.setZero(model_size);

                return 0;
            }
        }
        else
        {
            torque_contact_R_ = VectorXd::Zero(model_size);

            return 1;
        }
    }
    else
    {
        if (co_dof == 12)
        {
            Vector12d ContactForce_ = J_CR_INV_T.rightCols(reduced_model_dof_) * torque_input - P_CR;

            Vector3d P1_ = cc_[0].xc_pos - com_pos;
            Vector3d P2_ = cc_[1].xc_pos - com_pos;

            Matrix3d Rotyaw = rotateWithZ(-link_[0].rpy(2));

            Eigen::Matrix<double, 12, 12> force_rot_yaw;
            force_rot_yaw.setZero();
            for (int i = 0; i < 4; i++)
            {
                force_rot_yaw.block(i * 3, i * 3, 3, 3) = Rotyaw;
            }

            Vector6d ResultantForce_;
            ResultantForce_.setZero();

            Vector12d ResultRedistribution_;
            ResultRedistribution_.setZero();

            Vector12d F12 = force_rot_yaw * ContactForce_;

            double eta_cust = 0.99;
            double foot_length = 0.26;
            double foot_width = 0.1;

            double eta = 0.95;

            ContactRedistributetwomod(0.99, foot_length, foot_width, 1.0, 0.9, 0.9, Rotyaw * P1_, Rotyaw * P2_, F12, ResultantForce_, ResultRedistribution_, eta);
            Vector12d fc_redist_ = force_rot_yaw.transpose() * ResultRedistribution_;

            Vector12d desired_force;
            desired_force.setZero();

            desired_force.segment(6, 6) = -ContactForce_.segment(6, 6) + fc_redist_.segment(6, 6);
            torque_contact_R_ = V2_R.transpose() * (J_CR_INV_T.rightCols(reduced_model_dof_).bottomRows(6) * V2_R.transpose()).inverse() * desired_force.segment(6, 6);

            return 1;
        }
        else
        {
            torque_contact_R_.setZero(reduced_model_dof_);

            return 0;
        }
    }
}

int RobotData::SolveOSQP(OsqpEigen::Solver *osqp, const MatrixXd &H, VectorXd &g, const MatrixXd &A, VectorXd &lb, VectorXd &ub, VectorXd &ans, bool init)
{
    Eigen::SparseMatrix<double> sA_ = A.sparseView(1E-5);
    Eigen::SparseMatrix<double> sH_ = H.sparseView(1E-5);

    int var_num = H.cols();
    int const_num = A.rows();
    int result = 0;

    if (init)
    {
        osqp->clearSolver();
        osqp->data()->clearHessianMatrix();
        osqp->data()->clearLinearConstraintsMatrix();

        osqp->data()->setNumberOfVariables(var_num);
        osqp->data()->setNumberOfConstraints(const_num);

        osqp->data()->setHessianMatrix(sH_);
        osqp->data()->setGradient(g);
        osqp->data()->setLinearConstraintsMatrix(sA_);
        osqp->data()->setUpperBound(ub);
        osqp->data()->setLowerBound(lb);

        osqp->settings()->setWarmStart(true);
        osqp->settings()->setVerbosity(false);
        osqp->settings()->setAlpha(1.0);

        if (!osqp->initSolver())
            return 0;

        if (osqp->solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
            return 0;

        ans = osqp->getSolution();

        return 1;
    }
    else
    {

        osqp->updateHessianMatrix(sH_);
        osqp->updateGradient(g);
        osqp->updateLinearConstraintsMatrix(sA_);
        osqp->updateUpperBound(ub);
        osqp->updateLowerBound(lb);
        if (osqp->solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
            return 0;

        ans = osqp->getSolution();

        return 1;
    }
}

int RobotData::SolveQPoases(CQuadraticProgram &qp, const MatrixXd &H, const VectorXd &g, const MatrixXd &A, const VectorXd &lb, const VectorXd &ub, VectorXd &ans, bool init)
{
    int var_num = H.cols();
    int const_num = A.rows();
    int result = 0;

    if (qp.CheckProblemSize(var_num, const_num))
    {
        if (init)
        {
            qp.InitializeProblemSize(var_num, const_num);
        }
    }
    else
    {
        qp.InitializeProblemSize(var_num, const_num);
    }

    qp.UpdateMinProblem(H, g);
    qp.UpdateSubjectToAx(A, lb, ub);

    if (qp.SolveQPoases(600, ans))
    {
        return 1;
    }
    else
    {
        std::cout << "Contact Reduced qp solve failed" << std::endl;
        return 0;
    }
}
