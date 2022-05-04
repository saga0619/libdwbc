#ifndef WBHQP_ROBOTDATA_HPP
#define WBHQP_ROBOTDATA_HPP

#include "link.hpp"
#include "contact_constraint.hpp"
#include "task.hpp"
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <cstdarg>

using namespace Eigen;

class RobotData
{
private:
    /* data */
public:
    RobotData(/* args */);
    ~RobotData();

    unsigned int system_dof_;
    unsigned int model_dof_;
    unsigned int contact_dof_;

    double total_mass_;

    RigidBodyDynamics::Model model_;

    Vector3d com_pos;
    Vector3d com_vel;

    MatrixXd J_com_;

    MatrixXd A_;
    MatrixXd A_inv_;

    VectorXd q_;
    VectorXd q_dot_;
    VectorXd q_ddot_;

    VectorXd G_;
    VectorXd torque_grav_;
    VectorXd torque_task_;
    VectorXd torque_contact_;

    MatrixXd Lambda_contact;
    MatrixXd J_C;
    MatrixXd J_C_INV_T;
    MatrixXd N_C;

    MatrixXd P_C;

    MatrixXd W;
    MatrixXd W_inv;
    MatrixXd V2;

    MatrixXd NwJw;

    std::vector<Link> link_;
    std::vector<ContactConstraint> cc_;
    std::vector<TaskSpace> ts_;

    CQuadraticProgram qp_contact_;

    void UpdateKinematics(const VectorXd q_virtual, const VectorXd q_dot_virtual, const VectorXd q_ddot_virtual)
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
    }
    /*
    Add Contact constraint

    contact_type: 0: plane contact(6dof) 1: point contact(3dof)

    */
    void AddContactConstraint(int link_number, int contact_type, Vector3d contact_point, Vector3d contact_vector, double contact_x = 0, double contact_y = 0, bool verbose = false);

    void ClearContactConstraint();

    void UpdateContactConstraint();

    void CalcContactConstraint(bool update = true);

    void ClearTaskSpace()
    {
        ts_.clear();
    }

    void AddTaskSpace(int task_mode, int task_dof, bool verbose = false)
    {
        if (verbose)
            std::cout << "#" << ts_.size() << " Task Space Added with mode " << taskmode_str[task_mode] << std::endl;

        ts_.push_back(TaskSpace(task_mode, ts_.size(), task_dof, system_dof_));
    }

    void AddTaskSpace(int task_mode, int link_number, Vector3d task_point, bool verbose = false)
    {
        if (verbose)
            std::cout << "#" << ts_.size() << " Task Space Added : " << link_[link_number].name_ << " " << taskmode_str[task_mode] << " at point : " << task_point.transpose() << std::endl;

        ts_.push_back(TaskSpace(task_mode, ts_.size(), link_number, link_[link_number].link_id_, task_point, model_dof_));
    }

    void SetTaskSpace(int heirarchy, const MatrixXd &f_star, const MatrixXd &J_task = MatrixXd::Zero(1, 1))
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

    void UpdateTaskSpace()
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

    void CalcTaskSpace(bool update = true)
    {
        if (update)
            UpdateTaskSpace();
        for (int i = 0; i < ts_.size(); i++)
        {
            ts_[i].CalcJKT(A_inv_, N_C, W_inv);
        }
    }

    void CalcTaskTorque(bool hqp = true)
    {
        torque_task_.setZero(model_dof_);

        if (hqp)
        {
            VectorXd torque_limit;
            torque_limit.setConstant(model_dof_, 1000);
            for (int i = 0; i < ts_.size(); i++)
            {
                if (i == 0)
                {
                    MatrixXd Null_identity = MatrixXd::Identity(model_dof_, model_dof_);

                    ts_[i].GetTaskTorque(cc_, Null_identity, torque_limit, torque_grav_, NwJw, J_C_INV_T, P_C);

                    torque_task_ = ts_[i].J_kt_ * ts_[i].Lambda_task_ * (ts_[i].f_star_ + ts_[i].f_star_qp_);
                }
                else
                {
                    MatrixXd Null_identity = MatrixXd::Identity(model_dof_, model_dof_);
                    ts_[i].GetTaskTorque(cc_, Null_identity, torque_limit, torque_grav_, NwJw, J_C_INV_T, P_C);

                    torque_task_ += ts_[i - 1].Null_task_ * (ts_[i].J_kt_ * ts_[i].Lambda_task_ * (ts_[i].f_star_ + ts_[i].f_star_qp_));
                }

                if (i != ts_.size() - 1)
                    ts_[i].CalcNullMatrix(A_inv_, N_C);
            }
        }
        else
        {
            for (int i = 0; i < ts_.size(); i++)
            {
                if (i == 0)
                {
                    ts_[i].torque_h_ = ts_[i].J_kt_ * ts_[i].Lambda_task_ * ts_[i].f_star_;

                    std::cout << ts_[i].J_task_ << std::endl;

                    std::cout << ts_[i].J_kt_ << std::endl;

                    std::cout << ts_[i].Lambda_task_ << std::endl;

                    std::cout << ts_[i].f_star_ << std::endl;

                    std::cout << "torque task " << std::endl;

                    std::cout << ts_[i].torque_h_.transpose() << std::endl;
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
        }
    }

    /*
    Init model data with rbdl
    verbose 2 : Show all link Information
    verbose 1 : Show link id information
    verbose 0 : disable verbose
    */
    void InitModelData(std::string urdf_path, bool floating, int verbose)
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

    VectorXd CalcGravCompensation()
    {
        torque_grav_ = W_inv * (A_inv_.bottomRows(model_dof_) * (N_C * G_));
        P_C = J_C_INV_T * G_;
        return torque_grav_;
    }

    void CalcGravCompensation(VectorXd &grav_torque)
    {
        torque_grav_ = W_inv * (A_inv_.bottomRows(model_dof_) * (N_C * G_));
        P_C = J_C_INV_T * G_;

        grav_torque = torque_grav_;
    }

    template <typename... Types>
    void SetContact(Types... args)
    {
        std::vector<bool> v;
        (v.push_back(args), ...);

        if (cc_.size() < v.size())
        {
            std::cout << "Contact Constraint size mismatch ! input size : " << v.size() << " contact constraint size : " << cc_.size() << std::endl;
        }
        else
        {
            int itr = 0;

            for (auto n : v)
            {
                std::cout << "setting " << link_[cc_[itr].link_number_].name_ << " contact : " << bool_cast(n) << std::endl;
                cc_[itr++].SetContact(n);
            }

            if (cc_.size() > v.size())
            {
                for (int i = v.size(); i < cc_.size(); i++)
                {
                    std::cout << "setting " << link_[cc_[itr].link_number_].name_ << " contact : false" << std::endl;
                    cc_[itr++].SetContact(false);
                }
            }

            int contact_dof = 0;

            for (int i = 0; i < cc_.size(); i++)
            {
                if (cc_[i].contact)
                {
                    contact_dof += cc_[i].contact_dof_;
                }
            }

            contact_dof_ = contact_dof;
        }
    }

    VectorXd getContactForce(const VectorXd &command_torque)
    {
        return J_C_INV_T.rightCols(model_dof_) * (command_torque)-P_C;
    }

    void CalcContactRedistribute()
    {

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

        // Vector12d Fc = rd_.J_C_INV_T.rightCols(MODEL_DOF) * Control_Torque - rd_.J_C_INV_T * rd_.G;

        // Vector3d cpos_l, cpos_r;

        // cpos_l = rd_.link_[Left_Foot].rotm.transpose() * (rd_.link_[COM_id].xpos - rd_.link_[Left_Foot].xpos);
        // cpos_r = rd_.link_[Right_Foot].rotm.transpose() * (rd_.link_[COM_id].xpos - rd_.link_[Right_Foot].xpos);

        // Matrix3d crot_l, crot_r;

        // crot_r = DyrosMath::rotateWithX(-atan(cpos_r(1) / cpos_r(2))) * DyrosMath::rotateWithY(atan(cpos_r(0) / sqrt(cpos_r(1) * cpos_r(1) + cpos_r(2) * cpos_r(2))));

        // crot_l = DyrosMath::rotateWithX(-atan(cpos_l(1) / cpos_l(2))) * DyrosMath::rotateWithY(atan(cpos_l(0) / sqrt(cpos_l(1) * cpos_l(1) + cpos_l(2) * cpos_l(2))));

        // Vector3d cres_l, cres_r;

        // cres_l = crot_l.transpose() * cpos_l;

        // cres_r = crot_r.transpose() * cpos_r;

        // Vector12d ContactForce;
        // Eigen::Matrix<double, 12, 12> RotM_;

        int contact_dof = 0;
        int contact_index = cc_.size();
        for (int i = 0; i < contact_index; i++)
        {
            contact_dof += cc_[i].contact_dof_;
        }

        if (contact_dof == 6)
        {
            torque_contact_ = VectorXd::Zero(model_dof_);
        }
        else
        {

            Eigen::MatrixXd crot_matrix;
            Eigen::MatrixXd RotW;

            crot_matrix.setZero(contact_dof, contact_dof);

            RotW.setIdentity(contact_dof, contact_dof);

            int acc_cdof = 0;

            for (int i = 0; i < cc_.size(); i++)
            {
                Vector3d cv = cc_[i].rotm.transpose() * (com_pos - cc_[i].xc_pos);
                Matrix3d cm = rotateWithX(-atan(cv(1) / cv(2))) * rotateWithY(atan(cv(0) / sqrt(cv(1) * cv(1) + cv(2) * cv(2))));
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

            VectorXd control_torque = torque_grav_ + torque_task_;

            int total_const = 0;

            for (int i = 0; i < cc_.size(); i++)
            {
                total_const += cc_[i].constraint_number_;
            }

            qp_contact_.InitializeProblemSize(contact_dof_ - 6, total_const);

            static MatrixXd H;
            static VectorXd g;
            static MatrixXd A_t;

            // std::cout<<"2"<<std::endl;
            A_t = RotW * crot_matrix * J_C_INV_T.rightCols(model_dof_) * NwJw;

            // std::cout<<"3"<<std::endl;
            H = A_t.transpose() * A_t;

            // std::cout<<"4"<<std::endl;
            g = (RotW * crot_matrix * (J_C_INV_T.rightCols(model_dof_) * control_torque - P_C)).transpose() * A_t;

            // std::cout<<"5"<<std::endl;
            qp_contact_.UpdateMinProblem(H, g);

            // Constraint

            MatrixXd A_const_a;
            A_const_a.setZero(total_const, contact_dof_);

            MatrixXd A__mat;

            MatrixXd A_rot;

            A_rot.setZero(contact_dof_, contact_dof_);

            int const_idx = 0;
            int contact_idx = 0;

            for (int i = 0; i < cc_.size(); i++)
            {
                A_rot.block(contact_idx, contact_idx, 3, 3) = cc_[i].rotm.transpose(); // rd_.ee_[i].rotm.transpose();
                A_rot.block(contact_idx + 3, contact_idx + 3, 3, 3) = cc_[i].rotm.transpose();

                A_const_a.block(const_idx, contact_idx, 4, 6) = cc_[i].GetZMPConstMatrix4x6();
                A_const_a.block(const_idx + CONTACT_CONSTRAINT_ZMP, contact_idx, 6, 6) = cc_[i].GetForceConstMatrix6x6();

                const_idx += cc_[i].constraint_number_;
                contact_idx += cc_[i].contact_dof_;

                // Force Constraint
                // Will be added
            }

            Eigen::MatrixXd Atemp = A_const_a * A_rot * J_C_INV_T.rightCols(model_dof_);

            // std::cout<<"0"<<std::endl;
            Eigen::VectorXd bA = A_const_a * (A_rot * P_C) - Atemp * control_torque;

            // A_const_a *A_rot *(P_C - J_C_INV_T.rightCols(model_dof_) * control_torque);

            // std::cout<<"1"<<std::endl;

            Eigen::VectorXd ubA;
            ubA.setConstant(total_const, 1E+6);

            // std::cout<<"2"<<std::endl;

            qp_contact_.UpdateSubjectToAx(Atemp * NwJw, bA, ubA + bA);

            // std::cout << "3" << std::endl;

            // qp_torque_contact_.UpdateSubjectToAx()

            // std::cout<<"6"<<std::endl;
            Eigen::VectorXd qp_result;

            if (qp_contact_.SolveQPoases(100, qp_result))
            {
                torque_contact_ = NwJw * qp_result;
            }
            else
            {
                std::cout << "contact qp solve failed" << std::endl;
                torque_contact_.setZero(model_dof_);
            }
        }
    }
};

RobotData::RobotData(/* args */)
{
}

RobotData::~RobotData()
{
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

    Lambda_contact = (J_C * A_inv_ * J_C.transpose()).llt().solve(Eigen::MatrixXd::Identity(contact_dof_, contact_dof_));

    J_C_INV_T = Lambda_contact * J_C * A_inv_;

    N_C = MatrixXd::Identity(system_dof_, system_dof_) - J_C.transpose() * J_C_INV_T;

    W = A_inv_.bottomRows(model_dof_) * N_C.rightCols(model_dof_);

    PinvCOD(W, W_inv, V2);

    NwJw = V2.transpose() * (J_C_INV_T.rightCols(model_dof_).topRows(6) * V2.transpose()).inverse();
}

#endif