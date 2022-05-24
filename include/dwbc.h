#ifndef WBHQP_ROBOTDATA_HPP
#define WBHQP_ROBOTDATA_HPP

#include <rbdl/addons/urdfreader/urdfreader.h>
#include <cstdarg>

#include "link.h"
#include "contact_constraint.h"
#include "task.h"
#include "wbd.h"
#include <qpSWIFT/qpSWIFT.h>

using namespace Eigen;
namespace DWBC
{

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

        void UpdateKinematics(const VectorXd q_virtual, const VectorXd q_dot_virtual, const VectorXd q_ddot_virtual);

        /*
        Add Contact constraint

        contact_type: 0: plane contact(6dof) 1: point contact(3dof)

        */
        void AddContactConstraint(int link_number, int contact_type, Vector3d contact_point, Vector3d contact_vector, double contact_x = 0, double contact_y = 0, bool verbose = false);
        void ClearContactConstraint();
        void UpdateContactConstraint();
        void CalcContactConstraint(bool update = true);

        template <typename... Types>
        void SetContact(Types... args);

        void CalcContactRedistribute();
        VectorXd getContactForce(const VectorXd &command_torque);

        void ClearTaskSpace();
        void AddTaskSpace(int task_mode, int task_dof, bool verbose = false);
        void AddTaskSpace(int task_mode, int link_number, Vector3d task_point, bool verbose = false);
        void SetTaskSpace(int heirarchy, const MatrixXd &f_star, const MatrixXd &J_task = MatrixXd::Zero(1, 1));
        void UpdateTaskSpace();
        void CalcTaskSpace(bool update = true);
        void qpSWIFT_test();
        void CalcTaskTorque(bool hqp = true, bool init = true);
        void CalcTaskTorqueQP(TaskSpace &ts_, const MatrixXd &task_null_matrix_, const VectorXd &torque_limit, const VectorXd &torque_prev, const MatrixXd &NwJw, const MatrixXd &J_C_INV_T, const MatrixXd &P_C, bool init_trigger = true);

        /*
        Init model data with rbdl
        verbose 2 : Show all link Information
        verbose 1 : Show link id information
        verbose 0 : disable verbose
        */
        void InitModelData(std::string urdf_path, bool floating, int verbose);
        VectorXd CalcGravCompensation();
        void CalcGravCompensation(VectorXd &grav_torque);

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
    };

    template <typename... Types>
    void RobotData::SetContact(Types... args)
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
        CalcContactConstraint();
    }
}

#endif