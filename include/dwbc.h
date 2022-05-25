#ifndef WBHQP_ROBOTDATA_HPP
#define WBHQP_ROBOTDATA_HPP

#include <rbdl/addons/urdfreader/urdfreader.h>
#include <cstdarg>

#include "link.h"
#include "contact_constraint.h"
#include "task.h"
#include "wbd.h"

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

        /*
        Clear all stored ContactConstraint
        */
        void ClearContactConstraint();

        /*
        Update Conact space information
        */
        void UpdateContactConstraint();

        /*
        Calculate Contact Constraint Dynamics
        bool update : Call UpdateContactConstraint()
        */
        void CalcContactConstraint(bool update = true);

        /*
        Set Contact status, true or false
        */
        template <typename... Types>
        void SetContact(Types... args);

        /*
        Calculate Contact Redistribution Torque
        */
        void CalcContactRedistribute(bool init = true);

        /*
        Calculate Contact Force with command Torque
        */
        VectorXd getContactForce(const VectorXd &command_torque);

        /*
        Add Task Space
        */
        void AddTaskSpace(int task_mode, int task_dof, bool verbose = false);
        void AddTaskSpace(int task_mode, int link_number, Vector3d task_point, bool verbose = false);

        /*
        Clear all task space
        */
        void ClearTaskSpace();

        /*
        Set fstar or jtask or specific task heirarchy
        */
        void SetTaskSpace(int heirarchy, const MatrixXd &f_star, const MatrixXd &J_task = MatrixXd::Zero(1, 1));

        /*
        Update Task Space information
        */
        void UpdateTaskSpace();

        /*
        Calculate Task Space dynamics
        */
        void CalcTaskSpace(bool update = true);

        /*
        Calculate Heirarcical task torque.
        */
        void CalcTaskTorque(bool hqp = true, bool init = true);

        /*
        Calculate heirarchy task torque
        */
        void CalcTaskTorqueQP(TaskSpace &ts_, const MatrixXd &task_null_matrix_, const VectorXd &torque_limit, const VectorXd &torque_prev, const MatrixXd &NwJw, const MatrixXd &J_C_INV_T, const MatrixXd &P_C, bool init_trigger = true);

        /*
        Init model data with rbdl
        verbose 2 : Show all link Information
        verbose 1 : Show link id information
        verbose 0 : disable verbose
        */
        void InitModelData(std::string urdf_path, bool floating, int verbose);

        /*
        Calculate Gravity Compensation
        */
        VectorXd CalcGravCompensation();
        void CalcGravCompensation(VectorXd &grav_torque);
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
                // std::cout << "setting " << link_[cc_[itr].link_number_].name_ << " contact : " << bool_cast(n) << std::endl;
                cc_[itr++].SetContact(n);
            }

            if (cc_.size() > v.size())
            {
                for (int i = v.size(); i < cc_.size(); i++)
                {
                    // std::cout << "setting " << link_[cc_[itr].link_number_].name_ << " contact : false" << std::endl;
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