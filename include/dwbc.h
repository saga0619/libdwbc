#ifndef WBHQP_ROBOTDATA_HPP
#define WBHQP_ROBOTDATA_HPP

#include <rbdl/addons/urdfreader/urdfreader.h>
#include <cstdarg>

#include "link.h"
#include "contact_constraint.h"
#include "task.h"
#include "wbd.h"
#include "util.h"

using namespace Eigen;

#ifdef COMPILE_QPSWIFT
#define INFTY 1.0E+10
#include <qpSWIFT/qpSWIFT.h>
class TaskSpaceQP : public DWBC::TaskSpace
{
public:
    QP *qp_;

    TaskSpaceQP(int task_mode, int heirarchy, int task_dof, int model_dof)
        : TaskSpace(task_mode, heirarchy, task_dof, model_dof) {}

    TaskSpaceQP(int task_mode, int heirarchy, int link_number, int link_id, const Vector3d &task_point, int model_dof)
        : TaskSpace(task_mode, heirarchy, link_number, link_id, task_point, model_dof) {}
};
#else
#include "qp_wrapper.h"
// class TaskSpaceQP : public DWBC::TaskSpace
// {
// public:
//     DWBC::CQuadraticProgram qp_;

//     TaskSpaceQP(int task_mode, int heirarchy, int task_dof, int model_dof)
//         : DWBC::TaskSpace(task_mode, heirarchy, task_dof, model_dof) {}

//     TaskSpaceQP(int task_mode, int heirarchy, int link_number, int link_id, const Vector3d &task_point, int model_dof)
//         : DWBC::TaskSpace(task_mode, heirarchy, link_number, link_id, task_point, model_dof) {}
// };
#endif

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

        Vector3d com_pos; //COM pos
        Vector3d com_vel; //COM vel

        MatrixXd J_com_; //COM jacobian 

        MatrixXd A_;
        MatrixXd A_inv_;

        VectorXd B_;

        MatrixXd CMM_; /*Centroidal momentum matrix*/

        VectorXd q_system_;
        VectorXd q_dot_system_;
        VectorXd q_ddot_system_;

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

        bool torque_limit_set_;

        bool save_mat_file_;
        bool check_mat_file_;

        VectorXd torque_limit_;

        std::vector<Link> link_;
        std::vector<ContactConstraint> cc_;
        std::vector<TaskSpace> ts_;
#ifdef COMPILE_QPSWIFT
        std::vector<QP *> qp_task_;
        QP *qp_contact_;
#else
        std::vector<CQuadraticProgram> qp_task_;
        CQuadraticProgram qp_contact_;
#endif

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

        void SetTorqueLimit(const VectorXd &torque_limit); /* Set torque limit */

        void UpdateKinematics(const VectorXd q_virtual, const VectorXd q_dot_virtual, const VectorXd q_ddot_virtual, bool update_kinematics = true);

        /*
        Add Contact constraint

        contact_type: 0: plane contact(6dof) 1: point contact(3dof) 2: line contact (1dof)
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
        int CalcContactRedistribute(bool init = true);

        /*
        Calculate Contact Force with command Torque
        */
        VectorXd getContactForce(const VectorXd &command_torque);

        /*
        Add Task Space
        */
        void AddTaskSpace(int task_mode, int task_dof, bool verbose = false);
        void AddTaskSpace(int task_mode, int link_number, Vector3d task_point, bool verbose = false);
        void ClearQP();
        void AddQP();
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
         if update == true, automatically update TaskSpace Dynamics ( DWBC::RobotData::UpdateTaskSpace(); )
        */
        void CalcTaskSpace(bool update = true);

        /*
        Calculate Heirarcical task torque.
        */
        int CalcTaskControlTorque(bool init, bool hqp = true, bool update_task_space = true);

        /*
        Calculate heirarchy task torque
        */
        int CalcSingleTaskTorqueWithQP(TaskSpace &ts_, const MatrixXd &task_null_matrix_, const VectorXd &torque_prev, const MatrixXd &NwJw, const MatrixXd &J_C_INV_T, const MatrixXd &P_C, bool init_trigger = true);

        void CalcTaskSpaceTorqueHQPWithThreaded(bool init);

        void CopyKinematicsData(RobotData &target_rd);

        /*

        */

        VectorXd GetControlTorque(bool task_control = false, bool init = true);
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