#ifndef WBHQP_ROBOTDATA_HPP
#define WBHQP_ROBOTDATA_HPP

// #define EIGEN_DONT_VECTORIZE

#include <rbdl/addons/urdfreader/urdfreader.h>
#include <cstdarg>

#include "dwbc_link.h"
#include "dwbc_contact_constraint.h"
#include "dwbc_task.h"
#include "dwbc_wbd.h"
// #include "dwbc_util.h"

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
#include "dwbc_qp_wrapper.h"
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

    enum
    {
        ADD_LINK,
        ADD_LINK_WITH_FIXED_JOINT,
        ADD_LINK_WITH_REVOLUTE_JOINT,
        ADD_LINK_WITH_6DOF_JOINT,
        DELETE_LINK,
    };

    class RobotData
    {
    private:
        /* data */
    public:
        RobotData(/* args */);
        ~RobotData();

        // degree of freedom including floating base (robot dof + 6)
        unsigned int system_dof_;

        // degree of freedom of robot model (system dof - 6)
        unsigned int model_dof_;

        // degree of freedom of contact
        unsigned int contact_dof_;

        bool is_floating_;

        double total_mass_;

        double control_time_;

        RigidBodyDynamics::Model model_; // rbdl model of original model.

        RigidBodyDynamics::Model model_contact_; // rbdl model of contact model.
        RigidBodyDynamics::Model model_virtual_; // rbdl model of virtual model.

        Vector3d com_pos; // COM pos
        Vector3d com_vel; // COM vel

        MatrixXd J_com_; // COM jacobian ( also can be accesed from link.back().jac)

        // Mass matrix of Robot system_dof_ * system_dof_
        MatrixXd A_;
        MatrixXd A_inv_;

        // Non linear effect of Robot system_dof_ * 1
        VectorXd B_;

        /* Centroidal momentum matrix 3 * model_dof */
        MatrixXd CMM_;

        Vector3d ang_momentum_; // angular momentum

        VectorXd q_system_;      // q size           : (system_dof +1)
        VectorXd q_dot_system_;  // qdot size        : (system_dof)
        VectorXd q_ddot_system_; // qddot size       : (system_dof)

        VectorXd G_;              // gravity vector   : (system_dof)
        VectorXd torque_grav_;    // gravity torque   : (system_dof)
        VectorXd torque_task_;    // task torque      : (system_dof)
        VectorXd torque_contact_; // contact torque   : (system_dof)

        MatrixXd Lambda_contact; // contact space inertia matrix         : (contact_dof * contact_dof)
        MatrixXd J_C;            // contact jacobian                     : (contact_dof * system_dof)
        MatrixXd J_C_INV_T;      // contact jacobian inverse transpose   : (system_dof * contact_dof)
        MatrixXd N_C;            // null space of contact jacobian       : (system_dof * system_dof)

        MatrixXd P_C; // contact projection matrix

        MatrixXd W;     // W matrix
        MatrixXd W_inv; // W inverse matrix
        MatrixXd V2;    // V2 matrix

        MatrixXd NwJw; // NwJw matrix

        bool torque_limit_set_; // torque limit set flag

        VectorXd torque_limit_; // joint actuation limit information : (system_dof)

        std::vector<Link> link_;            // link information : contain all link data and the last link is center of mass information.
        std::vector<Joint> joint_;          // joint information : contain all joint data
        std::vector<ContactConstraint> cc_; // contact constraint information
        std::vector<TaskSpace> ts_;         // task space information
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

        void LoadModelData(std::string urdf_path, bool floating, int verbose = 0);

        void InitModelData(int verbose = 0);
        void InitModelWithLinkJoint(RigidBodyDynamics::Model& lmodel, std::vector<Link>& links, std::vector<Joint>& joints);

        void InitializeMatrix();
        /*
        Calculate Gravity Compensation
        */
        VectorXd CalcGravCompensation();
        void CalcGravCompensation(VectorXd &grav_torque);

        void SetTorqueLimit(const VectorXd &torque_limit); /* Set torque limit */

        void UpdateKinematics(const VectorXd q_virtual, const VectorXd q_dot_virtual, const VectorXd q_ddot_virtual, bool update_kinematics = true);
        void UpdateKinematics(bool update_kinematics = true);

        /*
        Add Contact constraint

        contact_type: 0: plane contact(6dof) 1: point contact(3dof) 2: line contact (1dof)
        */
        void AddContactConstraint(int link_number, int contact_type, Vector3d contact_point, Vector3d contact_vector, double contact_x = 0, double contact_y = 0, bool verbose = false);

        /*
        Add Contact constraint
        find contact link with link name. ignore the case of string
        */
        void AddContactConstraint(const char *link_name, int contact_type, Vector3d contact_point, Vector3d contact_vector, double contact_x = 0, double contact_y = 0, bool verbose = false);

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
        int CalcContactConstraint(bool update = true);

        /*
        Set Contact status, true or false
        */
        template <typename... Types>
        void SetContact(Types... args);

        /*
        Calculate Contact Redistribution Torque
        */
        int CalcContactRedistribute(VectorXd torque_input, bool init = true);
        int CalcContactRedistribute(bool init = true);

        /*
        Calculate Contact Force with command Torque
        */
        VectorXd getContactForce(const VectorXd &command_torque);

        /*
        Calculate Angular Momentum Matrix : (3 * system_dof)
        */
        MatrixXd CalcAngularMomentumMatrix();
        void CalcAngularMomentumMatrix(MatrixXd &cmm);
        /*
        Add Task Space
        */
        void AddTaskSpace(int task_mode, int task_dof, bool verbose = false);
        void AddTaskSpace(int task_mode, int link_number, Vector3d task_point, bool verbose = false);
        void AddTaskSpace(int task_mode, const char *link_name, Vector3d task_point, bool verbose = false); // find link with link name, ignore the case of string
        void ClearQP();
        void AddQP();
        /*
        Clear all task space
        */
        void ClearTaskSpace();

        /*
        Set fstar or jtask or specific task heirarchy
        */
        void SetTaskSpace(int heirarchy, const VectorXd &f_star, const MatrixXd &J_task = MatrixXd::Zero(1, 1));

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
        model modification
        */
        void DeleteLink(std::string link_name, bool verbose = false);
        void DeleteLink(int link_number, bool verbose = false);

        void AddLink(int parent_id, const char *link_name, int joint_type, const Vector3d &joint_axis, const Matrix3d &joint_rotm, const Vector3d &joint_trans, double body_mass, const Vector3d &com_position, const Matrix3d &inertia, bool verbose = false);

        /*
        joint_type :
            JOINT_FLOATING,
            JOINT_REVOLUTE,
            JOINT_PRISMATIC,
            JOINT_FIXED
        */
        void AddLink(const Joint &joint, const Link &link, bool verbose = false);
        // void AddLink(Link &link, int joint_type, const Vector3d &joint_axis, bool verbose = false);

        /*
        MODE :
            ADD_LINK,
            ADD_LINK_WITH_FIXED_JOINT,
            ADD_LINK_WITH_REVOLUTE_JOINT,
            ADD_LINK_WITH_FLOATING_JOINT,
            DELETE_LINK,
        */
        void InitAfterModelMod(int mode, int link_id, bool verbose = false);

        void ChangeLinkToFixedJoint(std::string link_name, bool verbose = false);

        void printLinkInfo();
        int getLinkID(std::string link_name);

        void CalcCOMInertia(std::vector<Link> &links, MatrixXd &com_inertia, VectorXd &com_momentum);

        VectorXd GetControlTorque(bool task_control = false, bool init = true);
        void CreateVModel(std::vector<Link> &links, std::vector<Joint> &joints);
        void UpdateVModel(RigidBodyDynamics::Model &vmodel, VectorXd &q_virtual, VectorXd &q_dot_virtual, VectorXd &q_ddot_virtual, std::vector<Link> &links, std::vector<Joint> &joints);
        void CalcVirtualInertia(RigidBodyDynamics::Model &vmodel, std::vector<Link> &links, std::vector<Joint> &joints, Matrix3d &new_inertia, Vector3d &new_com, double &new_mass);
        void ChangeLinkInertia(std::string link_name, Matrix3d &com_inertia, Vector3d &com_position, double com_mass, bool verbose = false);
        void CalcVirtualCMM(RigidBodyDynamics::Model v_model, std::vector<Link> &_link, Vector3d &com_pos, MatrixXd &cmm, bool verbose = false);
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

            int contact_dof = 0;
            for (auto n : v)
            {
                // std::cout << "setting " << link_[cc_[itr].link_number_].name_ << " contact : " << bool_cast(n) << std::endl;
                cc_[itr].SetContact(n);

                if (n == true)
                {
                    contact_dof += cc_[itr].contact_dof_;
                }

                itr++;
            }

            contact_dof_ = contact_dof;
            if (cc_.size() > v.size())
            {
                for (int i = v.size(); i < cc_.size(); i++)
                {
                    // std::cout << "setting " << link_[cc_[itr].link_number_].name_ << " contact : false" << std::endl;
                    cc_[itr++].SetContact(false);
                }
            }
        }
        CalcContactConstraint();
    }
}

#endif