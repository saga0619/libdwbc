#ifndef WBHQP_LINK_H
#define WBHQP_LINK_H

#include "rbdl/rbdl.h"
#include <iostream>
#include <Eigen/Dense>
#include "dwbc_math.h"

using namespace Eigen;

namespace DWBC
{   
    enum
    {
        JOINT_FLOATING_BASE,
        JOINT_6DOF,
        JOINT_REVOLUTE,
        JOINT_PRISMATIC,
        JOINT_FIXED
    };

    class Joint
    {
    public:
        Joint();
        Joint(int joint_type);
        Joint(int joint_type, const Vector3d &joint_axis);
        int joint_type_;
        Vector3d joint_axis_;

        Vector3d joint_translation_;
        Matrix3d joint_rotation_;

        Vector3d parent_translation_;
        Matrix3d parent_rotation_;

        RigidBodyDynamics::Joint ToRBDLJoint();
    };

    class Link
    {
    public:
        Link();
        Link(RigidBodyDynamics::Model &model_, const unsigned int link_id);
        ~Link();

        std::string name_;

        MatrixXd jac_;

        MatrixXd jac_com_;

        Vector3d xpos;
        Vector3d xipos;
        Vector3d v;
        Vector3d vi;
        Vector3d w;
        Matrix3d rotm;

        Vector3d joint_trans;
        Matrix3d joint_rotm;

        Vector3d parent_trans;
        Matrix3d parent_rotm;

        RigidBodyDynamics::Math::SpatialVector vw;

        Vector3d com_position_l_;
        Matrix3d inertia;

        // id of body from rbdl
        int body_id_;

        // id of parent link
        int parent_id_;

        std::vector<int> child_id_;

        // id of current link
        int link_id_;

        double mass;

        void UpdateAll(RigidBodyDynamics::Model &model_, const Eigen::VectorXd &q_virtual_, const Eigen::VectorXd &q_dot_virtual_);

        /*
        Update Position and Velocity data of link
        xpos, rotm, v, w
        */
        void UpdatePos(RigidBodyDynamics::Model &model_, const Eigen::VectorXd &q_virtual_, const Eigen::VectorXd &q_dot_virtual_);

        /*
        Upate Jacobian and COM jacobian of link
        */
        void UpdateJac(RigidBodyDynamics::Model &model_, const Eigen::VectorXd &q_virtual_);

        /*
        Get Jacobian of Specific position
        */
        MatrixXd GetPointJac(RigidBodyDynamics::Model &model_, const Eigen::VectorXd &q_virtual_, const Eigen::Vector3d &point);

        // /*
        // Get Position of
        // */
        void GetPointPos(RigidBodyDynamics::Model &model_, const Eigen::VectorXd &q_virtual_, const Eigen::VectorXd &q_dot_virtual_, Eigen::Vector3d &local_pos, Eigen::Vector3d &global_pos);

        Quaterniond GetQuat();

        // void UpdateJacDot(RigidBodyDynamics::Model &model_, const Eigen::VectorXd &q_virtual, const Eigen::VectorXd &q_dot_virtual);
        MatrixXd GetJacDot(RigidBodyDynamics::Model &model_, const Eigen::VectorXd &q_virtual, const Eigen::VectorXd &q_dot_virtual, const Eigen::Vector3d &point_jac);

        // Get spatial transform matrix 6x6 from link to base, Matrix * local_spacial_vector = global_spatial_vector
        MatrixXd GetSpatialTranform();

        // Get adjoint matrix 6x6 from link to base, Matrix * local_spacial_vector = global_spatial_vector
        MatrixXd GetAdjointMatrix();

        MatrixXd GetSpatialInertiaMatrix();

        RigidBodyDynamics::Body ToRBDLBody();


        // Combine two link. Add link_add to current link. rotm and xpos is the position of link_add in current link frame
        void AddLink(Link &link_add, Matrix3d &rotm, Vector3d &xpos);

        // Print Link Data
        void Print();
    };
}

#endif