#ifndef WBHQP_LINK_H
#define WBHQP_LINK_H

#include "rbdl/rbdl.h"
#include <iostream>
#include <Eigen/Dense>
#include "math.h"

using namespace Eigen;

namespace DWBC
{
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


        RigidBodyDynamics::Math::SpatialVector vw;

        Vector3d com_position_l_;
        Matrix3d inertia;

        // id of body from rbdl
        unsigned int body_id_;

        // id of parent link
        unsigned int parent_id_;

        std::vector<unsigned int> child_id_;

        // id of current link
        unsigned int link_id_;

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

    };
}

#endif