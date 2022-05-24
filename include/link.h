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
    private:
        /* data */
    public:
        Link(/* args */);
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

        unsigned int link_id_;

        double mass;

        void UpdateAll(RigidBodyDynamics::Model &model_, const Eigen::VectorXd &q_virtual_, const Eigen::VectorXd &q_dot_virtual_);

        void UpdatePos(RigidBodyDynamics::Model &model_, const Eigen::VectorXd &q_virtual_, const Eigen::VectorXd &q_dot_virtual_);

        void UpdateJac(RigidBodyDynamics::Model &model_, const Eigen::VectorXd &q_virtual_);

        MatrixXd GetPointJac(RigidBodyDynamics::Model &model_, const Eigen::VectorXd &q_virtual_, const Eigen::Vector3d &point);
    };
}

#endif