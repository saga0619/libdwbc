#ifndef WBHQP_LINK_HPP
#define WBHQP_LINK_HPP

#include "rbdl/rbdl.h"
#include <iostream>
#include <Eigen/Dense>
#include "math.hpp"

using namespace Eigen;

static MatrixXd j3_temp;

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

Link::Link(/* args */)
{
}

Link::Link(RigidBodyDynamics::Model &model_, const unsigned int link_id)
{
    link_id_ = link_id;
    inertia = model_.mBodies[link_id_].mInertia;
    mass = model_.mBodies[link_id_].mMass;
    com_position_l_ = model_.mBodies[link_id_].mCenterOfMass;
    jac_.setZero(6, model_.qdot_size);
    jac_com_.setZero(6, model_.qdot_size);
    name_ = model_.GetBodyName(link_id);
}

Link::~Link()
{
}

void Link::UpdateAll(RigidBodyDynamics::Model &model_, const Eigen::VectorXd &q_virtual_, const Eigen::VectorXd &q_dot_virtual_)
{
    UpdatePos(model_, q_virtual_, q_dot_virtual_);
    UpdateJac(model_, q_virtual_);
}

void Link::UpdatePos(RigidBodyDynamics::Model &model_, const Eigen::VectorXd &q_virtual_, const Eigen::VectorXd &q_dot_virtual_)
{
    xpos = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_virtual_, link_id_, Eigen::Vector3d::Zero(), false);
    xipos = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_virtual_, link_id_, model_.mBodies[link_id_].mCenterOfMass, false);
    rotm = (RigidBodyDynamics::CalcBodyWorldOrientation(model_, q_virtual_, link_id_, false)).transpose();
    vw = RigidBodyDynamics::CalcPointVelocity6D(model_, q_virtual_, q_dot_virtual_, link_id_, Eigen::Vector3d::Zero(), false);

    v = vw.segment(3, 3);
    w = vw.segment(0, 3);
}

void Link::UpdateJac(RigidBodyDynamics::Model &model_, const Eigen::VectorXd &q_virtual_)
{
    jac_.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, link_id_, Eigen::Vector3d::Zero(), jac_, false);
    jac_.topRows(3).swap(jac_.bottomRows(3));

    jac_com_.setZero();
    RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, link_id_, com_position_l_, jac_com_, false);
    jac_com_.topRows(3).swap(jac_com_.bottomRows(3));
}

MatrixXd Link::GetPointJac(RigidBodyDynamics::Model &model_, const Eigen::VectorXd &q_virtual_, const Eigen::Vector3d &point_jac)
{
    MatrixXd j_temp;
    j_temp.setZero(6, model_.qdot_size);

    RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, link_id_, point_jac, j_temp, false);

    j_temp.topRows(3).swap(j_temp.bottomRows(3));

    return j_temp;
}

#endif