#include "link.h"

using namespace DWBC;

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

MatrixXd Link::UpdateJacDot(RigidBodyDynamics::Model &model, const Eigen::VectorXd &q_virtual, const Eigen::VectorXd &q_dot_virtual, const Eigen::Vector3d &point_jac)
{

    Eigen::MatrixXd G;

    G.setZero(6, model.qdot_size);

    RigidBodyDynamics::Math::SpatialTransform point_trans =
        RigidBodyDynamics::Math::SpatialTransform(Matrix3d::Identity(), v);

    assert(G.rows() == 6 && G.cols() == model.qdot_size);

    unsigned int reference_body_id = link_id_;

    if (model.IsFixedBodyId(link_id_))
    {
        unsigned int fbody_id = link_id_ - model.fixed_body_discriminator;
        reference_body_id = model.mFixedBodies[fbody_id].mMovableParent;
    }

    unsigned int j = reference_body_id;

    while (j != 0)
    {
        unsigned int q_index = model.mJoints[j].q_index;

        if (model.mJoints[j].mJointType != RigidBodyDynamics::JointTypeCustom)
        {
            if (model.mJoints[j].mDoFCount == 1)
            {
                G.block(0, q_index, 6, 1) = point_trans.apply(
                                                           model.X_base[j].inverse().apply(
                                                               model.S[j]))
                                                .block<6, 1>(0, 0);
            }
            else if (model.mJoints[j].mDoFCount == 3)
            {
                G.block(0, q_index, 6, 3) = ((point_trans * model.X_base[j].inverse()).toMatrix() * model.multdof3_S[j]).block<6, 3>(0, 0);
            }
        }
        else
        {
            unsigned int k = model.mJoints[j].custom_joint_index;

            G.block(0, q_index, 6, model.mCustomJoints[k]->mDoFCount) = ((point_trans * model.X_base[j].inverse()).toMatrix() * model.mCustomJoints[k]->S).block(0, 0, 6, model.mCustomJoints[k]->mDoFCount);
        }

        j = model.lambda[j];
    }

    return G;
}
