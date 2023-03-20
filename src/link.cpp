#include "dwbc_link.h"

namespace DWBC
{

    Link::Link(/* args */)
    {
    }

    Link::Link(RigidBodyDynamics::Model &model_, const unsigned int body_id)
    {
        body_id_ = body_id;
        inertia = model_.mBodies[body_id_].mInertia;
        mass = model_.mBodies[body_id_].mMass;
        com_position_l_ = model_.mBodies[body_id_].mCenterOfMass;
        jac_.setZero(6, model_.qdot_size);
        jac_com_.setZero(6, model_.qdot_size);
        name_ = model_.GetBodyName(body_id_);
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
        xpos = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_virtual_, body_id_, Eigen::Vector3d::Zero(), false);
        xipos = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_virtual_, body_id_, model_.mBodies[body_id_].mCenterOfMass, false);
        rotm = (RigidBodyDynamics::CalcBodyWorldOrientation(model_, q_virtual_, body_id_, false)).transpose();
        vw = RigidBodyDynamics::CalcPointVelocity6D(model_, q_virtual_, q_dot_virtual_, body_id_, Eigen::Vector3d::Zero(), false);

        v = vw.segment(3, 3);
        w = vw.segment(0, 3);
    }

    void Link::UpdateJac(RigidBodyDynamics::Model &model_, const Eigen::VectorXd &q_virtual_)
    {
        jac_.setZero();
        RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, body_id_, Eigen::Vector3d::Zero(), jac_, false);
        jac_.topRows(3).swap(jac_.bottomRows(3));

        jac_com_.setZero();
        RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, body_id_, com_position_l_, jac_com_, false);
        jac_com_.topRows(3).swap(jac_com_.bottomRows(3));
    }

    MatrixXd Link::GetPointJac(RigidBodyDynamics::Model &model_, const Eigen::VectorXd &q_virtual_, const Eigen::Vector3d &point_jac)
    {
        MatrixXd j_temp;
        j_temp.setZero(6, model_.qdot_size);

        RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, body_id_, point_jac, j_temp, false);

        j_temp.topRows(3).swap(j_temp.bottomRows(3));

        return j_temp;
    }

    Quaterniond Link::GetQuat()
    {
        return Quaterniond(rotm);
    }

    void Link::GetPointPos(RigidBodyDynamics::Model &model_, const Eigen::VectorXd &q_virtual_, const Eigen::VectorXd &q_dot_virtual_, Eigen::Vector3d &local_pos, Eigen::Vector3d &global_pos)
    {
        global_pos = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_virtual_, body_id_, local_pos, false);
    }

    MatrixXd Link::GetJacDot(RigidBodyDynamics::Model &_model, const Eigen::VectorXd &q_virtual, const Eigen::VectorXd &q_dot_virtual, const Eigen::Vector3d &point_jac)
    {

        Eigen::MatrixXd G;

        G.setZero(6, _model.qdot_size);

        RigidBodyDynamics::Math::SpatialTransform point_trans =
            RigidBodyDynamics::Math::SpatialTransform(Matrix3d::Identity(), v);

        assert(G.rows() == 6 && G.cols() == _model.qdot_size);

        unsigned int reference_body_id = body_id_;

        if (_model.IsFixedBodyId(body_id_))
        {
            unsigned int fbody_id = body_id_ - _model.fixed_body_discriminator;
            reference_body_id = _model.mFixedBodies[fbody_id].mMovableParent;
        }

        unsigned int j = reference_body_id;

        while (j != 0)
        {
            unsigned int q_index = _model.mJoints[j].q_index;

            if (_model.mJoints[j].mJointType != RigidBodyDynamics::JointTypeCustom)
            {
                if (_model.mJoints[j].mDoFCount == 1)
                {
                    G.block(0, q_index, 6, 1) = point_trans.apply(
                                                               _model.X_base[j].inverse().apply(
                                                                   _model.S[j]))
                                                    .block<6, 1>(0, 0);
                }
                else if (_model.mJoints[j].mDoFCount == 3)
                {
                    G.block(0, q_index, 6, 3) = ((point_trans * _model.X_base[j].inverse()).toMatrix() * _model.multdof3_S[j]).block<6, 3>(0, 0);
                }
            }
            else
            {
                unsigned int k = _model.mJoints[j].custom_joint_index;

                G.block(0, q_index, 6, _model.mCustomJoints[k]->mDoFCount) = ((point_trans * _model.X_base[j].inverse()).toMatrix() * _model.mCustomJoints[k]->S).block(0, 0, 6, _model.mCustomJoints[k]->mDoFCount);
            }

            j = _model.lambda[j];
        }

        return G;
    }

}