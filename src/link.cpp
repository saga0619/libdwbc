#include "dwbc_link.h"

namespace DWBC
{
    Joint::Joint()
    {
        joint_type_ = -1;
    }

    Joint::Joint(int joint_type)
    {
        joint_type_ = joint_type;
    }

    Joint::Joint(int joint_type, const Vector3d &joint_axis)
    {
        joint_type_ = joint_type;
        joint_axis_ = joint_axis;
    }

    RigidBodyDynamics::Joint Joint::ToRBDLJoint()
    {
        if (joint_type_ == JOINT_REVOLUTE)
        {
            return RigidBodyDynamics::Joint(RigidBodyDynamics::JointTypeRevolute, joint_axis_);
        }
        else
        {
            // cout undefined joint type
            std::cout << "joint type not defined" << std::endl;

            return RigidBodyDynamics::Joint();
        }
    }

    Link::Link(/* args */)
    {
        mass = -1;
        body_id_ = -1;
    }

    Link::Link(RigidBodyDynamics::Model &model_, const unsigned int body_id)
    {
        body_id_ = body_id;
        inertia = model_.mBodies[body_id_].mInertia;
        mass = model_.mBodies[body_id_].mMass;
        com_position_l_ = model_.mBodies[body_id_].mCenterOfMass;
        name_ = model_.GetBodyName(body_id_);

        joint_rotm = model_.X_T[body_id_].E.transpose();
        joint_trans = model_.X_T[body_id_].r;
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

        // std::cout << name_ << std::endl;
        // std::cout << parent_rotm << std::endl;
        // std::cout << parent_trans << std::endl;
    }

    void Link::UpdateJac(RigidBodyDynamics::Model &model_, const Eigen::VectorXd &q_virtual_)
    {
        jac_.setZero(6, model_.qdot_size);
        RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, body_id_, Eigen::Vector3d::Zero(), jac_, false);
        jac_.topRows(3).swap(jac_.bottomRows(3));

        jac_com_.setZero(6, model_.qdot_size);
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

    MatrixXd Link::GetSpatialTranform()
    {
        return spatialTransformMatrix(xpos, rotm);
    }

    MatrixXd Link::GetAdjointMatrix()
    {
        MatrixXd adjoint = MatrixXd::Zero(6, 6);

        adjoint.block(0, 0, 3, 3) = rotm;
        adjoint.block(3, 3, 3, 3) = rotm;
        adjoint.block(3, 0, 3, 3) = skew(xpos) * rotm;

        return adjoint;
    }

    MatrixXd Link::GetSpatialInertiaMatrix()
    {
        MatrixXd i_temp = MatrixXd::Zero(6, 6);

        i_temp.block(0, 0, 3, 3) = inertia + mass * skew(com_position_l_) * skew(com_position_l_).transpose();
        i_temp.block(0, 3, 3, 3) = mass * skew(com_position_l_);
        i_temp.block(3, 0, 3, 3) = mass * skew(com_position_l_).transpose();
        i_temp.block(3, 3, 3, 3) = mass * Matrix3d::Identity();

        return i_temp;
    }
    using namespace std;

    void Link::Print()
    {
        cout << "link name : " << name_ << endl;
        cout << "rbdl id : " << body_id_ << endl;
        cout << "link mass : " << mass << endl;
        cout << "link com : " << com_position_l_.transpose() << endl;
        cout << "link inertia : " << inertia << endl;
        // cout << "link xpos : " << xpos << endl;
        // cout << "link rotm : " << rotm << endl;
        cout << "joint trans : " << joint_trans.transpose() << endl;
        cout << "join rotm : " << joint_rotm << endl;

        cout << "parent trans : " << parent_trans.transpose() << endl;
        cout << "parent torm : " << parent_rotm << endl;
    }

    RigidBodyDynamics::Body Link::ToRBDLBody()
    {
        if (mass < 0)
        {
            std::cout << "error : mass not defined" << std::endl;
        }
        return RigidBodyDynamics::Body(mass, com_position_l_, (RigidBodyDynamics::Math::Matrix3d)inertia);
    }

    void Link::AddLink(Link &other_link, Matrix3d &rotm, Vector3d &xpos)
    {
        double new_mass = mass + other_link.mass;



        Vector3d other_com = rotm * other_link.com_position_l_ + xpos;
        Vector3d new_com = (mass * com_position_l_ + other_link.mass * other_com) / new_mass;

        Matrix3d com_skm = skew(other_link.com_position_l_);
        Matrix3d inertia_other = other_link.inertia + other_link.mass * com_skm * com_skm.transpose(); 

        Matrix3d com_skm_this = skew(com_position_l_);
        Matrix3d inertia_this = inertia + mass * com_skm_this * com_skm_this.transpose(); 

        Matrix3d com_skm_other = skew(other_com);

        Matrix3d inertia_other_com_rotated_this_origin = rotm * other_link.inertia * rotm.transpose() + other_link.mass * com_skm_other * com_skm_other.transpose();
        Matrix3d inertia_summed = inertia_this + inertia_other_com_rotated_this_origin;
        Matrix3d new_inertia = inertia_summed - new_mass * skew(new_com) * skew(new_com).transpose();
        
        mass = new_mass;
        com_position_l_ = new_com;
        inertia = new_inertia;
    }

}