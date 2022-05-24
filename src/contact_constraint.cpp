#include "contact_constraint.h"

namespace DWBC
{
    ContactConstraint::ContactConstraint()
    {
    }

    ContactConstraint::ContactConstraint(RigidBodyDynamics::Model &md_, int link_number, int link_id, int contact_type, Vector3d contact_point, Vector3d contact_vector, double lx, double ly)
    {
        contact_point_ = contact_point;

        contact_type_ = contact_type;

        link_number_ = link_number;

        link_id_ = link_id;

        if (contact_type == CONTACT_6D)
        {
            contact_dof_ = 6;
            constraint_number_ = CONTACT_CONSTRAINT_ZMP + CONTACT_CONSTRAINT_FORCE; // + CONTACT_CONSTRAINT_PRESS;
        }
        else if (contact_type == CONTACT_POINT)
        {
            contact_dof_ = 3;
            constraint_number_ = CONTACT_CONSTRAINT_FORCE; // + CONTACT_CONSTRAINT_PRESS;
        }

        contact_direction_ = contact_vector;

        contact_plane_x = lx;
        contact_plane_x = ly;

        j_contact.setZero(6, md_.qdot_size);

        SetFrictionRatio(0.2, 0.2, 0.2);
    }

    ContactConstraint::~ContactConstraint()
    {
    }

    void ContactConstraint::Update(RigidBodyDynamics::Model &model_, const VectorXd q_virtual_)
    {
        xc_pos = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_virtual_, link_id_, contact_point_, false);
        rotm = (RigidBodyDynamics::CalcBodyWorldOrientation(model_, q_virtual_, link_id_, false)).transpose();

        j_contact.setZero();
        RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, link_id_, contact_point_, j_contact, false);

        j_contact.topRows(3).swap(j_contact.bottomRows(3));
    }
    void ContactConstraint::SetContact(bool cont)
    {
        contact = cont;
    }

    void ContactConstraint::EnableContact()
    {
        contact = true;
    }

    void ContactConstraint::DisableContact()
    {
        contact = false;
    }

    void ContactConstraint::SetFrictionRatio(double x, double y, double z)
    {
        friction_ratio_x = x;
        friction_ratio_y = y;
        friction_ratio_z = z;
    }

    MatrixXd ContactConstraint::GetZMPConstMatrix4x6()
    {
        return GetZMPConstMatrix(contact_plane_x, contact_plane_y);
    }

    MatrixXd ContactConstraint::GetForceConstMatrix6x6()
    {
        return GetForceConstMatrix(friction_ratio_x, friction_ratio_y, friction_ratio_z);
    }

    MatrixXd ContactConstraint::GetContactConstMatrix()
    {
        MatrixXd cc_m_ = MatrixXd::Zero(CONTACT_CONSTRAINT_ZMP + CONTACT_CONSTRAINT_FORCE, 6);
        cc_m_.block(0, 0, 4, 6) = GetZMPConstMatrix4x6();
        cc_m_.block(4, 0, 6, 6) = GetForceConstMatrix6x6();

        return cc_m_;
    }

    MatrixXd ContactConstraint::GetPressConstMatrix3x6()
    {
        return MatrixXd::Zero(3, 6);
    }
}