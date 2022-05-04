#ifndef WBHQP_CONTACTCONSTRAINT_HPP
#define WBHQP_CONTACTCONSTRAINT_HPP

#include "rbdl/rbdl.h"
#include <iostream>
#include <Eigen/Dense>
#include "math.hpp"

using namespace Eigen;

#define CONTACT_CONSTRAINT_ZMP 4
#define CONTACT_CONSTRAINT_FORCE 6
#define CONTACT_CONSTRAINT_PRESS 0

enum
{
    CONTACT_6D,
    CONTACT_POINT,
};

class ContactConstraint
{
private:
    /* data */
public:
    ContactConstraint()
    {
    }

    ContactConstraint(RigidBodyDynamics::Model &md_, int link_number, int link_id, int contact_type, Vector3d contact_point, Vector3d contact_vector, double lx, double ly)
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
            constraint_number_ = CONTACT_CONSTRAINT_FORCE; //+ CONTACT_CONSTRAINT_PRESS;
        }

        contact_direction_ = contact_vector;

        contact_plane_x = lx;
        contact_plane_x = ly;

        j_contact.setZero(6, md_.qdot_size);

        SetFrictionRatio(0.2,0.2,0.2);
    }

    ~ContactConstraint()
    {
    }

    bool contact;

    Vector3d xc_pos;
    int link_id_;
    int link_number_;
    Matrix3d rotm;

    unsigned int contact_type_; // 0 : 6dof contact, 1 : point Contact
    unsigned int contact_dof_;
    unsigned int constraint_number_;

    std::vector<unsigned int> constraint_m_;

    Vector3d contact_point_;
    Vector3d contact_direction_;

    MatrixXd j_contact;

    double contact_plane_x;
    double contact_plane_y;

    double friction_ratio_x;
    double friction_ratio_y;
    double friction_ratio_z;

    void Update(RigidBodyDynamics::Model &model_, const VectorXd q_virtual_)
    {
        xc_pos = RigidBodyDynamics::CalcBodyToBaseCoordinates(model_, q_virtual_, link_id_, contact_point_, false);
        rotm = (RigidBodyDynamics::CalcBodyWorldOrientation(model_, q_virtual_, link_id_, false)).transpose();

        j_contact.setZero();
        RigidBodyDynamics::CalcPointJacobian6D(model_, q_virtual_, link_id_, contact_point_, j_contact, false);

        j_contact.topRows(3).swap(j_contact.bottomRows(3));
    }
    void SetContact(bool cont)
    {
        contact = cont;
    }

    void EnableContact()
    {
        contact = true;
    }

    void DisableContact()
    {
        contact = false;
    }

    void SetFrictionRatio(double x, double y, double z)
    {
        friction_ratio_x = x;
        friction_ratio_y = y;
        friction_ratio_z = z;
    }

    MatrixXd GetZMPConstMatrix4x6()
    {
        MatrixXd zmp_const_mat = MatrixXd::Zero(4, 6);

        zmp_const_mat(0, 2) = -contact_plane_x;
        zmp_const_mat(0, 4) = -1;

        zmp_const_mat(1, 2) = -contact_plane_x;
        zmp_const_mat(1, 4) = 1;

        zmp_const_mat(2, 2) = -contact_plane_y;
        zmp_const_mat(2, 3) = -1;

        zmp_const_mat(3, 2) = -contact_plane_y;
        zmp_const_mat(3, 3) = 1;


        return zmp_const_mat;
    }

    MatrixXd GetForceConstMatrix6x6()
    {

        MatrixXd force_const_matrix = MatrixXd::Zero(6, 6);

        force_const_matrix(0, 0) = 1.0;
        force_const_matrix(0, 2) = -friction_ratio_x;
        force_const_matrix(1, 0) = -1.0;
        force_const_matrix(1, 2) = -friction_ratio_x;

        force_const_matrix(2, 1) = 1.0;
        force_const_matrix(2, 2) = -friction_ratio_y;
        force_const_matrix(3, 1) = -1.0;
        force_const_matrix(3, 2) = -friction_ratio_y;

        force_const_matrix(4, 5) = 1.0;
        force_const_matrix(4, 2) = -friction_ratio_z;
        force_const_matrix(5, 5) = -1.0;
        force_const_matrix(5, 2) = -friction_ratio_z;

        return force_const_matrix;
    }

    MatrixXd GetContactConstMatrix()
    {
        MatrixXd cc_m_ = MatrixXd::Zero(CONTACT_CONSTRAINT_ZMP + CONTACT_CONSTRAINT_FORCE, 6);
        cc_m_.block(0,0,4,6) = GetZMPConstMatrix4x6();
        cc_m_.block(4,0,6,6) = GetForceConstMatrix6x6();

        return cc_m_;
        

        // int total_const = 0;
        // for(int i=0;i<constraint_m_.size();i++)
        // {
        //     total_const += constraint_m_[i];
        // }

        // MatrixXd cc_m_ = MatrixXd::Zero(total_const,6);

        // cc_m_.block(0,0,constraint_m_[i],6 ) = GetZMPConstMatrix4x6();
        // cc_m_.block(constraint_m_[i - i], 0, constraint_m_[i])
        // MatrixXd contact_const_m_ = MatrixXd::Zero()
    }
};

#endif