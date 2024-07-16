#ifndef WBHQP_CONTACTCONSTRAINT_HPP
#define WBHQP_CONTACTCONSTRAINT_HPP

#include "rbdl/rbdl.h"
#include <iostream>
#include <Eigen/Dense>
#include "dwbc_math.h"
#include "dwbc_wbd.h"

using namespace Eigen;

#define CONTACT_CONSTRAINT_ZMP 4
#define CONTACT_CONSTRAINT_FORCE 6
#define CONTACT_CONSTRAINT_PRESS 3

namespace DWBC
{

    enum CONTACT_TYPE
    {
        CONTACT_6D,
        CONTACT_POINT,
        CONTACT_LINK,
        CONTACT_LINE,
    };

    class ContactConstraint
    {
    private:
        /* data */
    public:
        bool contact;

        Vector3d xc_pos;
        int rbdl_body_id_; 
        int link_number_;
        Matrix3d rotm;

        Vector3d zmp_pos;

        unsigned int contact_type_; // 0 : 6dof contact, 1 : point Contact
        unsigned int contact_dof_;
        unsigned int constraint_number_;

        std::vector<unsigned int> constraint_m_;

        Vector3d contact_point_;
        Vector3d contact_direction_;

        MatrixXd j_contact;

        double contact_plane_x_;
        double contact_plane_y_;

        double friction_ratio_;
        double friction_ratio_z_;

        bool fz_limiter_switch_;
        // int fz_limiter_size_;
        double fz_limit_;

        ContactConstraint();
        ContactConstraint(RigidBodyDynamics::Model &md_, int link_number, int link_id, int contact_type, Vector3d contact_point, Vector3d contact_vector, double contact_plate_size_x, double contact_plate_size_y);

        ~ContactConstraint();

        void Update(RigidBodyDynamics::Model &model_, const VectorXd q_virtual_);
        void SetContact(bool cont);

        void EnableContact();

        void DisableContact();
        void SetFrictionRatio(double friction_ratio, double friction_ratio_z);
        MatrixXd GetZMPConstMatrix4x6();

        MatrixXd GetForceConstMatrix6x6();
        MatrixXd GetContactConstMatrix();

        MatrixXd GetPressConstMatrix3x6();
    };
}

#endif