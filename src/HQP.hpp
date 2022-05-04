#include "task.hpp"
#include "robotdata.hpp"
#include "qp_wrapper.h"
#include "math.hpp"

class HQP
{
private:
    /* data */
public:
    HQP(/* args */);
    ~HQP();

    std::vector<Task> tl_;
    std::vector<ContactConstraint> cl_;

    unsigned int model_dof;

    MatrixXd A_;
    MatrixXd A_INV;

    MatrixXd J_C;
    MatrixXd J_C_INV_T;
    MatrixXd Lambda_C;

    MatrixXd N_C;

    MatrixXd W;

    MatrixXd W_INV;

    MatrixXd V2;

    VectorXd G_;

    MatrixXd NwJw;

    VectorXd torque_gravity;

    VectorXd torque_contact;

    // Calc WBC Data
    // JKT and Task Lambda matrix
    void CalcWBC();

    // calculate QP optimization
    void CalcQP();

    unsigned int task_dof;
    unsigned int contact_dof;
};

HQP::HQP(/* args */)
{
}

HQP::~HQP()
{
}

// void InitContactConstraint(HQP &hqp, const unsigned int number, const unsigned int link_id, const double contact_plane_x, const double contact_plane_y, const Vector3d local_contact_point, const Vector3d contact_direction)
void InitContactConstraint(HQP &hqp, const unsigned int number, const unsigned int contact_type, const double contact_plane_x, const double contact_plane_y, const Vector3d local_contact_point, const Vector3d contact_direction)
{
    hqp.cl_[number].contact_point_ = local_contact_point;

    hqp.cl_[number].contact_plane_x = contact_plane_x;
    hqp.cl_[number].contact_plane_y = contact_plane_y;

    hqp.cl_[number].contact_type_ = contact_type;

    if (hqp.cl_[number].contact_type_ == 0)
    {
        hqp.cl_[number].contact_dof = 6;
    }
}

void UpdateMassMatrix(HQP &hqp, const MatrixXd &A_)
{
    hqp.A_ = A_;

    hqp.A_INV = hqp.A_.inverse();
}

void UpdateContactConstraint(HQP &hqp)
{
    hqp.Lambda_C = (hqp.J_C * hqp.A_INV * hqp.J_C.transpose()).inverse();

    hqp.J_C_INV_T = hqp.Lambda_C * hqp.J_C * hqp.A_INV;
}

void CalcContactNull(HQP &hqp)
{
    hqp.W = hqp.A_INV.bottomRows(hqp.model_dof);

    PinvQR(hqp.W, hqp.W_INV, hqp.V2);

    hqp.NwJw = hqp.V2.transpose() * (hqp.J_C_INV_T.transpose().topRightCorner(hqp.contact_dof, hqp.model_dof) * hqp.V2.transpose()).inverse();
    // PinvQR()
}

void InitTaskSpace(HQP &hqp, const unsigned int heirarchy, const unsigned int task_dof)
{
    hqp.tl_[heirarchy].task_dof = task_dof;
}

void UpdateGravityVector(HQP &hqp, const MatrixXd &G)
{
    hqp.G_ = G;
}

void calculateGravityCompensationTorque(HQP &hqp, const MatrixXd &G)
{
    hqp.torque_gravity = (hqp.W_INV * (hqp.A_INV.bottomRows(hqp.model_dof) * (hqp.N_C * hqp.G_)));
}

bool UpdateTask(HQP &hqp, unsigned int heirarchy, const MatrixXd &J_task, const VectorXd &f_star)
{
    if (hqp.tl_.size() < heirarchy)
    {
        return false;
    }

    hqp.tl_[heirarchy].J_task_ = J_task;
    hqp.tl_[heirarchy].f_star_ = f_star;
}

bool calcTaskOSWBC(HQP &hqp, unsigned int heirarchy)
{
    hqp.tl_[heirarchy].Lambda_task_ = hqp.tl_[heirarchy].J_task_ * hqp.A_INV * hqp.N_C * hqp.tl_[heirarchy].J_task_.transpose();

    hqp.tl_[heirarchy].Q_ = hqp.tl_[heirarchy].Lambda_task_ * hqp.tl_[heirarchy].J_task_ * hqp.A_INV * hqp.N_C.rightCols(hqp.model_dof);

    hqp.tl_[heirarchy].J_kt_ = hqp.W_INV * hqp.tl_[heirarchy].Q_.transpose() * PinvQR(hqp.tl_[heirarchy].Q_ * hqp.W_INV * hqp.tl_[heirarchy].Q_.transpose());
}

bool calcTaskHQP(HQP &hqp, unsigned int heirarchy)
{
}

void AddTask(const HQP &hqp, const MatrixXd &J_task, const VectorXd &f_star)
{
    // hqp.task_.push_back({.J_task = J_task, .f_star = f_star});
}



// void InitTaskSpace(HQP &hqp, const unsigned int heirarchy, const unsigned int link_id, const Vector3d local_task_point)

//////////////////////

/*




*/
