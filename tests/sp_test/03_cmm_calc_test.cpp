#include <chrono>
#include "dwbc.h"
#include <unistd.h>
#include <iomanip>

#ifndef URDF_DIR
#define URDF_DIR ""
#endif

#undef COMPARISON_EPS
#define COMPARISON_EPS 1.0E-12

using namespace DWBC;

int main(void)
{
    srand((unsigned int)time(0));
    bool verbose = false;
    int repeat_time = 1000;
    RobotData rd_;
    std::string resource_path = URDF_DIR;
    std::string urdf_name = "/dyros_tocabi.urdf";
    std::string urdf_path = resource_path + urdf_name;
    VectorXd tlim, q, qdot, qddot, fstar_1, fstar_2;

    rd_.LoadModelData(urdf_path, true, false);

    q.setZero(rd_.model_.q_size);
    qdot.setZero(rd_.model_.qdot_size);
    qddot.setZero(rd_.model_.qdot_size);
    qdot.setRandom(rd_.model_.qdot_size);
    qdot.segment(0, 6).setZero();
    tlim.setConstant(rd_.model_dof_, 500);

    fstar_1.setZero(6);
    fstar_1 << 0.1, 2.0, 0.1, 0.1, -0.1, 0.1;
    // fstar_1 << 0.1, 2.0, 0.1; //, 0.1, -0.1, 0.1;

    rd_.SetTorqueLimit(tlim);
    // verbose = true;
    q << 0, 0, 0.92983, 0, 0, 0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0, 0, 0,
        0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 0, 0, 0, 0, 1;

    rd_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    rd_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    // rd_.AddTaskSpace(TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero(), verbose);
    rd_.AddTaskSpace(TASK_LINK_6D, "pelvis_link", Vector3d::Zero(), verbose);

    rd_.UpdateKinematics(q, qdot, qddot);
    rd_.SetContact(true, true);
    rd_.SetTaskSpace(0, fstar_1);
    rd_.CalcGravCompensation();
    rd_.CalcTaskControlTorque(true);
    rd_.CalcContactRedistribute(true);

    std::cout << "momentum matrix test : " << std::endl;

    Vector3d com_p__;
    double tmass__;

    MatrixXd mm__;
    VectorXd cm__;
    mm__.setZero(6, 6);
    cm__.setZero(6);
    Vector3d com_v__;
    rd_.CalcCOMInertia(rd_.link_, mm__, cm__);

    std::cout << "mm : " << std::endl;
    std::cout << mm__ << std::endl;

    std::cout << "cm : " << cm__.transpose() << std::endl;

    std::cout << "linear momentum m * v : " << tmass__ * com_v__.transpose() << std::endl;

    // std::cout << rd_.link_[10]
    std::cout << std::endl;
    std::cout << "Ang meomentum base calc 1 : " << std::endl;
    std::cout << rd_.ang_momentum_.transpose() << std::endl;

    MatrixXd cmm_;

    Eigen::MatrixXd H_C = MatrixXd::Zero(6, rd_.system_dof_);

    Eigen::VectorXd HCV = VectorXd::Zero(6);

    for (int i = 0; i < rd_.link_.size() - 1; i++)
    {

        Eigen::MatrixXd rotm = MatrixXd::Identity(6, 6);
        rotm.block(0, 0, 3, 3) = rd_.link_[i].rotm;
        rotm.block(3, 3, 3, 3) = rd_.link_[i].rotm;
        // Eigen::MatrixXd j_temp = MatrixXd::Zero(6, rd_.system_dof_);
        // j_temp.topRows(3) = rd_.link_[i].jac_.bottomRows(3);
        // j_temp.bottomRows(3) = rd_.link_[i].jac_.topRows(3);

        Eigen::VectorXd wv = VectorXd::Zero(6);
        wv.segment(0, 3) = rd_.link_[i].w;
        wv.segment(3, 3) = rd_.link_[i].v;

        Eigen::MatrixXd tmat = MatrixXd::Zero(6, 6);

        // rd_.link_[i].GetSpatialTranform();
        tmat.block(0, 0, 3, 3) = rd_.link_[i].rotm.transpose();
        tmat.block(3, 3, 3, 3) = rd_.link_[i].rotm.transpose();

        tmat.block(0, 3, 3, 3) = rd_.link_[i].rotm.transpose() * skew(-rd_.link_[i].rotm.transpose() * rd_.link_[i].xpos); // * rd_.link_[i].rotm;

        HCV += (rd_.link_[i].GetSpatialTranform() * rd_.link_[i].GetSpatialInertiaMatrix() * rotm.transpose() * wv);
    }

    std::cout << "momentum matrix at origin frame : " << std::endl;
    std::cout << HCV.transpose() << std::endl;

    Eigen::MatrixXd HCT = HCV.segment(0, 3) - skew(rd_.link_.back().xpos) * HCV.segment(3, 3);

    std::cout << "ang momentum base calc 2 : " << std::endl;
    std::cout << (HCT).transpose() << std::endl;

    // std::cout << (rd_.CalcAngularMomentumMatrix() * qdot).transpose() << std::endl;

    std::cout << "cmm fast calc : " << std::endl;

    Vector3d cmm_res = rd_.CMM_ * qdot.segment(6, rd_.model_dof_);

    std::cout << (cmm_res).transpose() << std::endl;

    Vector3d lin_momentum = rd_.link_.back().mass * rd_.link_.back().v;
    std::cout << "lin momentum " << std::endl;
    std::cout << lin_momentum.transpose() << std::endl;

    // std::cout << "cmm space to com : " << std::endl;

    // std::cout << (cmm_res + skew(rd_.link_[0].xpos - rd_.link_.back().xpos) * lin_momentum).transpose() << std::endl;

    {
        using namespace RigidBodyDynamics;

        // using namespace RigidBodyDynamics::Math;

        Model model = rd_.model_;
        UpdateKinematicsCustom(model, &q, &qdot, &qddot);

        for (size_t i = 1; i < model.mBodies.size(); i++)
        {
            model.Ic[i] = model.I[i];
            model.hc[i] = model.Ic[i].toMatrix() * model.v[i];
            model.hdotc[i] = model.Ic[i] * model.a[i] + Math::crossf(model.v[i],
                                                                     model.Ic[i] * model.v[i]);
        }

        Math::SpatialRigidBodyInertia Itot(0., Math::Vector3d(0., 0., 0.), Math::Matrix3d::Zero());
        Math::SpatialVector htot(Math::SpatialVector::Zero());
        Math::SpatialVector hdot_tot(Math::SpatialVector::Zero());

        for (size_t i = model.mBodies.size() - 1; i > 0; i--)
        {
            unsigned int lambda = model.lambda[i];

            if (lambda != 0)
            {
                model.Ic[lambda] = model.Ic[lambda] + model.X_lambda[i].applyTranspose(
                                                          model.Ic[i]);
                model.hc[lambda] = model.hc[lambda] + model.X_lambda[i].applyTranspose(
                                                          model.hc[i]);
            }
            else
            {
                Itot = Itot + model.X_lambda[i].applyTranspose(model.Ic[i]);
                htot = htot + model.X_lambda[i].applyTranspose(model.hc[i]);
            }
        }
        double mass = Itot.m;
        Math::Vector3d com = Itot.h / mass;
        std::cout << "mass = " << mass << " com = " << com.transpose() << " htot = " << htot.transpose() << std::endl;

        // Math::SpatialTransform Xtrans(com);

        Math::Vector3d angular_momentum;

        // std::cout << htot.transpose() << std::endl;

        htot = Math::Xtrans(com).applyAdjoint(htot);
        angular_momentum.set(htot[0], htot[1], htot[2]);

        std::cout << angular_momentum.transpose() << std::endl;
        std::cout << htot[3] << " " << htot[4] << " " << htot[5] << std::endl;
    }

    return 0;
}
