#include <chrono>
#include "dwbc.h"
#include <unistd.h>
#include <iomanip>

#ifndef URDF_DIR
#define URDF_DIR ""
#endif

using namespace DWBC;
using namespace std;

Eigen::MatrixXd calculateX(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B)
{
    // Step 1: Compute the eigen decomposition of B
    Eigen::EigenSolver<Eigen::MatrixXd> eigenSolverB(B);
    Eigen::MatrixXd Q = eigenSolverB.eigenvectors().real();
    Eigen::MatrixXd D = eigenSolverB.eigenvalues().real().asDiagonal();

    // Step 2: Compute the Cholesky decomposition of B
    Eigen::LLT<Eigen::MatrixXd> choleskyB = B.llt();
    Eigen::MatrixXd L = choleskyB.matrixL();

    // Step 3: Initialize a matrix Y (this will require further computations)
    Eigen::MatrixXd Y; // You will need to find a way to compute Y

    // Step 4: Compute X as Y * L
    Eigen::MatrixXd X = Y * L;

    return X;
}

// get input variable 9 double
//  int main()

int main(void)
{
    std::string urdf_file = std::string(URDF_DIR) + "/dyros_tocabi_ub.urdf";

    // std::cout << urdf_file << std::endl;

    RobotData rd_;

    rd_.LoadModelData(urdf_file, false, false);

    VectorXd tlim;
    VectorXd q = VectorXd::Zero(rd_.model_.q_size);
    VectorXd qdot = VectorXd::Zero(rd_.model_.qdot_size);

    // qdot.segment(6,

    qdot.setRandom(rd_.model_.qdot_size);
    VectorXd qddot = VectorXd::Zero(rd_.model_.qdot_size);

    q << 0, 0, 0,
        0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0;

    rd_.UpdateKinematics(q, qdot, qddot);

    std::cout << rd_.link_.back().mass << std::endl;

    MatrixXd jac_com = rd_.link_.back().jac_com_;

    vector<MatrixXd> jac_com_ul;
    vector<double> mass_ul;

    for (int i = 1; i < rd_.link_.size() - 1; i++)
    {
        // std::cout << i << " : " << rd_.link_[i].name_ << " : " << rd_.link_[i].mass << " per : " << rd_.link_[i].mass / rd_.total_mass_ << std::endl;
        jac_com_ul.push_back(rd_.link_[i].jac_com_);
        mass_ul.push_back(rd_.link_[i].mass);
    }

    MatrixXd new_jac_com = MatrixXd::Zero(3, rd_.model_.qdot_size);

    for (int i = 0; i < jac_com_ul.size(); i++)
    {
        new_jac_com += mass_ul[i] * jac_com_ul[i].block(0, 0, 3, rd_.model_.qdot_size) / rd_.total_mass_;
    }

    // calculate the rotational cmm matrix.

    MatrixXd new_jac_com_rot = MatrixXd::Zero(3, rd_.model_.qdot_size);

    MatrixXd lambda_ = (new_jac_com * rd_.A_.inverse() * new_jac_com.transpose()).inverse();

    std::cout << "urdf modifed upperbody model : " << rd_.model_.dof_count << " dof " << std::endl;
    std::cout << " mass : " << rd_.total_mass_ << std::endl;

    // std::cout << lambda_ << std::endl;

    RigidBodyDynamics::Model model;
    RigidBodyDynamics::Addons::URDFReadFromFile(urdf_file.c_str(), &model, false, false);

    model.gravity = Vector3d(0, 0, -9.81);

    RigidBodyDynamics::UpdateKinematicsCustom(model, &q, &qdot, &qddot);
    MatrixXd A_mat;
    A_mat.setZero(model.qdot_size, model.qdot_size);
    RigidBodyDynamics::CompositeRigidBodyAlgorithm(model, q, A_mat, true);

    vector<Vector3d> xpos;
    vector<MatrixXd> jac_com_l;
    vector<double> mass_l;

    double total_mass = 0;

    // std::cout << 0 << " : " << model.GetBodyName(0) << std::endl;

    for (int i = 1; i < model.mBodies.size(); i++)
    {
        xpos.push_back(RigidBodyDynamics::CalcBodyToBaseCoordinates(model, q, i, model.mBodies[i].mCenterOfMass, false));

        MatrixXd jac_temp;
        jac_temp.setZero(6, model.qdot_size);
        RigidBodyDynamics::CalcPointJacobian6D(model, q, i, model.mBodies[i].mCenterOfMass, jac_temp, false);
        jac_com_l.push_back(jac_temp);

        mass_l.push_back(model.mBodies[i].mMass);
        total_mass += model.mBodies[i].mMass;

        // std::cout << i << " : " << model.GetBodyName(i) << " : " << xpos.back().transpose() << model.mBodies[i].mMass << " accu : " << total_mass << std::endl;
    }

    // std::cout <<

    // std::cout << "total mass : " << total_mass << std::endl;

    MatrixXd com_jac_calc = MatrixXd::Zero(3, model.qdot_size);

    for (int i = 0; i < jac_com_l.size(); i++)
    {
        com_jac_calc = com_jac_calc + mass_l[i] * jac_com_l[i].block(3, 0, 3, model.qdot_size) / total_mass;

        // std::cout << i << " : " << mass_l[i] / total_mass << std::endl;
    }

    // for (int i = 0; i < jac_com_l.size(); i++)
    // {
    //     std::cout << jac_com_l[i] << std::endl;
    // }

    MatrixXd lambda_calc = (com_jac_calc * A_mat.inverse() * com_jac_calc.transpose()).inverse();

    // std::cout << lambda_calc << std::endl;

    std::vector<Link> link_vec;

    for (int i = 1; i < rd_.link_.size() - 1; i++)
    {
        link_vec.push_back(rd_.link_[i]);
    }

    // cmm matrix :
    VectorXd momtum_mat;
    MatrixXd inertial_mat;

    Vector3d ub_com_pos = rd_.link_.back().xpos;

    rd_.CalcCOMInertia(link_vec, inertial_mat, momtum_mat);

    Eigen::MatrixXd H_C = MatrixXd::Zero(6, model.qdot_size);

    for (int i = 0; i < link_vec.size(); i++)
    {
        Eigen::Matrix3d r_ = link_vec[i].rotm;
        Eigen::Matrix3d i_ = link_vec[i].inertia;
        Eigen::Vector3d c_ = link_vec[i].com_position_l_;
        Eigen::Vector3d x_ = link_vec[i].xpos;
        double m_ = link_vec[i].mass;

        H_C.topRows(3) += (r_ * (i_ + skew(c_) * skew(c_).transpose() * m_) * r_.transpose() + skew(x_) * r_ * skew(c_).transpose() * m_ * r_.transpose()) * link_vec[i].jac_.bottomRows(3) + (r_ * skew(c_) * m_ * r_.transpose() + m_ * skew(x_)) * link_vec[i].jac_.topRows(3);
        H_C.bottomRows(3) += m_ * r_ * skew(c_).transpose() * r_.transpose() * link_vec[i].jac_.bottomRows(3) + m_ * link_vec[i].jac_.topRows(3);
    }
    MatrixXd cmm;
    cmm = H_C;
    cmm.block(0, 0, 3, model.qdot_size) = H_C.topRows(3) - skew(ub_com_pos) * H_C.bottomRows(3);

    MatrixXd jac_inertial = inertial_mat.inverse() * cmm;

    // swap bottom 3 top 3
    jac_inertial.topRows(3).swap(jac_inertial.bottomRows(3));

    // std::cout << "jac_u * jac_u' : \n"
    //           << (jac_inertial * jac_inertial.transpose()).inverse() << std::endl;
    MatrixXd lambda_calc2_inv = (jac_inertial * A_mat.inverse() * jac_inertial.transpose());
    MatrixXd lambda_calc2 = lambda_calc2_inv.inverse();

    MatrixXd inertial_mat2;
    inertial_mat2.setZero(6, 6);
    inertial_mat2.block(0, 0, 3, 3) = inertial_mat.block(3, 3, 3, 3);
    inertial_mat2.block(3, 3, 3, 3) = inertial_mat.block(0, 0, 3, 3);

    // for (int i = 0; i < link_vec.size(); i++)
    // {
    //     std::cout << "link " << i << " pose : " << link_vec[i].xpos.transpose() << " mass : " << link_vec[i].mass << std::endl;
    // }

    // std::cout << "link_vec.size() : " << link_vec.size() << " link 0 pose : " << link_vec[0].xpos.transpose() << std::endl;

    // std::cout << "ub com pose : " << ub_com_pos.transpose() << std::endl;

    std::cout << "Mass matrix of virtual 6D body : inertial_mat2" << std::endl;
    std::cout << inertial_mat2 << std::endl;

    // std::cout << "jac_inertial : " << std::endl;
    // std::cout << jac_inertial << std::endl;

    std::cout << "lambda calc of upperbody cmm : lambda_calc2 " << std::endl;
    std::cout << lambda_calc2 << std::endl;

    // std::cout << "A_mat : " << std::endl;
    // std::cout << A_mat << std::endl;

    // std::cout << link_vec[0].name_ << std::endl;
    // std::cout << link_vec[0].jac_ << std::endl;

    // MatrixXd B = inertial_mat2.inverse();
    // MatrixXd A = lambda_calc2_inv;

    // Eigen::LLT<Eigen::MatrixXd> choleskyB1 = A.llt();
    // Eigen::MatrixXd L1 = choleskyB1.matrixL();

    // // std::cout << "MAtrix A : "<<std::endl;
    // // std::cout << A << std::endl;

    // // std::cout << "L1* L1' : " << std::endl;
    // // std::cout << L1 * L1.transpose() << std::endl;

    // Eigen::LLT<Eigen::MatrixXd> choleskyB2 = B.llt();
    // Eigen::MatrixXd L2 = choleskyB2.matrixL();

    // // std::cout << "Matrix B : " <<std::endl;
    // // std::cout << B << std::endl;
    // // std::cout << " L2 * L2' : "<<std::endl;
    // // std::cout << L2 * L2.transpose() << std::endl;

    // // std::cout << "L1 : " << std::endl;
    // // std::cout << L1 << std::endl;

    // // std::cout << "L2 : " << std::endl;
    // // std::cout << L2 << std::endl;

    // Eigen::MatrixXd X = L1 * L2.inverse();

    // std::cout << "X : " << std::endl;
    // std::cout << X << std::endl;

    // std::cout << " Proof : X * B * X.transpose : " << std::endl;
    // std::cout << (X * B * X.transpose()).inverse() << std::endl;

    // jac = X

    // example task torque calculation

    RobotData rd2_;

    std::string urdf2_file = std::string(URDF_DIR) + "/dyros_tocabi.urdf";
    // std::string desired_control_target = "pelvis_link";
    std::string desired_control_target = "COM";
    VectorXd fstar_1;
    fstar_1.setZero(6);
    fstar_1 << 0.1, 0.5, 0.1, 0.1, -0.1, 0.1;

    // fstar_1
    rd2_.LoadModelData(urdf2_file, true, false);

    VectorXd t2lim;
    VectorXd q2 = VectorXd::Zero(rd2_.model_.q_size);
    VectorXd q2dot = VectorXd::Zero(rd2_.model_.qdot_size);
    VectorXd q2ddot = VectorXd::Zero(rd2_.model_.qdot_size);

    // rd_.ChangeLinkToFixedJoint("head_link", verbose);

    tlim.setConstant(rd2_.model_dof_, 500);
    rd2_.SetTorqueLimit(tlim);
    // verbose = true;
    q2 << 0, 0, 0.92983, 0, 0, 0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0, 0, 0,
        0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;

    bool verbose = false;
    rd2_.UpdateKinematics(q2, q2dot, q2ddot);
    rd2_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    rd2_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);

    rd2_.AddTaskSpace(TASK_LINK_6D, desired_control_target.c_str(), Vector3d::Zero(), verbose);
    // rd2_.AddTaskSpace(TASK_LINK_ROTATION, desired_control_target.c_str(), Vector3d::Zero(), verbose);

    MatrixXd origin_A = rd2_.A_;

    MatrixXd origin_A_66 = rd2_.A_.block(0, 0, 6, 6);

    rd2_.SetContact(true, true);

    rd2_.CreateVModel();
    rd2_.UpdateVModel();

    // std::cout << rd2_.link_v_.back().
    // std::cout << rd2_.link_v_.back().inertia << std::endl;

    // std::cout << rd2_.link_v_[0].name_ << std::endl;
    // std::cout << rd2_.link_v_[0].jac_com_ << std::endl;

    // caculate lambda matrix of virtual model.

    MatrixXd lambda_calc_v = (rd2_.link_v_.back().jac_com_ * rd2_.A_v_.inverse() * rd2_.link_v_.back().jac_com_.transpose()).inverse();

    std::cout << "lambda of virtual model : \n";
    std::cout << lambda_calc_v << std::endl;

    // std::cout << " amat of virtual model : \n";
    // std::cout << rd2_.A_v_ << std::endl;

    rd2_.SetTaskSpace(0, fstar_1);
    // rd2_.SetTaskSpace(1, fstar_1.segment(3, 3));

    rd2_.CalcGravCompensation();
    rd2_.CalcTaskControlTorque(true);
    rd2_.CalcContactRedistribute(true);
    // std::cout << rd_.torque_grav_.transpose() << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "Original MODEL ::: task lambda of plevis : \n";
    std::cout << rd2_.ts_[0].Lambda_task_ << std::endl;

    std::cout << " QP Fstar modified : " << rd2_.ts_[0].f_star_.transpose() + rd2_.ts_[0].f_star_qp_.transpose() << std::endl;
    std::cout << "Result Task Torque : " << std::endl;

    std::cout << rd2_.torque_grav_.transpose() << std::endl;

    std::cout << rd2_.torque_task_.transpose() << std::endl;

    MatrixXd s_k = MatrixXd::Zero(rd2_.model_dof_, rd2_.model_dof_ + 6);
    s_k.block(0, 6, rd2_.model_dof_, rd2_.model_dof_) = MatrixXd::Identity(rd2_.model_dof_, rd2_.model_dof_);
    MatrixXd give_me_fstar = rd2_.ts_[0].J_task_ * rd2_.A_inv_ * rd2_.N_C * s_k.transpose();

    std::cout << "fstar : " << (give_me_fstar * rd2_.torque_task_).transpose() << std::endl;

    std::cout << "upperbody lambda " << std::endl;
    int upbody_link = rd2_.getLinkID("Upperbody_Link");

    std::cout << (rd2_.link_[upbody_link].jac_com_ * rd2_.A_inv_ * rd2_.N_C * rd2_.link_[upbody_link].jac_com_.transpose()).inverse() << std::endl;

    std::cout << "-----------------------------------------------------------------" << std::endl;
    // rd2_.DeleteLink("Waist1_Link", verbose);

    int lfoot = rd2_.getLinkID("l_ankleroll_link");
    int rfoot = rd2_.getLinkID("r_ankleroll_link");
    int pelvis = rd2_.getLinkID("pelvis_link");

    VectorXd q3 = VectorXd::Zero(rd2_.model_.q_size);
    VectorXd q3dot = VectorXd::Zero(rd2_.model_.qdot_size);
    VectorXd q3ddot = VectorXd::Zero(rd2_.model_.qdot_size);

    // q3 << 0, 0, 0.92983, 0, 0, 0,
    //     0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
    //     0.0, 0.0, -0.24, 0.6, -0.36, 0.0;

    // rd2_.UpdateKinematics(q3, q3dot, q3ddot);
    // rd2_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    // rd2_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);

    // rd2_.SetContact(true, true);

    // jac of lfoot :

    // std::cout <<"lfoot jac : \n";
    // std::cout << -skew(rd2_.link_[lfoot].xipos- rd2_.link_[pelvis].xpos) << std::endl;
    // std::cout << rd2_.link_[lfoot].jac_com_ << std::endl;

    // std::cout <<"rfoot jac : \n";
    // std::cout << -skew(rd2_.link_[rfoot].xipos - rd2_.link_[pelvis].xpos) << std::endl;
    // std::cout << rd2_.link_[rfoot].jac_com_ << std::endl;

    // std::cout <<"pelvis jac : \n";

    // std::cout << -skew(rd2_.link_[pelvis].xipos - rd2_.link_[pelvis].xpos) << std::endl;
    // std::cout << rd2_.link_[pelvis].jac_com_ << std::endl;

    /*


    SET 1 configuration


    */
    // std::cout << "original A_ : \n";
    // std::cout << rd2_.A_ << std::endl;
    // std::cout << " A_cs_ : \n";
    // std::cout << rd2_.A_cs_ << std::endl;
    // std::cout << " A_v_ : \n";

    // std::cout << rd2_.A_v_ << std::endl;

    // rd2_.UpdateVModel(rd2_.model_v_,)

    MatrixXd new_A_matrix;
    // std::cout <<rd2_.model_.qdot_size<<std::endl;
    MatrixXd floating_jac = MatrixXd::Zero(6, rd2_.model_cs_.qdot_size + 6);
    MatrixXd floating_jac2 = MatrixXd::Zero(6, rd2_.model_cs_.qdot_size + 6);

    Matrix3d rotm = rd2_.link_[pelvis].rotm;

    floating_jac.block(0, 0, 3, 3) = Matrix3d::Identity();
    floating_jac.block(3, 3, 3, 3) = rotm;
    floating_jac.block(0, 3, 3, 3) = -rotm * skew(ub_com_pos);
    floating_jac.block(0, 18, 6, 6) = MatrixXd::Identity(6, 6);
    // floating_jac.block(0, 18, 6, 6) = X.block(0, 0, 6, 6);
    // floating_jac.block(3,18,3,6) = X.block(0,0,3,6);

    floating_jac2.block(0, 18, 6, 6) = MatrixXd::Identity(6, 6);
    // std::cout << "Jacobian of UpperBody : \n";
    // std::cout << floating_jac << std::endl;

    // MatrixXd mass_matrix_ub;
    // mass_matrix_ub.setZero(6,6);
    // mass_matrix_ub.block(0,0,3,3) = inertial_mat.block(3,3,3,3);
    // mass_matrix_ub.block(3,3,3,3) = inertial_mat.block(0,0,3,3);

    // std::cout << "mass matrix ub : \n";
    // std::cout << mass_matrix_ub << std::endl;
    // new_A_matrix = floating_jac.transpose() * lambda_calc2 * 0.2 * floating_jac;
    // inertial_mat2 = inertial_mat2 * 0.1;

    Eigen::MatrixXd pelvis_mat_additonal;
    pelvis_mat_additonal.setZero(6, 6);

    double add_mass = 10;

    pelvis_mat_additonal(0, 0) = add_mass;
    pelvis_mat_additonal(1, 1) = add_mass;
    pelvis_mat_additonal(2, 2) = add_mass;

    pelvis_mat_additonal(3, 3) = 1;
    pelvis_mat_additonal(4, 4) = 1;
    pelvis_mat_additonal(5, 5) = 0;

    Eigen::MatrixXd add_mass_mat;
    add_mass_mat.setZero(24, 24);
    // add_mass_mat = pelvis_jh

    Eigen::MatrixXd pelvis_jac;
    pelvis_jac.setZero(6, 24);
    pelvis_jac.block(0, 0, 6, 18) = rd2_.link_[pelvis].jac_com_;

    // std::cout << "pelvis com jacobian : " << -skew(rd2_.link_[pelvis].com_position_l_) << std::endl;
    // std::cout << pelvis_jac << std::endl;

    // add_mass_mat = pelvis_jac.transpose() * pelvis_mat_additonal * pelvis_jac;

    Eigen::MatrixXd inertial_mat_small;
    inertial_mat_small = inertial_mat2 * 0.5;

    new_A_matrix = floating_jac.transpose() * inertial_mat2 * floating_jac;

    std::cout << "new A matrix : \n";
    std::cout << new_A_matrix << std::endl;

    MatrixXd A = new_A_matrix.block(0, 0, 6, 6);
    MatrixXd B = lambda_calc2;

    // Eigen::LLT<Eigen::MatrixXd> choleskyA2 = A.llt();
    // Eigen::MatrixXd L1 = choleskyA2.matrixL();

    // Eigen::LLT<Eigen::MatrixXd> choleskyB2 = B.llt();
    // Eigen::MatrixXd L2 = choleskyB2.matrixL();

    // new_A_matrix.block(18, 0, 6, 6) = L2 * L1.transpose();
    // new_A_matrix.block(0, 18, 6, 6) = L1 * L2.transpose();
    new_A_matrix.block(18, 18, 6, 6) = lambda_calc2;

    std::cout << "new A matrix2222 : \n";
    std::cout << new_A_matrix << std::endl;
    // new_A_matrix.setZero();

    MatrixXd new_A_mat_all;
    new_A_mat_all.setZero(24, 24);
    new_A_mat_all.block(0, 0, 18, 18) = rd2_.A_cs_;

    new_A_mat_all = new_A_mat_all + new_A_matrix; // + add_mass_mat;

    // new_A_mat_all.block(0, 0, 6, 6) = origin_A_66;

    // std::cout << "new A matrix : \n";
    // std::cout << new_A_matrix << std::endl;

    // std::cout << "mass matrix : \n";
    // std::cout << rd2_.A_ << std::endl;

    // std::cout << "mass matrix of A " << std::endl;
    // std::cout << rd2_.A_ << std::endl;

    // std::cout << "new A matrix all : \n";
    // std::cout << new_A_mat_all << std::endl;

    int contact_dof_ = 12;
    int system_dof_ = 24;
    int model_dof_ = 18;

    MatrixXd Lambda_contact, J_C_INV_T, N_C, W, W_inv, V2, NwJw, P_C;

    Lambda_contact.setZero(contact_dof_, contact_dof_);
    J_C_INV_T.setZero(contact_dof_, system_dof_);
    N_C.setZero(system_dof_, system_dof_);

    W.setZero(model_dof_, model_dof_);
    W_inv.setZero(model_dof_, model_dof_);

    int contact_null_dof = contact_dof_ - 6;

    V2.setZero(contact_null_dof, model_dof_);
    NwJw.setZero(model_dof_, model_dof_);

    P_C.setZero(contact_dof_, system_dof_);

    MatrixXd J_C = MatrixXd::Zero(contact_dof_, system_dof_);
    J_C.block(0, 0, 12, 18) = rd2_.J_C.block(0, 0, 12, 18);

    MatrixXd A_inv_2 = new_A_mat_all.inverse();

    int res_cal = CalculateContactConstraint(J_C, A_inv_2, Lambda_contact, J_C_INV_T, N_C, W, NwJw, W_inv, V2);

    MatrixXd J_task_;
    J_task_.setZero(6, system_dof_);
    J_task_.block(0, 0, 6, 18) = rd2_.link_[pelvis].jac_.block(0, 0, 6, 18);

    MatrixXd J_temp_;
    double r_total_mass = rd2_.link_.back().mass; // + rd2_.link_v_.back().mass;

    MatrixXd J_temp_2;
    J_temp_2.setZero(3, 39);
    for (int i = 0; i < 13; i++)
    {
        J_temp_2.block(0, 0, 3, 39) += rd2_.link_[i].jac_com_.block(0, 0, 3, 39) * rd2_.link_[i].mass;
    }

    std::cout << "J_temp2 " << std::endl;
    std::cout << J_temp_2.block(0, 0, 3, 24) << std::endl;

    std::cout << " link v " << std::endl;
    std::cout << rd2_.link_v_.back().jac_com_ << std::endl;

    MatrixXd testMatrix = MatrixXd::Zero(24, 39);

    testMatrix.block(0, 0, 18, 18) = MatrixXd::Identity(18, 18);
    testMatrix.block(18, 18, 6, 21) = rd2_.link_v_.back().jac_com_;

    std::cout << "mass calc result : \n"
              << (testMatrix * rd2_.A_inv_ * testMatrix.transpose()).inverse();

    // J_temp_.setZero(3, system_dof_);
    // J_temp_.block(0, 0, 3, 18) = J_temp_2.block(0, 0, 3, 18);

    std::cout << "upper * mass " << std::endl;
    std::cout << rd2_.link_v_.back().mass * floating_jac.block(0, 0, 3, 24) << std::endl;

    std::cout << "rtotal mass : " << r_total_mass << std::endl;

    J_task_.block(0, 0, 3, 24) = (J_temp_2.block(0, 0, 3, 24) + rd2_.link_v_.back().mass * floating_jac.block(0, 0, 3, 24)) / r_total_mass;

    // J_task_.block(0, 0, 3, 24) = rd2_.link_.back().jac_com_ * testMatrix.transpose();

    // std::cout << "J_task : \n";
    // std::cout << J_task_ << std::endl;

    // std::cout << "Original COM jacobian : " << std::endl;
    // std::cout << rd2_.link_.back().jac_com_ << std::endl;

    // std::cout << "J_task2 :\n ";
    // std::cout << rd2_.link_.back().jac_com_ * testMatrix.transpose() << std::endl;

    // J_task

    // J_task_.block(0, 0, 3, 18) = (rd2_.link_.back().mass * rd2_.link_.back().jac_com_.block(0, 0, 3, 18) + rd2_.link_v_.back().mass * floating_jac.block(0, 0, 3, 18)) / r_total_mass;
    // std::cout << r_total_mass << " : total mass " << std::endl;

    MatrixXd J_kt_;
    J_kt_.setZero(6, model_dof_);
    MatrixXd Lambda_task_;
    Lambda_task_.setZero(6, 6);

    CalculateJKT(J_task_, A_inv_2, N_C, W_inv, J_kt_, Lambda_task_);

    VectorXd torque_recalc = VectorXd::Zero(33);
    torque_recalc.segment(0, 18) = J_kt_ * Lambda_task_ * fstar_1;

    std::cout << "\n\n"
              << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "1. RESULT WITH upperbody matrix to inertial matrix : \n";

    std::cout << "Aex : \n";
    std::cout << new_A_mat_all << std::endl;

    std::cout << "Aex inv : \n";
    std::cout << new_A_mat_all.inverse() << std::endl;

    std::cout << "Jacobian of UpperBody : \n";
    std::cout << floating_jac << std::endl;

    std::cout << "lambda of upperbody : (J_upper * Aex_inv * J_upper')_inverse() " << std::endl;
    std::cout << (floating_jac * new_A_mat_all.inverse() * floating_jac.transpose()).inverse() << std::endl;

    // std::cout << "new a mat" << std::endl;
    // std::cout << new_A_matrix << std::endl;

    // std::cout << "A_inv : \n";
    // std::cout << A_inv_2 << std::endl;
    // std::cout << "Jac contact : \n";
    // std::cout << J_C << std::endl;
    // std::cout << "N_C : \n"
    //           << N_C << std::endl;

    // std::cout << "Lambda task : \n";
    // std::cout << Lambda_task_ << std::endl;

    std::cout << "task torque : \n";
    std::cout << (J_kt_ * Lambda_task_ * fstar_1).transpose() << std::endl;
    std::cout << (rd2_.link_v_.back().jac_com_.transpose() * torque_recalc.segment(12, 6)).transpose() << std::endl;

    std::cout << "fstar 2 :";
    std::cout << (give_me_fstar * torque_recalc).transpose() << std::endl;

    // std::cout << "origin A - new A :\n"
    //           << origin_A_66 - new_A_mat_all.block(0, 0, 6, 6) << std::endl;

    // std::cout << "j pelvis " << std::endl;
    // std::cout << J_task_ << std::endl;
    /*


    SET 2 configuration


    */

    // floating_jac.block(0, 18, 6, 6) = X;

    MatrixXd new_A_ub_2 = floating_jac.transpose() * inertial_mat2 * floating_jac;
    MatrixXd A_ex;
    A_ex.setZero(24, 24);
    A_ex.block(0, 0, 18, 18) = rd2_.A_;
    A_ex = A_ex + new_A_ub_2;

    // A_ex.block(0, 0, 6, 6) = origin_A_66;

    MatrixXd Lambda_contact2, J_C_INV_T2, N_C2, W2, W_inv2, V22, NwJw2, P_C2;

    Lambda_contact2.setZero(contact_dof_, contact_dof_);
    J_C_INV_T2.setZero(contact_dof_, system_dof_);
    N_C2.setZero(system_dof_, system_dof_);

    W2.setZero(model_dof_, model_dof_);
    W_inv2.setZero(model_dof_, model_dof_);

    // int contact_null_dof = contact_dof_ - 6;

    V22.setZero(contact_null_dof, model_dof_);
    NwJw2.setZero(model_dof_, model_dof_);

    P_C2.setZero(contact_dof_, system_dof_);

    MatrixXd J_C2 = MatrixXd::Zero(contact_dof_, system_dof_);
    J_C2.block(0, 0, 12, 18) = rd2_.J_C;

    MatrixXd A_ex_inv = A_ex.inverse();

    int res2_cal = CalculateContactConstraint(J_C2, A_ex_inv, Lambda_contact2, J_C_INV_T2, N_C2, W2, NwJw2, W_inv2, V22);

    MatrixXd J_task_2;
    J_task_2.setZero(6, system_dof_);
    J_task_2.block(0, 0, 6, 18) = rd2_.link_[pelvis].jac_;

    MatrixXd J_kt_2;
    J_kt_2.setZero(6, model_dof_);
    MatrixXd Lambda_task_2;
    Lambda_task_2.setZero(6, 6);

    CalculateJKT(J_task_2, A_ex_inv, N_C2, W_inv2, J_kt_2, Lambda_task_2);

    VectorXd torque_recalc2 = VectorXd::Zero(33);
    torque_recalc2.segment(0, 18) = J_kt_2 * Lambda_task_2 * fstar_1;

    std::cout << "\n\n"
              << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "2. RESULT WITH upperbody matrix to lambda matrix : \n";

    // std::cout << "Aex : \n";
    // std::cout << A_ex << std::endl;

    std::cout << "lambda of upperbody : (J_upper * Aex_inv * J_upper')_inverse() " << std::endl;
    std::cout << (floating_jac * A_ex.inverse() * floating_jac.transpose()).inverse() << std::endl;

    // std::cout << "A_inv : \n";
    // std::cout << A_ex_inv << std::endl;
    // std::cout << "Jac contact : \n";
    // std::cout << J_C2 << std::endl;
    // std::cout << "N_C : \n"
    //           << N_C2 << std::endl;

    std::cout << "Lambda task : \n";
    std::cout << Lambda_task_2 << std::endl;

    std::cout << "task torque : \n";
    std::cout << (J_kt_2 * Lambda_task_2 * fstar_1).transpose() << std::endl;
    std::cout << "fstar 2 :";
    std::cout << (give_me_fstar * torque_recalc2).transpose() << std::endl;

    // std::cout << "origin A - new A :\n"
    //           << origin_A_66 - A_ex.block(0, 0, 6, 6) << std::endl;

    // std::cout << "j pelvis " << std::endl;
    // std::cout << J_task_2 << std::endl;

    std::cout << "\n\n"
              << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "3. RESULT COMPARISON!! : \n";

    // std::cout << "Aex comparison : \n";
    // std::cout << A_ex - new_A_mat_all << std::endl;

    // std::cout << "Aex inv comaparison : \n";
    // std::cout << A_ex_inv - A_inv_2 << std::endl;

    // std::cout << "A_inv : \n";
    // std::cout << rd2_.A_.inverse() << std::endl;

    // std::cout << "inverse ..? \n";
    // std::cout << A_ex.inverse() * A_ex << std::endl;

    std::cout << "N_C comparison : " << std::endl;
    std::cout << N_C - N_C2 << std::endl;

    std::cout << "lambda_task comparison" << std::endl;
    std::cout << Lambda_task_2 - Lambda_task_ << std::endl;

    std::cout << "A_inv *N_C comparison" << std::endl;
    std::cout << A_ex_inv * N_C2 - A_inv_2 * N_C << std::endl;

    std::cout << "lambda contact comparison " << std::endl;
    std::cout << Lambda_contact2 - Lambda_contact << std::endl;

    std::cout << "J_C_INV_T comparison " << std::endl;
    std::cout << J_C_INV_T2 - J_C_INV_T << std::endl;

    std::cout << "J_C*A_inv comparison " << std::endl;
    std::cout << J_C2 * A_ex_inv - J_C * A_inv_2 << std::endl;

    std::cout << "A_inv comp block 66" << std::endl;
    std::cout << (A_ex_inv - A_inv_2).block(18, 18, 6, 6) << std::endl;
    std::cout << "A_inv comp block inv 66" << std::endl;
    std::cout << (A_ex_inv - A_inv_2).block(18, 18, 6, 6).inverse() << std::endl;
    std::cout << "0.5 A upper : " << std::endl;
    std::cout << 2 * new_A_matrix.block(18, 18, 6, 6) << std::endl;
    std::cout << "0.5 A upper inverse : " << std::endl;
    std::cout << (2 * new_A_matrix.block(18, 18, 6, 6)).inverse() << std::endl;

    std::cout << "jkt2 comparison " << std::endl;
    std::cout << J_kt_2 << std::endl;
    std::cout << "jkt 1 " << std::endl;
    std::cout << J_kt_ << std::endl;
    // std::cout << ""

    std::cout << "orgin A_inv " << std::endl;
    std::cout << origin_A.inverse() << std::endl;

    std::cout << "A inv of new A " << std::endl;
    std::cout << A_ex_inv << std::endl;

    return 0;
}
