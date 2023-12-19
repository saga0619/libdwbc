#include <chrono>
#include "dwbc.h"
#include <unistd.h>
#include <iomanip>

#ifndef URDF_DIR
#define URDF_DIR ""
#endif

using namespace DWBC;
using namespace std;

int main(void)
{
    std::string resource_path = URDF_DIR;
    std::string urdf_name = "/dyros_tocabi.urdf";
    std::string urdf_path = resource_path + urdf_name;
    RobotData rd_;
    rd_.LoadModelData(urdf_path, true, false);

    VectorXd tlim;
    VectorXd q = VectorXd::Zero(rd_.model_.q_size);
    VectorXd qdot = VectorXd::Zero(rd_.model_.qdot_size);
    VectorXd qddot = VectorXd::Zero(rd_.model_.qdot_size);

    std::string desired_control_target = "pelvis_link";
    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "Desired Control Target : " << desired_control_target << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;
    VectorXd fstar_1;
    fstar_1.setZero(6);
    fstar_1 << 0.1, 0.5, 0.1, 0.1, -0.1, 0.1;

    // fstar_1
    std::cout << "-----------------------------------------------------------------" << std::endl;
    std::cout << "Desired Task FSTAR : " << fstar_1.transpose() << std::endl;
    std::cout << "-----------------------------------------------------------------" << std::endl;
    // rd_.ChangeLinkToFixedJoint("head_link", verbose);

    tlim.setConstant(rd_.model_dof_, 500);
    rd_.SetTorqueLimit(tlim);
    // verbose = true;

    q << 0, 0, 0.92983, 0, 0, 0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0, 0, 0,
        0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;

    bool verbose = false;

    rd_.UpdateKinematics(q, qdot, qddot);
    rd_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    rd_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    rd_.AddContactConstraint("l_wrist2_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    rd_.AddContactConstraint("r_wrist2_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);

    // rd_.AddTaskSpace(TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero(), verbose);
    rd_.AddTaskSpace(TASK_LINK_6D, desired_control_target.c_str(), Vector3d::Zero(), verbose);

    rd_.SetContact(1, 1, 0, 0);
    
    rd_.CreateVModel();

    return 0;
}
