#define CATCH_CONFIG_MAIN
#include "catch_amalgamated.hpp"
#include "dwbc.h"
#include "dwbc_test_util.h"

#ifndef URDF_DIR
#define URDF_DIR ""
#endif

#undef COMPARISON_EPS
#define COMPARISON_EPS 1.0E-12

using namespace DWBC;

#define REQUIRE_MESSAGE(cond, msg) \
    do                             \
    {                              \
        INFO(msg);                 \
        REQUIRE(cond);             \
    } while ((void)0, 0)

#define CHECK_MESSAGE(cond, msg) \
    do                           \
    {                            \
        INFO(msg);               \
        CHECK(cond);             \
    } while ((void)0, 0)
// TEST_CASE("MODEL MODIFICATION TEST")
// {
//     RobotData rd_;
//     std::string resource_path = URDF_DIR;
//     std::string urdf_name = "/dyros_tocabi.urdf";
//     std::string urdf_path = resource_path + urdf_name;
//     rd_.LoadModelData(urdf_path, true, false);
//     VectorXd q;
//     VectorXd qdot;
//     VectorXd qddot;
//     q.setZero(rd_.model_.q_size);
//     qdot.setZero(rd_.model_.qdot_size);
//     qddot.setZero(rd_.model_.qdot_size);

//     q << 0, 0, 0.92983, 0, 0, 0,
//         0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
//         0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
//         0, 0, 0,
//         0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
//         0, 0,
//         -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;

//     rd_.UpdateKinematics(q, qdot, qddot);

//     int left_foot_id = 6;
//     int right_foot_id = 12;

//     rd_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
//     rd_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
//     rd_.AddContactConstraint("l_wrist2_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);
//     rd_.AddContactConstraint("r_wrist2_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);

//     rd_.AddTaskSpace(TASK_LINK_6D, "pelvis_link", Vector3d::Zero());
//     rd_.AddTaskSpace(TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero());

//     // delete head

//     VectorXd fstar_1;
//     fstar_1.setZero(6);
//     fstar_1(0) = 0.1;
//     fstar_1(1) = 4.0;
//     fstar_1(2) = 0.1;

//     fstar_1(3) = 0.1;
//     fstar_1(4) = -0.1;
//     fstar_1(5) = 0.1;

//     bool init = true;

//     VectorXd tlim;
//     tlim.setConstant(rd_.model_dof_, 300);

//     rd_.SetTorqueLimit(tlim);

//     RobotData rd2_;
//     rd2_.LoadModelData(urdf_path, true, false);

//     rd_.UpdateKinematics(q, qdot, qddot);

//     rd_.CopyKinematicsData(rd2_);
//     // rd_.save_mat_file_ = true;

//     rd2_.SetContact(true, true);

//     rd2_.SetTaskSpace(0, fstar_1);

//     rd2_.SetTaskSpace(1, fstar_1.segment(3, 3));

//     rd2_.CalcGravCompensation();

//     CHECK(rd2_.CalcTaskControlTorque(true));

//     CHECK(rd2_.CalcContactRedistribute(true));

//     std::cout << "before modification " << std::endl;

//     std::cout << "grav torque : " << std::endl;
//     std::cout << rd2_.torque_grav_.transpose() << std::endl;
//     std::cout << "task torque : " << std::endl;
//     std::cout << rd2_.torque_task_.transpose() << std::endl;
//     std::cout << "contact torque : " << std::endl;
//     std::cout << rd2_.torque_contact_.transpose() << std::endl;

//     MatrixXd w_temp, w_inv_temp;

//     w_temp = rd2_.W;
//     w_inv_temp = rd2_.W_inv;

//     rd2_.printLinkInfo();

//     int head_id = rd2_.getLinkID("head_link");

//     int neck_id = rd2_.getLinkID("neck_link");

//     std::cout << "mjoint size : " << rd2_.model_.mJoints.size() << std::endl;
//     std::cout << "mbody size : " << rd2_.model_.mBodies.size() << std::endl;

//     // rd2_.link_[head_id].

//     // Vector3d joint_trans = rd2_.link_[head_id].;

//     rd2_.DeleteLink("head_link");

//     // rd2_.AddBody("neck_link","head_link",Eigen::Matrix3d::Identity(),)

//     VectorXd q2;
//     VectorXd qdot2;
//     VectorXd qddot2;
//     q2.setZero(rd2_.model_.q_size);
//     qdot2.setZero(rd2_.model_.qdot_size);
//     qddot2.setZero(rd2_.model_.qdot_size);

//     q2 << 0, 0, 0.92983, 0, 0, 0,
//         0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
//         0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
//         0, 0, 0,
//         0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
//         0,
//         -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;

//     rd2_.UpdateKinematics(q2, qdot2, qddot2);

//     std::cout << "update complete" << std::endl;

//     rd2_.printLinkInfo();
//     rd2_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
//     std::cout << "add modification 1" << std::endl;
//     rd2_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
//     std::cout << "add modification 2" << std::endl;
//     rd2_.AddContactConstraint("l_wrist2_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);
//     std::cout << "add modification 3" << std::endl;
//     rd2_.AddContactConstraint("r_wrist2_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);
//     std::cout << "add modification 4" << std::endl;

//     rd2_.AddTaskSpace(TASK_LINK_6D, "pelvis_link", Vector3d::Zero());
//     std::cout << "add modification 5" << std::endl;
//     rd2_.AddTaskSpace(TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero());
//     std::cout << "add" << std::endl;

//     std::cout << "after modification 1" << std::endl;

//     rd2_.SetContact(true, true);
//     std::cout << "after modification 2" << std::endl;

//     // //w, winv compare
//     // std::cout << "w compare" << std::endl;
//     // std::cout << "w before" << std::endl;
//     // std::cout << w_temp << std::endl;
//     // std::cout << "w after" << std::endl;
//     // std::cout << rd2_.W << std::endl;

//     // //diff
//     // std::cout << "w diff" << std::endl;
//     // std::cout << rd2_.W - w_temp << std::endl;

//     // std::cout << "w_inv compare" << std::endl;
//     // std::cout << "w_inv before" << std::endl;
//     // std::cout << w_inv_temp << std::endl;
//     // std::cout << "w_inv after" << std::endl;
//     // std::cout << rd2_.W_inv << std::endl;

//     // //diff
//     // std::cout << "w_inv diff" << std::endl;
//     // std::cout << rd2_.W_inv - w_inv_temp << std::endl;

//     rd2_.SetTaskSpace(0, fstar_1);
//     std::cout << "after modification 3" << std::endl;

//     rd2_.SetTaskSpace(1, fstar_1.segment(3, 3));
//     std::cout << "after modification 4" << std::endl;

//     rd2_.CalcGravCompensation();
//     std::cout << "after modification 5" << std::endl;

//     CHECK(rd2_.CalcTaskControlTorque(true));

//     std::cout << "after modification 6" << std::endl;
//     CHECK(rd2_.CalcContactRedistribute(true));

//     std::cout << "after modification 7" << std::endl;

//     std::cout << "grav torque : " << std::endl;
//     std::cout << rd2_.torque_grav_.transpose() << std::endl;
//     std::cout << "task torque : " << std::endl;
//     std::cout << rd2_.torque_task_.transpose() << std::endl;
//     std::cout << "contact torque : " << std::endl;
//     std::cout << rd2_.torque_contact_.transpose() << std::endl;
// }

TEST_CASE("LIBDWBC CALCULATION VERIFICATION : CASE 1", "[LIBDWBC]")
{
    RobotData rd_;
    std::string resource_path = URDF_DIR;
    std::string urdf_name = "/dyros_tocabi.urdf";
    std::string urdf_path = resource_path + urdf_name;

    rd_.LoadModelData(urdf_path, true, false);
    VectorXd q = VectorXd::Zero(rd_.system_dof_ + 1);
    VectorXd qdot = VectorXd::Zero(rd_.system_dof_);
    VectorXd qddot = VectorXd::Zero(rd_.system_dof_);

    VectorXd fstar_1;
    fstar_1.setZero(6);

    VectorXd fstar_2;
    fstar_2.setZero(3);
    std::string case_set = "";

    q << 0, 0, 0.92983, 0, 0, 0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0, 0, 0,
        0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;

    fstar_1 << 0.1, 4.0, 0.1, 0.1, -0.1, 0.1;
    fstar_2 << 0.1, -0.1, 0.1;

    case_set = "/1";

    rd_.UpdateKinematics(q, qdot, qddot);

    int left_foot_id = 6;
    int right_foot_id = 12;

    rd_.AddContactConstraint(left_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    rd_.AddContactConstraint(right_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    rd_.AddContactConstraint(23, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);
    rd_.AddContactConstraint(31, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);

    rd_.AddTaskSpace(TASK_LINK_6D, 0, Vector3d::Zero());
    rd_.AddTaskSpace(TASK_LINK_ROTATION, 15, Vector3d::Zero());

    VectorXd tlim;
    tlim.setConstant(rd_.model_dof_, 300);

    rd_.SetTorqueLimit(tlim);

    rd_.UpdateKinematics(q, qdot, qddot);

    // rd_.AddContactConstraint(left_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    // rd_.AddContactConstraint(right_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);

    // rd_.AddTaskSpace(TASK_LINK_6D, 0, Vector3d::Zero());
    // rd_.AddTaskSpace(TASK_LINK_ROTATION, 15, Vector3d::Zero());

    rd_.SetContact(true, true);

    std::string f_name = "/J_C";

    REQUIRE_MESSAGE(check_binary((case_set + f_name).c_str(), rd_.J_C), "J_C is not correct");

    f_name = "/A_inv_";
    REQUIRE_MESSAGE(check_binary((case_set + f_name).c_str(), rd_.A_inv_), "A_inv_ is not correct");

    f_name = "/Lambda_contact";
    REQUIRE_MESSAGE(check_binary((case_set + f_name).c_str(), rd_.Lambda_contact), "Lambda_contact is not correct");

    f_name = "/J_C_INV_T";
    REQUIRE_MESSAGE(check_binary((case_set + f_name).c_str(), rd_.J_C_INV_T), "J_C_INV_T is not correct");

    f_name = "/N_C";
    REQUIRE_MESSAGE(check_binary((case_set + f_name).c_str(), rd_.N_C), "N_C is not correct");

    f_name = "/W";
    REQUIRE_MESSAGE(check_binary((case_set + f_name).c_str(), rd_.W), "W is not correct");

    f_name = "/NwJw";
    REQUIRE_MESSAGE(check_binary((case_set + f_name).c_str(), rd_.NwJw), "NwJw is not correct");

    f_name = "/W_inv";
    REQUIRE_MESSAGE(check_binary((case_set + f_name).c_str(), rd_.W_inv), "W_inv is not correct");

    rd_.SetTaskSpace(0, fstar_1);

    rd_.SetTaskSpace(1, fstar_2);

    f_name = "/torque_grav_";
    REQUIRE_MESSAGE(check_binary((case_set + f_name).c_str(), rd_.CalcGravCompensation()), "Gravity torque is not correct");

    CHECK(rd_.CalcTaskControlTorque(true));

    f_name = "/torque_task_";
    REQUIRE_MESSAGE(check_binary((case_set + f_name).c_str(), rd_.torque_task_), "Task torque is not correct");

    CHECK(rd_.CalcContactRedistribute(true));

    f_name = "/torque_contact_";
    REQUIRE_MESSAGE(check_binary((case_set + f_name).c_str(), rd_.torque_contact_), "Contact torque is not correct");
}

TEST_CASE("LIBDWBC CALCULATION VERIFICATION : CASE 2", "[LIBDWBC]")
{
    RobotData rd_;
    std::string resource_path = URDF_DIR;
    std::string urdf_name = "/dyros_tocabi.urdf";
    std::string urdf_path = resource_path + urdf_name;

    rd_.LoadModelData(urdf_path, true, false);
    VectorXd q = VectorXd::Zero(rd_.system_dof_ + 1);
    VectorXd qdot = VectorXd::Zero(rd_.system_dof_);
    VectorXd qddot = VectorXd::Zero(rd_.system_dof_);

    VectorXd fstar_1;
    fstar_1.setZero(6);

    VectorXd fstar_2;
    fstar_2.setZero(3);
    std::string case_set = "";

    q << 0, 0, 0.92983, 0, 0, 0,
        0.1, 0.0, -0.24, 0.5, -0.6, 0.0,
        0.05, 0.0, -0.21, 0.7, -0.31, 0.0,
        0, 0, 0,
        0.2, 0.5, 1.5, -1.27, -1.2, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 1.3, 0.1, 1.3, 0, 1;

    fstar_1 << 0.4, 2.0, 0.1, 0.3, -0.1, 0.1;
    fstar_2 << 0.1, 0.1, 0.1;

    case_set = "/2";

    rd_.UpdateKinematics(q, qdot, qddot);

    int left_foot_id = 6;
    int right_foot_id = 12;

    rd_.AddContactConstraint(left_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    rd_.AddContactConstraint(right_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    rd_.AddContactConstraint(23, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);
    rd_.AddContactConstraint(31, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);

    rd_.AddTaskSpace(TASK_LINK_6D, 0, Vector3d::Zero());
    rd_.AddTaskSpace(TASK_LINK_ROTATION, 15, Vector3d::Zero());

    VectorXd tlim;
    tlim.setConstant(rd_.model_dof_, 300);

    rd_.SetTorqueLimit(tlim);

    rd_.UpdateKinematics(q, qdot, qddot);

    // rd_.AddContactConstraint(left_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    // rd_.AddContactConstraint(right_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);

    // rd_.AddTaskSpace(TASK_LINK_6D, 0, Vector3d::Zero());
    // rd_.AddTaskSpace(TASK_LINK_ROTATION, 15, Vector3d::Zero());

    rd_.SetContact(true, true);

    std::string f_name = "/J_C";

    REQUIRE_MESSAGE(check_binary((case_set + f_name).c_str(), rd_.J_C), "J_C is not correct");

    f_name = "/A_inv_";
    REQUIRE_MESSAGE(check_binary((case_set + f_name).c_str(), rd_.A_inv_), "A_inv_ is not correct");

    f_name = "/Lambda_contact";
    REQUIRE_MESSAGE(check_binary((case_set + f_name).c_str(), rd_.Lambda_contact), "Lambda_contact is not correct");

    f_name = "/J_C_INV_T";
    REQUIRE_MESSAGE(check_binary((case_set + f_name).c_str(), rd_.J_C_INV_T), "J_C_INV_T is not correct");

    f_name = "/N_C";
    REQUIRE_MESSAGE(check_binary((case_set + f_name).c_str(), rd_.N_C), "N_C is not correct");

    f_name = "/W";
    REQUIRE_MESSAGE(check_binary((case_set + f_name).c_str(), rd_.W), "W is not correct");

    f_name = "/NwJw";
    REQUIRE_MESSAGE(check_binary((case_set + f_name).c_str(), rd_.NwJw), "NwJw is not correct");

    f_name = "/W_inv";
    REQUIRE_MESSAGE(check_binary((case_set + f_name).c_str(), rd_.W_inv), "W_inv is not correct");

    rd_.SetTaskSpace(0, fstar_1);

    rd_.SetTaskSpace(1, fstar_2);

    f_name = "/torque_grav_";
    REQUIRE_MESSAGE(check_binary((case_set + f_name).c_str(), rd_.CalcGravCompensation()), "Gravity torque is not correct");

    CHECK(rd_.CalcTaskControlTorque(true));

    f_name = "/torque_task_";
    REQUIRE_MESSAGE(check_binary((case_set + f_name).c_str(), rd_.torque_task_), "Task torque is not correct");

    CHECK(rd_.CalcContactRedistribute(true));

    f_name = "/torque_contact_";
    REQUIRE_MESSAGE(check_binary((case_set + f_name).c_str(), rd_.torque_contact_), "Contact torque is not correct");
}

TEST_CASE("KINEMATICS CALCULATION BENCHMARK", "[KINEMATICS_CALCULATION_BENCHMARK]")
{
    RobotData rd_;
    std::string resource_path = URDF_DIR;
    std::string urdf_name = "/dyros_tocabi.urdf";
    std::string urdf_path = resource_path + urdf_name;
    rd_.LoadModelData(urdf_path, true, false);
    VectorXd q;
    VectorXd qdot;
    VectorXd qddot;
    q.setZero(rd_.model_.q_size);
    qdot.setZero(rd_.model_.qdot_size);
    qddot.setZero(rd_.model_.qdot_size);

    BENCHMARK("random kinematics calculation")
    {
        q.setRandom();
        qdot.setRandom();
        rd_.UpdateKinematics(q, qdot, qddot);
    };
}

TEST_CASE("CONTACT SPACE CALCULATION BENCHMARK")
{
    RobotData rd_;
    std::string resource_path = URDF_DIR;
    std::string urdf_name = "/dyros_tocabi.urdf";
    std::string urdf_path = resource_path + urdf_name;

    rd_.LoadModelData(urdf_path, true, false);
    VectorXd q = VectorXd::Zero(rd_.system_dof_ + 1);
    VectorXd qdot = VectorXd::Zero(rd_.system_dof_);
    VectorXd qddot = VectorXd::Zero(rd_.system_dof_);

    q << 0, 0, 0.92983, 0, 0, 0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0, 0, 0,
        0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;

    rd_.UpdateKinematics(q, qdot, qddot);

    int left_foot_id = 6;
    int right_foot_id = 12;

    rd_.AddContactConstraint(left_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    rd_.AddContactConstraint(right_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    rd_.AddContactConstraint(23, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);
    rd_.AddContactConstraint(31, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);

    rd_.AddTaskSpace(TASK_LINK_6D, 0, Vector3d::Zero());
    rd_.AddTaskSpace(TASK_LINK_ROTATION, 15, Vector3d::Zero());

    VectorXd fstar_1;
    fstar_1.setZero(6);
    fstar_1 << 0.1, 4.0, 0.1, 0.1, -0.1, 0.1;

    VectorXd tlim;
    tlim.setConstant(rd_.model_dof_, 300);

    rd_.SetTorqueLimit(tlim);

    rd_.UpdateKinematics(q, qdot, qddot);
    BENCHMARK("kinematics calculation")
    {
        rd_.UpdateKinematics(q, qdot, qddot);
    };

    // rd_.AddContactConstraint(left_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    // rd_.AddContactConstraint(right_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);

    // rd_.AddTaskSpace(TASK_LINK_6D, 0, Vector3d::Zero());
    // rd_.AddTaskSpace(TASK_LINK_ROTATION, 15, Vector3d::Zero());

    rd_.SetContact(true, true);

    BENCHMARK("contact space calculation")
    {
        rd_.SetContact(true, true);
    };

    rd_.SetTaskSpace(0, fstar_1);
    rd_.SetTaskSpace(1, fstar_1.segment(3, 3));

    BENCHMARK("grav torque calculation")
    {
        rd_.CalcGravCompensation();
    };

    BENCHMARK("Update task space")
    {
        rd_.CalcTaskSpace();
    };

    rd_.CalcTaskControlTorque(true);

    BENCHMARK("task control calculation (Update task + QP calculation + matrix preperation)")
    {
        rd_.CalcTaskControlTorque(false);
    };

    BENCHMARK("QP Calculation Benchmark")
    {
        VectorXd qpres;
        for (int i = 0; i < rd_.qp_task_.size(); i++)
        {
            rd_.qp_task_[i].SolveQPoases(300, qpres);
        }
    };

    rd_.CalcContactRedistribute(true);

    BENCHMARK("contact control calculation")
    {
        rd_.CalcContactRedistribute(false);
    };
}
TEST_CASE("CENTROIDAL MOMENTUM MATRIX TEST")
{
    RobotData rd_;
    std::string resource_path = URDF_DIR;
    std::string urdf_name = "/dyros_tocabi.urdf";
    std::string urdf_path = resource_path + urdf_name;
    rd_.LoadModelData(urdf_path, true, false);
    VectorXd q;
    VectorXd qdot;
    VectorXd qddot;
    q.setZero(rd_.model_.q_size);
    qdot.setZero(rd_.model_.qdot_size);
    qddot.setZero(rd_.model_.qdot_size);

    q << 0, 0, 0.92983, 0, 0, 0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0, 0, 0,
        0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;

    qdot.setRandom(rd_.model_.qdot_size);
    qdot.segment(0, 6).setZero();

    // std::cout << qdot.transpose() << std::endl;

    rd_.UpdateKinematics(q, qdot, qddot);
    Eigen::VectorXd ans1, ans2;

    BENCHMARK("angular momentum matrix calculation")
    {
        ans1 = rd_.CalcAngularMomentumMatrix() * qdot;
    };

    Eigen::MatrixXd cmm = Eigen::MatrixXd::Zero(3, rd_.model_.qdot_size);
    BENCHMARK("angular momentum matrix calculation2")
    {
        rd_.CalcAngularMomentumMatrix(cmm);

        ans2 = cmm * qdot;
    };
    // std::cout << std::endl;
    // std::cout << rd_.ang_momentum_.transpose() << std::endl;
    // std::cout << (rd_.CalcAngularMomentumMatrix() * qdot).transpose() << std::endl;
    // std::cout << (rd_.CMM_ * qdot).transpose() << std::endl;

    REQUIRE(ans1.isApprox((ans2), 1e-5) == true);

    Eigen::MatrixXd H_C = MatrixXd::Zero(6, rd_.system_dof_);

    int i = 0;
    Eigen::MatrixXd rotm = MatrixXd::Identity(6, 6);
    Eigen::MatrixXd j_temp = MatrixXd::Zero(6, rd_.system_dof_);

    // BENCHMARK("angular momentum matrix calculation")
    // {
    //     rotm.block(0, 0, 3, 3) = rd_.link_[i].rotm;
    //     rotm.block(3, 3, 3, 3) = rd_.link_[i].rotm;
    // };
    // BENCHMARK("angular momentum matrix calculation2")
    // {
    //     rd_.link_[i].GetSpatialInertiaMatrix();
    // };

    // BENCHMARK("angular momentum matrix calculation3")
    // {
    //     j_temp.topRows(3) = rd_.link_[i].jac_.bottomRows(3);
    //     j_temp.bottomRows(3) = rd_.link_[i].jac_.topRows(3);
    // };
    // MatrixXd H_TEMP1, H_temp2;

    // BENCHMARK("angular momentum matrix calculation4")
    // {
    //     H_TEMP1 = ((rd_.link_[i].GetSpatialTranform() * rd_.link_[i].GetSpatialInertiaMatrix()) * rotm.transpose());
    // };
    // BENCHMARK("angular momentum matrix calculation5")
    // {
    //     H_temp2 = H_TEMP1 * j_temp;
    // };
}
TEST_CASE("MODEL MODIFICATION BENCHMARK")
{
    bool verbose = false;

    RobotData rd_;
    std::string resource_path = URDF_DIR;
    std::string urdf_name = "/dyros_tocabi.urdf";
    std::string urdf_path = resource_path + urdf_name;

    rd_.LoadModelData(urdf_path, true, false);
    VectorXd q = VectorXd::Zero(rd_.system_dof_ + 1);
    VectorXd qdot = VectorXd::Zero(rd_.system_dof_);
    VectorXd qddot = VectorXd::Zero(rd_.system_dof_);

    VectorXd fstar_1;
    fstar_1.setZero(6);
    fstar_1 << 0.1, 4.0, 0.1, 0.1, -0.1, 0.1;

    VectorXd tlim;
    tlim.setConstant(rd_.model_dof_, 500);

    rd_.SetTorqueLimit(tlim);
    q << 0, 0, 0.92983, 0, 0, 0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0, 0, 0,
        0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;

    rd_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    rd_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);

    rd_.AddTaskSpace(TASK_LINK_6D, "pelvis_link", Vector3d::Zero(), verbose);
    rd_.AddTaskSpace(TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero(), verbose);

    rd_.UpdateKinematics(q, qdot, qddot);
    rd_.SetContact(true, true);

    rd_.SetTaskSpace(0, fstar_1);
    rd_.SetTaskSpace(1, fstar_1.segment(3, 3));

    rd_.CalcGravCompensation();
    rd_.CalcTaskControlTorque(true);
    rd_.CalcContactRedistribute(true);

    BENCHMARK("ORIGINAL MODEL CALCULATION")
    {
        rd_.UpdateKinematics(q, qdot, qddot);
        rd_.SetContact(true, true);

        rd_.SetTaskSpace(0, fstar_1);
        rd_.SetTaskSpace(1, fstar_1.segment(3, 3));

        rd_.CalcGravCompensation();
        rd_.CalcTaskControlTorque(false);
        rd_.CalcContactRedistribute(false);
    };

    std::vector<Link> Link_RArm;
    std::vector<Joint> joint_RArm;
    for (int i = 13; i <= 33; i++)
    {
        Link_RArm.push_back(rd_.link_[i]);
        joint_RArm.push_back(rd_.joint_[i]);
    }
    rd_.DeleteLink("Waist1_Link", verbose);

    Link_RArm[2].parent_id_ = rd_.getLinkID("pelvis_link");
    joint_RArm[2].joint_type_ = JOINT_6DOF;
    rd_.AddLink(joint_RArm[2], Link_RArm[2], verbose); // UpperBody

    rd_.ClearContactConstraint();
    rd_.ClearTaskSpace();

    tlim.resize(rd_.model_dof_);
    tlim.setConstant(rd_.model_dof_, 300);

    rd_.SetTorqueLimit(tlim);
    q << 0, 0, 0.92983, 0, 0, 0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0, 0, 0,
        0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;
    rd_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    rd_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    rd_.AddTaskSpace(TASK_LINK_6D, "pelvis_link", Vector3d::Zero(), verbose);
    rd_.AddTaskSpace(TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero(), verbose);

    VectorXd q2;
    VectorXd qdot2;
    VectorXd qddot2;
    q2.setZero(rd_.model_.q_size);
    qdot2.setZero(rd_.model_.qdot_size);
    qddot2.setZero(rd_.model_.qdot_size);
    q2 << 0, 0, 0.92983, 0, 0, 0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0, 0, 0,
        0, 0, 0, 1;

    rd_.UpdateKinematics(q2, qdot2, qddot2);
    rd_.SetContact(true, true);

    rd_.SetTaskSpace(0, fstar_1);
    rd_.SetTaskSpace(1, fstar_1.segment(3, 3));

    rd_.CalcGravCompensation();
    rd_.CalcTaskControlTorque(true);
    rd_.CalcContactRedistribute(true);

    BENCHMARK("MODEL MODIFIED CONTROL CALCULATION")
    {
        rd_.UpdateKinematics(q2, qdot2, qddot2);
        rd_.SetContact(true, true);

        rd_.SetTaskSpace(0, fstar_1);
        rd_.SetTaskSpace(1, fstar_1.segment(3, 3));

        rd_.CalcGravCompensation();
        rd_.CalcTaskControlTorque(false);
        rd_.CalcContactRedistribute(false);
    };
}

TEST_CASE("COPY AND CALCULATION TEST")
{
    RobotData rd_;
    std::string resource_path = URDF_DIR;
    std::string urdf_name = "/dyros_tocabi.urdf";
    std::string urdf_path = resource_path + urdf_name;
    rd_.LoadModelData(urdf_path, true, false);
    VectorXd q;
    VectorXd qdot;
    VectorXd qddot;
    q.setZero(rd_.model_.q_size);
    qdot.setZero(rd_.model_.qdot_size);
    qddot.setZero(rd_.model_.qdot_size);

    q << 0, 0, 0.92983, 0, 0, 0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0, 0, 0,
        0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;

    rd_.UpdateKinematics(q, qdot, qddot);

    int left_foot_id = 6;
    int right_foot_id = 12;

    rd_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    rd_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    rd_.AddContactConstraint("l_wrist2_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);
    rd_.AddContactConstraint("r_wrist2_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);

    rd_.AddTaskSpace(TASK_LINK_6D, "pelvis_link", Vector3d::Zero());
    rd_.AddTaskSpace(TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero());

    VectorXd fstar_1;
    fstar_1.setZero(6);
    fstar_1(0) = 0.1;
    fstar_1(1) = 4.0;
    fstar_1(2) = 0.1;

    fstar_1(3) = 0.1;
    fstar_1(4) = -0.1;
    fstar_1(5) = 0.1;

    bool init = true;

    VectorXd tlim;
    tlim.setConstant(rd_.model_dof_, 300);

    rd_.SetTorqueLimit(tlim);

    RobotData rd2_;
    rd2_.LoadModelData(urdf_path, true, false);

    // Gen Matrix File
    // rd2_.check_mat_file_ = true;
    rd_.UpdateKinematics(q, qdot, qddot);

    rd_.CopyKinematicsData(rd2_);
    // rd_.save_mat_file_ = true;

    rd2_.SetContact(true, true);

    REQUIRE_MESSAGE(check_binary("/1/J_C", rd2_.J_C), "J_C is not correct");

    REQUIRE_MESSAGE(check_binary("/1/A_inv_", rd2_.A_inv_), "A_inv_ is not correct");

    REQUIRE_MESSAGE(check_binary("/1/Lambda_contact", rd2_.Lambda_contact), "Lambda_contact is not correct");

    REQUIRE_MESSAGE(check_binary("/1/J_C_INV_T", rd2_.J_C_INV_T), "J_C_INV_T is not correct");

    REQUIRE_MESSAGE(check_binary("/1/N_C", rd2_.N_C), "N_C is not correct");

    REQUIRE_MESSAGE(check_binary("/1/W", rd2_.W), "W is not correct");

    REQUIRE_MESSAGE(check_binary("/1/NwJw", rd2_.NwJw), "NwJw is not correct");

    REQUIRE_MESSAGE(check_binary("/1/W_inv", rd2_.W_inv), "W_inv is not correct");

    rd2_.SetTaskSpace(0, fstar_1);

    rd2_.SetTaskSpace(1, fstar_1.segment(3, 3));

    REQUIRE_MESSAGE(check_binary("/1/torque_grav_", rd2_.CalcGravCompensation()), "Gravity torque is not correct");

    CHECK(rd2_.CalcTaskControlTorque(true));

    REQUIRE_MESSAGE(check_binary("/1/torque_task_", rd2_.torque_task_), "Task torque is not correct");

    CHECK(rd2_.CalcContactRedistribute(true));

    REQUIRE_MESSAGE(check_binary("/1/torque_contact_", rd2_.torque_contact_), "Contact torque is not correct");
}

/*
if (save_mat_file_)
{
    std::string fname = "/h" + std::to_string(ts_.heirarchy_) + "mat";

    write_binary(fname.c_str(), H);

    fname = "/g" + std::to_string(ts_.heirarchy_) + "mat";

    write_binary(fname.c_str(), g);

    fname = "/A" + std::to_string(ts_.heirarchy_) + "mat";

    write_binary(fname.c_str(), A);

    fname = "/ubA" + std::to_string(ts_.heirarchy_) + "mat";

    write_binary(fname.c_str(), ubA);

    fname = "/lbA" + std::to_string(ts_.heirarchy_) + "mat";

    write_binary(fname.c_str(), lbA);
}

if (check_mat_file_)
{
    std::cout << "Matrix Check Mode !! Task Heirarchy : " << ts_.heirarchy_ << std::endl;

    std::string hpath = "/h" + std::to_string(ts_.heirarchy_) + "mat";
    std::cout << std::setw(30) << std::right << "H : " << check_binary(hpath.c_str(), H) << std::endl;

    hpath = "/g" + std::to_string(ts_.heirarchy_) + "mat";
    std::cout << std::setw(30) << std::right << "g : " << check_binary(hpath.c_str(), g) << std::endl;

    hpath = "/A" + std::to_string(ts_.heirarchy_) + "mat";
    std::cout << std::setw(30) << std::right << "A : " << check_binary(hpath.c_str(), A) << std::endl;

    hpath = "/ubA" + std::to_string(ts_.heirarchy_) + "mat";
    std::cout << std::setw(30) << std::right << "ubA : " << check_binary(hpath.c_str(), ubA) << std::endl;

    hpath = "/lbA" + std::to_string(ts_.heirarchy_) + "mat";
    std::cout << std::setw(30) << std::right << "lbA : " << check_binary(hpath.c_str(), lbA) << std::endl;
}

if (save_mat_file_)
{
    write_binary("/hcontact_mat", H);
    write_binary("/gcontact_mat", g);
    write_binary("/Acontact_mat", A_);
    write_binary("/ubAcontact_mat", ubA);
    write_binary("/lbAcontact_mat", lbA);
}

if (check_mat_file_)
{
    std::cout << "Matrix Check Mode !! Contact Redistribution : " << std::endl;

    std::cout << std::setw(30) << std::right << "H : " << check_binary("/hcontact_mat", H) << std::endl;
    std::cout << std::setw(30) << std::right << "g : " << check_binary("/gcontact_mat", g) << std::endl;
    std::cout << std::setw(30) << std::right << "A : " << check_binary("/Acontact_mat", A_) << std::endl;
    std::cout << std::setw(30) << std::right << "ubA : " << check_binary("/ubAcontact_mat", ubA) << std::endl;
    std::cout << std::setw(30) << std::right << "lbA : " << check_binary("/lbAcontact_mat", lbA) << std::endl;
}
*/