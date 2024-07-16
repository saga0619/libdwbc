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

    rd_.AddTaskSpace(0, TASK_LINK_6D, 0, Vector3d::Zero());
    rd_.AddTaskSpace(1, TASK_LINK_ROTATION, 15, Vector3d::Zero());

    VectorXd tlim;
    tlim.setConstant(rd_.model_dof_, 300);

    rd_.SetTorqueLimit(tlim);

    rd_.UpdateKinematics(q, qdot, qddot);

    // rd_.AddContactConstraint(left_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    // rd_.AddContactConstraint(right_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);

    // rd_.AddTaskSpace(TASK_LINK_6D, 0, Vector3d::Zero());
    // rd_.AddTaskSpace(TASK_LINK_ROTATION, 15, Vector3d::Zero());

    rd_.SetContact(true, true);
    rd_.CalcContactConstraint();

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

    rd_.AddTaskSpace(0, TASK_LINK_6D, 0, Vector3d::Zero());
    rd_.AddTaskSpace(1, TASK_LINK_ROTATION, 15, Vector3d::Zero());

    VectorXd tlim;
    tlim.setConstant(rd_.model_dof_, 300);

    rd_.SetTorqueLimit(tlim);

    rd_.UpdateKinematics(q, qdot, qddot);

    // rd_.AddContactConstraint(left_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    // rd_.AddContactConstraint(right_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);

    // rd_.AddTaskSpace(TASK_LINK_6D, 0, Vector3d::Zero());
    // rd_.AddTaskSpace(TASK_LINK_ROTATION, 15, Vector3d::Zero());

    rd_.SetContact(true, true);
    rd_.CalcContactConstraint();

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

    rd_.CalcGravCompensation();

    f_name = "/torque_grav_";
    REQUIRE_MESSAGE(check_binary((case_set + f_name).c_str(), rd_.torque_grav_), "Gravity torque is not correct");

    CHECK(rd_.CalcTaskControlTorque(true));

    f_name = "/torque_task_";
    REQUIRE_MESSAGE(check_binary((case_set + f_name).c_str(), rd_.torque_task_), "Task torque is not correct");

    // std::cout << "task cf : " << rd_.ts_.back().contact_qp_.transpose() << std::endl;

    // VectorXd contact_force = rd_.getContactForce(rd_.torque_grav_ + rd_.torque_task_ + rd_.torque_contact_);
    // std::cout << "Contact Force 1 : " << contact_force.transpose() << std::endl;
    // contact_force = rd_.getContactForce(rd_.torque_grav_ + rd_.torque_contact_);
    // std::cout << "Contact Force 2 : " << contact_force.transpose() << std::endl;
    // contact_force = rd_.getContactForce(rd_.torque_contact_);
    // std::cout << "Contact Force 3 : " << contact_force.transpose() << std::endl;

    // rd_.getZMP(contact_force);

    // std::cout << "lf zmp : " << rd_.cc_[0].xc_pos.transpose() - rd_.cc_[0].zmp_pos.transpose() << std::endl;
    // std::cout << "rf zmp : " << rd_.cc_[1].xc_pos.transpose() - rd_.cc_[1].zmp_pos.transpose() << std::endl;

    CHECK(rd_.CalcContactRedistribute(true));

    // std::cout << "contact qp : " << rd_.cf_redis_qp_.transpose() << std::endl;
    // std::cout << "torque_contact_ : " << rd_.torque_contact_.transpose() << std::endl;
    // contact_force = rd_.getContactForce(rd_.torque_grav_ + rd_.torque_task_ + rd_.torque_contact_);
    // std::cout << "Contact Force : " << contact_force.transpose() << std::endl;

    // rd_.getZMP(contact_force);

    // std::cout << "lf zmp : " << rd_.cc_[0].xc_pos.transpose() - rd_.cc_[0].zmp_pos.transpose() << std::endl;
    // std::cout << "rf zmp : " << rd_.cc_[1].xc_pos.transpose() - rd_.cc_[1].zmp_pos.transpose() << std::endl;
    f_name = "/torque_contact_";
    REQUIRE_MESSAGE(check_binary((case_set + f_name).c_str(), rd_.torque_contact_), "Contact torque is not correct");
}

TEST_CASE("LIBDWBC CALCULATION VERIFICATION : CASE 3", "[LIBDWBC]")
{
    double rot_z = M_PI_2;

    Vector3d euler_rotation(0, 0, rot_z);

    // get quaternion from euler angle in radian, euler_rotation
    Eigen::Quaterniond qu = Eigen::AngleAxisd(euler_rotation[0], Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(euler_rotation[1], Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(euler_rotation[2], Eigen::Vector3d::UnitZ());

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

    q << 0, 0, 0.92983, qu.x(), qu.y(), qu.z(),
        0.1, 0.0, -0.24, 0.5, -0.6, 0.0,
        0.05, 0.0, -0.21, 0.7, -0.31, 0.0,
        0, 0, 0,
        0.2, 0.5, 1.5, -1.27, -1.2, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 1.3, 0.1, 1.3, 0, qu.w();

    fstar_1 << 0.4, 2.0, 0.1, 0.3, -0.1, 0.1;
    fstar_2 << 0.1, 0.1, 0.1;

    case_set = "/2";

    rd_.UpdateKinematics(q, qdot, qddot);

    fstar_1.segment(0, 3) = rd_.link_[0].rotm * fstar_1.segment(0, 3);
    fstar_1.segment(3, 3) = rd_.link_[0].rotm * fstar_1.segment(3, 3);

    fstar_2 = rd_.link_[0].rotm * fstar_2;

    int left_foot_id = 6;
    int right_foot_id = 12;

    rd_.AddContactConstraint(left_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    rd_.AddContactConstraint(right_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    rd_.AddContactConstraint(23, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);
    rd_.AddContactConstraint(31, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.04, 0.04);

    rd_.AddTaskSpace(0, TASK_LINK_6D, 0, Vector3d::Zero());
    rd_.AddTaskSpace(1, TASK_LINK_ROTATION, 15, Vector3d::Zero());

    VectorXd tlim;
    tlim.setConstant(rd_.model_dof_, 300);

    rd_.SetTorqueLimit(tlim);

    rd_.UpdateKinematics(q, qdot, qddot);

    // rd_.AddContactConstraint(left_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    // rd_.AddContactConstraint(right_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);

    // rd_.AddTaskSpace(TASK_LINK_6D, 0, Vector3d::Zero());
    // rd_.AddTaskSpace(TASK_LINK_ROTATION, 15, Vector3d::Zero());

    rd_.SetContact(true, true);
    rd_.CalcContactConstraint();

    std::string f_name = "/J_C";

    rd_.SetTaskSpace(0, fstar_1);
    rd_.SetTaskSpace(1, fstar_2);

    f_name = "/torque_grav_";
    REQUIRE_MESSAGE(check_binary((case_set + f_name).c_str(), rd_.CalcGravCompensation()), "Gravity torque is not correct");

    CHECK(rd_.CalcTaskControlTorque(true));
    f_name = "/torque_task_";
    REQUIRE_MESSAGE(check_binary((case_set + f_name).c_str(), rd_.torque_task_), "Task torque is not correct");

    CHECK(rd_.CalcContactRedistribute(true));

    std::cout << rd_.link_[0].rotm.inverse() << std::endl;

    VectorXd con_qp = rd_.ts_.back().contact_qp_;
    con_qp.segment(0, 3) = rd_.link_[0].rotm.inverse() * con_qp.segment(0, 3);
    con_qp.segment(3, 3) = rd_.link_[0].rotm.inverse() * con_qp.segment(3, 3);
    std::cout << "task cf : " << con_qp.transpose() << std::endl;
    std::cout << "contact qp : " << rd_.cf_redis_qp_.transpose() << std::endl;
    // std::cout << "torque_contact_ : " << rd_.torque_contact_.transpose() << std::endl;
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

    rd_.AddTaskSpace(0, TASK_LINK_6D, 0, Vector3d::Zero());
    rd_.AddTaskSpace(1, TASK_LINK_ROTATION, 15, Vector3d::Zero());

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
    rd_.CalcContactConstraint();

    BENCHMARK("contact space calculation")
    {
        rd_.SetContact(true, true);
        rd_.CalcContactConstraint();
    };

    rd_.SetTaskSpace(0, fstar_1);
    rd_.SetTaskSpace(1, fstar_1.segment(3, 3));
    rd_.CalcGravCompensation();
    BENCHMARK("grav torque calculation")
    {
        rd_.CalcGravCompensation();
    };
    rd_.CalcTaskSpace();
    BENCHMARK("Update task space")
    {
        rd_.CalcTaskSpace();
    };
    PinvCODWB(rd_.W, rd_.W_inv);
    BENCHMARK("Matrix decomposition")
    {
        PinvCODWB(rd_.W, rd_.W_inv);
    };

    rd_.CalcTaskControlTorque(true);

    BENCHMARK("task control calculation (Update task + QP calculation + matrix preperation)")
    {
        rd_.CalcTaskControlTorque(true, false);
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
        rd_.CalcContactRedistribute(true, false);
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
    // /0.1820621, -0.007949, 0.042889, 0.9823191
    q << 0, 0, 0.92983, 0, 0, 0.7071068,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0, 0, 0,
        0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 0.7071068;

    // q << 0, 0, 0.92983, 0, 0, 0,
    //     0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
    //     0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
    //     0, 0, 0,
    //     0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
    //     0, 0,
    //     -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;

    qdot.setRandom(rd_.model_.qdot_size);
    qdot.segment(0, 6).setZero();

    int left_foot_id = 6;
    int right_foot_id = 12;

    rd_.AddContactConstraint(left_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);
    rd_.AddContactConstraint(right_foot_id, CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075);

    // std::cout << qdot.transpose() << std::endl;

    rd_.UpdateKinematics(q, qdot, qddot);

    rd_.SetContact(true, true);
    rd_.CalcContactConstraint();

    Eigen::Vector3d ang_momentum_from_rbdl, ang_momentum_from_dwbc;

    Eigen::MatrixXd com_inertia;
    Eigen::VectorXd com_mom;

    double mass_;
    RigidBodyDynamics::Math::Vector3d com;
    RigidBodyDynamics::Math::Vector3d _com_acc;
    RigidBodyDynamics::Math::Vector3d _com_vel;
    RigidBodyDynamics::Math::Vector3d _ang_momentum;
    RigidBodyDynamics::Utils::CalcCenterOfMass(rd_.model_, q, qdot, &qddot, mass_, com, &_com_vel, &_com_acc, &_ang_momentum);
    // rd_.CalcCOMInertia(rd_.link_, com_inertia, com_mom);
    // BENCHMARK("calc com information")
    // {
    //     rd_.CalcCOMInertia(rd_.link_, com_inertia, com_mom);
    // };
    ang_momentum_from_rbdl = _ang_momentum;
    // std::cout << "\nang momentum from rbdl : " << _ang_momentum.transpose() << std::endl;
    // std::cout << "com : " << com.transpose() << std::endl;

    // std::cout << " com inertia : " << std::endl;
    // std::cout << com_inertia << std::endl;

    // std::cout << rd_.link_.back().inertia << std::endl;

    // std::cout << " off diagonal " << std::endl;
    // std::cout << rd_.total_mass_ * skew(rd_.com_pos - rd_.link_[0].xpos) << std::endl;

    Eigen::VectorXd ans3 = Eigen::VectorXd::Zero(6);
    // ans2 = rd_.CalcAngularMomentumMatrix() * qdot;
    ang_momentum_from_dwbc = (rd_.CMM_ * qdot).segment(3, 3);
    BENCHMARK("angular momentum matrix calculation")
    {
        ang_momentum_from_dwbc = (rd_.CMM_ * qdot).segment(3, 3);
    };

    // std::cout << "\nCMM from rd_ : " << std::endl
    //           << ang_momentum_from_dwbc.transpose() << std::endl;
    // std::cout << " com : " << rd_.com_pos.transpose() << std::endl;

    // BENCHMARK("angular momentum matrix calculation2")
    // {
    //     MatrixXd H_temp = MatrixXd::Zero(6, rd_.system_dof_);

    // std::cout << "vi of link 15 : " << rd_.link_[27].vi.transpose() << std::endl;
    // std::cout << "vi custom of link 15 : " << rd_.link_[27].v + rd_.link_[27].w.cross(rd_.link_[27].rotm * rd_.link_[27].com_position_l_) << std::endl;

    //     MatrixXd rot_temp = MatrixXd::Identity(6, 6);
    //     for (int i = 0; i < rd_.link_.size(); i++)
    //     {
    //         H_temp += rot_temp * rd_.link_[i].GetSpatialInertiaMatrix(false) * rd_.link_[i].jac_;
    //     }
    // };

    // std::cout << std::endl
    //           << "answer comparison : " << std::endl;
    // std::cout << (rd_.com_vel * rd_.total_mass_).transpose() << " " << ans1.transpose() << std::endl;

    // std::cout << rd_.link_[0].rotm << std::endl;

    // std::cout << "pelvis mass : " << rd_.link_[0].mass << " l com pos : " << rd_.link_[0].com_position_l_ << std::endl;
    // std::cout << "pelvis inertia : " << std::endl;
    // std::cout << rd_.link_[0].inertia << std::endl;

    // std::cout << "IC rbdl " << std::endl;
    // std::cout << rd_.model_.I[rd_.link_[0].body_id_] << std::endl;

    // std::cout << "qdot : " << qdot.transpose() << std::endl;
    // std::cout << cm_rot6 << std::endl;
    // std::cout << (cm_rot6 * rd_.A_.topRows(6) * qdot).transpose() << std::endl; // cmm calc, "Improved Computation of the Humanoid Centroidal Dynamics and Application for Whole-Body Control"

    // std::cout << "ang affect : " << A12.transpose() << std::endl;

    // std::cout << "Amatrix : " << std::endl;

    // std::cout << rd_.A_ << std::endl;

    // std::cout << "pelvis : " << rd_.link_[0].xpos.transpose() << std::endl;

    // std::cout << rd_.link_[0].jac_ << std::endl;

    // std::cout << "upperbody : " << rd_.link_[15].xpos.transpose() << std::endl;
    // std::cout << rd_.link_[15].jac_ << std::endl;

    // std::cout << "Right Hand : " << rd_.link_[31].xpos.transpose() << std::endl;
    // std::cout << rd_.link_[31].jac_ << std::endl;

    BENCHMARK("RBDL : UPDATEKINEMATICS ")
    {
        RigidBodyDynamics::UpdateKinematicsCustom(rd_.model_, &rd_.q_system_, &rd_.q_dot_system_, &rd_.q_ddot_system_);
    };

    BENCHMARK("RBDL : COMPOSITE ")
    {
        RigidBodyDynamics::CompositeRigidBodyAlgorithm(rd_.model_, rd_.q_system_, rd_.A_, false);
    };

    Eigen::MatrixXd cmm = Eigen::MatrixXd::Zero(3, rd_.model_.qdot_size);
    // BENCHMARK("angular momentum matrix calculation2")
    // {
    //     // rd_.CalcAngularMomentumMatrix(cmm);

    //     ans3 = rd_.CMM_ * qdot;
    // };

    //
    //
    // Eigen::MatrixXd A_rot_ = Eigen::MatrixXd::Identity(39, 39);
    // A_rot_.block(3, 3, 3, 3) = rd_.link_[0].rotm;
    // Eigen::MatrixXd A_global = A_rot_ * rd_.A_ * A_rot_.transpose();
    Eigen::Matrix3d upper_ic;
    Matrix6d i_temp_6 = Eigen::MatrixXd::Zero(6, 6);

    // BENCHMARK("UPPERBODY MASS MATRIX CALCULATION")
    // {
    //     i_temp_6.setZero();
    //     for (int i = 0; i < 21; i++)
    //     {

    //         // std::cout << "link " << i + 13 << " : " << rd_.link_[i + 13].name_ << std::endl;
    //         Matrix6d I_rotation = Matrix6d::Zero(6, 6);
    //         Matrix6d temp1 = Matrix6d::Identity(6, 6);
    //         temp1.block(0, 0, 3, 3) = rd_.link_[i + 13].rotm.transpose();
    //         temp1.block(3, 3, 3, 3) = rd_.link_[i + 13].rotm.transpose();

    //         temp1.block(0, 3, 3, 3) = -rd_.link_[i + 13].rotm.transpose() * skew(rd_.link_[i + 13].xpos - rd_.link_[0].xpos);
    //         i_temp_6 += temp1.transpose() * rd_.link_[i + 13].GetSpatialInertiaMatrix(false) * temp1;
    //     }
    //     double upper_mass = 42.6195;
    //     Matrix6d skm_temp = i_temp_6.block(3, 0, 3, 3) / upper_mass;
    //     Eigen::Vector3d com_upper(skm_temp(2, 1), skm_temp(0, 2), skm_temp(1, 0));
    //     upper_ic = i_temp_6.block(3, 3, 3, 3) - upper_mass * skew(com_upper) * skew(com_upper).transpose();
    // };

    // std::cout << " calculated upperbody mass mat : " << std::endl;
    // std::cout << i_temp_6 << std::endl;
    // std::cout << "upper IC : " << std::endl;
    // std::cout << rd_.link_[0].rotm.transpose() * upper_ic * rd_.link_[0].rotm << std::endl;
    // std::cout << " calcul
    // std::cout << " com x : " << rd_.link_.back().v.transpose() << std::endl;
    // std::cout << " com xi : " << rd_.link_.back().vi.transpose() << std::endl;
    // std::cout << " com jac : " << std::endl
    //           << rd_.link_.back().jac_ << std::endl;
    // std::cout << " com jac 2 : " << std::endl
    //           << rd_.link_.back().jac_com_ << std::endl;
    // Eigen::MatrixXd H_C = Eigen::MatrixXd::Zero(6, rd_.model_.qdot_size);

    BENCHMARK("SEQDYN CALC")
    {
        rd_.ReducedDynamicsCalculate();
    };

    // std::cout << "13" << std::endl;
    // std::cout << ang_momentum_from_dwbc.transpose() << std::endl;
    // std::cout << ang_momentum_from_rbdl.transpose() << std::endl;

    REQUIRE(ang_momentum_from_rbdl.segment(0, 3).isApprox((ang_momentum_from_dwbc.segment(0, 3)), 1e-5));
    // rd_.SequentialDynamicsCalculate(true);
    // Eigen::MatrixXd H_C = MatrixXd::Zero(6, rd_.system_dof_);

    // int i = 0;
    // Eigen::MatrixXd rotm = MatrixXd::Identity(6, 6);
    // Eigen::MatrixXd j_temp = MatrixXd::Zero(6, rd_.system_dof_);
    // Eigen::MatrixXd j_temp2 = rd_.link_[6].jac_;

    // Eigen::MatrixXd ans;

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

    rd_.AddTaskSpace(0, TASK_LINK_6D, "pelvis_link", Vector3d::Zero());
    rd_.AddTaskSpace(1, TASK_LINK_ROTATION, "upperbody_link", Vector3d::Zero());

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
    rd2_.CalcContactConstraint();

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

TEST_CASE("2LEVEL WBC TEST")
{
    double rot_z = M_PI_2;

    RobotData rd_;

    std::string urdf_file = std::string(URDF_DIR) + "/dyros_tocabi.urdf";

    std::string desired_control_target = "COM";

    VectorXd fstar_1;
    fstar_1.setZero(6);
    fstar_1 << 0.11, 0.5, 0.13, 0.12, -0.11, 0.05; // based on local frmae.

    // fstar_1
    rd_.LoadModelData(urdf_file, true, false);

    VectorXd t2lim;
    VectorXd q2 = VectorXd::Zero(rd_.model_.q_size);
    VectorXd q2dot = VectorXd::Zero(rd_.model_.qdot_size);
    VectorXd q2ddot = VectorXd::Zero(rd_.model_.qdot_size);

    VectorXd tlim;
    tlim.setConstant(rd_.model_dof_, 500);
    rd_.SetTorqueLimit(tlim);

    Vector3d euler_rotation(0, 0, rot_z);

    // get quaternion from euler angle in radian, euler_rotation
    Eigen::Quaterniond qu = Eigen::AngleAxisd(euler_rotation[0], Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(euler_rotation[1], Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(euler_rotation[2], Eigen::Vector3d::UnitZ());

    q2 << 0, 0, 0.92983, qu.x(), qu.y(), qu.z(),
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0, 0, 0,
        0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, qu.w();

    bool verbose = false;
    rd_.UpdateKinematics(q2, q2dot, q2ddot);
    rd_.AddContactConstraint("l_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);
    rd_.AddContactConstraint("r_ankleroll_link", CONTACT_TYPE::CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, verbose);

    rd_.AddTaskSpace(0, TASK_LINK_6D, desired_control_target.c_str(), Vector3d::Zero(), verbose);
    // rd2_.AddTaskSpace(TASK_LINK_ROTATION, desired_control_target.c_str(), Vector3d::Zero(), verbose);

    rd_.SetContact(true, true);
    rd_.CalcContactConstraint();

    fstar_1.segment(0, 3) = rd_.link_[0].rotm * fstar_1.segment(0, 3);
    fstar_1.segment(3, 3) = rd_.link_[0].rotm * fstar_1.segment(3, 3);

    rd_.SetTaskSpace(0, fstar_1);

    rd_.CalcGravCompensation();
    rd_.CalcTaskControlTorque(true);
    rd_.CalcContactRedistribute(true);

    MatrixXd original_J_task = rd_.ts_[0].J_task_;

    // std::cout << "Result Task Torque : " << std::endl;
    // std::cout << rd_.torque_grav_.transpose() << std::endl;
    // std::cout << rd_.torque_task_.transpose() << std::endl;

    MatrixXd s_k = MatrixXd::Zero(rd_.model_dof_, rd_.model_dof_ + 6);
    s_k.block(0, 6, rd_.model_dof_, rd_.model_dof_) = MatrixXd::Identity(rd_.model_dof_, rd_.model_dof_);
    MatrixXd give_me_fstar = rd_.ts_[0].J_task_ * rd_.A_inv_ * rd_.N_C * s_k.transpose();

    // std::cout << "fstar : " << (give_me_fstar * rd_.torque_task_).transpose() << std::endl;

    BENCHMARK("2LEVEL WBC")
    {
        rd_.UpdateKinematics(q2, q2dot, q2ddot);
        rd_.SetContact(true, true);
        rd_.CalcContactConstraint();
        rd_.SetTaskSpace(0, fstar_1);

        rd_.CalcGravCompensation();

        rd_.CalcTaskControlTorque(true);
        rd_.CalcContactRedistribute(true);
    };
};

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