#include <chrono>
#include <dwbc/dwbc.h>

class MyRobotData : public DWBC::RobotData
{
    int custum_data;
    double control_time;
};


using namespace DWBC;

int main()
{
    MyRobotData rd_;

    std::string urdf_path = "../../test/dyros_tocabi.urdf";

    rd_.InitModelData(urdf_path, true, 1);

    VectorXd q;
    VectorXd qdot;
    VectorXd qddot;

    q.setZero(rd_.model_.q_size);
    qdot.setZero(rd_.model_.qdot_size);
    qddot.setZero(rd_.model_.qdot_size);

    MatrixXd j_tmp;

    int repeat_time = 10000;

    auto t_start = std::chrono::high_resolution_clock::now();

    int mid_consume = 0;

    for (int i = 0; i < repeat_time; i++)
    {
        q.setRandom();
        qdot.setRandom();
        rd_.UpdateKinematics(q, qdot, qddot);

        // std::cout << Vector3d(0, 0, -9.81) << std::endl;

        // auto t_midPoint = std::chrono::high_resolution_clock::now();

        // for (int i = 0; i < rd_.link_.size(); i++)
        // {
        //     j_tmp = rd_.link_[i].JacCOM();

        // }

        // mid_consume += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t_midPoint).count();
    }

    auto t_dur = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t_start);

    // std::cout << q.transpose() << std::endl;
    // std::cout << qdot.transpose() << std::endl;

    std::cout << "Update Kinematics Random Test complete. : " << t_dur.count() / repeat_time << " us" << std::endl;

    q.setZero();
    q << 0, 0, 0.92983, 0, 0, 0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
        0, 0, 0,
        0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
        0, 0,
        -0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0, 1;

    // q << 0, 0, 0, 8.72949e-06, -0.000191618, -3.56927e-07, 7.21486e-07, -1.14801e-07, -0.233464, 0.581483, -0.348292, -1.31239e-06, -1.15455e-07, -4.6496e-07, -0.233484, 0.581517, -0.348293, -1.16079e-06, -3.29457e-07, -0.000329052, -1.22101e-07, 0.300002, 0.300095, 1.5, -1.27, -1.00115, 2.3016e-07, -1, -3.5045e-08, 1.36636e-06, -0.000406735, -0.300001, -0.300095, -1.5, 1.27, 1.00115, -4.30943e-08, 1, 5.41439e-09, 1;

    qdot.setZero();
    rd_.UpdateKinematics(q, qdot, qddot);

    // std::cout << "total mass : " << rd_.total_mass_ << std::endl;
    // std::cout << rd_.link_[0].xpos.transpose() << std::endl;
    // std::cout << rd_.link_[0].rotm;

    int left_foot_id = 6;
    int right_foot_id = 12;

    // Add ContactConstraint
    rd_.AddContactConstraint(left_foot_id, CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, true);
    rd_.AddContactConstraint(right_foot_id, CONTACT_6D, Vector3d(0.03, 0, -0.1585), Vector3d(0, 0, 1), 0.15, 0.075, true);
    // rd_.AddContactConstraint(33, 0, Vector3d(0, 0, 0), Vector3d(0, 0, 1), 0.1, 0.1, true);

    rd_.SetContact(true, true);
    // rd_.CalcContactConstraint();

    // Add taskspace info
    // rd_.AddTaskSpace(TASK_LINK_6D, 0, rd_.link_[0].com_position_l_, true);
    rd_.AddTaskSpace(TASK_LINK_6D, 0, Vector3d::Zero(), true);
    rd_.AddTaskSpace(TASK_LINK_ROTATION, 15, Vector3d::Zero(), true);

    VectorXd fstar_1;
    fstar_1.setZero(6);
    fstar_1(0) = 0.1;
    fstar_1(1) = 2.0;
    fstar_1(2) = 0.1;
    rd_.SetTaskSpace(0, fstar_1);
    rd_.SetTaskSpace(1, Vector3d::Zero());

    std::cout << "Calc Task Space" << std::endl;

    rd_.CalcTaskSpace(); // Calculate Task Spaces...

    std::cout << "Calc grav compensation " << std::endl;

    rd_.CalcGravCompensation(); // Calulate Gravity Compensation

    std::cout << "Calc Task Torque times..." << std::endl;

    rd_.CalcTaskTorque(true, true);
    t_start = std::chrono::high_resolution_clock::now();
    for (int i = 0; i < 10000; i++)
    {
        rd_.CalcTaskTorque(true, false);
    }
    t_dur = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - t_start);

    std::cout << "Calc Task Torque : " << t_dur.count() / 10000 << " us" << std::endl;

    std::cout << "Calc Contact Redistribute" << std::endl;

    rd_.CalcContactRedistribute();

    std::cout << " Grav Torque : " << rd_.torque_grav_.transpose() << std::endl;
    std::cout << " Task Torque : " << rd_.torque_task_.transpose() << std::endl;
    std::cout << "contact Torque : " << rd_.torque_contact_.transpose() << std::endl;

    // std::cout << "contact force before : " << rd_.getContactForce(rd_.torque_grav_).transpose() << std::endl;

    // std::cout << "contact force before : " << rd_.getContactForce(rd_.torque_grav_ + rd_.torque_task_).transpose() << std::endl;

    std::cout << "contact force after : " << rd_.getContactForce(rd_.torque_grav_ + rd_.torque_task_ + rd_.torque_contact_).transpose() << std::endl;

    // std::cout<<"V2"<<std::endl<<rd_.V2<<std::endl;

    // std::cout<<"NwJw"<<std::endl<<rd_.NwJw<<std::endl;

    // std::cout << "mid consume : " << mid_consume / repeat_time << " us" << std::endl;
}

// wholebodycontrollibrary

/*

wbcl

WholeBodyControlLibrary::

WholeBodyModel;





*/
