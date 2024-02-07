/*
* Reinforcement Learning and Whole Body Controller Bridge
* (C) 2022 Donghyeon Kim <kdh0429@snu.ac.kr>
*/
#include "rl_wbc_bridge.h"

RlWBCBridge::RlWBCBridge(int env_id)
{
    // Robot Model Loading
    std::string resource_path = URDF_DIR;
    std::string urdf_name = "/dyros_tocabi.urdf";
    std::string urdf_path = resource_path + urdf_name;
    rd_.LoadModelData(urdf_path, true, false);

    q.setZero(rd_.model_.q_size);
    qdot.setZero(rd_.model_.qdot_size);
    qddot.setZero(rd_.model_.qdot_size);
    ctrl_command_.resize(rd_.model_dof_);

    calculated_command_.setZero(rd_.model_dof_);

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

    rd_.AddTaskSpace(0, TASK_LINK_6D, 0, Vector3d::Zero());
    rd_.AddTaskSpace(1, TASK_LINK_ROTATION, 15, Vector3d::Zero());

    VectorXd tlim;
    tlim.setConstant(rd_.model_dof_, 300);
    rd_.SetTorqueLimit(tlim);
}

RlWBCBridge::~RlWBCBridge()
{

}

void RlWBCBridge::reflectAction(py::array_t<double> action)
{
//     double* action_ptr = (double*) action.request().ptr;

//     while (tc_.rd_.is_action_writing_)
//     {
//         clock_nanosleep(CLOCK_MONOTONIC, 0, &t_u10, NULL);
//     }

//     tc_.rd_.is_action_writing_ = true;

//     int data_idx = 0;

//     tc_.rd_.rl_action_phase_ = action_ptr[data_idx];
//     data_idx++;
    
//     for (int i = 0; i <6; i++)
//     {
//         tc_.rd_.rl_action_left_foot_[i] = action_ptr[data_idx];
//         data_idx++;
//     }

//     for (int i = 0; i <6; i++)
//     {
//         tc_.rd_.rl_action_right_foot_[i] = action_ptr[data_idx];
//         data_idx++;
//     }

//     tc_.rd_.is_action_writing_ = false;

}

void RlWBCBridge::UpdateKinematics(py::array_t<double> qpos, py::array_t<double> qvel, py::array_t<double> qacc)
{
    double* qpos_ptr = (double*) qpos.request().ptr;
    double* qvel_ptr = (double*) qvel.request().ptr;
    double* qacc_ptr = (double*) qacc.request().ptr;
    
    q(0) = qpos_ptr[0];
    q(1) = qpos_ptr[1];
    q(2) = qpos_ptr[2];
    q(3) = qpos_ptr[4];
    q(4) = qpos_ptr[5];
    q(5) = qpos_ptr[6];
    q(rd_.system_dof_) = qpos_ptr[3];
    for (int j=0; j<rd_.model_dof_; j++)
    {
        q(j+6) = qpos_ptr[j+7];
    }

    for (int j=0; j<rd_.system_dof_; j++)
    {
        qdot(j) = qvel_ptr[j];
        qddot(j) = qacc_ptr[j];
    }

    rd_.UpdateKinematics(q, qdot, qddot);
}

void RlWBCBridge::SetContact(bool left, bool right)
{
    rd_.SetContact(left, right);
    rd_.CalcContactConstraint();
    rd_.CalcTaskSpace();
}

void RlWBCBridge::SetTaskSpace(int heirarchy, py::array_t<double> f_star)
{
    double* f_star_ptr = (double*) f_star.request().ptr;
    Map<VectorXd> f_star_tmp(f_star_ptr, f_star.size());
    rd_.SetTaskSpace(heirarchy, f_star_tmp);
}

void RlWBCBridge::CalcTorque()
{
    task_init = false;
    rd_.CalcGravCompensation(); 
    rd_.CalcTaskControlTorque(task_init);
    rd_.CalcContactRedistribute(task_init);
}

std::vector<float> RlWBCBridge::getTorqueCommand()
{         
    calculated_command_ = rd_.torque_grav_ + rd_.torque_task_ + rd_.torque_contact_;
    
    for (int i = 0; i < rd_.model_dof_; i++){
        ctrl_command_[i] = calculated_command_(i);
    }
    return ctrl_command_;
}

void RlWBCBridge::Reset()
{
    task_init = true;
}


// Later, in binding code:
PYBIND11_MODULE(dwbc, m) {
py::class_<RlWBCBridge>(m, "RlWBCBridge")
    .def(py::init<int>())
    .def("reflectAction", &RlWBCBridge::reflectAction)
    .def("UpdateKinematics", &RlWBCBridge::UpdateKinematics)
    .def("SetContact", &RlWBCBridge::SetContact)
    .def("SetTaskSpace", &RlWBCBridge::SetTaskSpace)
    .def("CalcTorque", &RlWBCBridge::CalcTorque)
    .def("getTorqueCommand", &RlWBCBridge::getTorqueCommand)
    .def("Reset", &RlWBCBridge::Reset);
}