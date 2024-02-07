#include <signal.h>

#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>

#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <ctime>

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
namespace py = pybind11;

#include <chrono>
#include <dwbc.h>
#include <iomanip>
#ifndef URDF_DIR
#define URDF_DIR "./src"
#endif
using namespace DWBC;

volatile bool *prog_shutdown;

class MyRobotData : public DWBC::RobotData
{
public:
    MyRobotData() : RobotData()
    {
    }

    int custum_data;
    double control_time;

    VectorXd torque_command_;

};

class RlWBCBridge {

    public:
        RlWBCBridge(int env_id);
        ~RlWBCBridge();
        void reflectAction(py::array_t<double> action);
        void UpdateKinematics(py::array_t<double> qpos, py::array_t<double> qvel, py::array_t<double> qacc);
        void SetContact(bool left, bool right);
        void SetTaskSpace(int heirarchy, py::array_t<double> f_star);
        void CalcTorque();
        void Reset();

        std::vector<float> getTorqueCommand();

        static void SIGINT_handler(int sig)
        {
            std::cout << " CNTRL : shutdown Signal" << std::endl;
            *prog_shutdown = true;
        }

    private:
        MyRobotData rd_;

        VectorXd q;
        VectorXd qdot;
        VectorXd qddot;

        bool task_init = false;

        VectorXd calculated_command_;
        std::vector<float> ctrl_command_;
};