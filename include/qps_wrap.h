#ifndef DWBC_QPSWIFT_H
#define DWBC_QPSWIFT_H

#include <qpSWIFT/qpSWIFT.h>

using namespace Eigen;

namespace DWBC
{

    VectorXd qpSwiftSolve(QP *qpp, int var_size, int const_size, MatrixXd &H, VectorXd &G, MatrixXd &A, VectorXd &Ub, bool verbose);
};
#endif