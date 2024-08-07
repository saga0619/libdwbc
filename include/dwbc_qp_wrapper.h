#ifndef WBHQP_QUADRATICPROGRAM_H
#define WBHQP_QUADRATICPROGRAM_H
/*
QPoases rapper
min   0.5 * x' * H * x + x' * g
  x
    lbA < A x < ubA
    lb < x < ub
*/
#include <iostream>
#include <Eigen/Dense>
#include <qpOASES.hpp>

// #include <osqp++.h>

using namespace Eigen;
using namespace std;
using namespace qpOASES;

namespace DWBC
{

  class CQuadraticProgram
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  public:
    CQuadraticProgram();
    virtual ~CQuadraticProgram();

  public:
    void InitializeProblemSize(const int &num_var, const int &num_cons);
    bool CheckProblemSize(int numbar, int numcons);
    void UpdateMinProblem(const MatrixXd &H, const VectorXd &g);
    void UpdateSubjectToAx(const MatrixXd &A, const VectorXd &lbA, const VectorXd &ubA);
    void UpdateSubjectToAxUb(const MatrixXd &A, const VectorXd &ubA);
    void UpdateSubjectToX(const VectorXd &lb, const VectorXd &ub);
    void DeleteSubjectToAx();
    void DeleteSubjectToX();
    void PrintMinProb();
    void PrintSubjectToAx();
    void PrintSubjectTox();
    void EnableEqualityCondition(const double Tolerance);
    void DisableEqualityCondition();
    // VectorXd SolveQPoases(const int &num_max_iter, bool MPC = true);
    int SolveQPoases(const int &num_max_iter, VectorXd &solv, bool MPC = true);
    void InitOSQP(const int &num_var, const int &num_cons);
    // int SolveOSQP(VectorXd &solv, bool init);

    // OsqpEigen::Solver _osqp;

  private:
    SQProblem _QPprob;
    Options _options;

    bool _bInitialized;
    int _num_var;
    int _num_cons;
    MatrixXd _H;
    VectorXd _g;
    bool _bool_constraint_Ax;
    bool _bool_constraint_AxUb;
    MatrixXd _A;
    VectorXd _lbA;
    VectorXd _ubA;
    bool _bool_constraint_x;
    VectorXd _lb;
    VectorXd _ub;

    void Initialize();
  };

}

#endif // QUADRATICPROGRAM_H