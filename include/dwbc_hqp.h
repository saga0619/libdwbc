#include <dwbc_qp_wrapper.h>
#include <dwbc_math.h>
#include <vector>
namespace DWBC
{

    class HQP_Hierarch
    {
    public:
        HQP_Hierarch();
        ~HQP_Hierarch();

        void initialize(const int &hierarchy_level, const int &acceleration_size, const int &torque_size, const int &contact_size, const int &ineq_const_size, const int &eq_const_size);
        void updateConstraintMatrix(const MatrixXd &A, const VectorXd &a, const MatrixXd &B, const VectorXd &b);
        void updateConstraintWeight(const MatrixXd &V, const MatrixXd &W);
        void updateCostMatrix(const MatrixXd &H, const VectorXd &g);
        void normalizeConstraintMatrix();


        void solve(VectorXd &qp_ans, bool init = true);

        
        int hierarchy_level_;

        int acceleration_size_;
        int torque_size_;
        int contact_size_;

        int variable_size_;
        int eq_const_size_;
        int ineq_const_size_;
        
        int null_space_size_;

        VectorXd y_;         //decision variable robot status (acceleration, contactforce, torque)
        VectorXd u_;         //decision variable for previous eq constraint null-space

        bool enable_cost_;
        MatrixXd H_;         //cost matrix                             (variable_size_ x variable_size_)
        VectorXd g_;         //cost vector                             (variable_size_ x 1)
        
        /**
         * Ay+a <= 0
         * inequality constraint
         * to solve V(Ay+a) <= v
         */
        MatrixXd A_;         //inequality constraint matrix             (ineq_const_size_ x variable_size_) 
        VectorXd a_;         //inequality constraint vector             (ineq_const_size_ x 1)
        MatrixXd V_;         //inequality weigh matrix                  (ineq_const_size_ x ineq_const_size_)
        VectorXd v_;         //inequality slack (optimize variable)     (ineq_const_size_ x 1)

        /**
         * By+b = 0
         * equality constraint
         * to solve W(By+b) = w
         */
        MatrixXd B_;         //equality constraint matrix               (eq_const_size_ x variable_size_)
        VectorXd b_;         //equality constraint vector               (eq_const_size_ x 1)
        MatrixXd W_;         //equality weigh matrix                    (eq_const_size_ x eq_const_size_)
        VectorXd w_;         //equality slack (optimize variable)       (eq_const_size_ x 1)

        MatrixXd B_nulled_;  // B * Z                                   (variable_size_ x status_size_)


        MatrixXd Z_;         //null space projection matrix of B        (variable_size_ x status_size_)


        VectorXd y_ans_;
        VectorXd v_ans_;
        VectorXd w_ans_;

        MatrixXd qp_H_;
        VectorXd qp_g_;
        MatrixXd qp_A_;
        VectorXd qp_lbA_;
        VectorXd qp_ubA_;

        VectorXd qp_lb_;
        VectorXd qp_ub_;

        int qp_variable_size_;
        int qp_constraint_size_;

        CQuadraticProgram qp_solver_;
    };

    class HQP
    {
    public:
        HQP();
        ~HQP();

        void initialize(const int &acceleration_size, const int &torque_size, const int &contact_size);
        void addHierarchy(const int &ineq_const_size, const int &eq_const_size);

        void prepare(bool verbose = false);
        void solveSequential(bool init = true, bool verbose = false);
        void solveWeighted(bool init = true);
        // void solveSequential(bool init = true);
        void solvefirst(bool init = true);


        int acceleration_size_;
        int torque_size_;
        int contact_size_;

        std::vector<HQP_Hierarch> hqp_hs_;
    };

}
