#include <Eigen/Eigen>
#include <casadi/casadi.hpp>
#include <ctime>
#include <fstream>
#include <iostream>
#include <vector>

#include "cink/Types.hpp"

namespace cink {

/**
 * @brief Quadratic program solver wrapper for CasADi's qpsol class so that
 * Eigen-types can be used to solve generic quadratic programs by efficient
 * solvers supported by CasADi
 *
 */
class QPSolver {
   public:
    using Options = casadi::Dict;

    using Matrix = Eigen::MatrixX<Real>;
    using Vector = Eigen::VectorX<Real>;

    struct Input {};

    struct Output {
        Output(const Size &nx, const Size &nc) {
            x = Vector::Zero(nx);
            lambda = Vector::Zero(nc);
        }
        /// @brief Objective value
        Real f;
        /// @brief Primal solution
        Vector x;
        /// @brief Dual solution
        Vector lambda;
    };

    Size nx() const { return nx_; }
    Size nc() const { return nc_; }

    QPSolver(const Size &nx, const Size &nc,
             const std::string &solver = "qpoases", const Options &opts = {});
             
    void solve(const Eigen::Ref<const Matrix> &H,
               const Eigen::Ref<const Vector> &g,
               const Eigen::Ref<const Matrix> &A,
               const Eigen::Ref<const Vector> &ubA,
               const Eigen::Ref<const Vector> &lbA,
               const Eigen::Ref<const Vector> &ubx,
               const Eigen::Ref<const Vector> &lbx);

    Real getObjective() const { return out_.f; }
    Vector getPrimalSolution() const { return out_.x; }
    Vector getDualSolution() const { return out_.lambda; }

   private:
    Size nx_;
    Size nc_;
    Output out_;

    casadi::Function qp_;
};
}  // namespace ik
