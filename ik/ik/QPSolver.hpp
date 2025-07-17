#include <Eigen/Eigen>
#include <casadi/casadi.hpp>
#include <ctime>
#include <fstream>
#include <iostream>
#include <vector>

#include "ik/Types.hpp"

namespace ik {

/**
 * @brief Quadratic program solver wrapper for CasADi's qpsol class so that
 * Eigen-types can be used to solve generic quadratic programs
 *
 */
class QPSolver {
   public:
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

    QPSolver(const Size &nx, const Size &nc,
             const std::string &solver = "qpoases")
        : out_(nx, nc) {
        using SX = casadi::SX;
        // Create the program to take in arbitrary hessian and cost matrix of a
        // known size
        casadi::SX H = SX::sym("H", nx, nx);
        casadi::SX g = SX::sym("g", nx, 1);
        casadi::SX A = SX::sym("A", nc, nx);

        casadi::SX x = SX::sym("x", nx, 1);

        casadi::SX f = 0.5 * SX::dot(x, SX::mtimes(H, x)) + SX::dot(g, x);
        casadi::SX c = SX::mtimes(A, x);

        // Assemble all other entries as a parameter vector
        auto p = casadi::SX::vertcat({casadi::SX::reshape(H, nx * nx, 1), g,
                                      casadi::SX::reshape(A, nc * nx, 1)});

        // Create quadratic solver
        qp_ = casadi::qpsol("solver", solver,
                            {{"f", f}, {"g", c}, {"p", p}, {"x", x}});
    }

    void solve(const Eigen::Ref<const Matrix> &H,
               const Eigen::Ref<const Vector> &g,
               const Eigen::Ref<const Matrix> &A,
               const Eigen::Ref<const Vector> &ubA,
               const Eigen::Ref<const Vector> &lbA,
               const Eigen::Ref<const Vector> &ubx,
               const Eigen::Ref<const Vector> &lbx) {
        // Create a vector for the parameters?
        Vector p(H.size() + g.size() + A.size());
        p << Eigen::Map<const Vector>(H.data(), H.size()), g,
            Eigen::Map<const Vector>(A.data(), A.size());

        std::vector<const Real *> arg(casadi::NLPSOL_NUM_IN);
        arg[casadi::NlpsolInput::NLPSOL_P] = p.data();
        arg[casadi::NlpsolInput::NLPSOL_UBX] = ubx.data();
        arg[casadi::NlpsolInput::NLPSOL_LBX] = lbx.data();
        arg[casadi::NlpsolInput::NLPSOL_UBG] = ubA.data();
        arg[casadi::NlpsolInput::NLPSOL_LBG] = lbA.data();

        std::vector<Real *> res(casadi::NLPSOL_NUM_OUT);
        res[casadi::NlpsolOutput::NLPSOL_F] = &out_.f;
        res[casadi::NlpsolOutput::NLPSOL_X] = out_.x.data();
        res[casadi::NlpsolOutput::NLPSOL_LAM_G] = out_.lambda.data();

        // Solve
        qp_(arg, res);
    }

    Real getObjective() const { return out_.f; }
    Vector getPrimalSolution() const { return out_.x; }
    Vector getDualSolution() const { return out_.lambda; }

   private:
    Output out_;

    casadi::Function qp_;
};
}  // namespace ik
