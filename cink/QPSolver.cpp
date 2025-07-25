#include "cink/QPSolver.hpp"

namespace cink {

QPSolver::QPSolver(const Size &nx, const Size &nc, const std::string &solver,
                   const Options &opts)
    : nx_(nx), nc_(nc), out_(nx, nc) {
    // Create the QP solver
    conic_f_ = casadi::conic("casadi_qpsol", solver,
                             {{"h", casadi::Sparsity::dense({nx, nx})},
                              {"a", casadi::Sparsity::dense({nc, nx})}},
                             opts);
}

void QPSolver::solve(const Eigen::Ref<const Matrix> &H,
                     const Eigen::Ref<const Vector> &g,
                     const Eigen::Ref<const Matrix> &A,
                     const Eigen::Ref<const Vector> &ubA,
                     const Eigen::Ref<const Vector> &lbA,
                     const Eigen::Ref<const Vector> &ubx,
                     const Eigen::Ref<const Vector> &lbx) {
    std::vector<const double *> w(casadi::CONIC_NUM_IN);

    w[casadi::CONIC_H] = H.data();
    w[casadi::CONIC_G] = g.data();
    w[casadi::CONIC_A] = A.data();
    w[casadi::CONIC_Q] = nullptr;
    w[casadi::CONIC_P] = nullptr;
    w[casadi::CONIC_LBX] = lbx.data();
    w[casadi::CONIC_UBX] = ubx.data();
    w[casadi::CONIC_LBA] = lbA.data();
    w[casadi::CONIC_UBA] = ubA.data();
    w[casadi::CONIC_X0] = nullptr;
    w[casadi::CONIC_LAM_X0] = nullptr;
    w[casadi::CONIC_LAM_A0] = nullptr;

    std::vector<Real *> res(casadi::CONIC_NUM_OUT);
    res[casadi::CONIC_COST] = &out_.f;
    res[casadi::CONIC_X] = out_.x.data();
    res[casadi::CONIC_LAM_A] = out_.lambda.data();

    conic_f_(w, res);
}

}  // namespace cink
