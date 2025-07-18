#pragma once
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include "ik/Types.hpp"

namespace ik {

class Configuration {
   public:
    using SE3 = pinocchio::SE3Tpl<Real>;

    using Vector = Eigen::VectorXd;
    using Matrix = Eigen::MatrixXd;

    Configuration(const std::shared_ptr<Model> &model,
                  const std::shared_ptr<Data> &data, const Vector &q0)
        : model_(model), data_(data), q0_(q0), q_(q0) {
        update(q_);
        // fixme use the actual dimensions at compile time
        jacobian_ = Matrix::Zero(6, model->nv);
    }

    Size nq() const { return model_->nq; }
    Size nv() const { return model_->nv; }

    const Model &model() const { return *model_; }

    void update(const Vector &q, bool compute_jacobians = true) {
        this->q_ = q;
        pinocchio::framesForwardKinematics(*model_, *data_, q);
        if (compute_jacobians) {
            pinocchio::computeJointJacobians(*model_, *data_, q);
        }
    }

    SE3 getTransformFrameToWorld(const String &frame) const {
        const auto id = model_->getFrameId(frame);
        if (id == model_->frames.size()) {
            assert("ERROR: Frame does not exist!");
        }
        return data_->oMf[id];
    }

    const Matrix &getFrameJacobian(const String &frame) const {
        const auto id = model_->getFrameId(frame);
        if (id == model_->frames.size()) {
            assert("ERROR: Frame does not exist!");
        }
        pinocchio::getFrameJacobian(*model_, *data_, id, pinocchio::LOCAL,
                                    jacobian_);
        return jacobian_;
    }

    const Eigen::Vector3<Real> &getCentreOfMass() const {
        return pinocchio::centerOfMass(*model_, *data_, this->q_);
    }

    Matrix computeCentreOfMassJacobian() const {
        return pinocchio::jacobianCenterOfMass(*model_, *data_, q_);
    }

    void integrateInPlace(const Vector &v, const Real &dt) {
        std::cout << "q before " << this->q_ << std::endl;
        this->q_ = integrate(v, dt);
        std::cout << "q after " << this->q_ << std::endl;
        this->update(this->q_);
    }

    Vector integrate(const Vector &v, const Real &dt) {
        return pinocchio::integrate(*model_, q_, v * dt);
    }

    const Vector &configuration() const { return q_; }

   private:
    std::shared_ptr<Model> model_;
    std::shared_ptr<Data> data_;
    Vector q0_;
    Vector q_;

    Matrix jacobian_;
};

}  // namespace ik