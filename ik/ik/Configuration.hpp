#include "ik/Types.hpp"
namespace ik {

class Configuration {
   public:
    using SE3 = pinocchio::SE3Tpl<Real>;
    
    using Vector = Eigen::VectorXd;
    using Matrix = Eigen::MatrixXd;

    Configuration(const std::shared_ptr<Model> &model,
                  const std::shared_ptr<Data> &data, const Vector &q0)
        : model_(model), data_(data), q0_(q0) {
        jacobian_ = pinocchio::Data::Matrix6x::Zero(6, model->nv);
    }

    const Size &nq() const { return model_->nq; }
    const Size &nv() const { return model_->nv; }

    void update(const Vector &q, bool compute_jacobians = true) {
        pinocchio::framesForwardKinematics(*model_, *data_, q);
        if (compute_jacobians) {
            pinocchio::computeFrameJacobians(*model_, *data, q);
        }
    }

    SE3 getTransformFrameToWorld(const String &frame) const {
        const auto id = model_->getFrameId(frame);
        if (id == model_->frames.size()) {
            assert("ERROR: Frame does not exist!")
        }
        return data_->oMf[id];
    }

    const Matrix &getFrameJacobian(const String &frame,
                                   pinocchio::Type &type = LOCAL) const {
        const auto id = model_->getFrameId(frame);
        if (id == model_->frames.size()) {
            assert("ERROR: Frame does not exist!")
        }
        pinocchio::getFrameJacobian(*model_, *data_, pinocchio::LOCAL,
                                    jacobian_);
        return jacobian_;
    }

    void integrateInPlace(const Vector &v, const Real &dt) {
        this->q_ = integrate(v, dt);
    }

    Vector integrate(const Vector &v, const Real &dt) {
        return pinocchio::integrate(*model_, v);
    }

    const Vector &configuration() const { return q_; }

   private:
    Vector q_;
    Vector q0_;
    std::shared_ptr<Model> model_;
    std::shared_ptr<Data> data_;

    pinocchio::Data::Matrix6x jacobian_;
};

}  // namespace ik