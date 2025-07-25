#include "ik/Configuration.hpp"

namespace ik {

Configuration::Configuration(
    const std::shared_ptr<Model> &model, const std::shared_ptr<Data> &data,
    const Vector &q0, const std::shared_ptr<CollisionModel> &collision_model,
    const std::shared_ptr<CollisionData> &collision_data)
    : model_(model),
      data_(data),
      collision_model_(collision_model),
      collision_data_(collision_data),
      q0_(q0),
      q_(q0) {
    update(q_);
    jacobian_ = Matrix6x::Zero(6, model->nv);
}

pinocchio::JointIndex Configuration::getJointIndex(const String &joint) const {
    const auto id = model_->getJointId(joint);
    if (id == model_->joints.size()) {
        throw std::runtime_error("Joint does not exist!");
    }
    return id;
}

pinocchio::FrameIndex Configuration::getFrameIndex(const String &frame) const {
    const auto id = model_->getFrameId(frame);
    if (id == model_->frames.size()) {
        throw std::runtime_error("Frame does not exist!");
    }
    return id;
}

void Configuration::update(const Vector &q) {
    this->q_ = q;
    pinocchio::framesForwardKinematics(*model_, *data_, q);
    pinocchio::computeJointJacobians(*model_, *data_, q);

    if (collision_model_ && collision_data_) {
        pinocchio::updateGeometryPlacements(*model_, *data_, *collision_model_,
                                            *collision_data_);
        pinocchio::computeCollisions(*collision_model_, *collision_data_);
        pinocchio::computeDistances(*collision_model_, *collision_data_);
    }
}

Configuration::SE3 Configuration::getTransformFrameToWorld(
    const String &frame) const {
    const auto id = model_->getFrameId(frame);
    if (id == model_->frames.size()) {
        assert("ERROR: Frame does not exist!");
    }
    return data_->oMf[id];
}

const Configuration::Matrix6x &Configuration::getJointJacobian(
    const pinocchio::JointIndex &index,
    const pinocchio::ReferenceFrame &reference_frame) const {
    if (index == model_->joints.size()) {
        assert("ERROR: Joint does not exist!");
    }
    pinocchio::getJointJacobian(*model_, *data_, index, reference_frame,
                                jacobian_);
    return jacobian_;
}

const Configuration::Matrix6x &Configuration::getFrameJacobian(
    const pinocchio::FrameIndex &index,
    const pinocchio::ReferenceFrame &reference_frame) const {
    if (index == model_->frames.size()) {
        assert("ERROR: Frame does not exist!");
    }
    pinocchio::getFrameJacobian(*model_, *data_, index, reference_frame,
                                jacobian_);
    return jacobian_;
}

const Eigen::Vector3<Real> &Configuration::getCentreOfMass() const {
    return pinocchio::centerOfMass(*model_, *data_, this->q_);
}

Configuration::Matrix Configuration::computeCentreOfMassJacobian() const {
    return pinocchio::jacobianCenterOfMass(*model_, *data_, q_);
}

void Configuration::integrateInPlace(const Vector &v, const Real &dt) {
    this->q_ = integrate(v, dt);
    this->update(this->q_);
}

Configuration::Vector Configuration::integrate(const Vector &v,
                                               const Real &dt) {
    return pinocchio::integrate(*model_, q_, v * dt);
}

}  // namespace ik