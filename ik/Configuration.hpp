#pragma once
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/collision/collision.hpp>

#include "ik/Types.hpp"

namespace ik {

class Configuration {
   public:
    using SE3 = pinocchio::SE3Tpl<Real>;

    using Vector = Eigen::VectorXd;
    using Matrix = Eigen::MatrixXd;
    using Matrix6x = typename Data::Matrix6x;

    using CollisionModel = pinocchio::GeometryModel;
    using CollisionData = pinocchio::GeometryData;

    Configuration(
        const std::shared_ptr<Model> &model, const std::shared_ptr<Data> &data,
        const Vector &q0,
        const std::shared_ptr<CollisionModel> &collision_model = nullptr,
        const std::shared_ptr<CollisionData> &collision_data = nullptr)
        : model_(model),
          data_(data),
          collision_model_(collision_model),
          collision_data_(collision_data),
          q0_(q0),
          q_(q0) {
        update(q_);
        jacobian_ = Matrix6x::Zero(6, model->nv);
    }

    Size nq() const { return model_->nq; }
    Size nv() const { return model_->nv; }

    const Model &model() const { return *model_; }
    const Data &data() const { return *data_; }

    const CollisionModel &collisionModel() const { return *collision_model_; }
    const CollisionData &collisionData() const { return *collision_data_; }

    void update(const Vector &q) {
        this->q_ = q;
        pinocchio::framesForwardKinematics(*model_, *data_, q);
        pinocchio::computeJointJacobians(*model_, *data_, q);

        if (collision_model_ && collision_data_) {
            pinocchio::updateGeometryPlacements(
                *model_, *data_, *collision_model_, *collision_data_);
            pinocchio::computeCollisions(*collision_model_, *collision_data_);
            pinocchio::computeDistances(*collision_model_, *collision_data_);
        }
    }

    SE3 getTransformFrameToWorld(const String &frame) const {
        const auto id = model_->getFrameId(frame);
        if (id == model_->frames.size()) {
            assert("ERROR: Frame does not exist!");
        }
        return data_->oMf[id];
    }

    const Matrix6x &getJointJacobian(
        const pinocchio::JointIndex &index,
        const pinocchio::ReferenceFrame &reference_frame =
            pinocchio::LOCAL) const {
        if (index == model_->joints.size()) {
            assert("ERROR: Joint does not exist!");
        }
        pinocchio::getJointJacobian(*model_, *data_, index, reference_frame,
                                    jacobian_);
        return jacobian_;
    }

    const Matrix6x &getFrameJacobian(
        const pinocchio::FrameIndex &index,
        const pinocchio::ReferenceFrame &reference_frame =
            pinocchio::LOCAL) const {
        if (index == model_->frames.size()) {
            assert("ERROR: Frame does not exist!");
        }
        pinocchio::getFrameJacobian(*model_, *data_, index, reference_frame,
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
        this->q_ = integrate(v, dt);
        this->update(this->q_);
    }

    Vector integrate(const Vector &v, const Real &dt) {
        return pinocchio::integrate(*model_, q_, v * dt);
    }

    const Vector &configuration() const { return q_; }

   private:
    std::shared_ptr<Model> model_;
    std::shared_ptr<Data> data_;

    std::shared_ptr<CollisionModel> collision_model_;
    std::shared_ptr<CollisionData> collision_data_;

    Vector q0_;
    Vector q_;

    Matrix6x jacobian_;
};

}  // namespace ik