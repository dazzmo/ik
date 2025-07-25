#pragma once
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/collision/collision.hpp>

#include "cink/Types.hpp"

namespace cink {

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
        const std::shared_ptr<CollisionData> &collision_data = nullptr);

    Size nq() const { return model_->nq; }
    Size nv() const { return model_->nv; }

    const Vector &configuration() const { return q_; }

    const Model &model() const { return *model_; }
    const Data &data() const { return *data_; }

    const CollisionModel &collisionModel() const { return *collision_model_; }
    const CollisionData &collisionData() const { return *collision_data_; }

    bool hasRootJoint() const { return model_->existJointName("root_joint"); }

    pinocchio::JointIndex getJointIndex(const String &joint) const;
    pinocchio::FrameIndex getFrameIndex(const String &frame) const;

    void update(const Vector &q);

    SE3 getTransformFrameToWorld(const String &frame) const;

    const Matrix6x &getJointJacobian(
        const pinocchio::JointIndex &index,
        const pinocchio::ReferenceFrame &reference_frame =
            pinocchio::LOCAL) const;

    const Matrix6x &getFrameJacobian(
        const pinocchio::FrameIndex &index,
        const pinocchio::ReferenceFrame &reference_frame =
            pinocchio::LOCAL) const;

    const Eigen::Vector3<Real> &getCentreOfMass() const;
    Matrix computeCentreOfMassJacobian() const;

    void integrateInPlace(const Vector &v, const Real &dt);
    Vector integrate(const Vector &v, const Real &dt);

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