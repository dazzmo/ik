#include "ik/barriers/SelfCollision.hpp"

namespace ik {

SelfCollisionBarrier::SelfCollisionBarrier(const Configuration &cfg,
                                           const Size &max_collisions)
    : BarrierAbstract(max_collisions),
      max_collisions_(max_collisions),
      closest_indices_(max_collisions, 0) {}

void SelfCollisionBarrier::computeBarrier(const Configuration &cfg,
                                          Eigen::Ref<Vector> barrier) {
    // Assess the specific collision pairs

    // Determine the k-closest collision pairs
    std::vector<std::pair<Size, Real>> pair_distance_(
        cfg.collisionModel().collisionPairs.size());

    for (int k = 0; k < cfg.collisionModel().collisionPairs.size(); ++k) {
        // Assess the collision results for the pairs
        const pinocchio::CollisionPair &cp =
            cfg.collisionModel().collisionPairs[k];
        const auto &dr = cfg.collisionData().distanceResults[k];

        pair_distance_[k] = {k, dr.min_distance - min_distance_};
    }

    std::sort(pair_distance_.begin(), pair_distance_.end(),
              [](const std::pair<Size, Real> &a, const std::pair<Size, Real> &b)
                  -> bool { return a.second < b.second; });

    // Sort the list and determine the k-closest pairs
    for (Size k = 0; k < max_collisions_; ++k) {
        closest_indices_[k] = pair_distance_[k].first;
        barrier[k] = pair_distance_[k].second;
    }
}

void SelfCollisionBarrier::computeJacobian(const Configuration &cfg,
                                           Eigen::Ref<Matrix> jacobian) {
    jacobian.setZero();
    // Assess the specific collision pairs
    Size row = 0;
    for (const Size &idx : closest_indices_) {
        const auto &cp = cfg.collisionModel().collisionPairs[idx];
        const auto &dr = cfg.collisionData().distanceResults[idx];

        const auto &go_1 = cfg.collisionModel().geometryObjects[cp.first];
        const auto &go_2 = cfg.collisionModel().geometryObjects[cp.second];

        const auto &j1_id = go_1.parentJoint;
        const auto &j2_id = go_2.parentJoint;

        // Location of each contact point in world frame
        auto w1 = dr.nearest_points[0];
        auto w2 = dr.nearest_points[1];

        // Position of the contact within the joint frame
        auto r1 = w1 - cfg.data().oMi[j1_id].translation();
        auto r2 = w2 - cfg.data().oMi[j2_id].translation();

        if ((w1 - w2).isZero(1e-3)) continue;

        // Compute normal between nearest point
        const auto n = (w1 - w2).normalized();

        const auto J1 =
            cfg.getJointJacobian(j1_id, pinocchio::LOCAL_WORLD_ALIGNED);
        const auto J2 =
            cfg.getJointJacobian(j2_id, pinocchio::LOCAL_WORLD_ALIGNED);

        // compute row
        jacobian.row(row) = n.transpose() * J1.topRows(3) +
                            r1.cross(n).transpose() * J1.bottomRows(3);
        jacobian.row(row) -= n.transpose() * J2.topRows(3) +
                             r2.cross(n).transpose() * J2.bottomRows(3);
        row++;
    }
}

}  // namespace ik