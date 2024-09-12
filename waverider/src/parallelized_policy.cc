#include "waverider/parallelized_policy.h"

namespace waverider {
ParallelizedPolicy::ParallelizedPolicy(ObstaclePolicyTuning tuning)
    : tuning_(std::move(tuning)) {}

rmpcpp::PolicyValue<3> ParallelizedPolicy::evaluate(
    const std::vector<Eigen::Vector3f>& x_observations,
    const Eigen::Vector3f& x, const Eigen::Vector3f& xdot) const {
  Eigen::Vector3f Af_sum = Eigen::Vector3f::Zero();
  Eigen::Matrix3f A_sum = Eigen::Matrix3f::Zero();

  for (const auto& x_observation : x_observations) {
    // normalize gradient
    Eigen::Vector3f grad_d = x - x_observation;
    const double d_x = std::max(grad_d.norm(), 0.0001f);
    grad_d.normalize();

    // calculate repulsive part (f_rep)
    const double alpha_rep =
        tuning_.eta_rep * std::exp(-(d_x / tuning_.nu_rep));
    const Eigen::Vector3f f_rep = alpha_rep * grad_d;

    // calculate dampening part (f_damp)
    const double epsilon = 1e-6;  // Added term for numerical stability
    const double alpha_damp =
        tuning_.eta_damp / ((d_x / tuning_.nu_damp) + epsilon);

    // eq 68 in RMP paper
    const Eigen::Vector3f P_obs_x = std::max(0.0f, -xdot.dot(grad_d)) *
                                    (grad_d * grad_d.transpose()) * xdot;
    const Eigen::Vector3f f_damp = alpha_damp * P_obs_x;

    // set f
    Eigen::Vector3f f_temp{Eigen::Vector3f::Zero()};
    if (tuning_.enable_repulsor) {
      f_temp += f_rep;
    }
    if (tuning_.enable_damper) {
      f_temp -= f_damp;
    }

    // calculation of metric
    // transpose should work outside s() as well
    const Eigen::Vector3f v = s(f_temp);
    const Eigen::Matrix3f A_temp =
        wr(static_cast<float>(d_x), tuning_.r) * (v * v.transpose());

    // set A
    A_sum += A_temp;
    Af_sum += A_temp * f_temp;
  }

  return {(A_sum.completeOrthogonalDecomposition().pseudoInverse() * Af_sum)
              .cast<double>(),
          A_sum.cast<double>()};
}
}  // namespace waverider
