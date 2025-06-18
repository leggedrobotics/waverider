#include "waverider/ferrous_surface_policy.h"

namespace waverider {

  namespace {
    constexpr double kMinPotential = 1e-8; 
    constexpr double kMinWeight = 1e-6;
    constexpr double kMaxSigmoidInput = 50.0;
  }

//==========================================================================
// SingleFerrousSurfacePolicy
//==========================================================================
SingleFerrousSurfacePolicy::SingleFerrousSurfacePolicy(
  const FerrousSurfacePolicyTuning& tuning) {
  setTuning(tuning);
}

void SingleFerrousSurfacePolicy::setTuning(const FerrousSurfacePolicyTuning& tuning) {
  tuning_ = tuning.checkValid();
  A_static_ = tuning_.a * Matrix::Identity();
}

SingleFerrousSurfacePolicy::PValue SingleFerrousSurfacePolicy::evaluateAt(
  const SingleFerrousSurfacePolicy::PState& state) {

  const double signed_distance = computeSignedDistance(state.pos_);
  const double potential = computePotential(signed_distance);

  if (potential < kMinPotential) {
    return {Eigen::Vector3d::Zero(), A_static_};
  }

  const Vector potential_gradient = computePotentialGradient(state.pos_);

  const double distance_norm = space_.norm(potential_gradient);
  const double softmax_scale = h(distance_norm);
  
  Vector f = -potential_gradient / softmax_scale;

  return {f, A_static_};
}

double SingleFerrousSurfacePolicy::computePotential(const Vector& position) const {
  const double signed_distance = computeSignedDistance(position);
  std::cout << "Signed distance: " << signed_distance << std::endl;
  return computePotential(signed_distance);
}

double SingleFerrousSurfacePolicy::getSigmoidWeight(double potential) const {
  const double clamped_potential = std::clamp(tuning_.kappa * potential, -kMaxSigmoidInput, kMaxSigmoidInput);
  return sigmoid(clamped_potential);
}

double SingleFerrousSurfacePolicy::computeSignedDistance(const Vector& position) const {
  std::cout << "Computing signed distance for position: " << position.transpose() << std::endl;
  return (position - surface_.center).dot(surface_.normal);
}

double SingleFerrousSurfacePolicy::computePotential(double signed_distance) const {
  std::cout << "Computing potential for signed distance: " << signed_distance << std::endl;
  std::cout << "Max range (tuning_.r): " << tuning_.r << std::endl;
  if (signed_distance <= 0.0 || signed_distance > tuning_.r) {
    return 0.0;
  }

  std::cout << "Signed distance calc: " << tuning_.alpha * std::exp(-tuning_.lambda * signed_distance) << std::endl;
  return tuning_.alpha * std::exp(-tuning_.lambda * signed_distance);
}

Vector SingleFerrousSurfacePolicy::computePotentialGradient(const Vector& position) const {
  const double signed_distance = computeSignedDistance(position);

  if (signed_distance <= 0.0 || signed_distance > tuning_.r) {
    return Vector::Zero();
  }

  const double gradient_magnitude = -tuning_.alpha * tuning_.lambda * std::exp(-tuning_.lambda * signed_distance);
  return gradient_magnitude * surface_.normal;
}

double SingleFerrousSurfacePolicy::sigmoid(double x) const {
  const double clamped_x = std::clamp(x, -kMaxSigmoidInput, kMaxSigmoidInput);
  return 1.0 / (1.0 + std::exp(-clamped_x));
}

//==========================================================================
// FerrousSurfacePolicy
//==========================================================================

FerrousSurfacePolicy::FerrousSurfacePolicy(const FerrousSurfacePolicyTuning& tuning) {
  setTuning(tuning);
}

void FerrousSurfacePolicy::setTuning(const FerrousSurfacePolicyTuning& tuning) {
  tuning_ = tuning.checkValid();
  A_static_ = tuning_.a * Matrix::Identity();
  for (auto& pair : surface_policies_) {
    pair.second->setTuning(tuning_);
  }
}

FerrousSurfacePolicy::PValue FerrousSurfacePolicy::evaluateAt(
  const FerrousSurfacePolicy::PState& state) {
  if (surfaces_.empty()) {
    return {Vector::Zero(), A_static_};
  }
  return computeSigmoidWeighted(state);
}

void FerrousSurfacePolicy::addSurface(const std::string& surface_id, const Surface& surface) {
  surfaces_[surface_id] = surface;

  auto policy = std::make_shared<SingleFerrousSurfacePolicy>(tuning_);
  policy->setSurface(surface);
  surface_policies_[surface_id] = policy;
}

void FerrousSurfacePolicy::updateSurface(const std::string& surface_id, const Surface& surface) {
  surfaces_[surface_id] = surface;

  auto it = surface_policies_.find(surface_id);
  if (it != surface_policies_.end()) {
    it->second->setSurface(surface);
  } else {
    addSurface(surface_id, surface);
  }
}

void FerrousSurfacePolicy::removeSurface(const std::string& surface_id) {
  surfaces_.erase(surface_id);
  surface_policies_.erase(surface_id);
}

void FerrousSurfacePolicy::setSurfaces(const std::unordered_map<std::string, Surface>& surfaces) {
  clearSurfaces();
  for (const auto& pair : surfaces) {
    addSurface(pair.first, pair.second);
  }
}

std::vector<std::string> FerrousSurfacePolicy::getSurfaceIds() const {
  std::vector<std::string> surface_ids;
  surface_ids.reserve(surfaces_.size());
  for (const auto& pair : surfaces_) {
    surface_ids.push_back(pair.first);
  }
  return surface_ids;
}

FerrousSurfacePolicy::PValue FerrousSurfacePolicy::computeSigmoidWeighted(
  const FerrousSurfacePolicy::PState& state) const {
  const auto all_policies = getAllPolicies();
  std::cout << "Evaluating " << all_policies.size() << " ferrous surface policies" << std::endl;
  if (all_policies.empty()) {
    std::cout << "No ferrous surface policies available." << std::endl;
    return {Vector::Zero(), A_static_};
  }

  if (all_policies.size() == 1) {
    std::cout << "Only one ferrous surface policy available." << std::endl;
    const auto result = all_policies[0]->evaluateAt(state);
    return {global_weight_ * result.f_, global_weight_ * result.A_};
  }

  std::vector<double> potentials;
  std::vector<double> weights;
  double total_sigmoid_weight = 0.0;

  std::cout << "Computing potentials and weights for " << all_policies.size() << " policies." << std::endl;

  for (const auto& policy : all_policies) {
    std::cout << "Evaluating policy at position: " << state.pos_.transpose() << std::endl;

    const double potential = policy->computePotential(state.pos_);
    if (potential < kMinPotential) continue;

    std::cout << "Policy potential: " << potential << std::endl;

    const double sigmoid_weight = policy->getSigmoidWeight(potential);
    if (sigmoid_weight < kMinWeight) continue;

    std::cout << "Policy sigmoid weight: " << sigmoid_weight << std::endl;

    potentials.push_back(potential);
    weights.push_back(sigmoid_weight);
    total_sigmoid_weight += sigmoid_weight;
  }

  std::cout << "Total sigmoid weight: " << total_sigmoid_weight << std::endl;

  // Normalize weights
  for (auto& w: weights) {
    w /= total_sigmoid_weight;
    std::cout << "Normalized weight: " << w << std::endl;
  }

  Vector f_combined = Vector::Zero();
  Matrix A_combined = Matrix::Zero();

  for (size_t i = 0; i < all_policies.size(); ++i) {
    std::cout << "Before min weight check: "
              << "Potential: " << potentials[i] << ", "
              << "Weight: " << weights[i] << std::endl;

    if(weights[i] < kMinWeight) continue;

    std::cout << "Evaluating policy " << i << " with potential: " << potentials[i]
              << " and weight: " << weights[i] << std::endl;

    const auto individual_policy_results = all_policies[i]->evaluateAt(state);

    std::cout << "Policy " << i << " potential: " << potentials[i]
              << ", weight: " << weights[i] << std::endl;

    f_combined += weights[i] * individual_policy_results.f_;
    A_combined += weights[i] * individual_policy_results.A_;

    std::cout << "Policy " << i << ": "
              << "Potential: " << potentials[i] << ", "
              << "Weight: " << weights[i] << ", "
              << "f: " << individual_policy_results.f_.transpose() << ", "
              << "A: \n" << individual_policy_results.A_ << std::endl;
  }

  std::cout << "Combined f: " << f_combined.transpose() << std::endl;
  std::cout << "Combined A: \n" << A_combined << std::endl;
  
  return {global_weight_ * f_combined,
          global_weight_ * A_combined};
}

std::vector<std::shared_ptr<SingleFerrousSurfacePolicy>> FerrousSurfacePolicy::getAllPolicies() const {
  std::vector<std::shared_ptr<SingleFerrousSurfacePolicy>> policies;
  policies.reserve(surface_policies_.size());
  for (const auto& pair : surface_policies_) {
    policies.push_back(pair.second);
  }
  return policies;
}

  
}  // namespace waverider