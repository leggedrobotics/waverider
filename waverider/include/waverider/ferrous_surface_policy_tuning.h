#ifndef WAVERIDER_FERROUS_SURFACE_POLICY_TUNING_H
#define WAVERIDER_FERROUS_SURFACE_POLICY_TUNING_H

#include <wavemap/core/config/config_base.h>

namespace waverider {
struct FerrousSurfacePolicyTuning : wavemap::ConfigBase<FerrousSurfacePolicyTuning, 6> {
    //! Attractive gain
    float alpha = 20.f;
    //! Exponential decay rate
    float lambda = 2.0f;
    //! Maximum influence range
    float r = 2.0f;
    //! Sigmoid sharpness for multi-surface policies
    float kappa = 5.0f;
    //! Soft max tuning
    float c = 0.2f;
    //! Metric scaling (applied as A = a * Identity)
    float a = 10.f;

    static MemberMap memberMap;

    bool isValid(bool verbose) const override;
};

} // namespace waverider

#endif // WAVERIDER_FERROUS_SURFACE_POLICY_TUNING_H