#ifndef WAVERIDER_FERROUS_SURFACE_POLICY_H
#define WAVERIDER_FERROUS_SURFACE_POLICY_H

#include <rmpcpp/core/policy_base.h>
#include <waverider/ferrous_surface_policy_tuning.h>
#include <unordered_map>
#include <memory>

namespace waverider {

using Vector = rmpcpp::Space<3>::Vector; // Needs to be adjusted

struct Surface {
    Vector center = Vector::Zero();
    Vector normal = Vector::Zero();
    std::string surface_id;
};

class SingleFerrousSurfacePolicy : public rmpcpp::PolicyBase<rmpcpp::Space<3>> {
    public:
        /**
         * Sets up the policy for interacting with a single ferrous surface.
         * surface_ refers to the ferrous surface that the policy will interact with.
         * A is the metric to be used.
         * alpha, beta, and c are tuning parameters.
         */

        explicit SingleFerrousSurfacePolicy(const FerrousSurfacePolicyTuning& tuning);
        SingleFerrousSurfacePolicy() = default;

        void setTuning(const FerrousSurfacePolicyTuning& tuning);
        void setSurface(const Surface& surface) {
            surface_ = surface;
        }
        const Surface& getSurface() const {
            return surface_;
        }

        PValue evaluateAt(const PState& state) override;

        double computePotential(const Vector& position) const;
        double getSigmoidWeight(double potential) const;

    protected:
        FerrousSurfacePolicyTuning tuning_;
        Surface surface_;
        Matrix A_static_ = Matrix::Identity();

        double computeSignedDistance(const Vector& position) const;
        double computePotential(double signed_distance) const;
        Vector computePotentialGradient(const Vector& position) const;
        double sigmoid(double x) const;
        
        /**
         * Normalization helper function.
         */
        Vector s(const Vector& x) { return x / h(this->space_.norm(x)); }

        /**
         * Softmax helper function
         */
        double h(const double z) const {
            return (z + tuning_.c * std::log(1.0 + std::exp(-2.0 * tuning_.c * z)));
        }
};

class FerrousSurfacePolicy : public rmpcpp::PolicyBase<rmpcpp::Space<3>> {
public:
    /**
     * Sets up the policy for interacting with ferrous surfaces.
     * surfaces_ refer to ferrous surfaces that the policy will interact with.
     * A is the metric to be used.
     * alpha, beta, and c are tuning parameters.
     */
    explicit FerrousSurfacePolicy(const FerrousSurfacePolicyTuning& tuning);
    FerrousSurfacePolicy() = default;

    void setTuning(const FerrousSurfacePolicyTuning& tuning);
    void setGlobalWeight(double weight) {
        global_weight_ = weight;
    }

    PValue evaluateAt(const PState& state) override;

    void addSurface(const std::string& surface_id, const Surface& surface);
    void updateSurface(const std::string& surface_id, const Surface& surface);
    void removeSurface(const std::string& surface_id);
    void clearSurfaces() { surfaces_.clear(); surface_policies_.clear(); }

    void setSurfaces(const std::unordered_map<std::string, Surface>& surfaces);

    size_t getNumSurfaces() const {
        return surfaces_.size();
    }
    std::vector<std::string> getSurfaceIds() const;

protected:
    FerrousSurfacePolicyTuning tuning_;
    std::unordered_map<std::string, Surface> surfaces_;
    std::unordered_map<std::string, std::shared_ptr<SingleFerrousSurfacePolicy>> surface_policies_;

    double global_weight_ = 1.0; // Global weight for the policy
    Matrix A_static = Matrix::Identity();

    // Sigmoid-weighted combination
    PValue computeSigmoidWeighted(const PState& state) const;

    std::vector<std::shared_ptr<SingleFerrousSurfacePolicy>> getAllPolicies() const;
};
}; // namespace waverider

#endif // WAVERIDER_FERROUS_SURFACE_POLICY_H
