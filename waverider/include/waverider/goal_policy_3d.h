#ifndef WAVERIDER_GOAL_POLICY_3D_H_
#define WAVERIDER_GOAL_POLICY_3D_H_

#include "rmpcpp/core/policy_base.h"
#include "waverider/goal_policy_tuning.h"
#include <geometry_msgs/TransformStamped.h>

namespace waverider {
    class GoalPolicy3D : public rmpcpp::PolicyBase<rmpcpp::Space<3>> {
        public:
            /**
             * Sets up the policy.
             * target is the target to move to.
             * A is the metric to be used.
             * alpha, beta and c are tuning parameters.
             */
            explicit GoalPolicy3D(const GoalPolicyTuning& tuning);
            GoalPolicy3D() = default;

            void setTuning(const GoalPolicyTuning& tuning);

            void setTarget(const Vector& target) { target_ = target; }

            Vector& target() { return target_; }

            PValue evaluateAt(const PState& state) override;

        protected:
            GoalPolicyTuning tuning_;
            Vector target_ = Vector::Zero();
            Matrix A_static_;

            /**
             *  Normalization helper function.
             */
            Vector s(const Vector& x) { return x / h(this->space_.norm(x)); }
            /**
             * Softmax helper function
             */
            double h(const double z) const {
                return (z + tuning_.c * std::log(1.0 + std::exp(-2.0 * tuning_.c * z)));
            }
    };

    class SurfaceProjector {
        public:
            /** 
             * Project R3 policy onto surface tangent space
             */
            static rmpcpp::PolicyBase<rmpcpp::Space<3>>::PValue projectToSurface(
                const rmpcpp::PolicyBase<rmpcpp::Space<3>>::PValue& policy_3d,
                const geometry_msgs::TransformStamped& surface_transform);
        private:
            static Eigen::Vector3d extractSurfaceNormal(
                const geometry_msgs::TransformStamped& surface_transform);
    };
};

#endif  // WAVERIDER_GOAL_POLICY_3D_H_