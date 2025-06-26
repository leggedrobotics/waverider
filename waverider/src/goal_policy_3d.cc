#include "waverider/goal_policy_3d.h"

namespace waverider {
    GoalPolicy3D::GoalPolicy3D(const GoalPolicyTuning& tuning) {
        setTuning(tuning); 
    }
    void GoalPolicy3D::setTuning(const GoalPolicyTuning& tuning) {
        tuning_ = tuning.checkValid();
        A_static_ = tuning_.a * Matrix::Identity();
    }
    GoalPolicy3D::PValue GoalPolicy3D::evaluateAt(const PState& state) {
        const auto error = space_.minus(target_, state.pos_);
        double alpha_scaled = tuning_.alpha;
        if (tuning_.disable_attractor_near_goal) {
            alpha_scaled *= std::min(error.norm(), 1.0);
        }
        Vector f = alpha_scaled * s(error) - tuning_.beta * state.vel_;
        return {f, A_static_};
    }

    // Surface projection implementation
    rmpcpp::PolicyBase<rmpcpp::Space<3>>::PValue SurfaceProjector::projectToSurface(
        const rmpcpp::PolicyBase<rmpcpp::Space<3>>::PValue& policy_3d,
        const geometry_msgs::TransformStamped& surface_transform) {
      
        // Extract surface normal from transform
        Eigen::Vector3d surface_normal = extractSurfaceNormal(surface_transform);
        
        // Create projection matrix P = I - nn^T
        Eigen::Matrix3d P = Eigen::Matrix3d::Identity() - 
                            surface_normal * surface_normal.transpose();
        
        // Project force to surface tangent space
        Eigen::Vector3d projected_force = P * policy_3d.f_;
        
        // Return new PolicyValue with projected force and original metric
        return rmpcpp::PolicyBase<rmpcpp::Space<3>>::PValue{projected_force, policy_3d.A_};
    }

    Eigen::Vector3d SurfaceProjector::extractSurfaceNormal(
        const geometry_msgs::TransformStamped& surface_transform) {
      
        // Extract surface normal as Z-axis of surface frame
        Eigen::Quaternionf quat(surface_transform.transform.rotation.w,
                                surface_transform.transform.rotation.x,
                                surface_transform.transform.rotation.y,
                                surface_transform.transform.rotation.z);
        
        if (quat.norm() == 0.0f) {
            return Eigen::Vector3d::UnitZ(); // Default to Z-axis if invalid
        }
        
        quat.normalize();
        return quat.toRotationMatrix().col(2).cast<double>();
    }

}