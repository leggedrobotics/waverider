#ifndef WAVERIDER_ROS_ROS_SERVER_H_
#define WAVERIDER_ROS_ROS_SERVER_H_

#include <string>
#include <thread>

#include <anymal_msgs/AnymalState.h>
#include <rmpcpp/policies/simple_target_policy.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <wavemap/core/config/config_base.h>
#include <wavemap/core/config/value_with_unit.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap_ros/utils/tf_transformer.h>
#include <waverider/goal_policy.h>
#include <waverider/goal_policy_tuning.h>
#include <waverider/waverider_policy.h>
#include <waverider/yaw_policy.h>
#include <waverider/yaw_policy_tuning.h>

namespace waverider {
using wavemap::FloatingPoint;
using wavemap::SiUnit;
using wavemap::ValueWithUnit;

struct WaveriderServerConfig
    : wavemap::ConfigBase<WaveriderServerConfig, 14, GoalPolicyTuning,
                          YawPolicyTuning, ObstaclePolicyTuning> {
  std::string odom_frame = "odom";

  std::string robot_state_topic;
  std::string twist_command_topic;

  std::string goal_tf_frame;
  std::string ground_plane_tf_frame;
  ValueWithUnit<SiUnit::kSeconds, FloatingPoint> tf_lookup_delay = 0.05f;
  ValueWithUnit<SiUnit::kMeters, FloatingPoint> ground_plane_offset = 0.f;

  FloatingPoint occupancy_threshold = 0.1f;

  ValueWithUnit<SiUnit::kSeconds, FloatingPoint> control_period = 0.02f;
  ValueWithUnit<SiUnit::kSeconds, FloatingPoint> integrator_step_size = 0.005f;

  int publish_debug_visuals_every_n_iterations = 20;

  GoalPolicyTuning goal_policy;
  YawPolicyTuning yaw_policy;
  ObstaclePolicyTuning obstacle_policy;

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};

class WaveriderServer {
 public:
  WaveriderServer(ros::NodeHandle nh, ros::NodeHandle nh_private,
                  std::string map_frame);
  WaveriderServer(ros::NodeHandle nh, ros::NodeHandle nh_private,
                  const WaveriderServerConfig& config, std::string map_frame);

  void updateMap(const wavemap::MapBase& map);

  void startPlanningAsync();
  void stopPlanningAsync() {
    continue_async_planning_.store(false, std::memory_order_relaxed);
  }

  void robotStateCallback(anymal_msgs::AnymalState robot_state_msg);

 private:
  const WaveriderServerConfig config_;
  const std::string map_frame_;

  struct {
    std::optional<rmpcpp::SE3State> data;
    uint64_t time = 0u;
    std::mutex mutex;
  } robot_state_;

  // Wavemap-based obstacle avoidance policy
  GoalPolicy goal_policy_;
  YawPolicy yaw_policy_;
  WaveriderPolicy obstacle_policy_;

  // Asynchronous policy publishing logic
  std::atomic<bool> continue_async_planning_{false};
  std::thread async_planning_thread_;
  void asyncPlanningLoop();
  void evaluateAndPublishPolicy();

  // ROS interfaces
  void subscribeToTopics(ros::NodeHandle& nh);
  ros::Subscriber robot_state_sub_;
  wavemap::TfTransformer transformer_;

  void advertiseTopics(ros::NodeHandle& nh_private);
  ros::Publisher policy_pub_;
  ros::Publisher debug_pub_;

  std::optional<Point3D> getGoalFromTf();
  std::optional<Plane3D> getGroundPlaneFromTf();
};
}  // namespace waverider

#endif  // WAVERIDER_ROS_ROS_SERVER_H_
