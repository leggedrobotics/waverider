#ifndef WAVERIDER_ROS_ROS_SERVER_H_
#define WAVERIDER_ROS_ROS_SERVER_H_

#include <string>
#include <thread>
#include <vector>

#include <magnecko_msgs/MagneckoWaveriderState.h>
#include <rmpcpp/policies/simple_target_policy.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <tf2_msgs/TFMessage.h>
#include <wavemap/core/config/config_base.h>
#include <wavemap/core/config/string_list.h>
#include <wavemap/core/config/value_with_unit.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap_ros/utils/tf_transformer.h>
#include <waverider/goal_policy.h>
#include <waverider/goal_policy_tuning.h>
#include <waverider/ferrous_surface_policy.h>
#include <waverider/ferrous_surface_policy_tuning.h>
#include <waverider/obstacle_list_policy.h>
#include <waverider/waverider_policy.h>
#include <waverider/yaw_policy.h>
#include <waverider/yaw_policy_tuning.h>

namespace waverider {
using wavemap::FloatingPoint;
using wavemap::SiUnit;
using wavemap::StringList;
using wavemap::ValueWithUnit;

struct WaveriderServerConfig
    : wavemap::ConfigBase<WaveriderServerConfig, 23, StringList,
                          GoalPolicyTuning, FerrousSurfacePolicyTuning, YawPolicyTuning,
                          ObstaclePolicyTuning> {
  std::string odom_frame = "odom";

  std::string robot_state_topic;
  std::string ferrous_surfaces_topic;
  std::string twist_command_topic;
  StringList obstacle_aabb_topics;

  std::string goal_tf_frame;
  std::string ground_plane_tf_frame;
  ValueWithUnit<SiUnit::kSeconds, FloatingPoint> tf_lookup_delay = 0.05f;
  ValueWithUnit<SiUnit::kMeters, FloatingPoint> ground_plane_offset = 0.f;

  FloatingPoint occupancy_threshold = 0.1f;

  ValueWithUnit<SiUnit::kSeconds, FloatingPoint> control_period = 0.02f;
  ValueWithUnit<SiUnit::kSeconds, FloatingPoint> integrator_step_size = 0.005f;

  int publish_debug_visuals_every_n_iterations = 20;

  GoalPolicyTuning goal_policy;
  FerrousSurfacePolicyTuning ferrous_surfaces_policy;
  YawPolicyTuning yaw_policy;
  ObstaclePolicyTuning map_obstacles_policy;
  ObstaclePolicyTuning aabb_obstacles_policy;
  FloatingPoint goal_policy_marker_scale = 0.1f;
  FloatingPoint ferrous_surface_policy_marker_scale = 0.1f;
  FloatingPoint yaw_policy_marker_scale = 0.1f;
  FloatingPoint map_obstacles_policy_marker_scale = 0.1f;
  FloatingPoint aabb_obstacles_policy_marker_scale = 0.1f;

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

  void robotStateCallback(magnecko_msgs::MagneckoWaveriderState robot_state_msg);

 private:
  const WaveriderServerConfig config_;
  const std::string map_frame_;

  // Wavemap-based obstacle avoidance policy
  GoalPolicy goal_policy_;
  FerrousSurfacePolicy ferrous_surface_policy_;
  YawPolicy yaw_policy_;
  WaveriderPolicy map_obstacles_policy_;
  ObstacleListPolicy aabb_obstacles_policy_;

  // Asynchronous policy publishing logic
  std::atomic<bool> continue_async_planning_{false};
  std::thread async_planning_thread_;
  void asyncPlanningLoop();
  void evaluateAndPublishPolicy();

  // ROS interfaces
  void subscribeToTopics(ros::NodeHandle& nh);
  wavemap::TfTransformer transformer_;

  ros::Subscriber robot_state_sub_;
  struct {
    std::optional<rmpcpp::SE3State> data;
    uint64_t time = 0u;
    std::mutex mutex;
  } robot_state_;

  ros::Subscriber ferrous_surfaces_sub_;
  void ferrousSurfaceCallback(const tf2_msgs::TFMessage::ConstPtr& msg);

  std::vector<ros::Subscriber> aabb_subs_;
  struct {
    std::vector<ObstacleListPolicy::ObstacleList> data;
    std::mutex mutex;
  } aabb_lists_;
  void parseAabbMsg(const std_msgs::Float32MultiArray& msg,
                    ObstacleListPolicy::ObstacleList& aabb_list);

  void advertiseTopics(ros::NodeHandle& nh_private);
  ros::Publisher policy_pub_;
  ros::Publisher debug_pub_;

  std::optional<Point3D> getPositionGoalFromTf(ros::Time lookup_time);
  Eigen::Quaternion<float> getOrientationGoalFromTf(ros::Time lookup_time);
  std::optional<Plane3D> getGroundPlaneFromTf();
};
}  // namespace waverider

#endif  // WAVERIDER_ROS_ROS_SERVER_H_
