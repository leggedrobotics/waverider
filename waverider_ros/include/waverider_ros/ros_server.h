#ifndef WAVERIDER_ROS_ROS_SERVER_H_
#define WAVERIDER_ROS_ROS_SERVER_H_

#include <string>
#include <thread>

#include <alma_msgs/AlmaState.h>
#include <rmpcpp/policies/simple_target_policy.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <wavemap/core/config/config_base.h>
#include <wavemap/core/config/value_with_unit.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap_ros/utils/tf_transformer.h>
#include <waverider/waverider_policy.h>

namespace waverider {
using wavemap::FloatingPoint;
using wavemap::SiUnit;
using wavemap::ValueWithUnit;

struct WaveriderServerConfig : wavemap::ConfigBase<WaveriderServerConfig, 8> {
  std::string world_frame = "odom";

  std::string robot_state_topic;

  std::string goal_tf_frame;
  std::string ground_plane_tf_frame;
  ValueWithUnit<SiUnit::kSeconds, FloatingPoint> tf_lookup_delay = 0.05f;

  FloatingPoint occupancy_threshold = 0.1f;

  ValueWithUnit<SiUnit::kSeconds, FloatingPoint> control_period = 0.02f;

  int publish_debug_visuals_every_n_iterations = 20;

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};

class WaveriderServer {
 public:
  WaveriderServer(ros::NodeHandle nh, ros::NodeHandle nh_private);
  WaveriderServer(ros::NodeHandle nh, ros::NodeHandle nh_private,
                  const WaveriderServerConfig& config);

  void updateMap(const wavemap::MapBase& map);

  void startPlanningAsync();
  void stopPlanningAsync() {
    continue_async_planning_.store(false, std::memory_order_relaxed);
  }

  void robotStateCallback(alma_msgs::AlmaState robot_state_msg);

 private:
  const WaveriderServerConfig config_;

  struct {
    std::optional<rmpcpp::SE3State> data;
    std::mutex mutex;
  } robot_state_;

  // Wavemap-based obstacle avoidance policy
  WaveriderPolicy waverider_policy_;
  rmpcpp::SimpleTargetPolicy<rmpcpp::Space<3>> goal_attractor_policy_;

  // Asynchronous policy publishing logic
  std::atomic<bool> continue_async_planning_{false};
  std::thread async_planning_thread_;
  void asyncPlanningLoop();
  void evaluateAndPublishPolicy();

  // ROS interfaces
  void subscribeToTopics(ros::NodeHandle& nh);
  ros::Subscriber robot_state_sub_;
  wavemap::TfTransformer transformer_;
  uint64_t prev_time_ = 0u;
  Eigen::Vector3d prev_v_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d prev_w_ = Eigen::Vector3d::Zero();

  void advertiseTopics(ros::NodeHandle& nh_private);
  ros::Publisher policy_pub_;
  ros::Publisher debug_pub_;

  std::optional<Point3D> getGoalFromTf();
  std::optional<Plane3D> getGroundPlaneFromTf();
};
}  // namespace waverider

#endif  // WAVERIDER_ROS_ROS_SERVER_H_
