#ifndef WAVERIDER_ROS_ROS_SERVER_H_
#define WAVERIDER_ROS_ROS_SERVER_H_

#include <string>
#include <thread>

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

struct WaveriderServerConfig : wavemap::ConfigBase<WaveriderServerConfig, 5> {
  std::string world_frame = "odom";

  int publish_debug_visuals_every_n_iterations = 20;

  std::string robot_state_topic;
  std::string goal_tf_frame;
  ValueWithUnit<SiUnit::kSeconds, FloatingPoint> goal_tf_delay = 0.05f;

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

  void robotStateCallback();

 private:
  const WaveriderServerConfig config_;

  struct {
    // TODO(victorr): Switch to SE2 state
    std::optional<rmpcpp::SE3State> data;
    std::mutex mutex;
  } robot_state_;

  // Wavemap-based obstacle avoidance policy
  WaveriderPolicy waverider_policy_;

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
};
}  // namespace waverider

#endif  // WAVERIDER_ROS_ROS_SERVER_H_
