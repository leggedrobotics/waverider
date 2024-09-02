#include "waverider_ros/ros_server.h"

#include <rmpcpp/geometry/partial_geometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <wavemap/core/utils/profiler_interface.h>
#include <wavemap_ros_conversions/config_conversions.h>

#include "waverider_ros/policy_visuals.h"

namespace waverider {
DECLARE_CONFIG_MEMBERS(WaveriderServerConfig,
                      (world_frame)
                      (publish_debug_visuals_every_n_iterations)
                      (robot_state_topic)
                      (goal_tf_frame)
                      (goal_tf_delay));

bool WaveriderServerConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_NE(world_frame, "", verbose);
  all_valid &= IS_PARAM_NE(robot_state_topic, "", verbose);
  all_valid &= IS_PARAM_NE(goal_tf_frame, "", verbose);
  all_valid &= IS_PARAM_GE(goal_tf_delay, 0.f, verbose);

  return all_valid;
}

WaveriderServer::WaveriderServer(ros::NodeHandle nh, ros::NodeHandle nh_private)
    : WaveriderServer(nh, nh_private,
                      WaveriderServerConfig::from(
                          wavemap::param::convert::toParamValue(nh_private, ""))
                          .value()) {}

WaveriderServer::WaveriderServer(ros::NodeHandle nh, ros::NodeHandle nh_private,
                                 const WaveriderServerConfig& config)
    : config_(config.checkValid()) {
  subscribeToTopics(nh);
  advertiseTopics(nh_private);
}

void WaveriderServer::updateMap(const wavemap::MapBase& map) {
  ProfilerZoneScoped;

  // Get the world state
  Point3D robot_position;
  {
    std::scoped_lock lock(robot_state_.mutex);
    if (!robot_state_.data.has_value()) {
      ROS_WARN(
          "Robot position not yet initialized. Could not extract obstacles.");
      return;
    }
    robot_position = robot_state_.data.value().p().cast<FloatingPoint>();
  }

  // Extract the obstacles
  if (auto hashed_map = dynamic_cast<const wavemap::HashedWaveletOctree*>(&map);
      hashed_map) {
    waverider_policy_.updateObstacles(*hashed_map, robot_position);
  } else {
    ROS_WARN(
        "Waverider policies can currently only be extracted from maps of "
        "type wavemap::HashedWaveletOctree.");
  }
}

void WaveriderServer::startPlanningAsync() {
  ProfilerZoneScoped;

  if (continue_async_planning_.load(std::memory_order::memory_order_relaxed)) {
    ROS_INFO("Async planning already enabled.");
    return;
  }

  continue_async_planning_.store(true, std::memory_order::memory_order_relaxed);
  async_planning_thread_ =
      std::thread(&WaveriderServer::asyncPlanningLoop, this);
}

void WaveriderServer::robotStateCallback() {
  ProfilerZoneScoped;
  // TODO(smauq): Read current state from msg
  Eigen::Affine3d T_odom_body_ref = Eigen::Affine3d::Identity();
  T_odom_body_ref.translation();  // Position
  T_odom_body_ref.linear();       // Orientation as rotation matrix
  // TODO(smauq): Check in what frames these should be
  Eigen::Vector3d v;     // Velocity
  Eigen::Vector3d vdot;  // Acceleration
  Eigen::Vector3d w;     // Body angular velocity
  Eigen::Vector3d wdot;  // Body angular acceleration

  // Convert into world state
  {
    std::scoped_lock lock(robot_state_.mutex);
    robot_state_.data.emplace();
    robot_state_.data->p() = T_odom_body_ref.translation();
    robot_state_.data->q() = T_odom_body_ref.rotation();
    robot_state_.data->v() = T_odom_body_ref.rotation() * v;
    robot_state_.data->a() = T_odom_body_ref.rotation() * vdot;
    robot_state_.data->w() = T_odom_body_ref.rotation() * w;
    robot_state_.data->dw() = T_odom_body_ref.rotation() * wdot;
  }
}

void WaveriderServer::asyncPlanningLoop() {
  ProfilerZoneScoped;
  ros::WallRate rate(200.0);
  while (ros::ok() &&
         continue_async_planning_.load(std::memory_order_relaxed)) {
    evaluateAndPublishPolicy();
    rate.sleep();
  }
  ROS_INFO("Stopped async planning.");
}

void WaveriderServer::evaluateAndPublishPolicy() {
  ProfilerZoneScoped;
  if (!waverider_policy_.isReady()) {
    ROS_WARN("Policy not yet initialized.");
    return;
  }

  // Get the current robot state
  rmpcpp::SE3State current_state;
  {
    std::unique_lock lock(robot_state_.mutex);
    if (!robot_state_.data.has_value()) {
      ROS_WARN("World state not yet initialized. Could not evaluate policy.");
      return;
    }
    current_state = robot_state_.data.value();
  }

  // Get the goal position
  const auto goal = getGoalFromTf();
  if (!goal.has_value()) {
    ROS_INFO("Goal position not set. Will do nothing.");
    return;
  }

  // Evaluate the goal attraction policy
  // TODO(victorr): Make sure frames are consistent between state, goal and obs.
  // TODO(victorr): Update the goal attractor to also induce robot rotations

  // Evaluate the static obstacle avoidance policy
  // TODO(victorr): Update this to work in 2D and induce robot rotations
  const auto val_wavemap_r3_W =
      waverider_policy_.evaluateAt(current_state.r3());

  // Evaluate the dynamic obstacle avoidance policy
  // TODO(victorr): Add a policy that avoids all dynamic obstacle bounding boxes

  // Forward integrate the state and policy to obtain velocity reference
  // TODO(victorr): Forward integrate state and policy by 1.f/(locomotion rate)

  // Send velocity reference to the locomotion controller
  // TODO(smauq): Populate and publish velocity command msg
  //  policy_pub_.publish();

  // Publish debug visuals
  {
    static int i = 0;
    if (++i % config_.publish_debug_visuals_every_n_iterations == 0) {
      visualization_msgs::MarkerArray marker_array;
      // marker_array.markers.emplace_back(generateClearingMarker());
      addFilteredObstaclesToMarkerArray(waverider_policy_.getObstacleCells(),
                                        config_.world_frame, marker_array);
      marker_array.markers.emplace_back(robotPositionToMarker(
          current_state.p().cast<float>(), config_.world_frame));
      debug_pub_.publish(marker_array);
    }
  }
}

void WaveriderServer::subscribeToTopics(ros::NodeHandle& nh) {
  // TODO(smauq): Subscribe to current state topic
  //  robot_state_sub_ =
  //      nh.subscribe(config_.robot_state_topic, 1,
  //                   &WaveriderServer::currentStateCallback, this);
}

void WaveriderServer::advertiseTopics(ros::NodeHandle& nh_private) {
  // TODO(smauq): Advertise velocity reference
  //  policy_pub_ =
  //      nh_private.advertise<mav_reactive_planning::PolicyValue>("policy", 1);
  // Advertise debug visuals
  debug_pub_ = nh_private.advertise<visualization_msgs::MarkerArray>(
      "filtered_obstacles", 1);
}

std::optional<Point3D> WaveriderServer::getGoalFromTf() {
  ros::Time lookup_time =
      ros::Time::now() - ros::Duration(config_.goal_tf_delay);
  wavemap::Transformation3D T_W_G;
  if (transformer_.lookupTransform(config_.world_frame, config_.goal_tf_frame,
                                   lookup_time, T_W_G)) {
    return T_W_G.getPosition();
  }
  return std::nullopt;
}
}  // namespace waverider
