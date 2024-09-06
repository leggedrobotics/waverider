#include "waverider_ros/ros_server.h"

#include <geometry_msgs/TwistStamped.h>
#include <rmpcpp/eval/integrator.h>
#include <rmpcpp/geometry/partial_geometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <wavemap/core/utils/profiler_interface.h>
#include <wavemap_ros_conversions/config_conversions.h>
#include <waverider/geometry.h>

#include "waverider_ros/policy_visuals.h"

namespace waverider {
DECLARE_CONFIG_MEMBERS(WaveriderServerConfig,
                      (world_frame)
                      (robot_state_topic)
                      (goal_tf_frame)
                      (ground_plane_tf_frame)
                      (tf_lookup_delay)
                      (occupancy_threshold)
                      (control_period)
                      (control_gain)
                      (publish_debug_visuals_every_n_iterations)
                      (attractor_tuning)
                      (repulsor_tuning));

bool WaveriderServerConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_NE(world_frame, "", verbose);
  all_valid &= IS_PARAM_NE(robot_state_topic, "", verbose);
  all_valid &= IS_PARAM_NE(goal_tf_frame, "", verbose);
  all_valid &= IS_PARAM_NE(ground_plane_tf_frame, "", verbose);
  all_valid &= IS_PARAM_GE(tf_lookup_delay, 0.f, verbose);
  all_valid &= IS_PARAM_GT(control_period, 0.f, verbose);

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
  // Configure the policies
  waverider_policy_.setOccupancyThreshold(config_.occupancy_threshold);
  waverider_policy_.updateTuning(config_.repulsor_tuning);
  goal_attractor_policy_.setTuning(config_.attractor_tuning.alpha,
                                   config_.attractor_tuning.beta,
                                   config_.attractor_tuning.c);
  goal_attractor_policy_.setA(config_.attractor_tuning.a *
                              Eigen::Matrix3d::Identity());

  // Interface with ROS
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

  // Get the ground plane
  const auto ground_plane = getGroundPlaneFromTf();
  if (!ground_plane.has_value()) {
    ROS_WARN("Ground plane TF lookup failed. Could not extract obstacles.");
    return;
  }

  // Extract the obstacles
  if (auto hashed_map = dynamic_cast<const wavemap::HashedWaveletOctree*>(&map);
      hashed_map) {
    waverider_policy_.updateObstacles(*hashed_map, robot_position,
                                      *ground_plane);
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

void WaveriderServer::robotStateCallback(alma_msgs::AlmaState robot_state_msg) {
  ProfilerZoneScoped;

  uint64_t curr_time = robot_state_msg.header.stamp.toNSec();
  Eigen::Matrix3d R_odom_body_ref =
      Eigen::Quaterniond(robot_state_msg.pose.pose.orientation.w,
                         robot_state_msg.pose.pose.orientation.x,
                         robot_state_msg.pose.pose.orientation.y,
                         robot_state_msg.pose.pose.orientation.z)
          .toRotationMatrix();
  Eigen::Vector3d t_odom_body_ref(robot_state_msg.pose.pose.position.x,
                                  robot_state_msg.pose.pose.position.y,
                                  robot_state_msg.pose.pose.position.z);

  Eigen::Vector3d v(robot_state_msg.twist.twist.linear.x,
                    robot_state_msg.twist.twist.linear.y,
                    robot_state_msg.twist.twist.linear.z);
  Eigen::Vector3d w(robot_state_msg.twist.twist.angular.x,
                    robot_state_msg.twist.twist.angular.y,
                    robot_state_msg.twist.twist.angular.z);

  Eigen::Vector3d vdot;
  Eigen::Vector3d wdot;

  if (prev_time_ != 0u) {
    double dt = static_cast<double>(curr_time - prev_time_) / 1e9;
    vdot = (v - prev_v_) / dt;
    wdot = (w - prev_w_) / dt;
  } else {
    vdot.setZero();
    wdot.setZero();
  }

  // Convert into world state
  {
    std::scoped_lock lock(robot_state_.mutex);
    robot_state_.data.emplace();
    robot_state_.data->p() = t_odom_body_ref;
    robot_state_.data->q() = R_odom_body_ref;
    robot_state_.data->v() = R_odom_body_ref * v;
    robot_state_.data->a() = R_odom_body_ref * vdot;
    robot_state_.data->w() = R_odom_body_ref * w;
    robot_state_.data->dw() = R_odom_body_ref * wdot;
  }

  prev_time_ = curr_time;
  prev_v_ = v;
  prev_w_ = w;
}

void WaveriderServer::asyncPlanningLoop() {
  ProfilerZoneScoped;
  ros::WallRate rate(ros::Duration(config_.control_period));
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
  {
    const auto goal = getGoalFromTf();
    if (!goal.has_value()) {
      ROS_INFO("Goal position not set. Will do nothing.");
      return;
    }
    goal_attractor_policy_.setTarget(goal->cast<double>());
  }

  // Evaluate the goal attraction policy
  auto attractor_r3_value =
      goal_attractor_policy_.evaluateAt(current_state.r3());
  // TODO(victorr): Place the goal attractor frame slightly in front of body,
  //                to also induce robot rotations
  auto attractor_r2_value =
      R3toR2{}.at(current_state.r3()).pull(attractor_r3_value);

  // Evaluate the static obstacle avoidance policy
  auto waverider_r3_value = waverider_policy_.evaluateAt(current_state.r3());
  auto waverider_r2_value =
      R3toR2{}.at(current_state.r3()).pull(waverider_r3_value);

  // Evaluate the dynamic obstacle avoidance policy
  // TODO(victorr): Add a policy that avoids all dynamic obstacle bounding boxes

  // Forward integrate the state and policy to obtain velocity reference
  auto propagated_state = R3toR2{}.convertToQ(current_state.r3());
  rmpcpp::TrapezoidalIntegrator integrator{
      propagated_state, config_.control_gain * config_.control_period};
  auto f_total = (attractor_r2_value + waverider_r2_value).f_;
  integrator.step(f_total);
  const Eigen::Vector3d vel_r3{propagated_state.vel_.x(),
                               propagated_state.vel_.y(), 0.0};

  // Send velocity reference to the locomotion controller
  geometry_msgs::TwistStamped twist_msg;
  twist_msg.header.stamp = ros::Time().fromNSec(prev_time_);
  twist_msg.header.frame_id = "base";
  Eigen::Vector3d v_body = current_state.q().inverse() * vel_r3;
  twist_msg.twist.linear.x = v_body.x();
  twist_msg.twist.linear.y = v_body.y();
  twist_msg.twist.linear.z = v_body.z();
  policy_pub_.publish(twist_msg);

  // Publish debug visuals
  {
    static int i = 0;
    if (++i % config_.publish_debug_visuals_every_n_iterations == 0) {
      visualization_msgs::MarkerArray marker_array;
      // marker_array.markers.emplace_back(generateClearingMarker());
      addFilteredObstaclesToMarkerArray(waverider_policy_.getObstacleCells(),
                                        config_.world_frame, marker_array);
      marker_array.markers.emplace_back(goalPositionToMarker(
          goal_attractor_policy_.target().cast<FloatingPoint>(),
          config_.world_frame));
      marker_array.markers.emplace_back(robotPositionToMarker(
          current_state.p().cast<float>(), config_.world_frame));
      marker_array.markers.emplace_back(
          velocityCommandToMarker(current_state.p().cast<float>(),
                                  vel_r3.cast<float>(), config_.world_frame));
      debug_pub_.publish(marker_array);
    }
  }
}

void WaveriderServer::subscribeToTopics(ros::NodeHandle& nh) {
  robot_state_sub_ = nh.subscribe(config_.robot_state_topic, 1,
                                  &WaveriderServer::robotStateCallback, this);
}

void WaveriderServer::advertiseTopics(ros::NodeHandle& nh_private) {
  policy_pub_ = nh_private.advertise<geometry_msgs::TwistStamped>(
      "/base_tracker/commanded_twist", 1);
  // Advertise debug visuals
  debug_pub_ = nh_private.advertise<visualization_msgs::MarkerArray>(
      "filtered_obstacles", 1);
}

std::optional<Point3D> WaveriderServer::getGoalFromTf() {
  ros::Time lookup_time =
      ros::Time::now() - ros::Duration(config_.tf_lookup_delay);
  wavemap::Transformation3D T_W_G;
  if (transformer_.lookupTransform(config_.world_frame, config_.goal_tf_frame,
                                   lookup_time, T_W_G)) {
    return T_W_G.getPosition();
  }
  return std::nullopt;
}

std::optional<Plane3D> WaveriderServer::getGroundPlaneFromTf() {
  ros::Time lookup_time =
      ros::Time::now() - ros::Duration(config_.tf_lookup_delay);
  wavemap::Transformation3D T_W_G;
  if (transformer_.lookupTransform(config_.world_frame,
                                   config_.ground_plane_tf_frame, lookup_time,
                                   T_W_G)) {
    Plane3D ground_plane;
    ground_plane.normal = T_W_G.getRotation().rotate(Vector3D::UnitZ());
    ground_plane.offset = ground_plane.normal.dot(T_W_G.getPosition());
    return ground_plane;
  }
  return std::nullopt;
}
}  // namespace waverider
