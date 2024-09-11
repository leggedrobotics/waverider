#include "waverider_ros/ros_server.h"

#include <geometry_msgs/TwistStamped.h>
#include <rmpcpp/eval/integrator.h>
#include <rmpcpp/geometry/partial_geometry.h>
#include <visualization_msgs/MarkerArray.h>
#include <wavemap/core/utils/profiler_interface.h>
#include <wavemap_ros_conversions/config_conversions.h>
#include <waverider/geometry.h>
#include <waverider/yaw_policy.h>

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
                      (integrator_step_size)
                      (publish_debug_visuals_every_n_iterations)
                      (attractor_tuning)
                      (yaw_tuning)
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
  goal_attractor_policy_.setTuning(config_.attractor_tuning.alpha,
                                   config_.attractor_tuning.beta,
                                   config_.attractor_tuning.c);
  goal_attractor_policy_.setA(config_.attractor_tuning.a *
                              Eigen::Matrix3d::Identity());
  {
    YawPolicy::Matrix A_yaw = YawPolicy::Matrix::Zero();
    A_yaw(2, 2) = config_.yaw_tuning.a;
    yaw_policy_.setA(A_yaw);
  }
  yaw_policy_.setTuning(config_.yaw_tuning.alpha, config_.yaw_tuning.beta,
                        config_.yaw_tuning.c);
  waverider_policy_.setOccupancyThreshold(config_.occupancy_threshold);
  waverider_policy_.updateTuning(config_.repulsor_tuning);

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

void WaveriderServer::robotStateCallback(anymal_msgs::AnymalState robot_state_msg) {
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

  // Forward integrate the state and policy to obtain velocity reference
  auto propagated_state = SE3toSE2(current_state);
  rmpcpp::TrapezoidalIntegrator integrator{propagated_state,
                                           config_.integrator_step_size};
  const int num_integrator_steps =
      std::ceil(config_.control_period / config_.integrator_step_size);
  for (int step_idx = 0; step_idx < num_integrator_steps; ++step_idx) {
    // Reconstruct R3 state
    auto propagated_state_r3 = R3toSE2{}.convertToQ(propagated_state);
    propagated_state_r3.pos_.z() = current_state.p().z();
    propagated_state_r3.vel_.z() = current_state.v().z();
    propagated_state_r3.acc_.z() = current_state.a().z();

    // Evaluate the goal attraction policy
    auto attractor_r3_value =
        goal_attractor_policy_.evaluateAt(propagated_state_r3);
    auto attractor_se2_value =
        R3toSE2{}.at(propagated_state_r3).pull(attractor_r3_value);

    // Evaluate the yaw policy
    auto yaw_se2_value = yaw_policy_.evaluateAt(propagated_state);

    // Evaluate the static obstacle avoidance policy
    auto waverider_r3_value = waverider_policy_.evaluateAt(propagated_state_r3);
    auto waverider_se2_value =
        R3toSE2{}.at(propagated_state_r3).pull(waverider_r3_value);

    // Evaluate the dynamic obstacle avoidance policy
    // TODO(victorr): Add a policy that avoids all dynamic obstacle AABBs

    // Apply the policies by integrating their commanded accelerations
    auto f_total =
        (attractor_se2_value + yaw_se2_value + waverider_se2_value).f_;
    integrator.step(f_total);

    // Publish debug visuals
    if (step_idx == num_integrator_steps - 1) {
      static int i = 0;
      if (++i % config_.publish_debug_visuals_every_n_iterations == 0) {
        visualization_msgs::MarkerArray marker_array;
        addFilteredObstaclesToMarkerArray(waverider_policy_.getObstacleCells(),
                                          config_.world_frame, marker_array);
        marker_array.markers.emplace_back(goalPositionToMarker(
            goal_attractor_policy_.target().cast<FloatingPoint>(),
            config_.world_frame));
        marker_array.markers.emplace_back(robotPositionToMarker(
            current_state.p().cast<float>(), config_.world_frame));
        {
          const Vector3D v_r2{static_cast<float>(propagated_state.vel_.x()),
                              static_cast<float>(propagated_state.vel_.y()),
                              0.f};
          marker_array.markers.emplace_back(
              commandToMarker(current_state.p().cast<float>(), v_r2,
                              config_.world_frame, "v_r2", 0.f, 0.f, 1.f));
          const Vector3D v_yaw{0.f, 0.f,
                               static_cast<float>(propagated_state.vel_[2])};
          marker_array.markers.emplace_back(
              commandToMarker(current_state.p().cast<float>(), v_yaw,
                              config_.world_frame, "v_yaw", 0.f, 0.f, 1.f));
        }
        {
          const Vector3D f_attract_r2 = {
              static_cast<float>(attractor_se2_value.f_.x()),
              static_cast<float>(attractor_se2_value.f_.y()), 0.f};
          marker_array.markers.emplace_back(commandToMarker(
              current_state.p().cast<float>(), f_attract_r2,
              config_.world_frame, "f_attract_r2", 0.f, 1.f, 0.f));
          const Vector3D f_attract_yaw = {
              0.f, 0.f, static_cast<float>(attractor_se2_value.f_.z())};
          marker_array.markers.emplace_back(commandToMarker(
              current_state.p().cast<float>(), f_attract_yaw,
              config_.world_frame, "f_attract_yaw", 0.f, 1.f, 0.f));
        }
        {
          const Vector3D f_yaw_r2 = {static_cast<float>(yaw_se2_value.f_.x()),
                                     static_cast<float>(yaw_se2_value.f_.y()),
                                     0.f};
          marker_array.markers.emplace_back(
              commandToMarker(current_state.p().cast<float>(), f_yaw_r2,
                              config_.world_frame, "f_yaw_r2", 1.f, 1.f, 0.f));
          const Vector3D f_yaw_yaw = {0.f, 0.f,
                                      static_cast<float>(yaw_se2_value.f_.z())};
          marker_array.markers.emplace_back(
              commandToMarker(current_state.p().cast<float>(), f_yaw_yaw,
                              config_.world_frame, "f_yaw_yaw", 1.f, 1.f, 0.f));
        }
        {
          const Vector3D f_rep_r2 = {
              static_cast<float>(waverider_se2_value.f_.x()),
              static_cast<float>(waverider_se2_value.f_.y()), 0.f};
          marker_array.markers.emplace_back(
              commandToMarker(current_state.p().cast<float>(), f_rep_r2,
                              config_.world_frame, "f_rep_r2", 1.f, 0.f, 0.f));
          const Vector3D f_rep_yaw = {
              0.f, 0.f, static_cast<float>(waverider_se2_value.f_.z())};
          marker_array.markers.emplace_back(
              commandToMarker(current_state.p().cast<float>(), f_rep_yaw,
                              config_.world_frame, "f_rep_yaw", 1.f, 0.f, 0.f));
        }
        debug_pub_.publish(marker_array);
      }
    }
  }

  // Compute velocity reference
  const double yaw = propagated_state.pos_[2];
  Eigen::Vector3d v_body = current_state.q().inverse() * Eigen::Vector3d{propagated_state.vel_.x(), propagated_state.vel_.y(), 0.0};
  const double vel_yaw = propagated_state.vel_[2];

  // Send velocity reference to the locomotion controller
  geometry_msgs::TwistStamped twist_msg;
  twist_msg.header.stamp = ros::Time().fromNSec(prev_time_);
  twist_msg.header.frame_id = "base";
  twist_msg.twist.linear.x = v_body.x();
  twist_msg.twist.linear.y = v_body.y();
  twist_msg.twist.linear.z = v_body.z();
  twist_msg.twist.angular.z = 0.0; //vel_yaw;
  policy_pub_.publish(twist_msg);
}

void WaveriderServer::subscribeToTopics(ros::NodeHandle& nh) {
  robot_state_sub_ = nh.subscribe(config_.robot_state_topic, 1,
                                  &WaveriderServer::robotStateCallback, this);
}

void WaveriderServer::advertiseTopics(ros::NodeHandle& nh_private) {
  policy_pub_ = nh_private.advertise<geometry_msgs::TwistStamped>(
      "/path_planning_and_following/twist", 1);
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
