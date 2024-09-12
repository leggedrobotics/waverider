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
                      (odom_frame)
                      (robot_state_topic)
                      (goal_tf_frame)
                      (ground_plane_tf_frame)
                      (tf_lookup_delay)
                      (ground_plane_offset)
                      (occupancy_threshold)
                      (control_period)
                      (integrator_step_size)
                      (publish_debug_visuals_every_n_iterations)
                      (goal_policy)
                      (yaw_policy)
                      (obstacle_policy));

bool WaveriderServerConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_NE(odom_frame, "", verbose);
  all_valid &= IS_PARAM_NE(robot_state_topic, "", verbose);
  all_valid &= IS_PARAM_NE(goal_tf_frame, "", verbose);
  all_valid &= IS_PARAM_NE(ground_plane_tf_frame, "", verbose);
  all_valid &= IS_PARAM_GE(tf_lookup_delay, 0.f, verbose);
  all_valid &= IS_PARAM_GT(control_period, 0.f, verbose);

  return all_valid;
}

WaveriderServer::WaveriderServer(ros::NodeHandle nh, ros::NodeHandle nh_private,
                                 std::string map_frame)
    : WaveriderServer(nh, nh_private,
                      WaveriderServerConfig::from(
                          wavemap::param::convert::toParamValue(nh_private, ""))
                          .value(),
                      std::move(map_frame)) {}

WaveriderServer::WaveriderServer(ros::NodeHandle nh, ros::NodeHandle nh_private,
                                 const WaveriderServerConfig& config,
                                 std::string map_frame)
    : config_(config.checkValid()), map_frame_(std::move(map_frame)) {
  // Check that the map frame name is valid
  CHECK_NE(map_frame, "");

  // Configure goal policy
  goal_policy_.setTuning(config_.goal_policy.alpha, config_.goal_policy.beta,
                         config_.goal_policy.c,
                         config_.goal_policy.disable_attractor_near_goal);
  goal_policy_.setA(config_.goal_policy.a * Eigen::Matrix3d::Identity());
  // Configure yaw policy
  {
    YawPolicy::Matrix A_yaw = YawPolicy::Matrix::Zero();
    A_yaw(2, 2) = config_.yaw_policy.a;
    yaw_policy_.setA(A_yaw);
  }
  yaw_policy_.setTuning(config_.yaw_policy.alpha, config_.yaw_policy.beta,
                        config_.yaw_policy.c);
  // Configure obstacle policy
  obstacle_policy_.setOccupancyThreshold(config_.occupancy_threshold);
  obstacle_policy_.updateTuning(config_.obstacle_policy);

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
    obstacle_policy_.updateObstacles(*hashed_map, robot_position,
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

void WaveriderServer::robotStateCallback(
    anymal_msgs::AnymalState robot_state_msg) {
  ProfilerZoneScoped;
  // Get the velocities in body frame
  const Eigen::Vector3d B_v_B(robot_state_msg.twist.twist.linear.x,
                              robot_state_msg.twist.twist.linear.y,
                              robot_state_msg.twist.twist.linear.z);
  const Eigen::Vector3d B_w_B(robot_state_msg.twist.twist.angular.x,
                              robot_state_msg.twist.twist.angular.y,
                              robot_state_msg.twist.twist.angular.z);
  // Get the transform from body to odom
  const Eigen::Matrix3d O_R_B =
      Eigen::Quaterniond(robot_state_msg.pose.pose.orientation.w,
                         robot_state_msg.pose.pose.orientation.x,
                         robot_state_msg.pose.pose.orientation.y,
                         robot_state_msg.pose.pose.orientation.z)
          .toRotationMatrix();
  const Eigen::Vector3d O_t_B(robot_state_msg.pose.pose.position.x,
                              robot_state_msg.pose.pose.position.y,
                              robot_state_msg.pose.pose.position.z);
  // Get the transform from odom to map
  wavemap::Transformation3D M_T_O;
  const auto& timestamp = robot_state_msg.header.stamp;
  if (!transformer_.lookupTransform(map_frame_, config_.odom_frame, timestamp,
                                    M_T_O)) {
    LOG(WARNING) << "Could not look up transform from odom to map. "
                    "Ignoring robot pose update.";
    return;
  }
  const Eigen::Matrix3d M_R_O = M_T_O.getRotationMatrix().cast<double>();
  const Eigen::Vector3d M_t_O = M_T_O.getPosition().cast<double>();
  // Compute the transform from body to map
  const Eigen::Matrix3d M_R_B = M_R_O * O_R_B;
  const Eigen::Vector3d M_t_B = M_t_O + M_R_O * O_t_B;

  // Convert robot pose into map frame
  {
    std::scoped_lock lock(robot_state_.mutex);
    robot_state_.data.emplace();
    robot_state_.data->p() = M_t_B;
    robot_state_.data->q() = M_R_B;
    robot_state_.data->v() = M_R_B * B_v_B;
    robot_state_.data->w() = M_R_B * B_w_B;
    robot_state_.data->a().setZero();
    robot_state_.data->dw().setZero();
    robot_state_.time = timestamp.toNSec();
  }
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
  if (!obstacle_policy_.isReady()) {
    ROS_WARN("Policy not yet initialized.");
    return;
  }

  // Get the current robot state
  rmpcpp::SE3State current_state;
  uint64_t current_time;
  {
    std::unique_lock lock(robot_state_.mutex);
    if (!robot_state_.data.has_value()) {
      ROS_WARN("World state not yet initialized. Could not evaluate policy.");
      return;
    }
    current_state = robot_state_.data.value();
    current_time = robot_state_.time;
  }

  // Get the goal position
  {
    const auto goal = getGoalFromTf();
    if (!goal.has_value()) {
      ROS_INFO("Goal position not set. Will do nothing.");
      return;
    }
    goal_policy_.setTarget(goal->cast<double>());
  }

  // Forward integrate the state and policy to obtain velocity reference
  auto propagated_state = SE3toSE2(current_state);
  const double yaw_init = propagated_state.pos_.z();
  const double yaw_dot_init = propagated_state.vel_.z();
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
    auto attractor_r3_value = goal_policy_.evaluateAt(propagated_state_r3);
    auto attractor_se2_value =
        R3toSE2{}.at(propagated_state_r3).pull(attractor_r3_value);

    // Evaluate the yaw policy
    auto yaw_se2_value = yaw_policy_.evaluateAt(propagated_state);

    // Evaluate the static obstacle avoidance policy
    auto waverider_r3_value = obstacle_policy_.evaluateAt(propagated_state_r3);
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
        addFilteredObstaclesToMarkerArray(obstacle_policy_.getObstacleCells(),
                                          map_frame_, marker_array);
        marker_array.markers.emplace_back(goalPositionToMarker(
            goal_policy_.target().cast<FloatingPoint>(), map_frame_));
        marker_array.markers.emplace_back(
            robotPositionToMarker(current_state.p().cast<float>(), map_frame_));
        {
          const Vector3D v_r2{static_cast<float>(propagated_state.vel_.x()),
                              static_cast<float>(propagated_state.vel_.y()),
                              0.f};
          marker_array.markers.emplace_back(
              commandToMarker(current_state.p().cast<float>(), v_r2, map_frame_,
                              "v_r2", 0.f, 0.f, 1.f));
          const Vector3D v_yaw{0.f, 0.f,
                               static_cast<float>(propagated_state.vel_[2])};
          marker_array.markers.emplace_back(
              commandToMarker(current_state.p().cast<float>(), v_yaw,
                              map_frame_, "v_yaw", 0.f, 0.f, 1.f));
        }
        {
          const Vector3D f_attract_r2 = {
              static_cast<float>(attractor_se2_value.f_.x()),
              static_cast<float>(attractor_se2_value.f_.y()), 0.f};
          marker_array.markers.emplace_back(
              commandToMarker(current_state.p().cast<float>(), f_attract_r2,
                              map_frame_, "f_attract_r2", 0.f, 1.f, 0.f));
          const Vector3D f_attract_yaw = {
              0.f, 0.f, static_cast<float>(attractor_se2_value.f_.z())};
          marker_array.markers.emplace_back(
              commandToMarker(current_state.p().cast<float>(), f_attract_yaw,
                              map_frame_, "f_attract_yaw", 0.f, 1.f, 0.f));
        }
        {
          const Vector3D f_yaw_r2 = {static_cast<float>(yaw_se2_value.f_.x()),
                                     static_cast<float>(yaw_se2_value.f_.y()),
                                     0.f};
          marker_array.markers.emplace_back(
              commandToMarker(current_state.p().cast<float>(), f_yaw_r2,
                              map_frame_, "f_yaw_r2", 1.f, 1.f, 0.f));
          const Vector3D f_yaw_yaw = {0.f, 0.f,
                                      static_cast<float>(yaw_se2_value.f_.z())};
          marker_array.markers.emplace_back(
              commandToMarker(current_state.p().cast<float>(), f_yaw_yaw,
                              map_frame_, "f_yaw_yaw", 1.f, 1.f, 0.f));
        }
        {
          const Vector3D f_rep_r2 = {
              static_cast<float>(waverider_se2_value.f_.x()),
              static_cast<float>(waverider_se2_value.f_.y()), 0.f};
          marker_array.markers.emplace_back(
              commandToMarker(current_state.p().cast<float>(), f_rep_r2,
                              map_frame_, "f_rep_r2", 1.f, 0.f, 0.f));
          const Vector3D f_rep_yaw = {
              0.f, 0.f, static_cast<float>(waverider_se2_value.f_.z())};
          marker_array.markers.emplace_back(
              commandToMarker(current_state.p().cast<float>(), f_rep_yaw,
                              map_frame_, "f_rep_yaw", 1.f, 0.f, 0.f));
        }
        debug_pub_.publish(marker_array);
      }
    }
  }

  // Compute velocity reference
  // LOG(INFO) << "yaw before " << yaw_init << " after " <<
  // propagated_state.pos_.z(); LOG(INFO) << "yaw_dot before " << yaw_dot_init
  // << " after " << propagated_state.vel_.z();
  Eigen::Vector3d v_body = current_state.q().inverse() *
                           Eigen::Vector3d{propagated_state.vel_.x(),
                                           propagated_state.vel_.y(), 0.0};
  const double vel_yaw = propagated_state.vel_.z();

  // Send velocity reference to the locomotion controller
  geometry_msgs::TwistStamped twist_msg;
  twist_msg.header.stamp = ros::Time().fromNSec(current_time);
  twist_msg.header.frame_id = "base";
  twist_msg.twist.linear.x = v_body.x();
  twist_msg.twist.linear.y = v_body.y();
  twist_msg.twist.linear.z = v_body.z();
  twist_msg.twist.angular.z = vel_yaw;
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
  if (transformer_.lookupTransform(map_frame_, config_.goal_tf_frame,
                                   lookup_time, T_W_G)) {
    return T_W_G.getPosition();
  }
  return std::nullopt;
}

std::optional<Plane3D> WaveriderServer::getGroundPlaneFromTf() {
  ros::Time lookup_time =
      ros::Time::now() - ros::Duration(config_.tf_lookup_delay);
  wavemap::Transformation3D T_W_G;
  if (transformer_.lookupTransform(map_frame_, config_.ground_plane_tf_frame,
                                   lookup_time, T_W_G)) {
    Plane3D ground_plane;
    ground_plane.normal = T_W_G.getRotation().rotate(Vector3D::UnitZ());
    ground_plane.offset = ground_plane.normal.dot(T_W_G.getPosition()) +
                          config_.ground_plane_offset;
    return ground_plane;
  }
  return std::nullopt;
}
}  // namespace waverider
