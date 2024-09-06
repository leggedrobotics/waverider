#include "waverider_ros/policy_visuals.h"

namespace waverider {
void addFilteredObstaclesToMarkerArray(
    const waverider::ObstacleCells& policy_blocks,
    const std::string& world_frame,
    visualization_msgs::MarkerArray& marker_array) {
  // Add a marker for each resolution level
  const auto num_levels = static_cast<int>(policy_blocks.cell_widths.size());
  for (int i = 0; i < num_levels; i++) {
    if (policy_blocks.centers[i].empty()) {
      continue;
    }
    marker_array.markers.emplace_back(
        filteredObstacleLevelToMarker(i, policy_blocks.cell_widths[i],
                                      policy_blocks.centers[i], world_frame));
  }
}

visualization_msgs::Marker robotPositionToMarker(
    const Vector3D& robot_position, const std::string& world_frame) {
  visualization_msgs::Marker marker;

  marker.pose.orientation.w = 1.0;
  marker.pose.position.x = robot_position.x();
  marker.pose.position.y = robot_position.y();
  marker.pose.position.z = robot_position.z();
  marker.id = 100;
  marker.ns = "robot";
  marker.header.frame_id = world_frame;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.6;
  marker.scale.y = 0.6;
  marker.scale.z = 0.4;
  marker.color.r = 1.0;
  marker.color.a = 1.0;

  return marker;
}

visualization_msgs::Marker goalPositionToMarker(
    const Vector3D& robot_position, const std::string& world_frame) {
  visualization_msgs::Marker marker;

  marker.pose.orientation.w = 1.0;
  marker.pose.position.x = robot_position.x();
  marker.pose.position.y = robot_position.y();
  marker.pose.position.z = robot_position.z();
  marker.id = 100;
  marker.ns = "goal";
  marker.header.frame_id = world_frame;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.4;
  marker.scale.y = 0.4;
  marker.scale.z = 0.4;
  marker.color.g = 1.0;
  marker.color.a = 1.0;

  return marker;
}

visualization_msgs::Marker velocityCommandToMarker(
    const Vector3D& robot_position, const Vector3D& velocity_command,
    const std::string& world_frame) {
  visualization_msgs::Marker marker;

  const auto q = Eigen::Quaternion<FloatingPoint>::FromTwoVectors(
                     Vector3D::UnitX(), velocity_command)
                     .normalized();
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.orientation.w = q.w();
  marker.pose.position.x = robot_position.x();
  marker.pose.position.y = robot_position.y();
  marker.pose.position.z = robot_position.z();
  marker.id = 100;
  marker.ns = "velocity_command";
  marker.header.frame_id = world_frame;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = velocity_command.norm();
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  return marker;
}

visualization_msgs::Marker generateClearingMarker() {
  visualization_msgs::Marker marker;

  marker.action = visualization_msgs::Marker::DELETEALL;
  marker.ns = "clearing";
  marker.pose.orientation.w = 1.0;

  return marker;
}

visualization_msgs::Marker filteredObstacleLevelToMarker(
    int lvl, double size, const std::vector<Vector3D>& obstacle_centers,
    const std::string& world_frame) {
  static std::vector<std_msgs::ColorRGBA> colors;
  // init colors if needed
  if (colors.empty()) {
    colors.resize(7);
    colors[0].r = 250 / 255.0;
    colors[0].g = 250 / 255.0;
    colors[0].b = 110 / 255.0;
    colors[0].a = 1.0;

    colors[1].r = 170 / 255.0;
    colors[1].g = 228 / 255.0;
    colors[1].b = 121 / 255.0;
    colors[1].a = 1.0;

    colors[2].r = 100 / 255.0;
    colors[2].g = 201 / 255.0;
    colors[2].b = 135 / 255.0;
    colors[2].a = 1.0;

    colors[3].r = 35 / 255.0;
    colors[3].g = 170 / 255.0;
    colors[3].b = 143 / 255.0;
    colors[3].a = 1.0;

    colors[4].r = 0 / 255.0;
    colors[4].g = 137 / 255.0;
    colors[4].b = 138 / 255.0;
    colors[4].a = 1.0;

    colors[5].r = 23 / 255.0;
    colors[5].g = 104 / 255.0;
    colors[5].b = 119 / 255.0;
    colors[5].a = 1.0;

    colors[6].r = 42 / 255.0;
    colors[6].g = 72 / 255.0;
    colors[6].b = 88 / 255.0;
    colors[6].a = 1.0;
  }

  visualization_msgs::Marker marker;
  marker.header.frame_id = world_frame;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = lvl;
  marker.ns = "level_" + std::to_string(lvl);
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;
  marker.color = colors[lvl];
  // marker.color.r = 0.0;
  // marker.color.b = 1.0;
  // marker.color.g = 0.0;
  // marker.color.a = 1.0;
  marker.pose.orientation.w = 1.0;

  marker.points.resize(obstacle_centers.size());
  for (size_t i = 0; i < obstacle_centers.size(); ++i) {
    marker.points[i].x = obstacle_centers[i].x();
    marker.points[i].y = obstacle_centers[i].y();
    marker.points[i].z = obstacle_centers[i].z();
  }

  return marker;
}
}  // namespace waverider
