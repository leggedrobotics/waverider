#ifndef WAVERIDER_ROS_POLICY_VISUALS_H_
#define WAVERIDER_ROS_POLICY_VISUALS_H_

#include <string>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <waverider/obstacle_filter.h>

namespace waverider {
void addFilteredObstaclesToMarkerArray(
    const ObstacleCells& policy_blocks, const std::string& world_frame,
    visualization_msgs::MarkerArray& marker_array);

visualization_msgs::Marker generateClearingMarker();

visualization_msgs::Marker robotPositionToMarker(
    const Vector3D& robot_position, const std::string& world_frame);

visualization_msgs::Marker goalPositionToMarker(const Vector3D& robot_position,
                                                const std::string& world_frame);

visualization_msgs::Marker commandToMarker(const Vector3D& robot_position,
                                           const Vector3D& command,
                                           const std::string& world_frame,
                                           const std::string& ns, float r,
                                           float g, float b);

visualization_msgs::Marker filteredObstacleLevelToMarker(
    int lvl, double size, const std::vector<Vector3D>& obstacle_centers,
    const std::string& world_frame);
}  // namespace waverider

#endif  // WAVERIDER_ROS_POLICY_VISUALS_H_
