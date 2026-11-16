#pragma once

#include <geometry_msgs/msg/quaternion.hpp>

namespace rcf_common {
/*!
 * Convert a quaternion to RPY.  Uses ZYX order (yaw-pitch-roll), but returns
 * angles in (roll, pitch, yaw).
 */
void quat_to_rpy(const geometry_msgs::msg::Quaternion& q, double& roll, double& pitch, double& yaw);

double yaw_from_quat(const geometry_msgs::msg::Quaternion& q);
}  // namespace rcf_common
