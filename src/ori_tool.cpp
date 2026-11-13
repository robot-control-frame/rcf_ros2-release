#include "rcf_common/ori_tool.hpp"

#include "rcf_common/math_utilities.hpp"
namespace rcf_common {
void quat_to_rpy(const geometry_msgs::msg::Quaternion& q, double& roll, double& pitch, double& yaw) {
  double as = std::min(-2. * (q.x * q.z - q.w * q.y), .99999);
  yaw = std::atan2(2 * (q.x * q.y + q.w * q.z), square(q.w) + square(q.x) - square(q.y) - square(q.z));
  pitch = std::asin(as);
  roll = std::atan2(2 * (q.y * q.z + q.w * q.x), square(q.w) - square(q.x) - square(q.y) + square(q.z));
}

double yaw_from_quat(const geometry_msgs::msg::Quaternion& q) {
  double roll, pitch, yaw;
  quat_to_rpy(q, roll, pitch, yaw);
  return yaw;
}
}  // namespace rcf_common
