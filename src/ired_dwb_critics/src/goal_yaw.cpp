#include "ired_dwb_critics/goal_yaw.hpp"

#include <cmath>

#include "angles/angles.h"
#include "pluginlib/class_list_macros.hpp"

namespace ired_dwb_critics {

bool GoalYawCritic::prepare(const geometry_msgs::msg::Pose2D &,
                            const nav_2d_msgs::msg::Twist2D &,
                            const geometry_msgs::msg::Pose2D &goal,
                            const nav_2d_msgs::msg::Path2D &) {
  goal_yaw_ = goal.theta;
  return true;
}

double GoalYawCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D &traj) {
  if (traj.poses.empty()) {
    return 0.0;
  }

  const double final_yaw = traj.poses.back().theta;

  return std::abs(angles::shortest_angular_distance(final_yaw, goal_yaw_)) /
         M_PI;
}

} // namespace ired_dwb_critics

PLUGINLIB_EXPORT_CLASS(ired_dwb_critics::GoalYawCritic,
                       dwb_core::TrajectoryCritic)
