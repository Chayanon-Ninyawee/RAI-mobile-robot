#include "ired_dwb_critics/goal_yaw.hpp"

#include <algorithm>
#include <cmath>

#include "angles/angles.h"
#include "pluginlib/class_list_macros.hpp"

namespace ired_dwb_critics {

bool GoalYawCritic::prepare(const geometry_msgs::msg::Pose2D &pose,
                            const nav_2d_msgs::msg::Twist2D &,
                            const geometry_msgs::msg::Pose2D &goal,
                            const nav_2d_msgs::msg::Path2D &) {
  current_yaw_ = pose.theta;
  goal_yaw_ = goal.theta;
  return true;
}

double GoalYawCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D &traj) {
  if (traj.poses.empty()) {
    return 0.0;
  }

  const double current_error =
      std::abs(angles::shortest_angular_distance(current_yaw_, goal_yaw_));

  const double final_yaw = traj.poses.back().theta;

  const double final_error =
      std::abs(angles::shortest_angular_distance(final_yaw, goal_yaw_));

  // Reward trajectories that reduce the yaw error.
  //
  // 0.0 = maximum improvement
  // 1.0 = no improvement
  // >1.0 = yaw got worse
  double score = 1.0 + (final_error - current_error) / M_PI;

  return std::max(0.0, score);
}

} // namespace ired_dwb_critics

PLUGINLIB_EXPORT_CLASS(ired_dwb_critics::GoalYawCritic,
                       dwb_core::TrajectoryCritic)
