#include "ired_dwb_critics/goal_yaw.hpp"

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

  const double initial_error =
      std::abs(angles::shortest_angular_distance(current_yaw_, goal_yaw_));

  const double final_yaw = traj.poses.back().theta;

  const double final_error =
      std::abs(angles::shortest_angular_distance(final_yaw, goal_yaw_));

  // Rotate in place first.
  const double rotate_threshold = 0.17;

  if (initial_error > rotate_threshold) {
    bool translating = false;

    for (const auto &pose : traj.poses) {
      const double dx = pose.x - traj.poses.front().x;
      const double dy = pose.y - traj.poses.front().y;

      if (std::hypot(dx, dy) > 0.01) {
        translating = true;
        break;
      }
    }

    if (translating) {
      return 1000.0;
    }
  }

  // Once sufficiently aligned, prefer trajectories
  // that reduce the remaining goal yaw error.
  if (initial_error > 0.001) {
    return final_error / initial_error;
  }

  return final_error;
}

} // namespace ired_dwb_critics

PLUGINLIB_EXPORT_CLASS(ired_dwb_critics::GoalYawCritic,
                       dwb_core::TrajectoryCritic)
