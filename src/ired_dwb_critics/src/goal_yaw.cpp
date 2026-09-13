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

  double total_error = 0.0;
  double total_weight = 0.0;

  const size_t count = traj.poses.size();

  for (size_t i = 0; i < count; ++i) {
    const double yaw_error = std::abs(
        angles::shortest_angular_distance(traj.poses[i].theta, goal_yaw_));

    /*
     * Give earlier poses more importance.
     *
     * weight = 2.0 at the beginning
     * weight = 1.0 at the end
     *
     * This encourages the robot to start rotating early
     * instead of waiting until the end of the trajectory.
     */
    const double progress =
        static_cast<double>(i) / static_cast<double>(count - 1);

    const double weight = 2.0 - progress;

    total_error += yaw_error * weight;
    total_weight += weight;
  }

  const double average_error = total_error / total_weight;

  /*
   * Normalize to approximately 0..1.
   *
   * Compare against the current error so trajectories that
   * actually improve the heading are preferred.
   */
  double score = 1.0 + (average_error - current_error) / M_PI;

  return std::max(0.0, score);
}

} // namespace ired_dwb_critics

PLUGINLIB_EXPORT_CLASS(ired_dwb_critics::GoalYawCritic,
                       dwb_core::TrajectoryCritic)
