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

  const double yaw_error =
      std::abs(angles::shortest_angular_distance(current_yaw_, goal_yaw_));

  /*
   * If we are already facing the goal, don't care about rotation.
   */
  if (yaw_error < 0.10) {
    return 0.0;
  }

  /*
   * Look at the angular velocity used by this trajectory.
   *
   * We want the robot to rotate while it is moving.
   */
  const double wz = traj.velocity.theta;

  /*
   * Determine which direction the robot needs to rotate.
   *
   * Positive  -> counter-clockwise
   * Negative  -> clockwise
   */
  const double yaw_direction =
      angles::shortest_angular_distance(current_yaw_, goal_yaw_);

  /*
   * No rotation:
   * penalize it when there is still significant yaw error.
   */
  if (std::abs(wz) < 0.05) {
    return 1.0;
  }

  /*
   * Wrong rotation direction:
   * penalize heavily.
   */
  if ((yaw_direction > 0.0 && wz < 0.0) || (yaw_direction < 0.0 && wz > 0.0)) {
    return 1.0;
  }

  /*
   * Correct rotation direction.
   *
   * Smaller score = better.
   *
   * Prefer stronger rotation when the yaw error is large,
   * but don't make rotation dominate the translational critics.
   */
  const double rotation = std::min(std::abs(wz) / 0.8, 1.0);

  return 1.0 - rotation;
}

} // namespace ired_dwb_critics

PLUGINLIB_EXPORT_CLASS(ired_dwb_critics::GoalYawCritic,
                       dwb_core::TrajectoryCritic)
