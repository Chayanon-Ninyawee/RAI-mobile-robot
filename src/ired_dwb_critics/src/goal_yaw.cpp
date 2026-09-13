#include "ired_dwb_critics/goal_yaw.hpp"

#include <cmath>

#include "angles/angles.h"
#include "pluginlib/class_list_macros.hpp"

namespace ired_dwb_critics {

bool GoalYawCritic::prepare(const geometry_msgs::msg::Pose2D &pose,
                            const nav_2d_msgs::msg::Twist2D &vel,
                            const geometry_msgs::msg::Pose2D &goal,
                            const nav_2d_msgs::msg::Path2D &) {
  current_yaw_ = pose.theta;
  goal_yaw_ = goal.theta;
  current_wz_ = vel.theta;

  const double yaw_error =
      std::abs(angles::shortest_angular_distance(current_yaw_, goal_yaw_));

  // Start rotating again if we drift outside
  // the alignment threshold.
  const double rotate_threshold = 0.17;

  if (yaw_error > rotate_threshold) {
    settling_ = false;
    return true;
  }

  // Only start the settling timer once the robot
  // has actually stopped rotating.
  const double angular_velocity_threshold = 0.05;

  if (!settling_ && std::abs(current_wz_) < angular_velocity_threshold) {
    aligned_time_ = rclcpp::Clock(RCL_ROS_TIME).now();
    settling_ = true;
  }

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

  const double rotate_threshold = 0.17;

  // --------------------------------------------------
  // Phase 1: Rotate toward the goal heading.
  // --------------------------------------------------
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

    // Completely forbid translation while rotating.
    if (translating) {
      return 1000.0;
    }

    // Prefer trajectories that reduce the yaw error.
    return final_error / initial_error;
  }

  // --------------------------------------------------
  // Phase 2: Settling.
  // --------------------------------------------------
  if (settling_) {
    const double settle_time = 0.5;

    const double elapsed =
        (rclcpp::Clock(RCL_ROS_TIME).now() - aligned_time_).seconds();

    if (elapsed < settle_time) {
      bool translating = false;

      for (const auto &pose : traj.poses) {
        const double dx = pose.x - traj.poses.front().x;

        const double dy = pose.y - traj.poses.front().y;

        if (std::hypot(dx, dy) > 0.01) {
          translating = true;
          break;
        }
      }

      // Still settling, so forbid translation.
      if (translating) {
        return 1000.0;
      }

      // During settling, still prefer staying close
      // to the target heading.
      return final_error;
    }
  }

  // --------------------------------------------------
  // Phase 3: Settled.
  // Translation is now allowed.
  // --------------------------------------------------
  return final_error;
}

} // namespace ired_dwb_critics

PLUGINLIB_EXPORT_CLASS(ired_dwb_critics::GoalYawCritic,
                       dwb_core::TrajectoryCritic)
