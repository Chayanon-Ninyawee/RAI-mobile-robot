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

  // Consider the robot aligned within about 1.7 degrees.
  const double rotate_threshold = 0.07;

  // Robot is still rotating toward the goal.
  if (yaw_error > rotate_threshold) {
    phase_ = Phase::ROTATE;
    return true;
  }

  // We are aligned, so make sure the robot has actually stopped
  // rotating before starting the settling timer.
  const double angular_velocity_threshold = 0.05;

  if (phase_ == Phase::ROTATE &&
      std::abs(current_wz_) < angular_velocity_threshold) {
    phase_ = Phase::SETTLE;
    aligned_time_ = rclcpp::Clock(RCL_ROS_TIME).now();
  }

  // If already settling, wait for the settling period.
  if (phase_ == Phase::SETTLE) {
    const double settle_time = 0.5;

    const double elapsed =
        (rclcpp::Clock(RCL_ROS_TIME).now() - aligned_time_).seconds();

    if (elapsed >= settle_time) {
      phase_ = Phase::MOVE;
    }
  }

  return true;
}

double GoalYawCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D &traj) {
  const auto &velocity = traj.velocity;

  const double vx = velocity.x;
  const double vy = velocity.y;
  const double wz = velocity.theta;

  const double initial_error =
      std::abs(angles::shortest_angular_distance(current_yaw_, goal_yaw_));

  const double final_yaw =
      traj.poses.empty() ? current_yaw_ : traj.poses.back().theta;

  const double final_error =
      std::abs(angles::shortest_angular_distance(final_yaw, goal_yaw_));

  const double rotate_threshold = 0.07;

  const double velocity_epsilon = 0.01;

  // ------------------------------------------------------------
  // Phase 1: ROTATE
  //
  // Only angular velocity is allowed.
  // ------------------------------------------------------------
  if (phase_ == Phase::ROTATE) {

    // Reject anything that translates.
    if (std::abs(vx) > velocity_epsilon || std::abs(vy) > velocity_epsilon) {
      return 1000.0;
    }

    // Reject trajectories that do not rotate.
    if (std::abs(wz) <= velocity_epsilon) {
      return 1000.0;
    }

    // Prefer trajectories that reduce the goal yaw error.
    if (initial_error > rotate_threshold) {
      return final_error / initial_error;
    }

    return final_error;
  }

  // ------------------------------------------------------------
  // Phase 2: SETTLE
  //
  // Everything must be zero.
  // ------------------------------------------------------------
  if (phase_ == Phase::SETTLE) {

    if (std::abs(vx) > velocity_epsilon || std::abs(vy) > velocity_epsilon ||
        std::abs(wz) > velocity_epsilon) {
      return 1000.0;
    }

    return final_error;
  }

  // ------------------------------------------------------------
  // Phase 3: MOVE
  //
  // Allow translation in X/Y.
  // Do not allow rotation.
  // ------------------------------------------------------------

  // Reject rotation.
  if (std::abs(wz) > velocity_epsilon) {
    return 1000.0;
  }

  // Allow both vx and vy because this is an omni robot.
  // DWB can now choose the translation direction needed
  // to reach the target while keeping the final heading fixed.

  return final_error;
}

} // namespace ired_dwb_critics

PLUGINLIB_EXPORT_CLASS(ired_dwb_critics::GoalYawCritic,
                       dwb_core::TrajectoryCritic)
