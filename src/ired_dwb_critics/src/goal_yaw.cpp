#include "ired_dwb_critics/goal_yaw.hpp"

#include <cmath>

#include "angles/angles.h"
#include "pluginlib/class_list_macros.hpp"

namespace ired_dwb_critics {

bool GoalYawCritic::prepare(const geometry_msgs::msg::Pose2D &pose,
                            const nav_2d_msgs::msg::Twist2D &vel,
                            const geometry_msgs::msg::Pose2D &goal,
                            const nav_2d_msgs::msg::Path2D &) {
  current_x_ = pose.x;
  current_y_ = pose.y;

  current_yaw_ = pose.theta;
  goal_yaw_ = goal.theta;

  current_wz_ = vel.theta;

  goal_x_ = goal.x;
  goal_y_ = goal.y;

  goal_distance_ = std::hypot(goal_x_ - current_x_, goal_y_ - current_y_);

  // Start final rotation when we are close enough
  // to the goal position.
  const double rotate_distance = 0.10;

  if (goal_distance_ > rotate_distance) {
    phase_ = Phase::MOVE;
    return true;
  }

  // We are close to the goal.
  // Switch to final rotation.
  if (phase_ == Phase::MOVE) {
    phase_ = Phase::ROTATE;
  }

  const double yaw_error =
      std::abs(angles::shortest_angular_distance(current_yaw_, goal_yaw_));

  const double rotate_threshold = 0.07;

  // Once the final yaw is reached, wait for the robot
  // to physically stop rotating before finishing.
  if (phase_ == Phase::ROTATE && yaw_error <= rotate_threshold) {
    const double angular_velocity_threshold = 0.05;

    if (std::abs(current_wz_) < angular_velocity_threshold) {
      phase_ = Phase::SETTLE;

      aligned_time_ = rclcpp::Clock(RCL_ROS_TIME).now();
    }
  }

  if (phase_ == Phase::SETTLE) {
    const double settle_time = 0.5;

    const double elapsed =
        (rclcpp::Clock(RCL_ROS_TIME).now() - aligned_time_).seconds();

    if (elapsed >= settle_time) {
      // Stay in ROTATE after settling so the critic
      // continues protecting the final yaw.
      phase_ = Phase::ROTATE;
    }
  }

  return true;
}

double GoalYawCritic::scoreTrajectory(const dwb_msgs::msg::Trajectory2D &traj) {
  const auto &velocity = traj.velocity;

  const double vx = velocity.x;
  const double vy = velocity.y;
  const double wz = velocity.theta;

  const double final_yaw =
      traj.poses.empty() ? current_yaw_ : traj.poses.back().theta;

  const double final_error =
      std::abs(angles::shortest_angular_distance(final_yaw, goal_yaw_));

  const double yaw_error =
      std::abs(angles::shortest_angular_distance(current_yaw_, goal_yaw_));

  const double velocity_epsilon = 0.01;

  // ============================================================
  // PHASE 1: MOVE
  //
  // Far from the goal:
  // - Allow vx
  // - Allow vy
  // - Allow small wz
  //
  // Do NOT force the robot to maintain the final yaw yet.
  // ============================================================

  if (phase_ == Phase::MOVE) {

    // Strongly discourage rotation while travelling,
    // but do NOT completely forbid it.
    //
    // This allows DWB to make small heading corrections
    // instead of getting stuck.
    const double rotation_penalty = std::abs(wz) * 5.0;

    return final_error * 0.5 + rotation_penalty;
  }

  // ============================================================
  // PHASE 2: ROTATE
  //
  // Close to the goal:
  // - No vx
  // - No vy
  // - Only wz
  //
  // This forces the final heading correction to happen
  // in place.
  // ============================================================

  if (phase_ == Phase::ROTATE) {

    // Reject translation.
    if (std::abs(vx) > velocity_epsilon || std::abs(vy) > velocity_epsilon) {
      return 1000.0;
    }

    // If we still have yaw error, require rotation.
    if (yaw_error > 0.07 && std::abs(wz) <= velocity_epsilon) {
      return 1000.0;
    }

    // Prefer trajectories that reduce final yaw error.
    return final_error;
  }

  // ============================================================
  // PHASE 3: SETTLE
  //
  // Completely stop.
  // ============================================================

  if (phase_ == Phase::SETTLE) {

    if (std::abs(vx) > velocity_epsilon || std::abs(vy) > velocity_epsilon ||
        std::abs(wz) > velocity_epsilon) {
      return 1000.0;
    }

    return final_error;
  }

  return final_error;
}

} // namespace ired_dwb_critics

PLUGINLIB_EXPORT_CLASS(ired_dwb_critics::GoalYawCritic,
                       dwb_core::TrajectoryCritic)
