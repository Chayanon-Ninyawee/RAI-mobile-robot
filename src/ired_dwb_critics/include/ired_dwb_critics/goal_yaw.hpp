#ifndef IRED_DWB_CRITICS__GOAL_YAW_HPP_
#define IRED_DWB_CRITICS__GOAL_YAW_HPP_

#include "dwb_core/trajectory_critic.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ired_dwb_critics {

class GoalYawCritic : public dwb_core::TrajectoryCritic {
public:
  bool prepare(const geometry_msgs::msg::Pose2D &pose,
               const nav_2d_msgs::msg::Twist2D &vel,
               const geometry_msgs::msg::Pose2D &goal,
               const nav_2d_msgs::msg::Path2D &global_plan) override;

  double scoreTrajectory(const dwb_msgs::msg::Trajectory2D &traj) override;

private:
  double current_yaw_{0.0};
  double goal_yaw_{0.0};
  double current_wz_{0.0};

  rclcpp::Time aligned_time_;
  bool settling_{false};
};

} // namespace ired_dwb_critics

#endif // IRED_DWB_CRITICS__GOAL_YAW_HPP_
