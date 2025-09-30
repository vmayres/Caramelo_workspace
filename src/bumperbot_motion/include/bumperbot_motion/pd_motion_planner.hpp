#ifndef PD_MOTION_PLANNER_HPP
#define PD_MOTION_PLANNER_HPP

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "tf2_ros/buffer.h"

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace bumperbot_motion
{
class PDMotionPlanner : public nav2_core::Controller
{
public:
  PDMotionPlanner() = default;
  ~PDMotionPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &,
    std::string name, std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override;

  void cleanup() override;

  void activate() override;

  void deactivate() override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path & path) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;

protected:
  geometry_msgs::msg::PoseStamped getNextPose(const geometry_msgs::msg::PoseStamped & robot_pose);

  bool transformPlan(const std::string & frame);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>> next_pose_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string plugin_name_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  rclcpp::Logger logger_ {rclcpp::get_logger("PDMotionPlanner")};
  rclcpp::Clock::SharedPtr clock_;

  double kp_;
  double kd_;
  double step_size_;
  double max_linear_velocity_;
  double max_angular_velocity_;

  rclcpp::Time last_cycle_time_;
  double prev_angular_error_;
  double prev_linear_error_;

  nav_msgs::msg::Path global_plan_;
};

}  // namespace bumperbot_motion

#endif  // PD_MOTION_PLANNER_HPP