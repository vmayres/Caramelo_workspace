/**
 * @file mecanum_drive_controller.cpp
 * @brief Implementation of the mecanum drive controller for ROS 2
 *
 * @author Victor Oliveira Ayres (adapted from Addison Sears-Collins)
 * @date June 23, 2025
 */

#include "mecanum_drive_controller/mecanum_drive_controller.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace mecanum_drive_controller
{

MecanumDriveController::MecanumDriveController() 
: controller_interface::ControllerInterface(),
  params_(),
  odometry_(10) // Default rolling window size
{
}

controller_interface::CallbackReturn MecanumDriveController::on_init()
{
  try
  {
    // Initialize parameter client
    param_listener_ = std::make_shared<mecanum_drive_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration 
MecanumDriveController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  command_interfaces_config.names.push_back(params_.front_left_joint_name + "/velocity");
  command_interfaces_config.names.push_back(params_.front_right_joint_name + "/velocity");
  command_interfaces_config.names.push_back(params_.back_left_joint_name + "/velocity");
  command_interfaces_config.names.push_back(params_.back_right_joint_name + "/velocity");

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration 
MecanumDriveController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  state_interfaces_config.names.push_back(params_.front_left_joint_name + "/position");
  state_interfaces_config.names.push_back(params_.front_right_joint_name + "/position");
  state_interfaces_config.names.push_back(params_.back_left_joint_name + "/position");
  state_interfaces_config.names.push_back(params_.back_right_joint_name + "/position");

  return state_interfaces_config;
}

controller_interface::CallbackReturn MecanumDriveController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Update parameters
  params_ = param_listener_->get_params();

  // Setup odometry
  odometry_.setWheelParams(
    params_.wheel_separation, 
    params_.wheel_base,
    params_.wheel_radius, 
    params_.wheel_radius,
    params_.wheel_radius, 
    params_.wheel_radius);

  // Create velocity command subscriber
  velocity_command_subscriber_ = get_node()->create_subscription<Twist>(
    "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<Twist> msg) -> void
    {
      if (!subscriber_is_active_)
      {
        RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
        return;
      }
      if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
      {
        RCLCPP_WARN_ONCE(
          get_node()->get_logger(),
          "Received TwistStamped with zero timestamp, setting it to current time");
        msg->header.stamp = get_node()->get_clock()->now();
      }
      received_velocity_msg_ptr_.set(std::move(msg));
    });

  // Create odometry publisher
  odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
    "~/odom", rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ = 
    std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odometry_publisher_);

  // Create wheel commands publisher
  wheel_commands_publisher_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/mecanum_controller/commands", rclcpp::SystemDefaultsQoS());
  realtime_wheel_commands_publisher_ = 
    std::make_shared<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(
      wheel_commands_publisher_);

  RCLCPP_INFO(get_node()->get_logger(), "Mecanum drive controller configured successfully");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumDriveController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Clear out vectors in case of restart
  front_left_wheel_names_.clear();
  front_right_wheel_names_.clear();
  back_left_wheel_names_.clear();
  back_right_wheel_names_.clear();

  // Assign wheels names
  front_left_wheel_names_.push_back(params_.front_left_joint_name);
  front_right_wheel_names_.push_back(params_.front_right_joint_name);
  back_left_wheel_names_.push_back(params_.back_left_joint_name);
  back_right_wheel_names_.push_back(params_.back_right_joint_name);

  // Initialize odometry
  odometry_.init(get_node()->get_clock()->now());

  // Reset controller variables
  reset();

  subscriber_is_active_ = true;

  RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumDriveController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  subscriber_is_active_ = false;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MecanumDriveController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  auto logger = get_node()->get_logger();
  if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    if (!is_halted)
    {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }

  std::shared_ptr<Twist> last_command_msg;
  received_velocity_msg_ptr_.get(last_command_msg);

  if (last_command_msg == nullptr)
  {
    RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  const auto age_of_last_command = time - last_command_msg->header.stamp;

  // Check command timeout
  if (age_of_last_command > cmd_vel_timeout_)
  {
    last_command_msg->twist.linear.x = 0.0;
    last_command_msg->twist.linear.y = 0.0;
    last_command_msg->twist.angular.z = 0.0;
  }

  // Get current wheel positions
  const double front_left_pos = state_interfaces_[0].get_value();
  const double front_right_pos = state_interfaces_[1].get_value();
  const double back_left_pos = state_interfaces_[2].get_value();
  const double back_right_pos = state_interfaces_[3].get_value();

  // Update odometry
  if (odometry_.update(front_left_pos, front_right_pos, back_left_pos, back_right_pos, time))
  {
    // Publish odometry message
    if (realtime_odometry_publisher_->trylock())
    {
      auto & odometry_message = realtime_odometry_publisher_->msg_;
      odometry_message.header.stamp = time;
      odometry_message.header.frame_id = "odom";
      odometry_message.child_frame_id = "base_footprint";
      odometry_message.pose.pose.position.x = odometry_.getX();
      odometry_message.pose.pose.position.y = odometry_.getY();
      odometry_message.pose.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0, 0, odometry_.getHeading());
      odometry_message.pose.pose.orientation.x = q.x();
      odometry_message.pose.pose.orientation.y = q.y();
      odometry_message.pose.pose.orientation.z = q.z();
      odometry_message.pose.pose.orientation.w = q.w();

      odometry_message.twist.twist.linear.x = odometry_.getLinearX();
      odometry_message.twist.twist.linear.y = odometry_.getLinearY();
      odometry_message.twist.twist.angular.z = odometry_.getAngular();

      realtime_odometry_publisher_->unlockAndPublish();
    }
  }

  // Convert velocity command to wheel velocities using mecanum drive kinematics
  const double vx = last_command_msg->twist.linear.x;
  const double vy = last_command_msg->twist.linear.y;
  const double vtheta = last_command_msg->twist.angular.z;

  // Mecanum drive inverse kinematics
  const double wheel_base_half = params_.wheel_base / 2.0;
  const double wheel_separation_half = params_.wheel_separation / 2.0;
  const double wheel_radius = params_.wheel_radius;

  const double front_left_vel = (vx - vy - vtheta * (wheel_base_half + wheel_separation_half)) / wheel_radius;
  const double front_right_vel = (vx + vy + vtheta * (wheel_base_half + wheel_separation_half)) / wheel_radius;
  const double back_left_vel = (vx + vy - vtheta * (wheel_base_half + wheel_separation_half)) / wheel_radius;
  const double back_right_vel = (vx - vy + vtheta * (wheel_base_half + wheel_separation_half)) / wheel_radius;

  // Publish wheel commands for external hardware interface
  if (realtime_wheel_commands_publisher_->trylock())
  {
    auto & wheel_commands_msg = realtime_wheel_commands_publisher_->msg_;
    wheel_commands_msg.data.clear();
    wheel_commands_msg.data.resize(4);
    wheel_commands_msg.data[0] = front_left_vel;   // front_left_wheel
    wheel_commands_msg.data[1] = front_right_vel;  // front_right_wheel
    wheel_commands_msg.data[2] = back_left_vel;    // rear_left_wheel (back_left)
    wheel_commands_msg.data[3] = back_right_vel;   // rear_right_wheel (back_right)
    realtime_wheel_commands_publisher_->unlockAndPublish();
  }

  // Assign velocities to wheels
  command_interfaces_[0].set_value(front_left_vel);
  command_interfaces_[1].set_value(front_right_vel);
  command_interfaces_[2].set_value(back_left_vel);
  command_interfaces_[3].set_value(back_right_vel);

  previous_update_timestamp_ = time;
  return controller_interface::return_type::OK;
}

bool MecanumDriveController::reset()
{
  odometry_.resetOdometry();

  // Release the old queue
  std::queue<Twist> empty_queue;
  previous_commands_.swap(empty_queue);

  subscriber_is_active_ = false;
  received_velocity_msg_ptr_.set(std::make_shared<Twist>());
  is_halted = false;

  return true;
}

void MecanumDriveController::halt()
{
  const auto halt_wheels = [&]()
  {
    command_interfaces_[0].set_value(0.0);
    command_interfaces_[1].set_value(0.0);
    command_interfaces_[2].set_value(0.0);
    command_interfaces_[3].set_value(0.0);
  };

  halt_wheels();
}

}  // namespace mecanum_drive_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  mecanum_drive_controller::MecanumDriveController, controller_interface::ControllerInterface)
