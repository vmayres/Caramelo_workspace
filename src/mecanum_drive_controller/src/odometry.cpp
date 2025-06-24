/**
 * @file odometry.cpp
 * @brief Implementation of odometry calculations for mecanum drive robots
 *
 * @author Victor Oliveira Ayres (adapted from Addison Sears-Collins)
 * @date June 23, 2025
 */

#include "mecanum_drive_controller/odometry.hpp"

namespace mecanum_drive_controller
{

Odometry::Odometry(size_t velocity_rolling_window_size)
: timestamp_(0.0),
  x_(0.0),
  y_(0.0),
  heading_(0.0),
  linear_x_(0.0),
  linear_y_(0.0),
  angular_(0.0),
  wheel_separation_(0.0),
  wheel_base_(0.0),
  front_left_wheel_radius_(0.0),
  front_right_wheel_radius_(0.0),
  back_left_wheel_radius_(0.0),
  back_right_wheel_radius_(0.0),
  front_left_wheel_old_pos_(0.0),
  front_right_wheel_old_pos_(0.0),
  back_left_wheel_old_pos_(0.0),
  back_right_wheel_old_pos_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_x_accumulator_(velocity_rolling_window_size),
  linear_y_accumulator_(velocity_rolling_window_size),
  angular_accumulator_(velocity_rolling_window_size)
{
}

void Odometry::init(const rclcpp::Time & time)
{
  // Reset values
  resetOdometry();

  timestamp_ = time;
}

bool Odometry::update(
  double front_left_pos, double front_right_pos,
  double back_left_pos, double back_right_pos,
  const rclcpp::Time & time)
{
  /// Get current wheel joint positions
  const double fl_wheel_cur_pos = front_left_pos * front_left_wheel_radius_;
  const double fr_wheel_cur_pos = front_right_pos * front_right_wheel_radius_;
  const double bl_wheel_cur_pos = back_left_pos * back_left_wheel_radius_;
  const double br_wheel_cur_pos = back_right_pos * back_right_wheel_radius_;

  /// Estimate velocities using wheel positions
  const double dt = time.seconds() - timestamp_.seconds();
  
  if (dt < 0.0001)
    return false; // Interval too small

  const double fl_wheel_est_vel = (fl_wheel_cur_pos - front_left_wheel_old_pos_) / dt;
  const double fr_wheel_est_vel = (fr_wheel_cur_pos - front_right_wheel_old_pos_) / dt;
  const double bl_wheel_est_vel = (bl_wheel_cur_pos - back_left_wheel_old_pos_) / dt;
  const double br_wheel_est_vel = (br_wheel_cur_pos - back_right_wheel_old_pos_) / dt;

  /// Update old position values
  front_left_wheel_old_pos_ = fl_wheel_cur_pos;
  front_right_wheel_old_pos_ = fr_wheel_cur_pos;
  back_left_wheel_old_pos_ = bl_wheel_cur_pos;
  back_right_wheel_old_pos_ = br_wheel_cur_pos;

  updateFromVelocity(fl_wheel_est_vel, fr_wheel_est_vel, bl_wheel_est_vel, br_wheel_est_vel, time);
  return true;
}

bool Odometry::updateFromVelocity(
  double front_left_vel, double front_right_vel,
  double back_left_vel, double back_right_vel,
  const rclcpp::Time & time)
{
  timestamp_ = time;

  // Mecanum wheel kinematics
  // vx = (fl + fr + bl + br) / 4
  // vy = (-fl + fr + bl - br) / 4  
  // w = (-fl + fr - bl + br) / (4 * (wheel_separation + wheel_base) / 2)

  const double wheel_base_half = wheel_base_ / 2.0;
  const double wheel_separation_half = wheel_separation_ / 2.0;
  
  linear_x_ = (front_left_vel + front_right_vel + back_left_vel + back_right_vel) / 4.0;
  linear_y_ = (-front_left_vel + front_right_vel + back_left_vel - back_right_vel) / 4.0;
  angular_ = (-front_left_vel + front_right_vel - back_left_vel + back_right_vel) / 
             (4.0 * (wheel_base_half + wheel_separation_half));

  /// Integrate odometry
  integrateExact(linear_x_, linear_y_, angular_);

  return true;
}

void Odometry::updateOpenLoop(double linear_x, double linear_y, double angular, const rclcpp::Time & time)
{
  /// Save last linear and angular velocity
  linear_x_ = linear_x;
  linear_y_ = linear_y;
  angular_ = angular;

  /// Integrate odometry
  timestamp_ = time;
  integrateExact(linear_x, linear_y, angular);
}

void Odometry::resetOdometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
}

void Odometry::setWheelParams(
  double wheel_separation,
  double wheel_base,
  double front_left_wheel_radius,
  double front_right_wheel_radius,
  double back_left_wheel_radius,
  double back_right_wheel_radius)
{
  wheel_separation_ = wheel_separation;
  wheel_base_ = wheel_base;
  front_left_wheel_radius_ = front_left_wheel_radius;
  front_right_wheel_radius_ = front_right_wheel_radius;
  back_left_wheel_radius_ = back_left_wheel_radius;
  back_right_wheel_radius_ = back_right_wheel_radius;
}

void Odometry::setVelocityRollingWindowSize(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;
  resetAccumulators();
}

void Odometry::integrateRungeKutta2(double linear_x, double /*linear_y*/, double angular)
{
  const double dt = timestamp_.seconds() - timestamp_.seconds();
  const double direction = heading_ + angular * 0.5;

  /// Runge-Kutta 2nd order integration
  x_ += linear_x * std::cos(direction) * dt;
  y_ += linear_x * std::sin(direction) * dt;
  heading_ += angular * dt;
}

void Odometry::integrateExact(double linear_x, double linear_y, double angular)
{
  const double dt = timestamp_.seconds() - timestamp_.seconds();

  if (std::abs(angular) < 1e-6)
  {
    /// Angular velocity is zero, so there is no rotation
    const double direction = heading_;
    x_ += (linear_x * std::cos(direction) - linear_y * std::sin(direction)) * dt;
    y_ += (linear_x * std::sin(direction) + linear_y * std::cos(direction)) * dt;
  }
  else
  {
    /// Exact integration of the differential equation
    const double heading_old = heading_;
    const double r = linear_x / angular;
    const double h = linear_y / angular;
    heading_ += angular * dt;

    x_ += r * (std::sin(heading_) - std::sin(heading_old)) + h * (std::cos(heading_old) - std::cos(heading_));
    y_ += r * (std::cos(heading_old) - std::cos(heading_)) + h * (std::sin(heading_) - std::sin(heading_old));
  }
}

void Odometry::resetAccumulators()
{
  linear_x_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  linear_y_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
  angular_accumulator_ = RollingMeanAccumulator(velocity_rolling_window_size_);
}

}  // namespace mecanum_drive_controller
