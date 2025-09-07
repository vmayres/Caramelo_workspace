#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from tf_transformations import quaternion_from_euler


class NoisyController(Node):
    def __init__(self):
        super().__init__("noisy_controller")
        
        self.declare_parameter("wheel_radius", 0.0325)
        self.declare_parameter("wheel_base", 0.169)
        self.declare_parameter("wheel_separation", 0.169)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_base_ = self.get_parameter("wheel_base").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info(f"Using wheel radius {self.wheel_radius_}")
        self.get_logger().info(f"Using wheel base {self.wheel_base_}")
        self.get_logger().info(f"Using wheel separation {self.wheel_separation_}")

        self.prev_pos_ = [0.0, 0.0, 0.0, 0.0]  # FL, FR, RL, RR
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.jointCallback, 10)
        self.odom_pub_ = self.create_publisher(Odometry, "bumperbot_controller/odom_noisy", 10)

        # Fill the Odometry message with invariant parameters
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint_ekf"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        # Fill the TF message
        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint_noisy"

        self.prev_time_ = self.get_clock().now()

    def jointCallback(self, msg):
        # Mecanum: FL, FR, RL, RR
        # Add noise to each wheel
        pos = [msg.position[i] + np.random.normal(0, 0.005) for i in range(4)]
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_
        dpos = [pos[i] - self.prev_pos_[i] for i in range(4)]
        self.prev_pos_ = pos.copy()
        self.prev_time_ = Time.from_msg(msg.header.stamp)

        # Wheel angular velocities
        fi = [dpos[i] / (dt.nanoseconds / S_TO_NS) for i in range(4)]
        r = self.wheel_radius_
        L = self.wheel_base_
        W = self.wheel_separation_

        # Direct kinematics (robot velocities)
        vx = r / 4 * (fi[0] + fi[1] + fi[2] + fi[3])
        vy = r / 4 * (-fi[0] + fi[1] + fi[2] - fi[3])
        wz = r / (4 * (L + W)) * (-fi[0] + fi[1] - fi[2] + fi[3])

        # Integrate position
        self.x_ += vx * (dt.nanoseconds / S_TO_NS)
        self.y_ += vy * (dt.nanoseconds / S_TO_NS)
        self.theta_ += wz * (dt.nanoseconds / S_TO_NS)

        # Compose and publish odometry
        q = quaternion_from_euler(0, 0, self.theta_)
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.twist.twist.linear.x = vx
        self.odom_msg_.twist.twist.linear.y = vy
        self.odom_msg_.twist.twist.angular.z = wz
        self.odom_pub_.publish(self.odom_msg_)

        # TF
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.br_.sendTransform(self.transform_stamped_)


def main():
    rclpy.init()

    noisy_controller = NoisyController()
    rclpy.spin(noisy_controller)
    
    noisy_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
