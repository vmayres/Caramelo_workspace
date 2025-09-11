#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.constants import S_TO_NS
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler


class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')

        # Parameters
        self.declare_parameter('wheel_radius', 0.0325)
        self.declare_parameter('wheel_base', 0.169)
        self.declare_parameter('wheel_separation', 0.169)

        self.wheel_radius_ = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_base_ = self.get_parameter('wheel_base').get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter('wheel_separation').get_parameter_value().double_value

        self.get_logger().info(f'Using wheel radius {self.wheel_radius_}')
        self.get_logger().info(f'Using wheel base {self.wheel_base_}')
        self.get_logger().info(f'Using wheel separation {self.wheel_separation_}')

        # State (estilo bumperbot: variáveis separadas)
        self.fl_prev_pos_ = 0.0
        self.fr_prev_pos_ = 0.0
        self.rl_prev_pos_ = 0.0
        self.rr_prev_pos_ = 0.0
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

        # Publishers/Subscribers (alinhado ao mecanum_drive_controller)
        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, 'simple_velocity_controller/commands', 10)
        self.vel_sub_ = self.create_subscription(TwistStamped, 'mecanum_controller/reference', self.velCallback, 10)
        self.joint_sub_ = self.create_subscription(JointState, 'joint_states', self.jointCallback, 10)
        self.odom_pub_ = self.create_publisher(Odometry, 'mecanum_controller/odom', 10)

        # Mecanum conversion (estilo bumperbot: speed_conversion_ mapeia w->[vx,vy,wz])
        r = self.wheel_radius_
        L = self.wheel_base_
        W = self.wheel_separation_
        denom = 4.0
        self.speed_conversion_ = np.array([
            [ r/denom,  r/denom,  r/denom,  r/denom],   # vx
            [r/denom,  -r/denom,  -r/denom, r/denom],   # vy
            [ -r/(denom*(L+W)),  r/(denom*(L+W)), -r/(denom*(L+W)),  r/(denom*(L+W)) ]  # wz
        ])
        self.get_logger().info(f'Mecanum conversion matrix (3x4): {self.speed_conversion_}')

        # Odometry message invariant parts
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = 'odom'
        self.odom_msg_.child_frame_id = 'base_footprint'
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        # TF message
        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = 'odom'
        self.transform_stamped_.child_frame_id = 'base_footprint'

        # Time handling: initialize with node clock
        self.prev_time_ = self.get_clock().now()
    
    def velCallback(self, msg: TwistStamped):
        robot_speed = np.array([[msg.twist.linear.x],
                                [msg.twist.linear.y],
                                [msg.twist.angular.z]])
        pinv_conv = np.linalg.pinv(self.speed_conversion_)
        wheel_speed = np.matmul(pinv_conv, robot_speed)

        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[0, 0], wheel_speed[1, 0], wheel_speed[2, 0], wheel_speed[3, 0]]
        self.wheel_cmd_pub_.publish(wheel_speed_msg)

    def jointCallback(self, msg: JointState):
        pos = msg.position[:4]  # [FL, FR, RL, RR]
        now_stamp = Time.from_msg(msg.header.stamp)
        dt = now_stamp - self.prev_time_
        dt_s = dt.nanoseconds / S_TO_NS if dt.nanoseconds is not None else 0.0

        # Proteção contra dt inválido (primeiro pacote ou relógio igual/retrocedido)
        if dt_s <= 0.0:
            self.fl_prev_pos_ = pos[0]
            self.fr_prev_pos_ = pos[1]
            self.rl_prev_pos_ = pos[2]
            self.rr_prev_pos_ = pos[3]
            self.prev_time_ = now_stamp
            return

        # Deltas das rodas
        dp_fl = pos[0] - self.fl_prev_pos_
        dp_fr = pos[1] - self.fr_prev_pos_
        dp_rl = pos[2] - self.rl_prev_pos_
        dp_rr = pos[3] - self.rr_prev_pos_

        # Atualiza prev para próxima iteração
        self.fl_prev_pos_ = pos[0]
        self.fr_prev_pos_ = pos[1]
        self.rl_prev_pos_ = pos[2]
        self.rr_prev_pos_ = pos[3]
        self.prev_time_ = now_stamp

    # Velocidades angulares das rodas (rad/s)
        fi_fl = dp_fl / dt_s
        fi_fr = dp_fr / dt_s
        fi_rl = dp_rl / dt_s
        fi_rr = dp_rr / dt_s

        # Cinemática direta (robot frame) via fórmulas mecanum
        r = self.wheel_radius_
        L = self.wheel_base_
        W = self.wheel_separation_
        vx = (r / 4.0) * (fi_fl + fi_fr + fi_rl + fi_rr)
        vy = (r / 4.0) * (-fi_fl + fi_fr + fi_rl - fi_rr)
        wz = (r / (4.0 * (L + W))) * (-fi_fl + fi_fr - fi_rl + fi_rr)

        # Integração da pose (global)
        self.x_ += vx * dt_s
        self.y_ += vy * dt_s
        self.theta_ += wz * dt_s

        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.theta_)
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.pose.pose.orientation.x = qx
        self.odom_msg_.pose.pose.orientation.y = qy
        self.odom_msg_.pose.pose.orientation.z = qz
        self.odom_msg_.pose.pose.orientation.w = qw
        self.odom_msg_.twist.twist.linear.x = vx
        self.odom_msg_.twist.twist.linear.y = vy
        self.odom_msg_.twist.twist.angular.z = wz
        self.odom_pub_.publish(self.odom_msg_)

        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = qx
        self.transform_stamped_.transform.rotation.y = qy
        self.transform_stamped_.transform.rotation.z = qz
        self.transform_stamped_.transform.rotation.w = qw
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.br_.sendTransform(self.transform_stamped_)


def main():
    rclpy.init()
    node = SimpleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()