#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomTFBroadcaster(Node):
    """
    Subscribes to an Odometry topic (default: /mecanum_controller/odometry)
    and broadcasts a TF transform from odom -> base_footprint using the
    pose from the odometry message.

    Parameters:
    - odom_topic (string): Odometry topic to subscribe. Default: /mecanum_controller/odometry
    - parent_frame (string): Parent frame for TF. Default: odom
    - child_frame (string): Child frame for TF. Default: base_footprint
    - use_child_from_msg (bool): If true and msg.child_frame_id is set, use it. Default: true
    - use_parent_from_msg (bool): If true and msg.header.frame_id is set, use it. Default: true
    """

    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        # Parameters (seguindo o estilo do professor)
        self.declare_parameter('odom_topic', '/mecanum_controller/odometry')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_footprint')
        self.declare_parameter('use_child_from_msg', True)
        self.declare_parameter('use_parent_from_msg', True)

        self.odom_topic_ = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.parent_frame_ = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child_frame_ = self.get_parameter('child_frame').get_parameter_value().string_value
        self.use_child_from_msg_ = self.get_parameter('use_child_from_msg').get_parameter_value().bool_value
        self.use_parent_from_msg_ = self.get_parameter('use_parent_from_msg').get_parameter_value().bool_value

        # TF broadcaster e mensagem TF pré-alocada (como no exemplo do professor)
        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header.frame_id = self.parent_frame_
        self.transform_stamped_.child_frame_id = self.child_frame_

        # Subscriber para a odometria
        self.sub_ = self.create_subscription(Odometry, self.odom_topic_, self.on_odom, 10)

        self.get_logger().info(
            f"Odom TF broadcaster: topic='{self.odom_topic_}', parent='{self.parent_frame_}', child='{self.child_frame_}'"
        )

    def on_odom(self, msg: Odometry):
        # Atualiza frames, respeitando flags para usar o que vem na msg
        parent = self.parent_frame_
        child = self.child_frame_
        if self.use_parent_from_msg_ and msg.header.frame_id:
            parent = msg.header.frame_id
        if self.use_child_from_msg_ and msg.child_frame_id:
            child = msg.child_frame_id

        self.transform_stamped_.header.frame_id = parent
        self.transform_stamped_.child_frame_id = child

        # Timestamp
        self.transform_stamped_.header.stamp = msg.header.stamp

        # TF (seguindo o padrão de atribuição do professor, campo-a-campo)
        self.transform_stamped_.transform.translation.x = msg.pose.pose.position.x
        self.transform_stamped_.transform.translation.y = msg.pose.pose.position.y
        self.transform_stamped_.transform.translation.z = msg.pose.pose.position.z
        self.transform_stamped_.transform.rotation.x = msg.pose.pose.orientation.x
        self.transform_stamped_.transform.rotation.y = msg.pose.pose.orientation.y
        self.transform_stamped_.transform.rotation.z = msg.pose.pose.orientation.z
        self.transform_stamped_.transform.rotation.w = msg.pose.pose.orientation.w

        # Publica TF
        self.br_.sendTransform(self.transform_stamped_)


def main():
    rclpy.init()
    node = OdomTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
