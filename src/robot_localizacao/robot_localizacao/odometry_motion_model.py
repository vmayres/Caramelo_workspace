#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from math import sin, cos, sqrt, atan2, pi, fabs
import random
import time


# Função para calcular a diferença entre dois ângulos
def angle_diff(a, b):
    a =atan2(sin(a), cos(a))
    b =atan2(sin(b), cos(b))
    d1 = a - b
    d2 = 2 * pi - fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if fabs(d1) < fabs(d2):
        return d1
    else:
        return d2


class OdometryMotionModel(Node):
    def __init__(self):
        super().__init__('odometry_motion_model')
        self.is_first_odom_ = True
        self.last_odom_x = 0.0
        self.last_odom_y = 0.0
        self.last_odom_theta = 0.0

        # Declaração de parâmetros
        # alpha1, alpha2, alpha3, alpha4 são os parâmetros do modelo de movimento
        self.declare_parameter('alpha1', 0.1)
        self.declare_parameter('alpha2', 0.1)
        self.declare_parameter('alpha3', 0.1)
        self.declare_parameter('alpha4', 0.1)
        self.declare_parameter('num_samples', 300) # Número de amostras para o filtro de partículas

        # Obtenção dos parâmetros
        self.alpha1 = self.get_parameter('alpha1').get_parameter_value().double_value
        self.alpha2 = self.get_parameter('alpha2').get_parameter_value().double_value
        self.alpha3 = self.get_parameter('alpha3').get_parameter_value().double_value
        self.alpha4 = self.get_parameter('alpha4').get_parameter_value().double_value
        self.num_samples = self.get_parameter('num_samples').get_parameter_value().integer_value

        # Inicialização das amostras
        # Caso o número de amostras seja menor ou igual a zero, retorna um erro fatal
        if self.num_samples >= 0:
            self.samples = PoseArray()
            self.samples.poses = [Pose() for _ in range(self.num_samples)]
        else:
            self.get_logger().fatal("Numero invalido de amostras! Exit...")
            return

        # Inicialização dos tópicos
        # Assina a odometria e publica as amostras
        self.odom_sub_ = self.create_subscription(Odometry, "/mecanum_controller/odometry", self.odomCallback, 10)
        self.pose_array_pub_ = self.create_publisher(PoseArray, "/odometry_motion_model/samples", 10)


    def odomCallback(self, odom):
        # Extrai a orientação em ângulo (yaw) da odometria em quaternion
        q = [odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w]
        
        # Converte quaternion para ângulo (roll, pitch, yaw)
        roll, pitch, yaw = euler_from_quaternion(q)

        # Caso seja a primeira odometria recebida, inicializa as variáveis de controle
        if self.is_first_odom_:
            self.last_odom_x = odom.pose.pose.position.x
            self.last_odom_y = odom.pose.pose.position.y
            self.last_odom_theta = yaw

            self.samples.header.frame_id = odom.header.frame_id
            self.is_first_odom_ = False
            return

        # Calcula a mudança na posição e orientação
        # incrementos das posições x, y e theta
        delta_x = odom.pose.pose.position.x - self.last_odom_x 
        delta_y = odom.pose.pose.position.y - self.last_odom_y
        delta_theta = angle_diff(yaw, self.last_odom_theta)

        # Atualiza os últimos valores de odometria
        self.last_odom_x = odom.pose.pose.position.x
        self.last_odom_y = odom.pose.pose.position.y
        self.last_odom_theta = yaw

        if sqrt(pow(delta_y, 2) + pow(delta_x, 2)) < 0.001 and fabs(delta_theta) < 0.001:
            delta_rot1 = 0.0
        else:
            delta_rot1 = angle_diff(atan2(delta_y, delta_x), yaw)
        delta_trasl = sqrt(pow(delta_x, 2) + pow(delta_y, 2))
        delta_rot2 = angle_diff(delta_theta, delta_rot1)

        # Calcula as variâncias dos ruídos
        # rot1_variance, trasl_variance e rot2_variance são as variâncias dos ruídos
        rot1_variance = self.alpha1 * delta_rot1 + self.alpha2 * delta_trasl
        trasl_variance = self.alpha3 * delta_trasl + self.alpha4 *(delta_rot1 + delta_rot2)
        rot2_variance = self.alpha1 * delta_rot2 + self.alpha2 * delta_trasl

        random.seed(int(time.time()))
        for sample in self.samples.poses:
            # Adiciona ruído aos incrementos
            noisy_delta_rot1 = random.gauss(0.0, rot1_variance)
            noisy_delta_trasl = random.gauss(0.0, trasl_variance)
            noisy_delta_rot2 = random.gauss(0.0, rot2_variance)

            # Aplica os incrementos com ruído à amostra
            delta_rot1_draw = angle_diff(delta_rot1, noisy_delta_rot1)
            delta_trasl_draw = delta_trasl - noisy_delta_trasl
            delta_rot2_draw = angle_diff(delta_rot2, noisy_delta_rot2)

            # Atualiza a pose da amostra
            sample_q = [sample.orientation.x,
                        sample.orientation.y,
                        sample.orientation.z,
                        sample.orientation.w]
            sample_roll, sample_pitch, sample_yam = euler_from_quaternion(sample_q)
            sample.position.x += delta_trasl_draw * cos(sample_yam + delta_rot1_draw)
            sample.position.y += delta_trasl_draw * sin(sample_yam + delta_rot1_draw)
            q = quaternion_from_euler(0.0, 0.0, sample_yam + delta_rot1_draw + delta_rot2_draw)
            sample.orientation.x, sample.orientation.y, sample.orientation.z, sample.orientation.w = q

        self.last_odom_x = odom.pose.pose.position.x
        self.last_odom_y = odom.pose.pose.position.y
        self.last_odom_theta = yaw

        self.pose_array_pub_.publish(self.samples)

def main():
    rclpy.init()

    odometry_motion_model = OdometryMotionModel()
    rclpy.spin(odometry_motion_model)

    odometry_motion_model.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
