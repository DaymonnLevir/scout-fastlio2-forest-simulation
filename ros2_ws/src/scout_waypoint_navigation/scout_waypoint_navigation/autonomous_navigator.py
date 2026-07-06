#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)

    return math.atan2(sin_yaw, cos_yaw)


def limit(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(value, maximum))


def distance_value(message: Float32) -> float:
    if message.data < 0.0:
        return math.inf

    return float(message.data)


class AutonomousNavigator(Node):

    GO_TO_GOAL = 'IR_AO_DESTINO'
    TURN_AVOID = 'GIRAR_PARA_DESVIAR'
    AVOID_FORWARD = 'AVANCAR_NO_DESVIO'
    GOAL_REACHED = 'DESTINO_ALCANCADO'

    def __init__(self) -> None:
        super().__init__('autonomous_navigator')

        self.declare_parameter('target_x', 3.0)
        self.declare_parameter('target_y', 0.0)

        # Quando verdadeiro, cria um destino à frente da pose inicial.
        self.declare_parameter('relative_goal', False)
        self.declare_parameter('relative_distance', 3.0)

        self.declare_parameter('position_tolerance', 0.20)
        self.declare_parameter('angle_tolerance', 0.25)

        self.declare_parameter('linear_gain', 0.5)
        self.declare_parameter('angular_gain', 1.5)

        self.declare_parameter('max_linear_speed', 0.18)
        self.declare_parameter('max_angular_speed', 0.40)

        self.declare_parameter('obstacle_distance', 0.80)
        self.declare_parameter('avoid_turn_angle_deg', 80.0)
        self.declare_parameter('extra_turn_angle_deg', 30.0)
        self.declare_parameter('avoid_forward_distance', 1.20)
        self.declare_parameter('avoid_linear_speed', 0.12)

        self.declare_parameter('sensor_timeout', 2.0)

        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)

        self.relative_goal = bool(
            self.get_parameter('relative_goal').value
        )

        self.relative_distance = float(
            self.get_parameter('relative_distance').value
        )

        self.position_tolerance = float(
            self.get_parameter('position_tolerance').value
        )

        self.angle_tolerance = float(
            self.get_parameter('angle_tolerance').value
        )

        self.linear_gain = float(
            self.get_parameter('linear_gain').value
        )

        self.angular_gain = float(
            self.get_parameter('angular_gain').value
        )

        self.max_linear_speed = float(
            self.get_parameter('max_linear_speed').value
        )

        self.max_angular_speed = float(
            self.get_parameter('max_angular_speed').value
        )

        self.obstacle_distance = float(
            self.get_parameter('obstacle_distance').value
        )

        self.avoid_turn_angle = math.radians(
            float(self.get_parameter('avoid_turn_angle_deg').value)
        )

        self.extra_turn_angle = math.radians(
            float(self.get_parameter('extra_turn_angle_deg').value)
        )

        self.avoid_forward_distance = float(
            self.get_parameter('avoid_forward_distance').value
        )

        self.avoid_linear_speed = float(
            self.get_parameter('avoid_linear_speed').value
        )

        self.sensor_timeout = float(
            self.get_parameter('sensor_timeout').value
        )

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.front_distance = math.inf
        self.left_distance = math.inf
        self.right_distance = math.inf

        self.odometry_received = False
        self.obstacle_data_received = False
        self.target_ready = not self.relative_goal

        self.last_obstacle_time = None

        self.state = self.GO_TO_GOAL
        self.avoid_direction = 1.0
        self.avoid_target_yaw = 0.0
        self.avoid_start_x = 0.0
        self.avoid_start_y = 0.0

        self.log_counter = 0
        self.wait_counter = 0

        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/Odometry',
            self.odometry_callback,
            10
        )

        self.front_subscription = self.create_subscription(
            Float32,
            '/obstacle_distance/front',
            self.front_callback,
            10
        )

        self.left_subscription = self.create_subscription(
            Float32,
            '/obstacle_distance/left',
            self.left_callback,
            10
        )

        self.right_subscription = self.create_subscription(
            Float32,
            '/obstacle_distance/right',
            self.right_callback,
            10
        )

        self.control_timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            'Navegador autônomo iniciado.'
        )

        self.get_logger().info(
            'Aguardando /Odometry e informações de obstáculos...'
        )

        if not self.relative_goal:
            self.get_logger().info(
                f'Destino absoluto: x={self.target_x:.2f} m, '
                f'y={self.target_y:.2f} m'
            )

    def odometry_callback(self, message: Odometry) -> None:
        self.current_x = message.pose.pose.position.x
        self.current_y = message.pose.pose.position.y

        orientation = message.pose.pose.orientation

        self.current_yaw = quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )

        if not self.odometry_received:
            self.get_logger().info(
                f'Pose inicial: x={self.current_x:.3f}, '
                f'y={self.current_y:.3f}, '
                f'yaw={math.degrees(self.current_yaw):.1f} graus'
            )

        self.odometry_received = True

        if self.relative_goal and not self.target_ready:
            self.target_x = (
                self.current_x
                + self.relative_distance * math.cos(self.current_yaw)
            )

            self.target_y = (
                self.current_y
                + self.relative_distance * math.sin(self.current_yaw)
            )

            self.target_ready = True

            self.get_logger().info(
                f'Destino relativo calculado: '
                f'x={self.target_x:.3f} m, '
                f'y={self.target_y:.3f} m'
            )

    def update_obstacle_time(self) -> None:
        self.last_obstacle_time = self.get_clock().now()
        self.obstacle_data_received = True

    def front_callback(self, message: Float32) -> None:
        self.front_distance = distance_value(message)
        self.update_obstacle_time()

    def left_callback(self, message: Float32) -> None:
        self.left_distance = distance_value(message)
        self.update_obstacle_time()

    def right_callback(self, message: Float32) -> None:
        self.right_distance = distance_value(message)
        self.update_obstacle_time()

    def obstacle_data_is_recent(self) -> bool:
        if self.last_obstacle_time is None:
            return False

        elapsed = (
            self.get_clock().now() - self.last_obstacle_time
        ).nanoseconds / 1e9

        return elapsed <= self.sensor_timeout

    def publish_stop(self) -> None:
        self.cmd_publisher.publish(Twist())

    @staticmethod
    def format_distance(distance: float) -> str:
        if not math.isfinite(distance):
            return 'livre'

        return f'{distance:.2f} m'

    def choose_avoidance_direction(self) -> None:
        left_score = (
            self.left_distance
            if math.isfinite(self.left_distance)
            else 100.0
        )

        right_score = (
            self.right_distance
            if math.isfinite(self.right_distance)
            else 100.0
        )

        if left_score >= right_score:
            self.avoid_direction = 1.0
            side_name = 'ESQUERDA'
        else:
            self.avoid_direction = -1.0
            side_name = 'DIREITA'

        self.avoid_target_yaw = normalize_angle(
            self.current_yaw
            + self.avoid_direction * self.avoid_turn_angle
        )

        self.state = self.TURN_AVOID
        self.publish_stop()

        self.get_logger().warn(
            f'Obstáculo a {self.front_distance:.2f} m. '
            f'Esquerda={self.format_distance(self.left_distance)}, '
            f'Direita={self.format_distance(self.right_distance)}. '
            f'Desvio escolhido: {side_name}.'
        )

    def go_to_goal(self, distance_to_goal: float) -> Twist:
        command = Twist()

        if (
            math.isfinite(self.front_distance)
            and self.front_distance < self.obstacle_distance
        ):
            self.choose_avoidance_direction()
            return command

        delta_x = self.target_x - self.current_x
        delta_y = self.target_y - self.current_y

        desired_yaw = math.atan2(delta_y, delta_x)

        angle_error = normalize_angle(
            desired_yaw - self.current_yaw
        )

        command.angular.z = limit(
            self.angular_gain * angle_error,
            -self.max_angular_speed,
            self.max_angular_speed
        )

        if abs(angle_error) <= self.angle_tolerance:
            linear_speed = self.linear_gain * distance_to_goal
            linear_speed *= max(0.0, math.cos(angle_error))

            command.linear.x = limit(
                linear_speed,
                0.0,
                self.max_linear_speed
            )

        return command

    def turn_to_avoid(self) -> Twist:
        command = Twist()

        angle_error = normalize_angle(
            self.avoid_target_yaw - self.current_yaw
        )

        if abs(angle_error) < math.radians(7.0):
            self.publish_stop()

            self.avoid_start_x = self.current_x
            self.avoid_start_y = self.current_y
            self.state = self.AVOID_FORWARD

            self.get_logger().info(
                'Giro de desvio concluído. '
                'Iniciando avanço lateral.'
            )

            return command

        command.angular.z = limit(
            self.angular_gain * angle_error,
            -self.max_angular_speed,
            self.max_angular_speed
        )

        return command

    def avoid_forward(self) -> Twist:
        command = Twist()

        distance_traveled = math.hypot(
            self.current_x - self.avoid_start_x,
            self.current_y - self.avoid_start_y
        )

        if distance_traveled >= self.avoid_forward_distance:
            self.state = self.GO_TO_GOAL
            self.publish_stop()

            self.get_logger().info(
                f'Desvio lateral concluído após '
                f'{distance_traveled:.2f} m. '
                f'Retomando o destino.'
            )

            return command

        if (
            math.isfinite(self.front_distance)
            and self.front_distance < self.obstacle_distance
        ):
            self.avoid_target_yaw = normalize_angle(
                self.current_yaw
                + self.avoid_direction * self.extra_turn_angle
            )

            self.state = self.TURN_AVOID
            self.publish_stop()

            self.get_logger().warn(
                'Ainda há obstáculo durante o desvio. '
                'Aumentando o ângulo de contorno.'
            )

            return command

        command.linear.x = self.avoid_linear_speed
        command.angular.z = 0.0

        return command

    def control_loop(self) -> None:
        if not self.odometry_received or not self.target_ready:
            return

        if not self.obstacle_data_received:
            self.publish_stop()

            self.wait_counter += 1

            if self.wait_counter >= 40:
                self.get_logger().info(
                    'Aguardando dados do obstacle_detector...'
                )
                self.wait_counter = 0

            return

        if not self.obstacle_data_is_recent():
            self.publish_stop()

            self.wait_counter += 1

            if self.wait_counter >= 40:
                self.get_logger().warn(
                    'Dados de obstáculos desatualizados. '
                    'Robô mantido parado.'
                )
                self.wait_counter = 0

            return

        delta_x = self.target_x - self.current_x
        delta_y = self.target_y - self.current_y

        distance_to_goal = math.hypot(delta_x, delta_y)

        if distance_to_goal <= self.position_tolerance:
            if self.state != self.GOAL_REACHED:
                self.state = self.GOAL_REACHED
                self.publish_stop()

                self.get_logger().info(
                    'Destino alcançado!'
                )

                self.get_logger().info(
                    f'Posição final: x={self.current_x:.3f}, '
                    f'y={self.current_y:.3f}'
                )

                self.get_logger().info(
                    f'Erro restante: {distance_to_goal:.3f} m'
                )

            return

        if self.state == self.GO_TO_GOAL:
            command = self.go_to_goal(distance_to_goal)

        elif self.state == self.TURN_AVOID:
            command = self.turn_to_avoid()

        elif self.state == self.AVOID_FORWARD:
            command = self.avoid_forward()

        else:
            command = Twist()

        self.cmd_publisher.publish(command)

        self.log_counter += 1

        if self.log_counter >= 20:
            self.get_logger().info(
                f'Estado={self.state} | '
                f'Posição=({self.current_x:.2f}, '
                f'{self.current_y:.2f}) | '
                f'Distância ao destino={distance_to_goal:.2f} m | '
                f'Frente={self.format_distance(self.front_distance)} | '
                f'v={command.linear.x:.2f} m/s | '
                f'w={command.angular.z:.2f} rad/s'
            )

            self.log_counter = 0


def main(args=None) -> None:
    rclpy.init(args=args)

    node = AutonomousNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
