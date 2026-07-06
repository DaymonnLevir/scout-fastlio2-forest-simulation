#!/usr/bin/env python3

import csv
import math
from pathlib import Path
from typing import List, Tuple

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


def convert_distance(message: Float32) -> float:
    if message.data < 0.0:
        return math.inf

    return float(message.data)


class MissionNavigator(Node):

    GO_TO_GOAL = 'IR_AO_DESTINO'
    TURN_AVOID = 'GIRAR_PARA_DESVIAR'
    AVOID_FORWARD = 'AVANCAR_NO_DESVIO'
    PAUSE = 'PAUSA_ENTRE_PONTOS'
    MISSION_COMPLETE = 'MISSAO_CONCLUIDA'

    def __init__(self) -> None:
        super().__init__('mission_navigator')

        self.declare_parameter(
            'waypoints',
            [
                3.0, 0.0,
                3.0, 2.0,
                0.0, 2.0,
                0.0, 0.0,
            ]
        )

        self.declare_parameter('waypoints_relative', True)

        self.declare_parameter('position_tolerance', 0.20)
        self.declare_parameter('angle_tolerance', 0.25)

        self.declare_parameter('linear_gain', 0.5)
        self.declare_parameter('angular_gain', 1.5)

        self.declare_parameter('max_linear_speed', 0.18)
        self.declare_parameter('max_angular_speed', 0.40)

        self.declare_parameter('obstacle_distance', 0.80)
        self.declare_parameter('avoid_turn_angle_deg', 80.0)
        self.declare_parameter('extra_turn_angle_deg', 25.0)
        self.declare_parameter('avoid_forward_distance', 1.20)
        self.declare_parameter('avoid_linear_speed', 0.12)

        self.declare_parameter('sensor_timeout', 2.0)
        self.declare_parameter('pause_between_goals', 2.0)

        self.declare_parameter(
            'csv_path',
            '/root/scout_forest_project/mission_results.csv'
        )

        raw_waypoints = [
            float(value)
            for value in self.get_parameter('waypoints').value
        ]

        if len(raw_waypoints) < 2 or len(raw_waypoints) % 2 != 0:
            raise ValueError(
                'O parâmetro waypoints deve conter pares x, y.'
            )

        self.local_waypoints: List[Tuple[float, float]] = []

        for index in range(0, len(raw_waypoints), 2):
            self.local_waypoints.append(
                (
                    raw_waypoints[index],
                    raw_waypoints[index + 1]
                )
            )

        self.waypoints_relative = bool(
            self.get_parameter('waypoints_relative').value
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

        self.pause_between_goals = float(
            self.get_parameter('pause_between_goals').value
        )

        self.csv_path = Path(
            str(self.get_parameter('csv_path').value)
        )

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.initial_x = 0.0
        self.initial_y = 0.0
        self.initial_yaw = 0.0

        self.front_distance = math.inf
        self.left_distance = math.inf
        self.right_distance = math.inf

        self.global_waypoints: List[Tuple[float, float]] = []

        self.odometry_received = False
        self.obstacle_data_received = False
        self.mission_ready = False

        self.last_obstacle_time = None

        self.current_goal_index = 0
        self.state = self.GO_TO_GOAL

        self.avoid_direction = 1.0
        self.avoid_target_yaw = 0.0

        self.avoid_start_x = 0.0
        self.avoid_start_y = 0.0

        self.goal_deviations = 0
        self.total_deviations = 0

        self.mission_start_time = 0.0
        self.goal_start_time = 0.0
        self.pause_start_time = 0.0

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

        self.prepare_csv()

        self.get_logger().info(
            'Navegador de missão iniciado.'
        )

        self.get_logger().info(
            f'Quantidade de pontos: {len(self.local_waypoints)}'
        )

        self.get_logger().info(
            'Aguardando /Odometry e obstacle_detector...'
        )

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def prepare_csv(self) -> None:
        self.csv_path.parent.mkdir(
            parents=True,
            exist_ok=True
        )

        with self.csv_path.open(
            'w',
            newline='',
            encoding='utf-8'
        ) as file:
            writer = csv.writer(file)

            writer.writerow([
                'ponto',
                'local_x',
                'local_y',
                'destino_x',
                'destino_y',
                'final_x',
                'final_y',
                'erro_m',
                'tempo_s',
                'desvios',
            ])

    def build_global_waypoints(self) -> None:
        self.global_waypoints.clear()

        cosine = math.cos(self.initial_yaw)
        sine = math.sin(self.initial_yaw)

        for local_x, local_y in self.local_waypoints:
            if self.waypoints_relative:
                global_x = (
                    self.initial_x
                    + local_x * cosine
                    - local_y * sine
                )

                global_y = (
                    self.initial_y
                    + local_x * sine
                    + local_y * cosine
                )
            else:
                global_x = local_x
                global_y = local_y

            self.global_waypoints.append(
                (global_x, global_y)
            )

        self.get_logger().info(
            'Pontos da missão no referencial camera_init:'
        )

        for index, waypoint in enumerate(
            self.global_waypoints,
            start=1
        ):
            self.get_logger().info(
                f'P{index}: '
                f'x={waypoint[0]:.3f}, '
                f'y={waypoint[1]:.3f}'
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
            self.initial_x = self.current_x
            self.initial_y = self.current_y
            self.initial_yaw = self.current_yaw

            self.get_logger().info(
                f'Pose inicial: '
                f'x={self.initial_x:.3f}, '
                f'y={self.initial_y:.3f}, '
                f'yaw={math.degrees(self.initial_yaw):.1f} graus'
            )

            self.build_global_waypoints()

            self.mission_start_time = self.now_seconds()
            self.goal_start_time = self.mission_start_time

            self.mission_ready = True

            self.log_current_goal()

        self.odometry_received = True

    def update_obstacle_time(self) -> None:
        self.last_obstacle_time = self.get_clock().now()
        self.obstacle_data_received = True

    def front_callback(self, message: Float32) -> None:
        self.front_distance = convert_distance(message)
        self.update_obstacle_time()

    def left_callback(self, message: Float32) -> None:
        self.left_distance = convert_distance(message)
        self.update_obstacle_time()

    def right_callback(self, message: Float32) -> None:
        self.right_distance = convert_distance(message)
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

    def current_goal(self) -> Tuple[float, float]:
        return self.global_waypoints[self.current_goal_index]

    def log_current_goal(self) -> None:
        target_x, target_y = self.current_goal()

        self.get_logger().info(
            '========================================'
        )

        self.get_logger().info(
            f'Iniciando ponto '
            f'{self.current_goal_index + 1}/'
            f'{len(self.global_waypoints)}'
        )

        self.get_logger().info(
            f'Destino atual: '
            f'x={target_x:.3f}, '
            f'y={target_y:.3f}'
        )

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

        self.goal_deviations += 1
        self.total_deviations += 1

        self.state = self.TURN_AVOID
        self.publish_stop()

        self.get_logger().warn(
            f'Obstáculo a {self.front_distance:.2f} m. '
            f'Esquerda={self.format_distance(self.left_distance)}, '
            f'Direita={self.format_distance(self.right_distance)}. '
            f'Desvio escolhido: {side_name}.'
        )

    def go_to_goal(
        self,
        target_x: float,
        target_y: float,
        distance_to_goal: float
    ) -> Twist:
        command = Twist()

        if (
            math.isfinite(self.front_distance)
            and self.front_distance < self.obstacle_distance
        ):
            self.choose_avoidance_direction()
            return command

        desired_yaw = math.atan2(
            target_y - self.current_y,
            target_x - self.current_x
        )

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
            linear_speed *= max(
                0.0,
                math.cos(angle_error)
            )

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
            self.avoid_start_x = self.current_x
            self.avoid_start_y = self.current_y

            self.state = self.AVOID_FORWARD

            self.get_logger().info(
                'Giro concluído. '
                'Iniciando avanço do desvio.'
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

            self.get_logger().info(
                f'Desvio concluído após '
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

            self.get_logger().warn(
                'Obstáculo ainda presente no desvio. '
                'Aumentando o ângulo.'
            )

            return command

        command.linear.x = self.avoid_linear_speed

        return command

    def save_goal_result(
        self,
        target_x: float,
        target_y: float,
        error: float
    ) -> None:
        elapsed = self.now_seconds() - self.goal_start_time

        local_x, local_y = self.local_waypoints[
            self.current_goal_index
        ]

        with self.csv_path.open(
            'a',
            newline='',
            encoding='utf-8'
        ) as file:
            writer = csv.writer(file)

            writer.writerow([
                self.current_goal_index + 1,
                f'{local_x:.6f}',
                f'{local_y:.6f}',
                f'{target_x:.6f}',
                f'{target_y:.6f}',
                f'{self.current_x:.6f}',
                f'{self.current_y:.6f}',
                f'{error:.6f}',
                f'{elapsed:.3f}',
                self.goal_deviations,
            ])

        self.get_logger().info(
            f'Ponto {self.current_goal_index + 1} alcançado!'
        )

        self.get_logger().info(
            f'Posição final: '
            f'x={self.current_x:.3f}, '
            f'y={self.current_y:.3f}'
        )

        self.get_logger().info(
            f'Erro: {error:.3f} m | '
            f'Tempo: {elapsed:.2f} s | '
            f'Desvios: {self.goal_deviations}'
        )

    def finish_current_goal(
        self,
        target_x: float,
        target_y: float,
        error: float
    ) -> None:
        self.publish_stop()

        self.save_goal_result(
            target_x,
            target_y,
            error
        )

        last_goal = (
            self.current_goal_index
            >= len(self.global_waypoints) - 1
        )

        if last_goal:
            self.state = self.MISSION_COMPLETE

            total_time = (
                self.now_seconds()
                - self.mission_start_time
            )

            self.get_logger().info(
                '========================================'
            )

            self.get_logger().info(
                'MISSÃO CONCLUÍDA!'
            )

            self.get_logger().info(
                f'Tempo total: {total_time:.2f} s'
            )

            self.get_logger().info(
                f'Total de desvios: '
                f'{self.total_deviations}'
            )

            self.get_logger().info(
                f'Resultados salvos em: '
                f'{self.csv_path}'
            )

            return

        self.state = self.PAUSE
        self.pause_start_time = self.now_seconds()

        self.get_logger().info(
            f'Pausa de {self.pause_between_goals:.1f} s '
            f'antes do próximo ponto.'
        )

    def start_next_goal(self) -> None:
        self.current_goal_index += 1

        self.goal_deviations = 0
        self.goal_start_time = self.now_seconds()

        self.state = self.GO_TO_GOAL

        self.log_current_goal()

    def control_loop(self) -> None:
        if not self.odometry_received or not self.mission_ready:
            return

        if self.state == self.MISSION_COMPLETE:
            self.publish_stop()
            return

        if self.state == self.PAUSE:
            self.publish_stop()

            elapsed_pause = (
                self.now_seconds()
                - self.pause_start_time
            )

            if elapsed_pause >= self.pause_between_goals:
                self.start_next_goal()

            return

        if not self.obstacle_data_received:
            self.publish_stop()

            self.wait_counter += 1

            if self.wait_counter >= 40:
                self.get_logger().info(
                    'Aguardando obstacle_detector...'
                )

                self.wait_counter = 0

            return

        if not self.obstacle_data_is_recent():
            self.publish_stop()

            self.wait_counter += 1

            if self.wait_counter >= 40:
                self.get_logger().warn(
                    'Dados de obstáculos desatualizados. '
                    'Robô parado.'
                )

                self.wait_counter = 0

            return

        target_x, target_y = self.current_goal()

        distance_to_goal = math.hypot(
            target_x - self.current_x,
            target_y - self.current_y
        )

        if distance_to_goal <= self.position_tolerance:
            self.finish_current_goal(
                target_x,
                target_y,
                distance_to_goal
            )

            return

        if self.state == self.GO_TO_GOAL:
            command = self.go_to_goal(
                target_x,
                target_y,
                distance_to_goal
            )

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
                f'Ponto={self.current_goal_index + 1}/'
                f'{len(self.global_waypoints)} | '
                f'Estado={self.state} | '
                f'Posição=({self.current_x:.2f}, '
                f'{self.current_y:.2f}) | '
                f'Distância={distance_to_goal:.2f} m | '
                f'Frente={self.format_distance(self.front_distance)} | '
                f'v={command.linear.x:.2f} m/s | '
                f'w={command.angular.z:.2f} rad/s'
            )

            self.log_counter = 0


def main(args=None) -> None:
    rclpy.init(args=args)

    node = MissionNavigator()

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
