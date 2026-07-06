#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


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


class WaypointController(Node):

    def __init__(self) -> None:
        super().__init__('waypoint_controller')

        self.declare_parameter('target_x', 1.0)
        self.declare_parameter('target_y', 0.0)

        self.declare_parameter('position_tolerance', 0.15)
        self.declare_parameter('angle_tolerance', 0.25)

        self.declare_parameter('linear_gain', 0.5)
        self.declare_parameter('angular_gain', 1.5)

        self.declare_parameter('max_linear_speed', 0.20)
        self.declare_parameter('max_angular_speed', 0.40)

        self.target_x = float(self.get_parameter('target_x').value)
        self.target_y = float(self.get_parameter('target_y').value)

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

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.odometry_received = False
        self.initial_pose_reported = False
        self.goal_reached = False

        self.log_counter = 0

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

        self.control_timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            f'Destino configurado: x={self.target_x:.2f} m, '
            f'y={self.target_y:.2f} m'
        )

        self.get_logger().info(
            'Aguardando odometria do FAST-LIO em /Odometry...'
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

        self.odometry_received = True

        if not self.initial_pose_reported:
            self.get_logger().info(
                f'Posição inicial: x={self.current_x:.3f}, '
                f'y={self.current_y:.3f}, '
                f'yaw={math.degrees(self.current_yaw):.1f} graus'
            )

            self.initial_pose_reported = True

    def publish_stop(self) -> None:
        self.cmd_publisher.publish(Twist())

    def control_loop(self) -> None:
        if not self.odometry_received:
            return

        if self.goal_reached:
            self.publish_stop()
            return

        delta_x = self.target_x - self.current_x
        delta_y = self.target_y - self.current_y

        distance = math.hypot(delta_x, delta_y)

        desired_yaw = math.atan2(delta_y, delta_x)
        angle_error = normalize_angle(
            desired_yaw - self.current_yaw
        )

        if distance <= self.position_tolerance:
            self.publish_stop()
            self.goal_reached = True

            self.get_logger().info(
                'Destino alcançado!'
            )

            self.get_logger().info(
                f'Posição final: x={self.current_x:.3f}, '
                f'y={self.current_y:.3f}'
            )

            self.get_logger().info(
                f'Erro restante: {distance:.3f} m'
            )

            return

        command = Twist()

        angular_speed = self.angular_gain * angle_error

        command.angular.z = limit(
            angular_speed,
            -self.max_angular_speed,
            self.max_angular_speed
        )

        if abs(angle_error) > self.angle_tolerance:
            command.linear.x = 0.0
        else:
            linear_speed = self.linear_gain * distance
            linear_speed *= max(0.0, math.cos(angle_error))

            command.linear.x = limit(
                linear_speed,
                0.0,
                self.max_linear_speed
            )

        self.cmd_publisher.publish(command)

        self.log_counter += 1

        if self.log_counter >= 20:
            self.get_logger().info(
                f'Posição=({self.current_x:.2f}, '
                f'{self.current_y:.2f}) | '
                f'Distância={distance:.2f} m | '
                f'Erro angular={math.degrees(angle_error):.1f} graus | '
                f'v={command.linear.x:.2f} m/s | '
                f'w={command.angular.z:.2f} rad/s'
            )

            self.log_counter = 0


def main(args=None) -> None:
    rclpy.init(args=args)

    node = WaypointController()

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
