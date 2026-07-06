#!/usr/bin/env python3

import heapq
import math
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Float32


class ObstacleDetector(Node):

    def __init__(self) -> None:
        super().__init__('obstacle_detector')

        self.declare_parameter('z_min', -0.10)
        self.declare_parameter('z_max', 0.60)
        self.declare_parameter('minimum_range', 0.15)
        self.declare_parameter('maximum_range', 3.0)

        self.declare_parameter('front_half_angle_deg', 25.0)
        self.declare_parameter('side_max_angle_deg', 75.0)

        self.declare_parameter('obstacle_distance', 0.80)
        self.declare_parameter('sample_step', 2)

        self.z_min = float(self.get_parameter('z_min').value)
        self.z_max = float(self.get_parameter('z_max').value)

        self.minimum_range = float(
            self.get_parameter('minimum_range').value
        )

        self.maximum_range = float(
            self.get_parameter('maximum_range').value
        )

        self.front_half_angle = math.radians(
            float(self.get_parameter('front_half_angle_deg').value)
        )

        self.side_max_angle = math.radians(
            float(self.get_parameter('side_max_angle_deg').value)
        )

        self.obstacle_distance = float(
            self.get_parameter('obstacle_distance').value
        )

        self.sample_step = max(
            1,
            int(self.get_parameter('sample_step').value)
        )

        self.front_distance = math.inf
        self.left_distance = math.inf
        self.right_distance = math.inf

        self.front_points = 0
        self.left_points = 0
        self.right_points = 0

        self.cloud_received = False

        self.subscription = self.create_subscription(
            PointCloud2,
            '/livox/lidar/points',
            self.cloud_callback,
            qos_profile_sensor_data
        )

        self.obstacle_publisher = self.create_publisher(
            Bool,
            '/obstacle_ahead',
            10
        )

        self.front_publisher = self.create_publisher(
            Float32,
            '/obstacle_distance/front',
            10
        )

        self.left_publisher = self.create_publisher(
            Float32,
            '/obstacle_distance/left',
            10
        )

        self.right_publisher = self.create_publisher(
            Float32,
            '/obstacle_distance/right',
            10
        )

        self.status_timer = self.create_timer(
            0.5,
            self.publish_status
        )

        self.get_logger().info(
            'Detector iniciado. Aguardando /livox/lidar/points...'
        )

        self.get_logger().info(
            f'Obstáculo à frente será considerado abaixo de '
            f'{self.obstacle_distance:.2f} m.'
        )

    @staticmethod
    def robust_nearest_distance(distances: List[float]) -> float:
        if not distances:
            return math.inf

        quantity = min(5, len(distances))
        nearest = heapq.nsmallest(quantity, distances)

        return sum(nearest) / len(nearest)

    @staticmethod
    def publish_distance(publisher, distance: float) -> None:
        message = Float32()

        if math.isfinite(distance):
            message.data = float(distance)
        else:
            message.data = -1.0

        publisher.publish(message)

    @staticmethod
    def format_distance(distance: float) -> str:
        if not math.isfinite(distance):
            return 'livre'

        return f'{distance:.2f} m'

    def cloud_callback(self, message: PointCloud2) -> None:
        front_distances: List[float] = []
        left_distances: List[float] = []
        right_distances: List[float] = []

        points = point_cloud2.read_points(
            message,
            field_names=('x', 'y', 'z'),
            skip_nans=True
        )

        for index, point in enumerate(points):
            if index % self.sample_step != 0:
                continue

            x = float(point[0])
            y = float(point[1])
            z = float(point[2])

            if z < self.z_min or z > self.z_max:
                continue

            distance = math.hypot(x, y)

            if distance < self.minimum_range:
                continue

            if distance > self.maximum_range:
                continue

            angle = math.atan2(y, x)

            if abs(angle) <= self.front_half_angle:
                front_distances.append(distance)

            elif self.front_half_angle < angle <= self.side_max_angle:
                left_distances.append(distance)

            elif -self.side_max_angle <= angle < -self.front_half_angle:
                right_distances.append(distance)

        self.front_distance = self.robust_nearest_distance(
            front_distances
        )

        self.left_distance = self.robust_nearest_distance(
            left_distances
        )

        self.right_distance = self.robust_nearest_distance(
            right_distances
        )

        self.front_points = len(front_distances)
        self.left_points = len(left_distances)
        self.right_points = len(right_distances)

        self.cloud_received = True

    def publish_status(self) -> None:
        if not self.cloud_received:
            self.get_logger().info(
                'Aguardando a primeira nuvem de pontos...'
            )
            return

        obstacle_ahead = (
            math.isfinite(self.front_distance)
            and self.front_distance < self.obstacle_distance
        )

        obstacle_message = Bool()
        obstacle_message.data = obstacle_ahead
        self.obstacle_publisher.publish(obstacle_message)

        self.publish_distance(
            self.front_publisher,
            self.front_distance
        )

        self.publish_distance(
            self.left_publisher,
            self.left_distance
        )

        self.publish_distance(
            self.right_publisher,
            self.right_distance
        )

        status = 'OBSTÁCULO À FRENTE' if obstacle_ahead else 'CAMINHO LIVRE'

        self.get_logger().info(
            f'Frente={self.format_distance(self.front_distance)} '
            f'({self.front_points} pontos) | '
            f'Esquerda={self.format_distance(self.left_distance)} '
            f'({self.left_points} pontos) | '
            f'Direita={self.format_distance(self.right_distance)} '
            f'({self.right_points} pontos) | '
            f'{status}'
        )


def main(args=None) -> None:
    rclpy.init(args=args)

    node = ObstacleDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
