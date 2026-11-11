#!/usr/bin/env python3

import math

from geometry_msgs.msg import Point
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt32
from visualization_msgs.msg import Marker, MarkerArray


class SeabedMapPublisher(Node):
    MAP_FRAME = "world"
    RESOLUTION = 0.1
    WIDTH = 120
    HEIGHT = 120
    ORIGIN_X = -6.0
    ORIGIN_Y = -6.0
    FLOOR_Z = -2.9
    SEDIMENTS = (
        (1.6, -1.0, 0.09),
        (1.85, -0.85, 0.12),
        (2.1, -1.15, 0.08),
        (2.35, -0.9, 0.10),
        (2.7, 0.9, 0.11),
        (2.95, 1.15, 0.08),
        (3.2, 0.85, 0.13),
        (3.5, 1.0, 0.09),
    )

    def __init__(self) -> None:
        super().__init__("seabed_map_publisher")
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_publisher = self.create_publisher(OccupancyGrid, "/seabed/map", qos)
        self.markers_publisher = self.create_publisher(
            MarkerArray, "/seabed/targets", qos
        )
        self.removed_targets: set[int] = set()
        self.removed_subscriber = self.create_subscription(
            UInt32, "/rov/collection/target_removed", self.remove_target, 10
        )
        self.map_message = self.create_map()
        self.markers_message = self.create_markers()
        self.timer = self.create_timer(1.0, self.publish_map)
        self.get_logger().info(
            "Publishing fixed seabed map and 8 sediment targets in the world frame"
        )

    def create_map(self) -> OccupancyGrid:
        message = OccupancyGrid()
        message.header.frame_id = self.MAP_FRAME
        message.info.resolution = self.RESOLUTION
        message.info.width = self.WIDTH
        message.info.height = self.HEIGHT
        message.info.origin.position.x = self.ORIGIN_X
        message.info.origin.position.y = self.ORIGIN_Y
        message.info.origin.position.z = self.FLOOR_Z
        message.info.origin.orientation.w = 1.0
        cells = [0] * (self.WIDTH * self.HEIGHT)

        for target_id, (sediment_x, sediment_y, radius) in enumerate(self.SEDIMENTS, start=1):
            if target_id in self.removed_targets:
                continue
            min_x = max(0, math.floor((sediment_x - radius - self.ORIGIN_X) / self.RESOLUTION))
            max_x = min(
                self.WIDTH - 1,
                math.ceil((sediment_x + radius - self.ORIGIN_X) / self.RESOLUTION),
            )
            min_y = max(0, math.floor((sediment_y - radius - self.ORIGIN_Y) / self.RESOLUTION))
            max_y = min(
                self.HEIGHT - 1,
                math.ceil((sediment_y + radius - self.ORIGIN_Y) / self.RESOLUTION),
            )
            for grid_y in range(min_y, max_y + 1):
                world_y = self.ORIGIN_Y + (grid_y + 0.5) * self.RESOLUTION
                for grid_x in range(min_x, max_x + 1):
                    world_x = self.ORIGIN_X + (grid_x + 0.5) * self.RESOLUTION
                    if math.hypot(world_x - sediment_x, world_y - sediment_y) <= radius:
                        cells[grid_y * self.WIDTH + grid_x] = 100

        message.data = cells
        return message

    def create_markers(self) -> MarkerArray:
        marker_array = MarkerArray()
        floor = Marker()
        floor.header.frame_id = self.MAP_FRAME
        floor.ns = "seabed"
        floor.id = 0
        floor.type = Marker.CUBE
        floor.action = Marker.ADD
        floor.pose.position.z = self.FLOOR_Z - 0.015
        floor.pose.orientation.w = 1.0
        floor.scale.x = self.WIDTH * self.RESOLUTION
        floor.scale.y = self.HEIGHT * self.RESOLUTION
        floor.scale.z = 0.02
        floor.color.r = 0.34
        floor.color.g = 0.27
        floor.color.b = 0.19
        floor.color.a = 0.45
        marker_array.markers.append(floor)

        occupied = Marker()
        occupied.header.frame_id = self.MAP_FRAME
        occupied.ns = "occupied_map_cells"
        occupied.id = 1
        occupied.type = Marker.CUBE_LIST
        occupied.action = Marker.ADD
        occupied.pose.position.z = self.FLOOR_Z + 0.005
        occupied.pose.orientation.w = 1.0
        occupied.scale.x = self.RESOLUTION
        occupied.scale.y = self.RESOLUTION
        occupied.scale.z = 0.01
        occupied.color.r = 0.93
        occupied.color.g = 0.57
        occupied.color.b = 0.16
        occupied.color.a = 1.0
        for cell, value in enumerate(self.map_message.data):
            if value:
                grid_x = cell % self.WIDTH
                grid_y = cell // self.WIDTH
                occupied.points.append(
                    Point(
                        x=self.ORIGIN_X + (grid_x + 0.5) * self.RESOLUTION,
                        y=self.ORIGIN_Y + (grid_y + 0.5) * self.RESOLUTION,
                    )
                )
        marker_array.markers.append(occupied)

        for target_id in self.removed_targets:
            deleted = Marker()
            deleted.header.frame_id = self.MAP_FRAME
            deleted.ns = "sediment_targets"
            deleted.id = target_id + 1
            deleted.action = Marker.DELETE
            marker_array.markers.append(deleted)

        for target_id, (x, y, radius) in enumerate(self.SEDIMENTS, start=1):
            if target_id in self.removed_targets:
                continue
            marker = Marker()
            marker.header.frame_id = self.MAP_FRAME
            marker.ns = "sediment_targets"
            marker.id = target_id + 1
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = self.FLOOR_Z + radius
            marker.pose.orientation.w = 1.0
            marker.scale.x = radius * 2.0
            marker.scale.y = radius * 2.0
            marker.scale.z = radius * 2.0
            marker.color.r = 0.72
            marker.color.g = 0.40
            marker.color.b = 0.13
            marker.color.a = 1.0
            marker.lifetime.sec = 0
            marker_array.markers.append(marker)
        return marker_array

    def remove_target(self, message: UInt32) -> None:
        if message.data < 1 or message.data > len(self.SEDIMENTS):
            self.get_logger().error(f"Ignoring invalid removed target ID {message.data}")
            return
        if message.data in self.removed_targets:
            return
        self.removed_targets.add(message.data)
        self.map_message = self.create_map()
        self.markers_message = self.create_markers()
        self.get_logger().info(f"Removed sediment target {message.data} from map and markers")

    def publish_map(self) -> None:
        stamp = self.get_clock().now().to_msg()
        self.map_message.header.stamp = stamp
        for marker in self.markers_message.markers:
            marker.header.stamp = stamp
        self.map_publisher.publish(self.map_message)
        self.markers_publisher.publish(self.markers_message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SeabedMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
