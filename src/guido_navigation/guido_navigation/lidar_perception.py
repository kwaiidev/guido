"""LiDAR perception node for Guido autonomous navigation."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

from .common import bresenham, make_point_cloud2, world_to_grid
from .scan_filter import (
    KeepoutBox,
    clear_keepout_cells,
    cluster_sequential_points,
    mask_keepout_ranges,
    scan_ranges_to_points,
)


@dataclass
class Track:
    x: float
    y: float
    vx: float
    vy: float
    last_stamp_sec: float
    seen_count: int


class LidarPerception(Node):
    def __init__(self):
        super().__init__('guido_lidar_perception')

        self.declare_parameter('scan_topic', '/ldlidar_node/scan')
        self.declare_parameter('point_cloud_topic', '/navigation/points')
        self.declare_parameter('costmap_topic', '/navigation/local_costmap')
        self.declare_parameter('obstacle_markers_topic', '/navigation/obstacle_markers')
        self.declare_parameter('tracking_markers_topic', '/navigation/tracking_markers')
        self.declare_parameter('filtered_scan_topic', '/navigation/filtered_scan')
        self.declare_parameter('self_filter_marker_topic', '/navigation/self_filter_marker')
        self.declare_parameter('grid_width_m', 12.0)
        self.declare_parameter('grid_height_m', 12.0)
        self.declare_parameter('resolution', 0.10)
        self.declare_parameter('inflation_radius_m', 0.45)
        self.declare_parameter('cluster_tolerance_m', 0.35)
        self.declare_parameter('min_cluster_points', 3)
        self.declare_parameter('dynamic_match_distance_m', 0.90)
        self.declare_parameter('dynamic_speed_threshold_mps', 0.20)
        self.declare_parameter('max_range_m', 15.0)
        self.declare_parameter('min_range_m', 0.10)
        self.declare_parameter('self_filter_enabled', True)
        self.declare_parameter('publish_filtered_scan', True)
        self.declare_parameter('keepout_x_min_m', -0.50)
        self.declare_parameter('keepout_x_max_m', 0.50)
        self.declare_parameter('keepout_y_min_m', -0.45)
        self.declare_parameter('keepout_y_max_m', 0.45)

        scan_topic = str(self.get_parameter('scan_topic').value)
        point_cloud_topic = str(self.get_parameter('point_cloud_topic').value)
        costmap_topic = str(self.get_parameter('costmap_topic').value)
        obstacle_markers_topic = str(self.get_parameter('obstacle_markers_topic').value)
        tracking_markers_topic = str(self.get_parameter('tracking_markers_topic').value)
        filtered_scan_topic = str(self.get_parameter('filtered_scan_topic').value)
        self_filter_marker_topic = str(self.get_parameter('self_filter_marker_topic').value)

        self._grid_width_m = float(self.get_parameter('grid_width_m').value)
        self._grid_height_m = float(self.get_parameter('grid_height_m').value)
        self._resolution = float(self.get_parameter('resolution').value)
        self._inflation_radius_m = float(self.get_parameter('inflation_radius_m').value)
        self._cluster_tolerance_m = float(self.get_parameter('cluster_tolerance_m').value)
        self._min_cluster_points = int(self.get_parameter('min_cluster_points').value)
        self._dynamic_match_distance_m = float(
            self.get_parameter('dynamic_match_distance_m').value
        )
        self._dynamic_speed_threshold_mps = float(
            self.get_parameter('dynamic_speed_threshold_mps').value
        )
        self._max_range_m = float(self.get_parameter('max_range_m').value)
        self._min_range_m = float(self.get_parameter('min_range_m').value)
        self._self_filter_enabled = bool(self.get_parameter('self_filter_enabled').value)
        self._publish_filtered_scan = bool(self.get_parameter('publish_filtered_scan').value)
        self._keepout = KeepoutBox(
            x_min=float(self.get_parameter('keepout_x_min_m').value),
            x_max=float(self.get_parameter('keepout_x_max_m').value),
            y_min=float(self.get_parameter('keepout_y_min_m').value),
            y_max=float(self.get_parameter('keepout_y_max_m').value),
        )

        self._grid_width_cells = int(round(self._grid_width_m / self._resolution))
        self._grid_height_cells = int(round(self._grid_height_m / self._resolution))
        self._inflation_radius_cells = max(
            1, int(math.ceil(self._inflation_radius_m / self._resolution))
        )
        self._origin_x = -self._grid_width_m / 2.0
        self._origin_y = -self._grid_height_m / 2.0

        self._filtered_scan_pub = self.create_publisher(LaserScan, filtered_scan_topic, 10)
        self._point_cloud_pub = self.create_publisher(PointCloud2, point_cloud_topic, 10)
        self._costmap_pub = self.create_publisher(OccupancyGrid, costmap_topic, 10)
        self._obstacle_markers_pub = self.create_publisher(MarkerArray, obstacle_markers_topic, 10)
        self._tracking_markers_pub = self.create_publisher(MarkerArray, tracking_markers_topic, 10)
        self._self_filter_marker_pub = self.create_publisher(Marker, self_filter_marker_topic, 10)

        self._tracks: dict[int, Track] = {}
        self._next_track_id = 1

        self.create_subscription(LaserScan, scan_topic, self._scan_cb, 10)

        self.get_logger().info(
            'LiDAR perception online: '
            f'scan={scan_topic} filtered={filtered_scan_topic} '
            f'costmap={costmap_topic} cloud={point_cloud_topic} '
            f'self_filter={self._self_filter_enabled}'
        )

    def _scan_cb(self, message: LaserScan):
        stamp = message.header.stamp
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) / 1e9
        filtered_ranges = mask_keepout_ranges(
            message.ranges,
            angle_min=float(message.angle_min),
            angle_increment=float(message.angle_increment),
            keepout=self._keepout,
            enabled=self._self_filter_enabled,
        )
        filtered_scan = self._copy_scan_with_ranges(message, filtered_ranges)
        if self._publish_filtered_scan:
            self._filtered_scan_pub.publish(filtered_scan)
        self._self_filter_marker_pub.publish(
            self._build_keepout_marker(frame_id=message.header.frame_id, stamp=stamp)
        )

        points = self._scan_to_points(filtered_scan)
        cloud = make_point_cloud2(points, frame_id=message.header.frame_id, stamp=stamp)
        self._point_cloud_pub.publish(cloud)

        costmap = self._build_costmap(points, message.header.frame_id, stamp)
        self._costmap_pub.publish(costmap)

        clusters = self._cluster_points(points)
        obstacle_markers, tracking_markers = self._update_tracks(
            clusters,
            frame_id=message.header.frame_id,
            stamp=stamp,
            stamp_sec=stamp_sec,
        )
        self._obstacle_markers_pub.publish(obstacle_markers)
        self._tracking_markers_pub.publish(tracking_markers)

    def _scan_to_points(self, message: LaserScan) -> list[tuple[float, float]]:
        range_min = max(self._min_range_m, float(message.range_min))
        range_max = min(self._max_range_m, float(message.range_max))
        return scan_ranges_to_points(
            message.ranges,
            angle_min=float(message.angle_min),
            angle_increment=float(message.angle_increment),
            min_range_m=range_min,
            max_range_m=range_max,
        )

    def _build_costmap(self, points: list[tuple[float, float]], frame_id: str, stamp) -> OccupancyGrid:
        grid = np.full(
            (self._grid_height_cells, self._grid_width_cells),
            fill_value=-1,
            dtype=np.int8,
        )

        center = world_to_grid(
            0.0,
            0.0,
            origin_x=self._origin_x,
            origin_y=self._origin_y,
            resolution=self._resolution,
            width=self._grid_width_cells,
            height=self._grid_height_cells,
        )
        if center is None:
            raise RuntimeError('Local costmap center is outside the configured grid.')
        center_x, center_y = center

        obstacle_cells: list[tuple[int, int]] = []
        for x_coord, y_coord in points:
            cell = world_to_grid(
                x_coord,
                y_coord,
                origin_x=self._origin_x,
                origin_y=self._origin_y,
                resolution=self._resolution,
                width=self._grid_width_cells,
                height=self._grid_height_cells,
            )
            if cell is None:
                continue

            gx, gy = cell
            for free_x, free_y in bresenham(center_x, center_y, gx, gy):
                if free_x == gx and free_y == gy:
                    break
                grid[free_y, free_x] = 0

            grid[gy, gx] = 100
            obstacle_cells.append((gx, gy))

        for gx, gy in obstacle_cells:
            self._inflate_cell(grid, gx, gy)
        if self._self_filter_enabled:
            clear_keepout_cells(
                grid,
                keepout=self._keepout,
                origin_x=self._origin_x,
                origin_y=self._origin_y,
                resolution=self._resolution,
            )

        message = OccupancyGrid()
        message.header.frame_id = frame_id
        message.header.stamp = stamp
        message.info.resolution = self._resolution
        message.info.width = self._grid_width_cells
        message.info.height = self._grid_height_cells
        message.info.origin.position.x = self._origin_x
        message.info.origin.position.y = self._origin_y
        message.info.origin.orientation.w = 1.0
        message.data = grid.flatten().tolist()
        return message

    def _inflate_cell(self, grid: np.ndarray[Any, np.dtype[np.int8]], gx: int, gy: int):
        for dy in range(-self._inflation_radius_cells, self._inflation_radius_cells + 1):
            for dx in range(-self._inflation_radius_cells, self._inflation_radius_cells + 1):
                nx = gx + dx
                ny = gy + dy
                if nx < 0 or ny < 0 or nx >= self._grid_width_cells or ny >= self._grid_height_cells:
                    continue
                if math.hypot(dx, dy) > self._inflation_radius_cells:
                    continue
                grid[ny, nx] = max(int(grid[ny, nx]), 70)

    def _cluster_points(self, points: list[tuple[float, float]]) -> list[dict[str, float]]:
        return cluster_sequential_points(
            points,
            tolerance_m=self._cluster_tolerance_m,
            min_cluster_points=self._min_cluster_points,
        )

    @staticmethod
    def _copy_scan_with_ranges(message: LaserScan, ranges: list[float]) -> LaserScan:
        filtered_scan = LaserScan()
        filtered_scan.header = message.header
        filtered_scan.angle_min = message.angle_min
        filtered_scan.angle_max = message.angle_max
        filtered_scan.angle_increment = message.angle_increment
        filtered_scan.time_increment = message.time_increment
        filtered_scan.scan_time = message.scan_time
        filtered_scan.range_min = message.range_min
        filtered_scan.range_max = message.range_max
        filtered_scan.ranges = ranges
        filtered_scan.intensities = list(message.intensities)
        return filtered_scan

    def _build_keepout_marker(self, *, frame_id: str, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = 'self_filter'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = self._keepout.center_x
        marker.pose.position.y = self._keepout.center_y
        marker.pose.position.z = 0.02
        marker.pose.orientation.w = 1.0
        marker.scale.x = self._keepout.width
        marker.scale.y = self._keepout.height
        marker.scale.z = 0.04
        marker.color.a = 0.18
        marker.color.r = 1.0
        marker.color.g = 0.05
        marker.color.b = 0.05
        return marker

    def _update_tracks(self, clusters, *, frame_id: str, stamp, stamp_sec: float) -> tuple[MarkerArray, MarkerArray]:
        matched_ids: set[int] = set()
        new_tracks: dict[int, Track] = {}

        for cluster in clusters:
            best_track_id = None
            best_distance = self._dynamic_match_distance_m
            for track_id, track in self._tracks.items():
                if track_id in matched_ids:
                    continue
                distance = math.hypot(cluster['x'] - track.x, cluster['y'] - track.y)
                if distance < best_distance:
                    best_distance = distance
                    best_track_id = track_id

            if best_track_id is None:
                track_id = self._next_track_id
                self._next_track_id += 1
                new_tracks[track_id] = Track(
                    x=cluster['x'],
                    y=cluster['y'],
                    vx=0.0,
                    vy=0.0,
                    last_stamp_sec=stamp_sec,
                    seen_count=1,
                )
                matched_ids.add(track_id)
                cluster['track_id'] = track_id
                cluster['vx'] = 0.0
                cluster['vy'] = 0.0
                continue

            previous = self._tracks[best_track_id]
            dt = max(1e-3, stamp_sec - previous.last_stamp_sec)
            vx = (cluster['x'] - previous.x) / dt
            vy = (cluster['y'] - previous.y) / dt
            new_tracks[best_track_id] = Track(
                x=cluster['x'],
                y=cluster['y'],
                vx=vx,
                vy=vy,
                last_stamp_sec=stamp_sec,
                seen_count=previous.seen_count + 1,
            )
            matched_ids.add(best_track_id)
            cluster['track_id'] = best_track_id
            cluster['vx'] = vx
            cluster['vy'] = vy

        for track_id, track in self._tracks.items():
            if track_id in matched_ids:
                continue
            if (stamp_sec - track.last_stamp_sec) <= 0.5:
                new_tracks[track_id] = track

        self._tracks = new_tracks

        return (
            self._build_cluster_markers(clusters, frame_id=frame_id, stamp=stamp),
            self._build_tracking_markers(clusters, frame_id=frame_id, stamp=stamp),
        )

    def _build_cluster_markers(self, clusters, *, frame_id: str, stamp) -> MarkerArray:
        markers = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)

        for index, cluster in enumerate(clusters):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = 'obstacles'
            marker.id = index
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(cluster['x'])
            marker.pose.position.y = float(cluster['y'])
            diameter = max(0.20, 2.0 * float(cluster['radius']))
            marker.scale.x = diameter
            marker.scale.y = diameter
            marker.scale.z = 0.25
            marker.color.a = 0.85
            marker.color.r = 1.0
            marker.color.g = 0.35
            marker.color.b = 0.0
            markers.markers.append(marker)

        return markers

    def _build_tracking_markers(self, clusters, *, frame_id: str, stamp) -> MarkerArray:
        markers = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        markers.markers.append(delete_marker)

        for index, cluster in enumerate(clusters):
            velocity = math.hypot(float(cluster['vx']), float(cluster['vy']))
            is_dynamic = velocity >= self._dynamic_speed_threshold_mps

            text_marker = Marker()
            text_marker.header.frame_id = frame_id
            text_marker.header.stamp = stamp
            text_marker.ns = 'tracks'
            text_marker.id = index
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = float(cluster['x'])
            text_marker.pose.position.y = float(cluster['y'])
            text_marker.pose.position.z = 0.40
            text_marker.scale.z = 0.18
            text_marker.color.a = 1.0
            text_marker.color.r = 0.0 if is_dynamic else 0.9
            text_marker.color.g = 1.0
            text_marker.color.b = 0.1
            text_marker.text = (
                f"id={int(cluster['track_id'])} "
                f"v={velocity:.2f}m/s "
                f"{'dynamic' if is_dynamic else 'static'}"
            )
            markers.markers.append(text_marker)

            arrow_marker = Marker()
            arrow_marker.header.frame_id = frame_id
            arrow_marker.header.stamp = stamp
            arrow_marker.ns = 'track_vectors'
            arrow_marker.id = 1000 + index
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.scale.x = 0.05
            arrow_marker.scale.y = 0.10
            arrow_marker.scale.z = 0.10
            arrow_marker.color.a = 0.9
            arrow_marker.color.r = 1.0 if is_dynamic else 0.2
            arrow_marker.color.g = 0.2 if is_dynamic else 0.9
            arrow_marker.color.b = 0.0
            start = Point(x=float(cluster['x']), y=float(cluster['y']), z=0.10)
            end = Point(
                x=float(cluster['x'] + cluster['vx']),
                y=float(cluster['y'] + cluster['vy']),
                z=0.10,
            )
            arrow_marker.points = [start, end]
            markers.markers.append(arrow_marker)

        return markers


def main(args=None):
    rclpy.init(args=args)
    node = LidarPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
