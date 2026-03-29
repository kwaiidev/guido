"""Shared helpers for Guido's autonomous navigation nodes."""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
from typing import Iterable, Iterator

from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def distance_xy(x0: float, y0: float, x1: float, y1: float) -> float:
    return math.hypot(x1 - x0, y1 - y0)


def quaternion_from_yaw(yaw: float) -> Quaternion:
    quaternion = Quaternion()
    quaternion.z = math.sin(yaw / 2.0)
    quaternion.w = math.cos(yaw / 2.0)
    return quaternion


def yaw_from_quaternion(quaternion: Quaternion) -> float:
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def pose2d_from_pose(pose: Pose) -> Pose2D:
    return Pose2D(
        x=float(pose.position.x),
        y=float(pose.position.y),
        yaw=yaw_from_quaternion(pose.orientation),
    )


def make_pose_stamped(*, frame_id: str, stamp, pose: Pose2D) -> PoseStamped:
    message = PoseStamped()
    message.header.frame_id = frame_id
    message.header.stamp = stamp
    message.pose.position.x = pose.x
    message.pose.position.y = pose.y
    message.pose.orientation = quaternion_from_yaw(pose.yaw)
    return message


def world_to_grid(
    x: float,
    y: float,
    *,
    origin_x: float,
    origin_y: float,
    resolution: float,
    width: int,
    height: int,
) -> tuple[int, int] | None:
    gx = int((x - origin_x) / resolution)
    gy = int((y - origin_y) / resolution)
    if gx < 0 or gy < 0 or gx >= width or gy >= height:
        return None
    return gx, gy


def grid_to_world(
    gx: int,
    gy: int,
    *,
    origin_x: float,
    origin_y: float,
    resolution: float,
) -> tuple[float, float]:
    return (
        origin_x + (gx + 0.5) * resolution,
        origin_y + (gy + 0.5) * resolution,
    )


def grid_offset(gx: int, gy: int, width: int) -> int:
    return gy * width + gx


def bresenham(x0: int, y0: int, x1: int, y1: int) -> Iterator[tuple[int, int]]:
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx + dy

    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        doubled_error = 2 * error
        if doubled_error >= dy:
            error += dy
            x0 += sx
        if doubled_error <= dx:
            error += dx
            y0 += sy


def make_point_cloud2(
    points: Iterable[tuple[float, float]],
    *,
    frame_id: str,
    stamp,
) -> PointCloud2:
    points_list = list(points)
    buffer = bytearray()
    for x_coord, y_coord in points_list:
        buffer.extend(struct.pack('fff', float(x_coord), float(y_coord), 0.0))

    message = PointCloud2()
    message.header = Header(frame_id=frame_id, stamp=stamp)
    message.height = 1
    message.width = len(points_list)
    message.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = message.point_step * len(points_list)
    message.is_dense = True
    message.data = bytes(buffer)
    return message


def iter_point_cloud_xy(message: PointCloud2) -> Iterator[tuple[float, float]]:
    step = int(message.point_step)
    for offset in range(0, len(message.data), step):
        x_coord, y_coord, _ = struct.unpack_from('fff', message.data, offset)
        yield x_coord, y_coord


def copy_grid_data(message: OccupancyGrid) -> list[int]:
    return [int(value) for value in message.data]
