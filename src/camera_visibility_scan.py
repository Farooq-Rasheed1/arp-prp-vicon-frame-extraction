#!/usr/bin/env python3
"""Offline camera observability scan for the ARP/PRP real dataset.

This script intentionally avoids ROS runtime dependencies.  It reads the
SQLite rosbag2 files directly, extracts Vicon-derived ground-truth odometry
from the processed fusion bag, aligns those poses to raw OAK-D frame
timestamps, and estimates where the other robot should appear in each image.
"""

from __future__ import annotations

import argparse
import csv
import io
import math
import sqlite3
import struct
from bisect import bisect_left
from dataclasses import dataclass
from pathlib import Path

from PIL import Image


NS_PER_S = 1_000_000_000
PNG_SIGNATURE = b'\x89PNG\r\n\x1a\n'


@dataclass(frozen=True)
class CameraModel:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    @property
    def hfov_deg(self) -> float:
        return math.degrees(2.0 * math.atan(self.width / (2.0 * self.fx)))

    def scaled_to(self, width: int, height: int) -> 'CameraModel':
        sx = width / float(self.width)
        sy = height / float(self.height)
        return CameraModel(width, height, self.fx * sx, self.fy * sy, self.cx * sx, self.cy * sy)


@dataclass(frozen=True)
class CameraExtrinsics2D:
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_rad: float = 0.0


@dataclass(frozen=True)
class Pose2D:
    stamp_ns: int
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Projection:
    stamp_ns: int
    dt_ms: float
    rel_x_m: float
    rel_y_m: float
    range_m: float
    bearing_deg: float
    u_px: float
    bbox_w_px: float
    bbox_h_px: float
    in_front: bool
    in_hfov: bool
    in_range: bool

    @property
    def visible(self) -> bool:
        return self.in_front and self.in_hfov and self.in_range


class CdrReader:
    """Minimal little-endian CDR reader for the ROS messages used here."""

    def __init__(self, data: bytes) -> None:
        self.data = data
        self.offset = 4
        self.base = 4

    def align(self, size: int) -> None:
        remainder = (self.offset - self.base) % size
        if remainder:
            self.offset += size - remainder

    def int32(self) -> int:
        self.align(4)
        value = struct.unpack_from('<i', self.data, self.offset)[0]
        self.offset += 4
        return value

    def uint32(self) -> int:
        self.align(4)
        value = struct.unpack_from('<I', self.data, self.offset)[0]
        self.offset += 4
        return value

    def float64(self) -> float:
        self.align(8)
        value = struct.unpack_from('<d', self.data, self.offset)[0]
        self.offset += 8
        return value

    def string(self) -> str:
        size = self.uint32()
        raw = self.data[self.offset:self.offset + size]
        self.offset += size
        return raw.rstrip(b'\0').decode('utf-8', errors='replace')


def yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def parse_odometry(data: bytes) -> Pose2D:
    reader = CdrReader(data)
    sec = reader.int32()
    nanosec = reader.uint32()
    reader.string()  # header.frame_id
    reader.string()  # child_frame_id
    x = reader.float64()
    y = reader.float64()
    reader.float64()  # z
    qx = reader.float64()
    qy = reader.float64()
    qz = reader.float64()
    qw = reader.float64()
    return Pose2D(sec * NS_PER_S + nanosec, x, y, yaw_from_quaternion(qx, qy, qz, qw))


def topic_id(db_path: Path, topic_name: str) -> int:
    with sqlite3.connect(db_path) as con:
        row = con.execute('select id from topics where name = ?', (topic_name,)).fetchone()
    if row is None:
        raise SystemExit(f'topic not found in {db_path}: {topic_name}')
    return int(row[0])


def read_odometry_topic(db_path: Path, topic_name: str) -> list[Pose2D]:
    tid = topic_id(db_path, topic_name)
    poses: list[Pose2D] = []
    with sqlite3.connect(db_path) as con:
        rows = con.execute(
            'select data from messages where topic_id = ? order by timestamp',
            (tid,),
        )
        for (data,) in rows:
            poses.append(parse_odometry(bytes(data)))
    return poses


def read_message_timestamps(db_path: Path, topic_name: str) -> list[int]:
    tid = topic_id(db_path, topic_name)
    with sqlite3.connect(db_path) as con:
        rows = con.execute(
            'select timestamp from messages where topic_id = ? order by timestamp',
            (tid,),
        )
        return [int(row[0]) for row in rows]


def first_image_size(db_path: Path, topic_name: str) -> tuple[int, int]:
    tid = topic_id(db_path, topic_name)
    with sqlite3.connect(db_path) as con:
        row = con.execute(
            'select data from messages where topic_id = ? order by timestamp limit 1',
            (tid,),
        ).fetchone()
    if row is None:
        raise SystemExit(f'no images found in {db_path}: {topic_name}')
    data = bytes(row[0])
    start = data.find(PNG_SIGNATURE)
    if start < 0:
        raise SystemExit(f'first image did not contain PNG payload: {topic_name}')
    with Image.open(io.BytesIO(data[start:])) as image:
        return image.size


def nearest_pose(poses: list[Pose2D], stamps: list[int], stamp_ns: int) -> tuple[Pose2D, float]:
    index = bisect_left(stamps, stamp_ns)
    candidates = []
    if index < len(poses):
        candidates.append(poses[index])
    if index > 0:
        candidates.append(poses[index - 1])
    if not candidates:
        raise ValueError('no poses available')
    pose = min(candidates, key=lambda p: abs(p.stamp_ns - stamp_ns))
    return pose, (pose.stamp_ns - stamp_ns) / 1e6


def project_target(
    stamp_ns: int,
    self_pose: Pose2D,
    target_pose: Pose2D,
    dt_ms: float,
    camera: CameraModel,
    extrinsics: CameraExtrinsics2D,
    robot_width_m: float,
    robot_height_m: float,
    max_range_m: float,
) -> Projection:
    base_c = math.cos(self_pose.yaw)
    base_s = math.sin(self_pose.yaw)
    camera_x = self_pose.x + base_c * extrinsics.x_m - base_s * extrinsics.y_m
    camera_y = self_pose.y + base_s * extrinsics.x_m + base_c * extrinsics.y_m
    camera_yaw = self_pose.yaw + extrinsics.yaw_rad

    dx = target_pose.x - camera_x
    dy = target_pose.y - camera_y
    c = math.cos(camera_yaw)
    s = math.sin(camera_yaw)

    # ROS base frame convention: +x forward, +y left.  Pinhole image x is
    # positive to the right, so the lateral term is negated.
    rel_x = c * dx + s * dy
    rel_y = -s * dx + c * dy
    range_m = math.hypot(rel_x, rel_y)
    bearing = math.atan2(rel_y, rel_x)
    if rel_x > 0.05:
        u_px = camera.cx - camera.fx * (rel_y / rel_x)
        bbox_w = camera.fx * robot_width_m / rel_x
        bbox_h = camera.fy * robot_height_m / rel_x
    else:
        u_px = float('nan')
        bbox_w = float('nan')
        bbox_h = float('nan')
    return Projection(
        stamp_ns=stamp_ns,
        dt_ms=dt_ms,
        rel_x_m=rel_x,
        rel_y_m=rel_y,
        range_m=range_m,
        bearing_deg=math.degrees(bearing),
        u_px=u_px,
        bbox_w_px=bbox_w,
        bbox_h_px=bbox_h,
        in_front=rel_x > 0.05,
        in_hfov=0.0 <= u_px < camera.width if math.isfinite(u_px) else False,
        in_range=range_m <= max_range_m,
    )


def scan_camera(
    frame_stamps: list[int],
    self_poses: list[Pose2D],
    target_poses: list[Pose2D],
    camera: CameraModel,
    extrinsics: CameraExtrinsics2D,
    robot_width_m: float,
    robot_height_m: float,
    max_range_m: float,
    max_sync_ms: float,
) -> list[Projection]:
    self_stamps = [pose.stamp_ns for pose in self_poses]
    target_stamps = [pose.stamp_ns for pose in target_poses]
    projections: list[Projection] = []
    for stamp in frame_stamps:
        self_pose, self_dt = nearest_pose(self_poses, self_stamps, stamp)
        target_pose, target_dt = nearest_pose(target_poses, target_stamps, stamp)
        dt_ms = max(abs(self_dt), abs(target_dt))
        if dt_ms > max_sync_ms:
            continue
        projections.append(
            project_target(
                stamp,
                self_pose,
                target_pose,
                dt_ms,
                camera,
                extrinsics,
                robot_width_m,
                robot_height_m,
                max_range_m,
            )
        )
    return projections


def visible_segments(projections: list[Projection], max_gap_s: float = 0.25) -> list[list[Projection]]:
    segments: list[list[Projection]] = []
    current: list[Projection] = []
    max_gap_ns = int(max_gap_s * NS_PER_S)
    for projection in projections:
        if not projection.visible:
            if current:
                segments.append(current)
                current = []
            continue
        if current and projection.stamp_ns - current[-1].stamp_ns > max_gap_ns:
            segments.append(current)
            current = []
        current.append(projection)
    if current:
        segments.append(current)
    return segments


def write_csv(path: Path, projections: list[Projection]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow([
            'stamp_ns',
            'time_s_from_start',
            'visible',
            'rel_x_m',
            'rel_y_m',
            'range_m',
            'bearing_deg',
            'u_px',
            'bbox_w_px',
            'bbox_h_px',
            'sync_dt_ms',
        ])
        start = projections[0].stamp_ns if projections else 0
        for p in projections:
            writer.writerow([
                p.stamp_ns,
                f'{(p.stamp_ns - start) / 1e9:.3f}',
                int(p.visible),
                f'{p.rel_x_m:.4f}',
                f'{p.rel_y_m:.4f}',
                f'{p.range_m:.4f}',
                f'{p.bearing_deg:.3f}',
                f'{p.u_px:.2f}',
                f'{p.bbox_w_px:.2f}',
                f'{p.bbox_h_px:.2f}',
                f'{p.dt_ms:.3f}',
            ])


def summarize(name: str, projections: list[Projection]) -> str:
    visible = [p for p in projections if p.visible]
    segments = visible_segments(projections)
    lines = [
        f'## {name}',
        '',
        f'total image frames checked: {len(projections)}',
        f'candidate visible frames: {len(visible)} ({100.0 * len(visible) / max(1, len(projections)):.1f}%)',
        f'candidate visible segments: {len(segments)}',
    ]
    if visible:
        lines.extend([
            f'range while visible: {min(p.range_m for p in visible):.2f} m to {max(p.range_m for p in visible):.2f} m',
            f'bearing while visible: {min(p.bearing_deg for p in visible):.1f} deg to {max(p.bearing_deg for p in visible):.1f} deg',
            f'pixel u while visible: {min(p.u_px for p in visible):.0f} px to {max(p.u_px for p in visible):.0f} px',
            '',
            'Top candidate segments:',
        ])
        ranked = sorted(segments, key=len, reverse=True)[:10]
        start0 = projections[0].stamp_ns
        for idx, segment in enumerate(ranked, 1):
            mid = segment[len(segment) // 2]
            lines.append(
                f'{idx}. t={((segment[0].stamp_ns - start0) / 1e9):.1f}s..'
                f'{((segment[-1].stamp_ns - start0) / 1e9):.1f}s, '
                f'frames={len(segment)}, range~{mid.range_m:.2f}m, '
                f'bearing~{mid.bearing_deg:.1f}deg, u~{mid.u_px:.0f}px, '
                f'box~{mid.bbox_w_px:.0f}x{mid.bbox_h_px:.0f}px'
            )
    return '\n'.join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--fusion-db', required=True)
    parser.add_argument('--tb1-db', required=True)
    parser.add_argument('--tb2-db', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--robot-width-m', type=float, default=0.36)
    parser.add_argument('--robot-height-m', type=float, default=0.35)
    parser.add_argument('--tb1-camera-x-m', type=float, default=0.0)
    parser.add_argument('--tb1-camera-y-m', type=float, default=0.0)
    parser.add_argument('--tb1-camera-yaw-offset-deg', type=float, default=0.0)
    parser.add_argument('--tb2-camera-x-m', type=float, default=0.0)
    parser.add_argument('--tb2-camera-y-m', type=float, default=0.0)
    parser.add_argument('--tb2-camera-yaw-offset-deg', type=float, default=0.0)
    parser.add_argument('--max-range-m', type=float, default=6.0)
    parser.add_argument('--max-sync-ms', type=float, default=20.0)
    args = parser.parse_args()

    fusion_db = Path(args.fusion_db)
    tb1_db = Path(args.tb1_db)
    tb2_db = Path(args.tb2_db)
    output_dir = Path(args.output_dir)

    tb1_camera_info = CameraModel(1280, 720, 1026.157958984375, 1026.157958984375, 649.09716796875, 369.48046875)
    tb2_camera_info = CameraModel(1280, 720, 1032.688232421875, 1032.688232421875, 638.600830078125, 362.9078369140625)
    tb1_image_size = first_image_size(tb1_db, '/tb1/oakd/stereo/image_raw/compressedDepth')
    tb2_image_size = first_image_size(tb2_db, '/tb2/oakd/stereo/image_raw/compressedDepth')
    tb1_camera = tb1_camera_info.scaled_to(*tb1_image_size)
    tb2_camera = tb2_camera_info.scaled_to(*tb2_image_size)
    tb1_extrinsics = CameraExtrinsics2D(
        args.tb1_camera_x_m,
        args.tb1_camera_y_m,
        math.radians(args.tb1_camera_yaw_offset_deg),
    )
    tb2_extrinsics = CameraExtrinsics2D(
        args.tb2_camera_x_m,
        args.tb2_camera_y_m,
        math.radians(args.tb2_camera_yaw_offset_deg),
    )

    tb1_gt = read_odometry_topic(fusion_db, '/robot_a/ground_truth')
    tb2_gt = read_odometry_topic(fusion_db, '/robot_b/ground_truth')
    tb1_frames = read_message_timestamps(tb1_db, '/tb1/oakd/stereo/image_raw/compressedDepth')
    tb2_frames = read_message_timestamps(tb2_db, '/tb2/oakd/stereo/image_raw/compressedDepth')

    tb1_sees_tb2 = scan_camera(
        tb1_frames, tb1_gt, tb2_gt, tb1_camera, tb1_extrinsics,
        args.robot_width_m, args.robot_height_m, args.max_range_m, args.max_sync_ms,
    )
    tb2_sees_tb1 = scan_camera(
        tb2_frames, tb2_gt, tb1_gt, tb2_camera, tb2_extrinsics,
        args.robot_width_m, args.robot_height_m, args.max_range_m, args.max_sync_ms,
    )

    write_csv(output_dir / 'tb1_camera_sees_tb2.csv', tb1_sees_tb2)
    write_csv(output_dir / 'tb2_camera_sees_tb1.csv', tb2_sees_tb1)

    report = [
        '# Camera Visibility Scan',
        '',
        'Assumptions:',
        '',
        '- Uses Vicon-derived ground-truth odometry from the processed fusion bag.',
        '- Projects target poses through per-robot 2D camera extrinsics relative to the Vicon/base pose.',
        f'- TB1 camera extrinsics: x={tb1_extrinsics.x_m:.3f} m, y={tb1_extrinsics.y_m:.3f} m, yaw={args.tb1_camera_yaw_offset_deg:.3f} deg.',
        f'- TB2 camera extrinsics: x={tb2_extrinsics.x_m:.3f} m, y={tb2_extrinsics.y_m:.3f} m, yaw={args.tb2_camera_yaw_offset_deg:.3f} deg.',
        '- Projects only horizontal image location. Vertical placement should be refined with camera/base extrinsics.',
        f'- TB1 compressed depth image size: {tb1_camera.width}x{tb1_camera.height}; intrinsics scaled from camera_info.',
        f'- TB2 compressed depth image size: {tb2_camera.width}x{tb2_camera.height}; intrinsics scaled from camera_info.',
        f'- Range gate: {args.max_range_m:.1f} m. Max pose/image sync error: {args.max_sync_ms:.1f} ms.',
        f'- TB1 horizontal FOV from intrinsics: {tb1_camera.hfov_deg:.1f} deg.',
        f'- TB2 horizontal FOV from intrinsics: {tb2_camera.hfov_deg:.1f} deg.',
        '',
        summarize('TB1 camera -> TB2 target', tb1_sees_tb2),
        '',
        summarize('TB2 camera -> TB1 target', tb2_sees_tb1),
        '',
        'CSV outputs:',
        '',
        f'- {output_dir / "tb1_camera_sees_tb2.csv"}',
        f'- {output_dir / "tb2_camera_sees_tb1.csv"}',
    ]
    output_dir.mkdir(parents=True, exist_ok=True)
    report_path = output_dir / 'visibility_report.md'
    report_path.write_text('\n'.join(report) + '\n')
    print(report_path)
    print('\n'.join(report))


if __name__ == '__main__':
    main()
