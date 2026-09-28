#!/usr/bin/env python3
"""Slow section-aware wall exploration and mapping around the IROS loop."""

import argparse
import collections
import math
import os
import struct
import time

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, Joy, LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Float64, Header, String
from vesc_msgs.msg import VescStateStamped

from wall_lidar_core import (
    ScanPathTracker, future_footprint_hits, lidar_contact_mask,
    lidar_overload_reason,
)
from identified_centerline import chassis_points
from iros_track import IROS_MAP_LAP_LENGTH_M
from temporal_scan_v1 import fuse_recent_scans
from track_autonomy_core import CAR_FORWARD_IN_LIDAR
from wall_mapper import IncrementalWallMap, save_classified_ply

# Mapping copy of V1: cap forward motor requests without changing the planner.
MAP_FORWARD_SPEED_CAP = 0.45


def low_wall_points(msg, z_min=-0.14, z_max=0.11, max_range=3.5):
    data = point_cloud2.read_points(msg, field_names=("x", "y", "z"),
                                     skip_nans=True)
    if hasattr(data, "dtype") and data.dtype.names:
        points = np.column_stack([data[name] for name in ("x", "y", "z")])
    else:
        points = np.asarray(list(data), dtype=float).reshape(-1, 3)
    radius = np.linalg.norm(points[:, :2], axis=1)
    keep = (np.isfinite(points).all(axis=1) & (radius >= 0.10)
            & (radius <= max_range) & (points[:, 2] >= z_min)
            & (points[:, 2] < z_max))
    return points[keep], points


def laser_scan_points(msg, max_range=3.5):
    """Convert the proven car-oriented /scan projection to Unitree XY axes."""
    ranges = np.asarray(msg.ranges, dtype=float)
    angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
    keep = (np.isfinite(ranges) & (ranges >= 0.10)
            & (ranges <= max_range))
    forward = ranges[keep] * np.cos(angles[keep])
    lateral = ranges[keep] * np.sin(angles[keep])
    cosine, sine = math.cos(CAR_FORWARD_IN_LIDAR), math.sin(CAR_FORWARD_IN_LIDAR)
    lidar_x = cosine * forward - sine * lateral
    lidar_y = sine * forward + cosine * lateral
    return np.column_stack((lidar_x, lidar_y, np.zeros(len(lidar_x))))


class WallExplorerLap(Node):
    def __init__(self, drive=False, remote_run=False, max_seconds=65,
                 min_seconds=0, input_mode="scan", scan_topic="/scan",
                 cloud_topic="/unilidar/cloud",
                 lap_length_m=IROS_MAP_LAP_LENGTH_M, lap_offset_m=0.0,
                 initial_width_m=2.30, min_width_m=1.30,
                 max_width_m=3.80, survey_zigzag=True,
                 cloud_z_min=-0.14, cloud_z_max=0.11,
                 map_output="/tmp/iros_wall_map_latest.npz",
                 max_retained_points=12000, max_raw_points=96000,
                 contact_point_limit=80,
                 overload_output="/tmp/wall_explorer_overload.npz"):
        super().__init__("wall_explorer_lap")
        self.drive = drive
        self.remote_run = remote_run
        self.remote_used = False
        self.max_seconds = float(max_seconds)
        self.min_seconds = float(min_seconds)
        self.input_mode = input_mode
        self.scan_topic = scan_topic
        self.cloud_topic = cloud_topic
        self.cloud_z_min = float(cloud_z_min)
        self.cloud_z_max = float(cloud_z_max)
        self.map_output = map_output
        self.max_retained_points = int(max_retained_points)
        self.max_raw_points = int(max_raw_points)
        self.contact_point_limit = int(contact_point_limit)
        self.overload_output = overload_output
        self.tracker_settings = dict(
            lap_length_m=float(lap_length_m),
            lap_offset_m=float(lap_offset_m),
            initial_width_m=float(initial_width_m),
            min_width_m=float(min_width_m),
            max_width_m=float(max_width_m),
            survey_zigzag=bool(survey_zigzag),
        )
        self.state = "ready"
        self.reason = ""
        self.tracker = ScanPathTracker(**self.tracker_settings)
        self.wall_map = IncrementalWallMap(voxel_m=0.05)
        self.command = None
        self.lane = None
        self.scan_time = 0.0
        self.scan_count = 0
        self.odom_time = 0.0
        self.vesc_time = 0.0
        self.imu_time = 0.0
        self.joy_time = 0.0
        self.r1 = False
        self.l1 = False
        self.start_requested = False
        self.odom_position = None
        self.odom_yaw = None
        self.start_odom_position = None
        self.odom_distance = 0.0
        self.odom_speed = 0.0
        self.odom_signed_speed = 0.0
        self.vesc_speeds = collections.deque(maxlen=12)
        self.ground_speed = 0.0
        self.vesc_signed_speed = 0.0
        self.servo_command = float("nan")
        self.imu_yaw = None
        self.imu_turn = 0.0
        self.started = None
        self.last_progress = None
        self.sent_speed = 0.0
        self.brake_until = 0.0
        self.scan_history = collections.deque(maxlen=60)
        self.planning_history = collections.deque(maxlen=6)
        self.turn_sign_votes = collections.deque(maxlen=12)
        self.latest_points = np.empty((0, 3))
        self.latest_raw_count = 0
        self.latest_retained_count = 0
        self.latest_contact_count = 0
        self.recovery_attempts = 0
        self.recovery_start = 0.0
        self.recovery_reverse_start = None
        self.recovery_position = None
        self.recovery_distance = 0.0
        self.recovery_steer = 0.0
        self.recovery_clear_distance = None
        self.settle_until = 0.0
        self.forward_clear_count = 0
        self.blocked_scans = 0
        self.last_saved_map_cells = 0
        self.fatal_overload = False
        self.fatal_shutdown_at = None

        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 1)
        self.brake_pub = self.create_publisher(Float64, "/commands/motor/brake", 1)
        self.status_pub = self.create_publisher(String,
                                                "/wall_explorer/status", 1)
        self.classified_pub = self.create_publisher(
            PointCloud2, "/wall_explorer/classified_points", 1)
        if input_mode == "scan":
            self.create_subscription(LaserScan, scan_topic,
                                     self.on_laser_scan, qos)
        elif input_mode == "cloud":
            self.create_subscription(PointCloud2, cloud_topic,
                                     self.on_cloud, qos)
        else:
            raise ValueError("input_mode must be 'scan' or 'cloud'")
        self.create_subscription(Imu, "/unilidar/imu", self.on_imu, qos)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(VescStateStamped, "/sensors/core", self.on_vesc, 10)
        self.create_subscription(Float64, "/commands/servo/position",
                                 self.on_servo, 10)
        self.create_subscription(Joy, "/joy", self.on_joy, 10)
        self.create_timer(0.05, self.control)
        self.create_timer(1.0, self.report)
        self.create_timer(30.0, self.save_map_checkpoint)
        self.get_logger().info(
            "Wall explorer ready: %s input=%s lap=%.2fm (%s)" %
            ("MOTORS ENABLED" if drive else "DRY RUN", input_mode,
             self.tracker_settings["lap_length_m"],
             "survey weave" if survey_zigzag else "centred"))

    def on_laser_scan(self, msg):
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        self.process_points(
            laser_scan_points(msg), msg.header.stamp,
            raw_count=len(ranges),
            raw_capture={
                "raw_scan_ranges": ranges,
                "raw_scan_angle_min": np.asarray(msg.angle_min),
                "raw_scan_angle_increment": np.asarray(msg.angle_increment),
                "raw_scan_range_min": np.asarray(msg.range_min),
                "raw_scan_range_max": np.asarray(msg.range_max),
            })

    def on_cloud(self, msg):
        points, raw_points = low_wall_points(
            msg, z_min=self.cloud_z_min, z_max=self.cloud_z_max)
        self.process_points(
            points, msg.header.stamp, raw_count=len(raw_points),
            raw_capture={"raw_cloud_points": raw_points})

    def process_points(self, points, stamp, raw_count=0, raw_capture=None):
        # Even an empty but fresh scan is meaningful: it starts the bounded
        # no-wall protocol. Coverage/station support, not the old 50/80 global
        # point counts, determines whether a wall is usable downstream.
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        self.latest_raw_count = int(raw_count)
        self.latest_retained_count = len(points)
        overload_reason = self.overload_reason(points, raw_count)
        if overload_reason:
            self.handle_overload(overload_reason, points, raw_count,
                                 raw_capture=raw_capture)
            return
        self.latest_contact_count = int(np.count_nonzero(
            lidar_contact_mask(points)))
        now = time.monotonic()
        planning_points = fuse_recent_scans(
            points, self.planning_history, self.odom_distance,
            self.imu_turn, now)
        self.command, self.lane, reason = self.tracker.update(
            points, now, planning_points=planning_points,
            lap_distance=self.odom_distance, turn=self.imu_turn)
        self.publish_classified_points(self.tracker.last_observation, stamp)
        self.latest_points = points
        if self.lane.valid and abs(self.lane.steering_rad) >= 0.08:
            self.turn_sign_votes.append(math.copysign(1.0,
                                                      self.lane.steering_rad))
        self.scan_time = now
        self.scan_count += 1
        self.scan_history.append((points.copy(), now, self.odom_distance,
                                  self.imu_turn))
        self.planning_history.append((points.copy(), now,
                                      self.odom_distance, self.imu_turn))
        self.wall_map.update(
            self.tracker.last_observation, self.odom_position, self.odom_yaw,
            self.odom_distance, now)
        self.forward_clear_count = (self.forward_clear_count + 1
                                    if (self.command is not None and not reason
                                        and self.lane.valid)
                                    else 0)
        if self.state == "active" and reason:
            if reason in ("no clear swept path ahead", "obstacle at car footprint"):
                self.blocked_scans += 1
                if self.blocked_scans >= 3:
                    self.begin_recovery(reason, points)
            else:
                self.stop(reason)
        elif not reason:
            self.blocked_scans = 0

    def overload_reason(self, points, raw_count):
        if self.fatal_overload:
            return "overload fault already latched"
        return lidar_overload_reason(
            points, raw_count,
            max_retained_points=self.max_retained_points,
            contact_point_limit=self.contact_point_limit,
            max_raw_points=self.max_raw_points)

    def handle_overload(self, reason, points, raw_count, raw_capture=None):
        if self.fatal_overload:
            return
        self.fatal_overload = True
        message = ("FATAL LIDAR OVERLOAD: %s; raw=%d retained=%d" %
                   (reason, raw_count, len(points)))

        # Do this before compression or file I/O: a very large triggering scan
        # must never delay the zero-speed command and brake request.
        self.publish_drive(0.0, 0.0)
        self.publish_brake()
        status = String()
        status.data = message + "; stopped, capturing fault data"
        self.status_pub.publish(status)
        self.get_logger().fatal(status.data)

        output = os.path.abspath(os.path.expanduser(self.overload_output))
        root, _ = os.path.splitext(output)
        try:
            os.makedirs(os.path.dirname(output), exist_ok=True)
        except Exception as exc:
            message += "; cannot create overload output directory: %s" % exc
            self.get_logger().fatal(message)
            status.data = message
            self.status_pub.publish(status)
            self.stop(message)
            self.fatal_shutdown_at = time.monotonic() + 1.0
            return
        observation = self.tracker.last_observation
        pieces = observation.pieces if observation is not None else []
        piece_points = (np.concatenate([piece.points for piece in pieces])
                        if pieces else np.empty((0, 2)))
        wall_ids = (np.concatenate([
            np.full(len(piece.points), piece.wall, dtype=np.uint8)
            for piece in pieces]) if pieces else np.empty(0, dtype=np.uint8))
        piece_ids = (np.concatenate([
            np.full(len(piece.points), piece.piece_id, dtype=np.uint16)
            for piece in pieces]) if pieces else np.empty(0, dtype=np.uint16))
        confidences = (np.concatenate([
            np.full(len(piece.points), piece.confidence, dtype=np.float32)
            for piece in pieces]) if pieces else np.empty(0, dtype=np.float32))
        contact_mask = lidar_contact_mask(points)
        self.latest_contact_count = int(np.count_nonzero(contact_mask))
        trigger_points = np.column_stack((chassis_points(points), points[:, 2]))
        try:
            payload = dict(
                reason=np.asarray(message),
                recorded_unix_s=np.asarray(time.time()),
                controller_state=np.asarray(self.state),
                current_points=points,
                current_points_base=trigger_points,
                current_contact_mask=contact_mask,
                raw_count=np.asarray(raw_count),
                retained_count=np.asarray(len(points)),
                max_raw_points=np.asarray(self.max_raw_points),
                max_retained_points=np.asarray(self.max_retained_points),
                contact_point_limit=np.asarray(self.contact_point_limit),
                odom_position=(np.asarray(self.odom_position)
                               if self.odom_position is not None
                               else np.full(2, np.nan)),
                odom_yaw=np.asarray(self.odom_yaw
                                    if self.odom_yaw is not None else np.nan),
                odom_distance=np.asarray(self.odom_distance),
                imu_turn=np.asarray(self.imu_turn),
                classified_points=piece_points,
                classified_wall_id=wall_ids,
                classified_piece_id=piece_ids,
                classified_confidence=confidences,
            )
            if raw_capture:
                payload.update(raw_capture)
            np.savez_compressed(output, **payload)
            save_classified_ply(
                root + ".ply", trigger_points,
                np.zeros(len(points), dtype=np.uint8),
                contact_mask.astype(np.uint16),
                contact_mask.astype(np.float32))
            save_classified_ply(root + "_classified.ply", piece_points,
                                wall_ids, piece_ids, confidences)
            message += ("; dump=%s; trigger_ply=%s; classified_ply=%s" %
                        (output, root + ".ply", root + "_classified.ply"))
            report_path = root + ".txt"
            final_message = message + "; report=" + report_path
            temporary = report_path + ".tmp"
            with open(temporary, "w", encoding="utf-8") as stream:
                stream.write(final_message + "\n")
            os.replace(temporary, report_path)
            message = final_message
        except Exception as exc:
            message += "; overload artifact/report write failed: %s" % exc
        self.get_logger().fatal(message)
        status = String()
        status.data = message
        self.status_pub.publish(status)
        self.stop(message)
        self.fatal_shutdown_at = time.monotonic() + 1.0

    @staticmethod
    def _rgb_float(red, green, blue):
        packed = ((int(red) & 255) << 16
                  | (int(green) & 255) << 8
                  | (int(blue) & 255))
        return struct.unpack("f", struct.pack("I", packed))[0]

    def publish_classified_points(self, observation, stamp):
        """Publish each understood scan piece with wall/piece metadata."""
        rows = []
        pieces = (observation.pieces
                  if observation is not None and observation.pieces else [])
        for piece in pieces:
            if piece.wall == 1:
                base = (245, 65 + 18 * (piece.piece_id % 4), 30)
            else:
                base = (30, 105 + 20 * (piece.piece_id % 4), 255)
            shade = 0.55 + 0.45 * float(np.clip(piece.confidence, 0.0, 1.0))
            rgb = self._rgb_float(*(int(channel * shade)
                                    for channel in base))
            rows.extend((float(point[0]), float(point[1]), 0.0, rgb,
                         int(piece.wall), int(piece.piece_id),
                         float(piece.confidence))
                        for point in piece.points)
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32,
                       count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32,
                       count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32,
                       count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32,
                       count=1),
            PointField(name="wall_id", offset=16, datatype=PointField.UINT8,
                       count=1),
            PointField(name="piece_id", offset=18, datatype=PointField.UINT16,
                       count=1),
            PointField(name="confidence", offset=20,
                       datatype=PointField.FLOAT32, count=1),
        ]
        header = Header()
        header.stamp = stamp
        header.frame_id = "base_link"
        self.classified_pub.publish(
            point_cloud2.create_cloud(header, fields, rows))

    def save_map_checkpoint(self):
        if (not self.wall_map.cells
                or len(self.wall_map.cells) == self.last_saved_map_cells):
            return
        try:
            self.wall_map.save(
                self.map_output,
                reason=np.asarray("checkpoint"),
                lap_length_m=np.asarray(
                    self.tracker_settings["lap_length_m"]),
                estimated_width_m=np.asarray(self.tracker.width),
            )
            self.last_saved_map_cells = len(self.wall_map.cells)
        except Exception as exc:
            self.get_logger().warn("Could not checkpoint wall map: %s" % exc)

    def begin_recovery(self, reason, points):
        if self.recovery_attempts >= 3:
            self.stop("three recovery attempts did not clear the corner")
            return
        car_points = chassis_points(points)
        rear_visible = np.count_nonzero((car_points[:, 0] < -0.10)
                                        & (np.abs(car_points[:, 1]) > 0.15))
        if rear_visible < 12:
            self.stop("rear wall visibility insufficient for reverse")
            return
        turn = self.lane.steering_rad if self.lane and self.lane.valid else 0.0
        if len(self.turn_sign_votes) >= 5:
            vote = float(np.median(self.turn_sign_votes))
            if abs(vote) > 0.5:
                turn = vote * max(abs(turn), 0.20)
        elif abs(turn) < 0.05 and self.tracker.preview_steer is not None:
            turn = self.tracker.preview_steer
        if abs(turn) < 0.05:
            self.stop("turn direction unavailable for reverse")
            return
        reverse_sign = -math.copysign(1.0, turn)
        choices = [reverse_sign * angle for angle in
                   ((0.25, 0.20, 0.12) if reverse_sign > 0
                    else (0.30, 0.22, 0.12))]
        choices.append(0.0)
        # At a tight corner the preferred reverse turn can brush the wall
        # behind the car while the opposite reverse arc remains clear.
        choices.extend(-reverse_sign * angle for angle in (0.30, 0.22, 0.12))
        safe = [steer for steer in choices
                if future_footprint_hits(points, steer, travel=-0.16) < 3]
        if not safe:
            self.stop("no clear reverse path")
            return
        self.recovery_steer = safe[0]
        self.recovery_attempts += 1
        self.recovery_start = time.monotonic()
        self.recovery_reverse_start = None
        self.recovery_position = (None if self.odom_position is None
                                  else self.odom_position.copy())
        self.recovery_distance = 0.0
        self.forward_clear_count = 0
        self.state = "recovering"
        self.publish_drive(0.0, 0.0)
        self.publish_brake()
        self.get_logger().warn(
            "Forward path blocked (%s); reversing attempt %d with steer %+.2f" %
            (reason, self.recovery_attempts, self.recovery_steer))

    def on_imu(self, msg):
        q = msg.orientation
        if q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z < 0.5:
            return
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y),
                         1 - 2 * (q.y*q.y + q.z*q.z))
        if self.imu_yaw is not None and self.state == "active":
            delta = math.atan2(math.sin(yaw - self.imu_yaw),
                               math.cos(yaw - self.imu_yaw))
            if abs(delta) < 0.20:
                self.imu_turn += delta
        self.imu_yaw = yaw
        self.imu_time = time.monotonic()

    def on_odom(self, msg):
        position = np.array([msg.pose.pose.position.x,
                             msg.pose.pose.position.y], dtype=float)
        orientation = msg.pose.pose.orientation
        norm2 = (orientation.w * orientation.w
                 + orientation.x * orientation.x
                 + orientation.y * orientation.y
                 + orientation.z * orientation.z)
        if norm2 >= 0.5:
            self.odom_yaw = math.atan2(
                2.0 * (orientation.w * orientation.z
                       + orientation.x * orientation.y),
                1.0 - 2.0 * (orientation.y * orientation.y
                             + orientation.z * orientation.z),
            )
        step = (float(np.linalg.norm(position - self.odom_position))
                if self.odom_position is not None else 0.0)
        if self.odom_position is not None and self.state == "active":
            if 0.002 <= step < 0.10:
                self.odom_distance += step
                self.last_progress = time.monotonic()
                if (self.recovery_clear_distance is not None
                        and self.odom_distance-self.recovery_clear_distance >= 0.40):
                    self.recovery_attempts = 0
                    self.recovery_clear_distance = None
        self.odom_position = position
        self.odom_signed_speed = float(msg.twist.twist.linear.x)
        self.odom_speed = abs(self.odom_signed_speed)
        if self.state == "recovering" and self.recovery_position is not None:
            # Forward coasting can cancel reverse motion in a net-pose check.
            # VESC raw speed is negative in reverse on this car.
            if self.vesc_signed_speed < -0.03 and 0.002 <= step < 0.10:
                self.recovery_distance += step
        self.odom_time = time.monotonic()

    def on_vesc(self, msg):
        self.vesc_signed_speed = float(msg.state.speed) / 4614.0
        self.vesc_speeds.append(abs(self.vesc_signed_speed))
        self.ground_speed = float(np.median(self.vesc_speeds))
        self.vesc_time = time.monotonic()

    def on_servo(self, msg):
        self.servo_command = float(msg.data)

    def on_joy(self, msg):
        if len(msg.buttons) < 6:
            return
        old_l1 = self.l1
        self.l1 = bool(msg.buttons[4])
        self.r1 = bool(msg.buttons[5])
        self.joy_time = time.monotonic()
        if self.l1 and not old_l1 and self.state == "ready":
            self.start_requested = True
        if self.r1 and self.state in ("active", "recovering", "settling"):
            self.stop("R1 manual override", brake=False)

    def publish_drive(self, speed, steer):
        if speed > 0.0:
            speed = min(float(speed), MAP_FORWARD_SPEED_CAP)
        self.sent_speed = float(speed)
        if not self.drive or not rclpy.ok():
            return
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.drive_pub.publish(msg)

    def publish_brake(self):
        if self.drive and rclpy.ok():
            msg = Float64()
            msg.data = 8.0
            self.brake_pub.publish(msg)

    def stop(self, reason, brake=True):
        if self.state == "stopped":
            return
        self.state = "stopped"
        self.reason = reason
        self.brake_until = time.monotonic() + (0.7 if brake else 0.0)
        self.publish_drive(0.0, 0.0)
        if brake:
            self.publish_brake()
        if self.scan_history:
            frames, times, distances, turns = zip(*self.scan_history)
            lengths = np.array([len(frame) for frame in frames])
            try:
                np.savez_compressed("/tmp/wall_explorer_last_stop_scans.npz",
                                    points=np.concatenate(frames),
                                    offsets=np.r_[0, np.cumsum(lengths)],
                                    times=np.asarray(times),
                                    distances=np.asarray(distances),
                                    turns=np.asarray(turns), reason=reason)
            except Exception as exc:
                self.get_logger().warn("Could not save stop scans: %s" % exc)
        try:
            saved = self.wall_map.save(
                self.map_output,
                reason=np.asarray(reason),
                lap_length_m=np.asarray(
                    self.tracker_settings["lap_length_m"]),
                estimated_width_m=np.asarray(self.tracker.width),
            )
            self.get_logger().info("Saved classified wall map to %s" % saved)
        except Exception as exc:
            self.get_logger().warn("Could not save classified wall map: %s" % exc)
        self.get_logger().warn("Lap stopped: %s; distance %.2fm; turn %.2frad" %
                               (reason, self.odom_distance, self.imu_turn))

    def can_arm(self, now):
        if self.state != "ready" or self.r1:
            return False
        if not ((self.remote_run and not self.remote_used) or (self.start_requested
                                    and now-self.joy_time < 0.35)):
            return False
        if (self.scan_count < 12 or now-self.scan_time > 0.30
                or now-self.odom_time > 0.35 or now-self.imu_time > 0.25
                or now-self.vesc_time > 0.35):
            return False
        if self.lane is None or not self.lane.valid:
            return False
        if self.odom_speed > 0.03 or self.ground_speed > 0.03:
            return False
        if self.drive and (self.count_publishers("/drive") > 1
                           or self.brake_pub.get_subscription_count() < 1):
            return False
        return True

    def control(self):
        now = time.monotonic()
        if self.fatal_overload:
            self.publish_drive(0.0, 0.0)
            if (self.fatal_shutdown_at is None
                    or now < self.fatal_shutdown_at):
                self.publish_brake()
            elif rclpy.ok():
                self.get_logger().fatal(
                    "Overload data stored; shutting down wall explorer")
                rclpy.shutdown()
            return
        # After a reboot, vesc_to_odom waits for its first servo command
        # before publishing any odometry. Seed it through the normal drive
        # mux at zero speed so the odometry freshness check can arm the run.
        if (self.state == "ready" and self.drive and not self.r1
                and self.odom_time == 0.0
                and self.count_publishers("/drive") == 1):
            self.publish_drive(0.0, 0.0)
        if self.can_arm(now):
            self.state = "active"
            if self.remote_run:
                self.remote_used = True
            self.start_requested = False
            self.started = now
            self.last_progress = now
            self.odom_distance = 0.0
            self.imu_turn = 0.0
            self.start_odom_position = (None if self.odom_position is None
                                        else self.odom_position.copy())
            self.recovery_attempts = 0
            self.recovery_clear_distance = None
            self.blocked_scans = 0
            self.get_logger().info(
                "Wall-explorer lap started; section=%s %s lane %.2fm" %
                (self.tracker.section, self.lane.mode, self.lane.width_m))
        if self.state == "stopped":
            if now < self.brake_until and not self.r1:
                self.publish_brake()
            elif self.drive and not self.r1:
                self.publish_drive(0.0, 0.0)
            if now > self.brake_until + 0.4 and not self.l1 and not self.r1:
                self.state = "ready"
                self.reason = ""
                self.start_requested = False
                self.tracker = ScanPathTracker(**self.tracker_settings)
                self.planning_history.clear()
                self.get_logger().info("Ready for a new L1 press")
            return
        if self.state in ("recovering", "settling"):
            if (now-self.scan_time > 0.30 or now-self.odom_time > 0.35
                    or now-self.imu_time > 0.25 or now-self.vesc_time > 0.35
                    or (not self.remote_run and now-self.joy_time > 0.50)):
                self.stop("sensor or controller data stale during recovery")
                return
            if now-self.started >= self.max_seconds:
                self.stop("run time limit during recovery")
                return
            if self.drive and self.count_publishers("/drive") > 1:
                self.stop("another drive publisher appeared during recovery")
                return
            if self.state == "recovering":
                if self.recovery_reverse_start is None:
                    # At speed, a reverse command can arrive while the car is
                    # still coasting forward. Brake first, then begin the
                    # guarded reverse only once the wheels have settled.
                    self.publish_drive(0.0, 0.0)
                    self.publish_brake()
                    if (abs(self.vesc_signed_speed) > 0.05
                            or self.ground_speed > 0.08):
                        if now - self.recovery_start > 2.0:
                            self.stop("car did not settle before reverse")
                        return
                    self.recovery_reverse_start = now
                    self.recovery_distance = 0.0
                    self.get_logger().info("Wheels settled; starting guarded reverse")
                reverse_elapsed = now - self.recovery_reverse_start
                if (self.recovery_distance >= 0.15
                        or reverse_elapsed >= 2.4):
                    self.publish_drive(0.0, 0.0)
                    self.publish_brake()
                    self.state = "settling"
                    self.settle_until = now + 0.4
                    self.get_logger().info("Reverse moved %.2fm; rechecking forward lane" %
                                           self.recovery_distance)
                    return
                if (reverse_elapsed > 0.35
                        and self.vesc_signed_speed > 0.04):
                    self.recovery_reverse_start = None
                    self.publish_drive(0.0, 0.0)
                    self.publish_brake()
                    self.get_logger().warn("Forward wheel motion during reverse; braking again")
                    return
                if (reverse_elapsed > 1.5
                        and self.recovery_distance < 0.025):
                    self.stop("reverse command produced no wheel movement")
                    return
                if (len(self.latest_points) < 50 or
                        future_footprint_hits(self.latest_points,
                            self.recovery_steer, travel=-0.10) >= 3):
                    self.publish_drive(0.0, 0.0)
                    self.publish_brake()
                    forward_safe = any(
                        future_footprint_hits(self.latest_points, angle,
                                              travel=0.15) < 3
                        for angle in (-0.38, -0.25, -0.10, 0.0, 0.10, 0.25))
                    if not forward_safe:
                        self.stop("reverse and forward paths blocked")
                        return
                    self.state = "settling"
                    self.settle_until = now + 0.4
                    self.forward_clear_count = 0
                    self.get_logger().info(
                        "Reverse arc narrowed; braking to recheck forward lane")
                    return
                self.publish_drive(-0.35, self.recovery_steer)
                return
            if now < self.settle_until:
                self.publish_brake()
                return
            if self.forward_clear_count < 3:
                self.begin_recovery("forward path still blocked after reverse",
                                    self.latest_points)
            else:
                self.state = "active"
                self.last_progress = now
                self.recovery_clear_distance = self.odom_distance
                self.get_logger().info("Forward lane clear after reverse")
            return
        if self.state != "active":
            return
        if self.r1:
            self.stop("R1 manual override", brake=False)
            return
        if not self.remote_run and now-self.joy_time > 0.50:
            self.stop("controller signal stale")
            return
        if now-self.started >= self.max_seconds:
            self.stop("run time limit")
            return
        if (now-self.scan_time > 0.30 or now-self.odom_time > 0.35
                or now-self.imu_time > 0.25 or now-self.vesc_time > 0.35):
            self.stop("sensor data stale")
            return
        if self.drive and self.count_publishers("/drive") > 1:
            self.stop("another drive publisher appeared")
            return
        if self.drive and now-self.started > 2.0 and now-self.last_progress > 2.0:
            self.begin_recovery("no wheel progress", self.latest_points)
            return
        closure = (float(np.linalg.norm(
            self.odom_position-self.start_odom_position))
            if self.odom_position is not None and self.start_odom_position is not None
            else float("inf"))
        if (now-self.started >= self.min_seconds
                and self.odom_distance >= 5.0 and abs(self.imu_turn) >= 6.20
                and closure < 0.35):
            self.stop("one lap completed")
            return
        if 0 < self.blocked_scans < 3:
            self.publish_drive(0.0, self.tracker.steer)
            return
        if self.command is None:
            self.begin_recovery("forward path blocked at start",
                                self.latest_points)
            return
        target_speed, steer = self.command
        # The VESC command is itself in m/s. Use a brief stronger request at
        # near-zero wheel speed to clear static friction, then cap the rolling
        # request and reduce it before the slow-lap speed limit is reached.
        tight_turn = abs(steer) > 0.22
        mode = self.lane.mode if self.lane is not None else "none"
        reacquiring = mode.startswith("reacquire")
        single_wall = mode.startswith("single")
        cautious = reacquiring or single_wall or self.tracker.section.startswith("U_")
        moving = max(0.0, self.vesc_signed_speed,
                     self.odom_signed_speed, self.ground_speed)
        if self.odom_distance > 0.02 and now-self.last_progress < 0.15:
            # The wheel speed briefly reads zero between pulses even while
            # odometry shows the car rolling. Do not send another high launch
            # request on each of those zero samples.
            moving = max(moving, 0.10)
        if cautious and moving < 0.05:
            # Short launch pulse; publish_drive still caps this at 0.45 m/s.
            command_speed = 0.40
        elif cautious and moving < 0.10:
            command_speed = 0.34
        elif cautious:
            command_speed = float(np.clip(
                target_speed + 0.08 * (target_speed - moving),
                0.18, 0.34 if single_wall else 0.28))
        elif moving < 0.05:
            command_speed = 0.66 if tight_turn else 0.64
        elif moving < 0.10:
            command_speed = 0.65 if tight_turn else 0.63
        else:
            command_speed = float(np.clip(
                target_speed + (0.05 if tight_turn else 0.03)
                + 0.10*(target_speed-moving), 0.46,
                0.69 if tight_turn else 0.74))
        measured_speed = max(self.ground_speed, moving)
        if cautious and measured_speed > 0.42:
            command_speed = 0.0
        if measured_speed > 0.60:
            command_speed = min(command_speed, float(np.clip(
                0.72 - 0.70*(measured_speed-0.60), 0.50, 0.72)))
        if measured_speed > 0.72:
            command_speed = min(command_speed, 0.46)
        if measured_speed > 0.80:
            # Coast immediately through a short speed surge; demand more
            # motor speed only after the wheels have slowed again.
            command_speed = 0.0
        if (self.recovery_clear_distance is not None
                and self.odom_distance - self.recovery_clear_distance < 0.40):
            # Ease through the corner just cleared by the reverse manoeuvre.
            command_speed = min(command_speed, 0.40)
        self.publish_drive(command_speed, steer)

    def report(self):
        lane = self.lane
        details = ""
        if lane is not None:
            observation = self.tracker.last_observation
            current_wall1 = (len(observation.current_wall1)
                             if observation is not None
                             and observation.current_wall1 is not None else 0)
            current_wall2 = (len(observation.current_wall2)
                             if observation is not None
                             and observation.current_wall2 is not None else 0)
            details = (" | %s/%s width=%.2fm%s ahead=%.2fm "
                       "wall1=%d@%.2f wall2=%d@%.2f current=%d/%d "
                       "points=%d/%d contact=%d map=%d" %
                       (self.tracker.section, lane.mode, lane.width_m,
                        " locked" if self.tracker.width_locked else " seed",
                        lane.observed_ahead_m, lane.wall1_count,
                        lane.wall1_confidence, lane.wall2_count,
                        lane.wall2_confidence, current_wall1, current_wall2,
                        self.latest_raw_count, self.latest_retained_count,
                        self.latest_contact_count,
                        len(self.wall_map.cells)))
        steer = self.command[1] if self.command is not None else float("nan")
        closure = (float(np.linalg.norm(self.odom_position-self.start_odom_position))
                   if self.odom_position is not None and self.start_odom_position is not None
                   else float("nan"))
        message = ("%s%s%s | steer=%+.2frad servo=%.2f | distance=%.2fm turn=%.2frad closure=%.2fm v=%.2fm/s instant=%.2f cmd=%.2f" %
                   (self.state, (": " + self.reason) if self.reason else "",
                    details, steer, self.servo_command,
                    self.odom_distance, self.imu_turn, closure,
                    self.ground_speed, self.vesc_signed_speed,
                    self.sent_speed))
        out = String()
        out.data = message
        self.status_pub.publish(out)
        self.get_logger().info(message)

    def destroy_node(self):
        if self.drive and self.state in ("active", "recovering", "settling"):
            self.stop("process shutdown")
        elif self.wall_map.cells:
            try:
                self.wall_map.save(
                    self.map_output,
                    reason=np.asarray(self.reason or "process shutdown"),
                    lap_length_m=np.asarray(
                        self.tracker_settings["lap_length_m"]),
                    estimated_width_m=np.asarray(self.tracker.width),
                )
            except Exception as exc:
                self.get_logger().warn("Could not save wall map: %s" % exc)
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--drive", action="store_true")
    parser.add_argument("--remote-run", action="store_true")
    parser.add_argument("--max-seconds", type=float, default=1200)
    parser.add_argument("--min-seconds", type=float, default=0)
    parser.add_argument("--exit-on-stop", action="store_true")
    parser.add_argument("--input", choices=("scan", "cloud"), default="scan",
                        help="Prefer /scan; raw cloud needs a verified z band")
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--cloud-topic", default="/unilidar/cloud")
    parser.add_argument("--cloud-z-min", type=float, default=-0.14)
    parser.add_argument("--cloud-z-max", type=float, default=0.11)
    parser.add_argument("--lap-length-m", type=float,
                        default=IROS_MAP_LAP_LENGTH_M)
    parser.add_argument("--lap-offset-m", type=float, default=0.0)
    parser.add_argument("--initial-width-m", type=float, default=2.30)
    parser.add_argument("--min-width-m", type=float, default=1.30)
    parser.add_argument("--max-width-m", type=float, default=3.80)
    parser.add_argument("--no-survey-zigzag", dest="survey_zigzag",
                        action="store_false")
    parser.set_defaults(survey_zigzag=True)
    parser.add_argument("--map-output",
                        default="/tmp/iros_wall_map_latest.npz")
    parser.add_argument("--max-retained-points", type=int, default=12000,
                        help="fatal stop above this filtered point count")
    parser.add_argument("--max-raw-points", type=int, default=96000,
                        help="fatal stop above this pre-filter point count")
    parser.add_argument("--contact-point-limit", type=int, default=80,
                        help="fatal stop for this many near-body returns")
    parser.add_argument("--overload-output",
                        default="/tmp/wall_explorer_overload.npz")
    args = parser.parse_args()
    rclpy.init()
    node = WallExplorerLap(
        drive=args.drive,
        remote_run=args.remote_run,
        max_seconds=args.max_seconds,
        min_seconds=args.min_seconds,
        input_mode=args.input,
        scan_topic=args.scan_topic,
        cloud_topic=args.cloud_topic,
        lap_length_m=args.lap_length_m,
        lap_offset_m=args.lap_offset_m,
        initial_width_m=args.initial_width_m,
        min_width_m=args.min_width_m,
        max_width_m=args.max_width_m,
        survey_zigzag=args.survey_zigzag,
        cloud_z_min=args.cloud_z_min,
        cloud_z_max=args.cloud_z_max,
        map_output=args.map_output,
        max_retained_points=args.max_retained_points,
        max_raw_points=args.max_raw_points,
        contact_point_limit=args.contact_point_limit,
        overload_output=args.overload_output,
    )
    try:
        if args.exit_on_stop:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                if (node.started is not None and node.state == "stopped"
                        and time.monotonic() > node.brake_until + 0.15):
                    break
        else:
            rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
