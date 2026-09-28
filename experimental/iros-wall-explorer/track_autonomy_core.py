#!/usr/bin/env python3
"""Geometry, LiDAR map matching, and path following for this RoboRacer track.

This module has no ROS or actuator dependency so recorded scans can exercise the
same localization and steering calculations before the car is allowed to move.
"""

import math

import numpy as np
from scipy.spatial import cKDTree


# Operator measurement on 2026-09-24: 13 inches axle to axle.
WHEELBASE = 13.0 * 0.0254
# Operator measurements: LiDAR 14 in from rear bumper, rear axle 3.5 in
# from that bumper (20 in overall length; 13 in wheelbase; 10.5 in offset).
LIDAR_FORWARD_OF_BASE = 10.5 * 0.0254
# The LiDAR point-cloud x axis is rotated relative to the car's forward axis.
# Both stationary lane walls and the original driven trajectory indicate 61 deg.
CAR_FORWARD_IN_LIDAR = math.radians(61.0)
MAX_STEER = 0.38
CAR_WIDTH = 10.5 * 0.0254
BODY_REAR_OF_BASE = -3.5 * 0.0254
BODY_FRONT_OF_BASE = 16.5 * 0.0254
MIN_BODY_WALL_GAP = 0.03


def live_body_clearance(points):
    """Return the live LiDAR points nearest the measured car footprint.

    The narrow rearward stripe is a repeatable return from the car itself.
    Points outside that stripe remain eligible to detect a new obstacle.
    """
    points = np.asarray(points, dtype=float)
    forward = (points[:, 0] * math.cos(CAR_FORWARD_IN_LIDAR)
               + points[:, 1] * math.sin(CAR_FORWARD_IN_LIDAR))
    lateral = (-points[:, 0] * math.sin(CAR_FORWARD_IN_LIDAR)
               + points[:, 1] * math.cos(CAR_FORWARD_IN_LIDAR))
    self_return = ((forward > -0.48) & (forward < -0.15)
                   & (np.abs(lateral) < 0.06))
    along = forward[~self_return] + LIDAR_FORWARD_OF_BASE
    across = lateral[~self_return]
    outside_x = np.maximum(np.maximum(BODY_REAR_OF_BASE - along,
                                       along - BODY_FRONT_OF_BASE), 0.0)
    outside_y = np.maximum(np.abs(across) - CAR_WIDTH / 2, 0.0)
    return np.hypot(outside_x, outside_y)


def angle_wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def chassis_heading(lidar_yaw):
    return angle_wrap(lidar_yaw + CAR_FORWARD_IN_LIDAR)


def rear_axle_position(lidar_pose):
    x, y, lidar_yaw = lidar_pose
    yaw = chassis_heading(lidar_yaw)
    return np.array([
        x - LIDAR_FORWARD_OF_BASE * math.cos(yaw),
        y - LIDAR_FORWARD_OF_BASE * math.sin(yaw),
    ])


def constrain_nonholonomic_step(previous_pose, candidate_pose, wheel_travel):
    """Bound a scan-match update to motion possible for a rolling car.

    Wall ICP can slide a scan sideways along a near-parallel cardboard wall.
    Wheel travel and the IMU heading provide a physical constraint that stops
    a single scan match from placing the car inside that wall.
    """
    previous = np.asarray(previous_pose, dtype=float)
    candidate = np.asarray(candidate_pose, dtype=float)
    before = rear_axle_position(previous)
    after = rear_axle_position(candidate)
    mid_yaw = angle_wrap(chassis_heading(previous[2]) + 0.5 * angle_wrap(
        candidate[2] - previous[2]))
    forward = np.array([math.cos(mid_yaw), math.sin(mid_yaw)])
    left = np.array([-forward[1], forward[0]])
    delta = after - before
    distance = max(0.0, float(wheel_travel))
    along = float(np.clip(delta @ forward, -0.01, 0.025 + 1.6 * distance))
    across = float(np.clip(delta @ left, -0.012 - 0.15 * distance,
                           0.012 + 0.15 * distance))
    bounded_base = before + along * forward + across * left
    yaw = chassis_heading(candidate[2])
    bounded_lidar = bounded_base + LIDAR_FORWARD_OF_BASE * np.array([
        math.cos(yaw), math.sin(yaw)])
    return np.array([bounded_lidar[0], bounded_lidar[1], candidate[2]])


def pose_matrix(pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


def matrix_pose(transform):
    return np.array(
        [transform[0, 2], transform[1, 2], math.atan2(transform[1, 0], transform[0, 0])]
    )


def transform_xy(points, pose):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return points @ np.array([[c, s], [-s, c]]) + np.array([x, y])


def voxel_xy(points, size=0.02):
    if len(points) == 0:
        return points
    keys = np.floor(points[:, :2] / size).astype(np.int32)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return points[indices]


def _icp(points, target, tree, seed, cutoff, iterations=16):
    estimate = np.array(seed, dtype=float)
    for iteration in range(iterations):
        moved = transform_xy(points, estimate)
        distances, indices = tree.query(moved)
        matched = distances < (cutoff * 1.4 if iteration < 4 else cutoff)
        if np.count_nonzero(matched) < 25:
            break
        limit = np.quantile(distances[matched], 0.85)
        matched &= distances <= limit
        source = moved[matched]
        destination = target[indices[matched]]
        source_mean, destination_mean = source.mean(axis=0), destination.mean(axis=0)
        u, _, vt = np.linalg.svd(
            (source - source_mean).T @ (destination - destination_mean)
        )
        rotation = u @ np.diag([1.0, np.linalg.det(u @ vt)]) @ vt
        shift = destination_mean - source_mean @ rotation
        angle = math.atan2(rotation[0, 1], rotation[0, 0])
        updated = pose_matrix([shift[0], shift[1], angle]) @ pose_matrix(estimate)
        estimate = matrix_pose(updated)
        if np.linalg.norm(shift) < 0.0005 and abs(angle) < 0.0003:
            break
    distances, _ = tree.query(transform_xy(points, estimate))
    matched = distances < 0.12
    fraction = float(np.mean(matched))
    median = float(np.median(distances[matched])) if np.any(matched) else float("inf")
    score = float(np.mean(np.minimum(distances, 0.20) ** 2))
    return estimate, fraction, median, score


class TrackMap:
    def __init__(self, npz_path):
        saved = np.load(npz_path)
        self.path = np.asarray(saved["path"], dtype=float)
        self.yaw = np.asarray(saved["yaw"], dtype=float)
        self.speed_profile = (np.asarray(saved["speed_profile"], dtype=float)
                              if "speed_profile" in saved else None)
        self.curvature = (np.asarray(saved["curvature"], dtype=float)
                          if "curvature" in saved else None)
        self.walls = np.asarray(saved["walls"][:, :2], dtype=float)
        if self.path.shape != (180, 2) or len(self.walls) < 3000:
            raise ValueError("Track map has unexpected shape")
        if self.speed_profile is not None and (
                self.speed_profile.shape != (len(self.path),)
                or not np.isfinite(self.speed_profile).all()
                or np.any(self.speed_profile <= 0)):
            raise ValueError("Track speed profile is invalid")
        if self.curvature is not None and (
                self.curvature.shape != (len(self.path),)
                or not np.isfinite(self.curvature).all()):
            raise ValueError("Track curvature is invalid")
        self.wall_tree = cKDTree(self.walls)
        # The exported ring includes sparse returns up to 17 cm away from the
        # fitted barrier surface. Keep all returns for scan matching, but do
        # not let isolated interior points stand in for a physical barrier.
        self.safety_tree = self.wall_tree
        if all(key in saved for key in
               ("track_center", "inner_radius", "outer_radius")):
            center = np.asarray(saved["track_center"], dtype=float)
            inner = np.asarray(saved["inner_radius"], dtype=float)
            outer = np.asarray(saved["outer_radius"], dtype=float)
            delta = self.walls - center
            radius = np.linalg.norm(delta, axis=1)
            angle = np.arctan2(delta[:, 1], delta[:, 0])
            sector = np.mod(
                np.rint((angle + math.pi) * len(inner) / (2 * math.pi)).astype(int),
                len(inner),
            )
            on_barrier = np.minimum(
                np.abs(radius - inner[sector]),
                np.abs(radius - outer[sector]),
            ) < 0.08
            if np.count_nonzero(on_barrier) >= 3000:
                self.safety_tree = cKDTree(self.walls[on_barrier])
        self.path_tree = cKDTree(self.path)
        self.segment_lengths = np.linalg.norm(
            np.roll(self.path, -1, axis=0) - self.path, axis=1
        )
        self.lap_length = float(np.sum(self.segment_lengths))

    def wall_clearance(self, position):
        return float(self.safety_tree.query(np.asarray(position)[:2])[0])

    def body_clearance(self, rear_axle, car_yaw):
        """Map-point distance to the measured full-car rectangular footprint."""
        base = np.asarray(rear_axle, dtype=float)[:2]
        reach = BODY_FRONT_OF_BASE + CAR_WIDTH + 0.20
        indices = self.safety_tree.query_ball_point(base, reach)
        if not indices:
            return float("inf")
        offset = self.safety_tree.data[indices] - base
        cosine, sine = math.cos(car_yaw), math.sin(car_yaw)
        along = cosine * offset[:, 0] + sine * offset[:, 1]
        across = -sine * offset[:, 0] + cosine * offset[:, 1]
        outside_x = np.maximum(
            np.maximum(BODY_REAR_OF_BASE - along,
                       along - BODY_FRONT_OF_BASE), 0.0
        )
        outside_y = np.maximum(np.abs(across) - CAR_WIDTH / 2, 0.0)
        return float(np.hypot(outside_x, outside_y).min())

    def path_error(self, position):
        distance, index = self.path_tree.query(np.asarray(position)[:2])
        return float(distance), int(index)

    def localize_at_start(self, scans):
        points = voxel_xy(np.asarray(scans, dtype=float)[:, :2], 0.02)
        if len(points) < 150:
            return None
        results = []
        for yaw in (-0.20, 0.0, 0.20):
            for x in (-0.30, 0.0, 0.30):
                for y in (-0.30, 0.0, 0.30):
                    pose, fraction, median, score = _icp(
                        points, self.walls, self.wall_tree,
                        [x, y, yaw], cutoff=0.16, iterations=22,
                    )
                    if np.linalg.norm(pose[:2]) > 0.55 or abs(pose[2]) > 0.45:
                        continue
                    # The car is expected at the marked start, not elsewhere on the loop.
                    rank = score + 0.0003 * np.dot(pose[:2], pose[:2])
                    results.append((rank, pose, fraction, median))
        if not results:
            return None
        _, pose, fraction, median = min(results, key=lambda row: row[0])
        if fraction < 0.60 or median > 0.055:
            return None
        return pose, fraction, median

    def localize_on_upper_straight(self, scans):
        """Find a stationary LiDAR pose on the operator-identified upper straight."""
        points = voxel_xy(np.asarray(scans, dtype=float)[:, :2], 0.02)
        if len(points) < 150:
            return None
        seeds = []
        for index in range(20, 54, 2):
            car_yaw = self.yaw[index]
            forward = np.array([math.cos(car_yaw), math.sin(car_yaw)])
            left = np.array([-forward[1], forward[0]])
            for lateral in (-0.15, 0.0, 0.15):
                base = self.path[index] + lateral * left
                lidar = base + LIDAR_FORWARD_OF_BASE * forward
                seed = np.array([*lidar, angle_wrap(car_yaw - CAR_FORWARD_IN_LIDAR)])
                distances, _ = self.wall_tree.query(transform_xy(points, seed))
                score = float(np.mean(np.minimum(distances, 0.20) ** 2))
                seeds.append((score, seed))
        candidates = []
        for _, seed in sorted(seeds, key=lambda row: row[0])[:8]:
            pose, fraction, median, score = _icp(
                points, self.walls, self.wall_tree,
                seed, cutoff=0.16, iterations=20,
            )
            base = rear_axle_position(pose)
            path_error, index = self.path_error(base)
            if not 18 <= index <= 60 or path_error > 0.35:
                continue
            candidates.append((score, pose, fraction, median))
        if not candidates:
            return None
        _, pose, fraction, median = min(candidates, key=lambda row: row[0])
        if fraction < 0.70 or median > 0.025:
            return None
        return pose, fraction, median

    def localize_near_pose(self, scans, expected_pose):
        """Relocalize near a measured stop, avoiding lookalike wall sections."""
        expected = np.asarray(expected_pose, dtype=float)
        if expected.shape != (3,) or not np.isfinite(expected).all():
            raise ValueError("expected pose must contain finite x, y, yaw")
        points = voxel_xy(np.asarray(scans, dtype=float)[:, :2], 0.02)
        if len(points) < 150:
            return None
        candidates = []
        for dx in (-0.06, 0.0, 0.06):
            for dy in (-0.06, 0.0, 0.06):
                for dyaw in (-0.04, 0.0, 0.04):
                    seed = expected + [dx, dy, dyaw]
                    pose, fraction, median, score = _icp(
                        points, self.walls, self.wall_tree,
                        seed, cutoff=0.16, iterations=20,
                    )
                    if (np.linalg.norm(pose[:2] - expected[:2]) > 0.25
                            or abs(angle_wrap(pose[2] - expected[2])) > 0.15):
                        continue
                    base = rear_axle_position(pose)
                    if (self.wall_clearance(base) < 0.19
                            or self.body_clearance(base,
                                                   chassis_heading(pose[2]))
                            < MIN_BODY_WALL_GAP
                            or self.path_error(base)[0] > 0.24):
                        continue
                    # Along a long, nearly uniform wall, a lower scan residual
                    # can correspond to a different place on the same lane.
                    # The stopped pose from the preceding run is a stronger
                    # longitudinal cue than a few millimetres of ICP score.
                    drift = np.linalg.norm(pose[:2] - expected[:2])
                    rank = score + 0.15 * drift * drift
                    candidates.append((rank, pose, fraction, median))
        if not candidates:
            return None
        _, pose, fraction, median = min(candidates, key=lambda row: row[0])
        if fraction < 0.65 or median > 0.030:
            return None
        return pose, fraction, median

    def match_moving_scan(self, points, guess):
        points = np.asarray(points, dtype=float)[:, :2]
        if len(points) < 45:
            return None
        pose, fraction, median, _ = _icp(
            points, self.walls, self.wall_tree, guess, cutoff=0.14, iterations=12
        )
        translation = np.linalg.norm(pose[:2] - np.asarray(guess)[:2])
        yaw_error = abs(angle_wrap(pose[2] - guess[2]))
        if fraction < 0.38 or median > 0.070 or translation > 0.16 or yaw_error > 0.16:
            return None
        return pose, fraction, median


class MapLocalizer:
    """Fuse incremental KISS-ICP poses with bounded wall-map corrections."""

    def __init__(self, track):
        self.track = track
        self.anchor = None
        self.pose = None
        self.misses = 0
        self.last_quality = None

    def initialize(self, global_pose, kiss_pose):
        self.anchor = pose_matrix(global_pose) @ np.linalg.inv(pose_matrix(kiss_pose))
        self.pose = np.asarray(global_pose, dtype=float)
        self.misses = 0

    def update(self, kiss_pose, points, heading_reference=None,
               max_heading_deviation=None):
        if self.anchor is None:
            raise RuntimeError("Localizer must be initialized first")
        kiss_transform = pose_matrix(kiss_pose)
        predicted = matrix_pose(self.anchor @ kiss_transform)
        match = self.track.match_moving_scan(points, predicted)
        if match is None:
            self.pose = predicted.copy()
            if heading_reference is not None and max_heading_deviation is not None:
                error = angle_wrap(self.pose[2] - heading_reference)
                self.pose[2] = angle_wrap(
                    heading_reference + np.clip(
                        error, -max_heading_deviation, max_heading_deviation
                    )
                )
                self.anchor = pose_matrix(self.pose) @ np.linalg.inv(kiss_transform)
            self.misses += 1
            self.last_quality = None
            return self.pose, False
        aligned, fraction, median = match
        # Apply a bounded fraction of each correction to avoid steering jumps.
        change = aligned - predicted
        change[2] = angle_wrap(change[2])
        updated = predicted + 0.45 * change
        updated[2] = angle_wrap(updated[2])
        if heading_reference is not None and max_heading_deviation is not None:
            heading_error = angle_wrap(updated[2] - heading_reference)
            updated[2] = angle_wrap(
                heading_reference + np.clip(
                    heading_error, -max_heading_deviation, max_heading_deviation
                )
            )
        self.pose = updated
        self.anchor = pose_matrix(updated) @ np.linalg.inv(kiss_transform)
        self.misses = 0
        self.last_quality = (fraction, median)
        return self.pose, True


class PurePursuit:
    def __init__(self, track, lookahead=0.35):
        self.track = track
        self.lookahead = (0.30 if track.speed_profile is not None
                          and lookahead == 0.35 else float(lookahead))
        self.last_index = None

    def command(self, lidar_pose):
        yaw = chassis_heading(lidar_pose[2])
        base = rear_axle_position(lidar_pose)
        path_distance, nearest = self.track.path_error(base)
        if self.last_index is not None:
            advance = (nearest - self.last_index) % len(self.track.path)
            if advance > 35 and advance < len(self.track.path) - 8:
                return None, "path index jump"
        self.last_index = nearest
        wall_distance = self.track.wall_clearance(base)
        if path_distance > 0.48:
            return None, "too far from centerline"
        if wall_distance < 0.19:
            return None, "too close to mapped barrier"
        if self.track.body_clearance(base, yaw) < MIN_BODY_WALL_GAP:
            return None, "too close to mapped barrier for car footprint"

        distance = 0.0
        index = nearest
        for _ in range(len(self.track.path)):
            distance += self.track.segment_lengths[index]
            index = (index + 1) % len(self.track.path)
            if distance >= self.lookahead:
                break
        goal = self.track.path[index]
        alpha = angle_wrap(math.atan2(goal[1] - base[1], goal[0] - base[0]) - yaw)
        goal_distance = np.linalg.norm(goal - base)
        steering = math.atan2(
            2 * WHEELBASE * math.sin(alpha), max(goal_distance, 0.05)
        )
        if self.track.curvature is not None and path_distance < 0.08:
            path_yaw = self.track.yaw[nearest]
            left = np.array([-math.sin(path_yaw), math.cos(path_yaw)])
            lateral_error = float((base - self.track.path[nearest]) @ left)
            curve = float(self.track.curvature[nearest])
            geometric_steer = math.atan(WHEELBASE * curve)
            # On a bend, a short lookahead can briefly ask the car to turn
            # outward while it is already outside the curve. Keep the turn
            # flowing toward the upcoming path curvature instead.
            if curve < -0.35 and lateral_error > 0.015:
                steering = min(steering, 0.8 * geometric_steer)
            elif curve > 0.35 and lateral_error < -0.015:
                steering = max(steering, 0.8 * geometric_steer)
        steering = float(np.clip(steering, -MAX_STEER, MAX_STEER))
        if self.track.speed_profile is not None:
            # The saved profile includes upcoming curvature and full-car wall
            # clearance, so speed changes before each bend is entered.
            speed = float(self.track.speed_profile[nearest])
            if path_distance > 0.07:
                speed = min(speed, max(0.17, 0.27 - 0.55 * path_distance))
        else:
            speed = 0.32 - 0.09 * min(abs(steering) / MAX_STEER, 1.0)
            speed -= 0.04 * min(path_distance / 0.25, 1.0)
            speed = float(np.clip(speed, 0.26, 0.32))
        return (speed, steering, path_distance, wall_distance, nearest), None
