#!/usr/bin/env python3
"""Experimental section-aware wall path prediction and footprint checks."""

import math
from collections import deque
from dataclasses import replace
import numpy as np

from coherent_wall_tracker import CoherentWallIdentifier
from iros_track import (
    IROS_MAP_LAP_LENGTH_M, nominal_steering, section_at_m, turn_sign,
)
from identified_centerline import chassis_points, predict_centerline
from track_autonomy_core import (
    BODY_FRONT_OF_BASE, BODY_REAR_OF_BASE, CAR_WIDTH, MAX_STEER,
    WHEELBASE, LIDAR_FORWARD_OF_BASE,
)

MAX_LEFT_STEER = 0.26  # VESC servo_min=0.10 at gain=-1.2135, offset=0.42


def future_footprint_hits(lidar_points, steer, travel=0.22):
    """Count wall returns entering the car's swept body over a short command."""
    points = chassis_points(lidar_points)
    lidar_forward = points[:, 0] - LIDAR_FORWARD_OF_BASE
    self_return = ((lidar_forward > -0.48) & (lidar_forward < -0.15)
                   & (np.abs(points[:, 1]) < 0.06))
    # Stationary scans contain repeatable LiDAR/mount returns inside the
    # rear axle footprint (x=-0.08..0.08 m, |y|<0.15 m). These cannot be
    # track walls, but otherwise block every forward steering arc.
    body_reflection = ((points[:, 0] > BODY_REAR_OF_BASE - 0.02)
                       & (points[:, 0] < 0.11)
                       & (np.abs(points[:, 1]) < 0.16))
    points = points[~(self_return | body_reflection)]
    # Existing contacts that move clear should not prevent the car escaping:
    # the rear side can brush the wall on a forward turn, and the front can
    # already be beside a wall when a short reverse clears it.
    curvature = math.tan(steer) / WHEELBASE
    theta = curvature * travel
    if abs(curvature) < 1e-5:
        axle_x, axle_y = travel, 0.0
    else:
        axle_x = math.sin(theta) / curvature
        axle_y = (1.0 - math.cos(theta)) / curvature
    dx = points[:, 0] - axle_x
    dy = points[:, 1] - axle_y
    along = math.cos(theta) * dx + math.sin(theta) * dy
    across = -math.sin(theta) * dx + math.cos(theta) * dy
    end_x = np.maximum(np.maximum(BODY_REAR_OF_BASE - along,
                                   along - BODY_FRONT_OF_BASE), 0.0)
    end_y = np.maximum(np.abs(across) - CAR_WIDTH * 0.5, 0.0)
    end_gap = np.hypot(end_x, end_y)
    if travel > 0:
        side_gap = np.abs(points[:, 1]) - CAR_WIDTH * 0.5
        existing_contact = ((points[:, 0] > BODY_REAR_OF_BASE - 0.04)
                            & (points[:, 0] < 0.10)
                            & (side_gap > -0.01) & (side_gap < 0.04))
    else:
        existing_contact = ((points[:, 0] > 0.25)
                            & (points[:, 0] < BODY_FRONT_OF_BASE + 0.03)
                            & (np.abs(points[:, 1]) < CAR_WIDTH * 0.5 + 0.04))
    points = points[~(existing_contact & (end_gap >= 0.025))]
    hits = 0
    steps = max(2, math.ceil(abs(travel) / 0.04))
    for distance in np.linspace(travel / steps, travel, steps):
        curvature = math.tan(steer) / WHEELBASE
        theta = curvature * distance
        if abs(curvature) < 1e-5:
            axle_x, axle_y = distance, 0.0
        else:
            axle_x = math.sin(theta) / curvature
            axle_y = (1.0 - math.cos(theta)) / curvature
        dx = points[:, 0] - axle_x
        dy = points[:, 1] - axle_y
        along = math.cos(theta) * dx + math.sin(theta) * dy
        across = -math.sin(theta) * dx + math.cos(theta) * dy
        outside_x = np.maximum(np.maximum(BODY_REAR_OF_BASE - along,
                                            along - BODY_FRONT_OF_BASE), 0.0)
        outside_y = np.maximum(np.abs(across) - CAR_WIDTH * 0.5, 0.0)
        gap = np.hypot(outside_x, outside_y)
        hits = max(hits, int(np.count_nonzero(gap < 0.025)))
    return hits


def lidar_contact_mask(lidar_points):
    """Mark returns in/just around the body after known self-reflections."""
    points = np.asarray(lidar_points, dtype=float).reshape(-1, 3)
    if not len(points):
        return np.zeros(0, dtype=bool)
    car = chassis_points(points)
    sensor_forward = car[:, 0] - LIDAR_FORWARD_OF_BASE
    self_return = ((sensor_forward > -0.48)
                   & (sensor_forward < -0.15)
                   & (np.abs(car[:, 1]) < 0.06))
    mount_return = ((car[:, 0] > BODY_REAR_OF_BASE - 0.02)
                    & (car[:, 0] < 0.11)
                    & (np.abs(car[:, 1]) < 0.16))
    usable = ~(self_return | mount_return)
    contact = (usable
               & (car[:, 0] > BODY_REAR_OF_BASE - 0.04)
               & (car[:, 0] < BODY_FRONT_OF_BASE + 0.10)
               & (np.abs(car[:, 1]) < CAR_WIDTH * 0.5 + 0.10))
    return contact


def lidar_overload_reason(lidar_points, raw_count, max_retained_points=12000,
                          contact_point_limit=80, max_raw_points=None):
    """Return a fatal overload/contact message, or an empty string."""
    points = np.asarray(lidar_points, dtype=float).reshape(-1, 3)
    retained = len(points)
    max_raw_points = (8 * max_retained_points if max_raw_points is None
                      else int(max_raw_points))
    if retained > max_retained_points:
        return ("retained point count %d exceeds limit %d (raw=%d)" %
                (retained, max_retained_points, raw_count))
    if raw_count > max_raw_points:
        return ("raw LiDAR point count %d exceeds driver flood limit %d" %
                (raw_count, max_raw_points))
    if not retained:
        return ""
    contact = lidar_contact_mask(points)
    contact_count = int(np.count_nonzero(contact))
    if contact_count >= contact_point_limit:
        return ("dense near-body cluster %d exceeds contact limit %d; "
                "possible track strike" %
                (contact_count, contact_point_limit))
    return ""


class ScanPathTracker:
    """Section-aware wall follower with bounded blind-wall reacquisition."""

    def __init__(self, lap_length_m=IROS_MAP_LAP_LENGTH_M, lap_offset_m=0.0,
                 initial_width_m=2.30, min_width_m=1.30,
                 max_width_m=3.80, survey_zigzag=True):
        if not min_width_m < initial_width_m < max_width_m:
            raise ValueError("initial width must lie inside min/max width")
        self.lap_length_m = float(lap_length_m)
        self.lap_offset_m = float(lap_offset_m)
        self.min_width = float(min_width_m)
        self.max_width = float(max_width_m)
        self.width = float(initial_width_m)
        self.width_locked = False
        self.width_history = deque(maxlen=7)
        self.identifier = CoherentWallIdentifier(
            lap_length_m=self.lap_length_m, max_range_m=3.5)
        self.last_observation = None
        self.section = "straight"
        self.survey_zigzag = bool(survey_zigzag)

        self.steer = 0.0
        self.last_valid = None
        self.last_valid_distance = None
        self.last_valid_turn = None
        self.last_update = None
        self.last_prediction = None
        self.preview_steer = None
        self.preview_time = None
        self.opposite_previews = 0
        self.desired_history = deque(maxlen=3)
        self.lost_since = None
        self.lost_distance = None

    def _section_guidance(self, desired, mode):
        """Keep one-wall U-turn estimates on the known section handedness."""
        sign = turn_sign(self.section)
        if sign == 0.0 or not (mode.startswith("single")
                               or mode.startswith("reacquire")):
            return desired
        nominal = nominal_steering(self.section, WHEELBASE,
                                   self.lap_length_m)
        if desired * sign <= 0.0:
            desired = nominal
        elif abs(desired) < 0.55 * abs(nominal):
            desired = sign * 0.55 * abs(nominal)
        if self.section.startswith("U_"):
            desired = 0.72 * desired + 0.28 * nominal
        return desired

    def _survey_correction(self, lane, lap_distance, both_current=True):
        """Small straight-only weave to sample both walls on a wide track."""
        if (not self.survey_zigzag or self.section != "straight"
                or lane.mode != "paired" or not both_current):
            return 0.0
        half_clearance = 0.5 * (lane.width_m - CAR_WIDTH)
        amplitude = min(0.18, max(0.0, 0.25 * (half_clearance - 0.35)))
        if amplitude <= 0.0:
            return 0.0
        target_y = amplitude * math.sin(2.0 * math.pi * lap_distance / 5.0)
        # Pure-pursuit correction to a gentle 1.2 m-ahead lateral target.
        return 0.45 * math.atan2(2.0 * WHEELBASE * target_y, 1.2 ** 2)

    @staticmethod
    def _motion_correct_rows(rows, travel, yaw):
        if rows is None:
            return None
        rows = np.asarray(rows, dtype=float).reshape(-1, 2)
        if not len(rows):
            return rows.copy()
        if abs(yaw) < 1e-6:
            shift = np.array([travel, 0.0])
        else:
            radius = travel / yaw
            shift = np.array([radius * math.sin(yaw),
                              radius * (1.0 - math.cos(yaw))])
        relative = rows - shift
        cosine, sine = math.cos(yaw), math.sin(yaw)
        return np.column_stack((
            cosine * relative[:, 0] + sine * relative[:, 1],
            -sine * relative[:, 0] + cosine * relative[:, 1],
        ))

    def _begin_or_continue_reacquisition(self, lane, now, lap_distance,
                                         turn, observation):
        """Return a bounded dead-reckoning lane/steer pair, or ``None``."""
        if self.last_prediction is None or self.last_valid is None:
            return None
        if self.lost_since is None:
            self.lost_since = now
            self.lost_distance = lap_distance
        elapsed = now - self.lost_since
        travelled = max(0.0, lap_distance - self.lost_distance)
        time_limit = 1.25 if self.width_locked else 0.45
        distance_limit = 0.38 if self.width_locked else 0.12
        if elapsed > time_limit or travelled > distance_limit:
            return None

        since_valid = max(0.0, lap_distance - self.last_valid_distance)
        yaw = (turn - self.last_valid_turn
               if self.last_valid_turn is not None else 0.0)
        estimated = replace(
            self.last_prediction,
            centerline=self._motion_correct_rows(
                self.last_prediction.centerline, since_valid, yaw),
            observed_centers=self._motion_correct_rows(
                self.last_prediction.observed_centers, since_valid, yaw),
            left_wall=self._motion_correct_rows(
                self.last_prediction.left_wall, since_valid, yaw),
            right_wall=self._motion_correct_rows(
                self.last_prediction.right_wall, since_valid, yaw),
            inferred_centers=self._motion_correct_rows(
                self.last_prediction.inferred_centers, since_valid, yaw),
        )

        sign = turn_sign(self.section)
        if sign:
            nominal = nominal_steering(self.section, WHEELBASE,
                                       self.lap_length_m)
            desired = (0.65 * self.steer + 0.35 * nominal
                       if self.steer * sign > 0.0 else nominal)
        else:
            # On a straight, preserve the last path while sweeping the nose a
            # few degrees side-to-side to reacquire a reflective tube.
            weave = 0.045 * math.sin(2.0 * math.pi * elapsed / 0.80)
            ahead = estimated.centerline[estimated.centerline[:, 0] > 0.15]
            if len(ahead):
                target = ahead[np.argmin(np.abs(ahead[:, 0] - 0.70))]
                alpha = math.atan2(target[1], target[0])
                path_steer = math.atan2(
                    2.0 * WHEELBASE * math.sin(alpha),
                    max(float(np.linalg.norm(target)), 0.05))
            else:
                path_steer = self.steer
            desired = 0.82 * path_steer + weave
        guessed = replace(
            estimated,
            valid=False,
            observed_ahead_m=0.0,
            steering_rad=desired,
            reason=lane.reason,
            mode="reacquire_%s" % self.section,
            section=self.section,
            wall1_count=len(observation.wall1),
            wall2_count=len(observation.wall2),
            wall1_confidence=observation.confidence1,
            wall2_confidence=observation.confidence2,
        )
        return guessed, desired

    def update(self, points, now, planning_points=None, lap_distance=0.0,
               turn=0.0):
        planning = points if planning_points is None else planning_points
        self.section = section_at_m(lap_distance, self.lap_length_m,
                                    self.lap_offset_m)
        observation = self.identifier.update(
            chassis_points(planning), self.section,
            current_chassis_xy=chassis_points(points))
        self.last_observation = observation
        lane = predict_centerline(
            planning,
            expected_width=self.width,
            min_width=self.min_width,
            max_width=self.max_width,
            wall_observation=observation,
            section=self.section,
        )
        current_wall1 = observation.current_wall1_visible
        current_wall2 = observation.current_wall2_visible
        current_wall_seen = current_wall1 or current_wall2
        both_current = current_wall1 and current_wall2
        exploring = False
        if lane.valid and current_wall_seen:
            self.lost_since = None
            self.lost_distance = None
            if (lane.mode.startswith("paired")
                    and both_current
                    and observation.wall1_confirmed
                    and observation.wall2_confirmed
                    and self.min_width <= lane.width_m <= self.max_width):
                self.width_history.append(lane.width_m)
                measured = float(np.median(self.width_history))
                if not self.width_locked:
                    self.width = measured
                    self.width_locked = True
                else:
                    change = float(np.clip(measured - self.width,
                                           -0.20, 0.20))
                    self.width += 0.25 * change

            self.last_valid = now
            self.last_valid_distance = lap_distance
            self.last_valid_turn = turn
            self.last_prediction = lane
            desired = lane.steering_rad
            is_preview = (lane.mode.startswith("paired_plus")
                          and lane.observed_ahead_m >= 0.65
                          and abs(desired) >= 0.06)
            if is_preview:
                opposing = (self.preview_steer is not None
                            and self.preview_steer * desired < 0.0
                            and self.preview_time is not None
                            and now - self.preview_time < 0.8)
                self.opposite_previews = (self.opposite_previews + 1
                                          if opposing else 0)
                if not opposing or self.opposite_previews >= 3:
                    self.preview_steer = desired
                    self.preview_time = now
                    self.opposite_previews = 0
            if (lane.mode.startswith("single")
                    and self.preview_steer is not None
                    and self.preview_time is not None
                    and now - self.preview_time < 0.8):
                if desired * self.preview_steer < 0.0:
                    desired = self.preview_steer
                elif (lane.observed_ahead_m < 0.55
                      or abs(desired) < 0.60 * abs(self.preview_steer)):
                    desired = 0.65 * self.preview_steer + 0.35 * desired
            desired = self._section_guidance(desired, lane.mode)
            desired += self._survey_correction(
                lane, lap_distance, both_current=both_current)
            self.desired_history.append(desired)
            if lane.mode.startswith("single") and len(self.desired_history) >= 3:
                desired = float(np.median(self.desired_history))
            lane.steering_rad = desired
        else:
            if lane.valid and not current_wall_seen:
                lane = replace(
                    lane,
                    valid=False,
                    reason="no stable wall piece in current scan",
                )
            reacquisition = self._begin_or_continue_reacquisition(
                lane, now, lap_distance, turn, observation)
            if reacquisition is None:
                return None, lane, "wall reacquisition limit exceeded"
            lane, desired = reacquisition
            exploring = True

        dt = 0.05 if self.last_update is None else float(np.clip(
            now - self.last_update, 0.02, 0.20))
        self.last_update = now
        blend = 0.70 if exploring else 0.55
        slew = 0.90 if exploring else 1.20
        target = self.steer + blend * (desired - self.steer)
        self.steer = float(np.clip(target, self.steer - slew * dt,
                                    self.steer + slew * dt))
        self.steer = float(np.clip(self.steer, -MAX_STEER, MAX_LEFT_STEER))

        # Only current returns participate in collision checks; fused history
        # and guessed walls may guide the path but can never declare it clear.
        choices = [self.steer]
        for offset in (0.06, -0.06, 0.12, -0.12, 0.19, -0.19):
            choices.append(float(np.clip(self.steer + offset,
                                         -MAX_STEER, MAX_LEFT_STEER)))
        if exploring:
            choices.extend((0.0, float(np.clip(
                nominal_steering(self.section, WHEELBASE,
                                 self.lap_length_m),
                -MAX_STEER, MAX_LEFT_STEER))))
        horizon = (0.12 if exploring else
                   0.16 if abs(self.steer) > 0.24 else 0.24)
        safe = [candidate for candidate in choices
                if future_footprint_hits(points, candidate,
                                         travel=horizon) < 3]
        if not safe:
            return None, lane, "no clear swept path ahead"
        chosen = min(safe, key=lambda candidate: abs(candidate - self.steer))
        self.steer = chosen

        if exploring:
            speed = 0.22
        elif lane.mode.startswith("single") or current_wall1 != current_wall2:
            speed = 0.34
        else:
            speed = 0.45
        if self.section.startswith("U_") or abs(chosen) > 0.22:
            speed = min(speed, 0.30)
        return (speed, chosen), lane, ""
