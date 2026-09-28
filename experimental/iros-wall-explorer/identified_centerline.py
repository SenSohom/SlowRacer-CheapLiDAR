#!/usr/bin/env python3
"""Extract and project a section-identified road center from a single low-wall LiDAR cloud.

Coordinates in the returned arrays are car coordinates with the rear axle at
(0, 0): x forward and y left. No offline map or ROS dependency is required.
"""

import math
from dataclasses import dataclass

import numpy as np

from track_autonomy_core import (
    CAR_FORWARD_IN_LIDAR, CAR_WIDTH, LIDAR_FORWARD_OF_BASE,
    MAX_STEER, WHEELBASE,
)


@dataclass
class LanePrediction:
    valid: bool
    centerline: np.ndarray
    observed_centers: np.ndarray
    left_wall: np.ndarray
    right_wall: np.ndarray
    width_m: float
    observed_ahead_m: float
    steering_rad: float
    reason: str
    mode: str = "none"
    inferred_centers: np.ndarray = None
    section: str = "unknown"
    wall1_count: int = 0
    wall2_count: int = 0
    wall1_confidence: float = 0.0
    wall2_confidence: float = 0.0


def chassis_points(lidar_points):
    """Convert Unitree LiDAR XYZ into x/y from the car's rear axle."""
    points = np.asarray(lidar_points, dtype=float)
    cosine = math.cos(CAR_FORWARD_IN_LIDAR)
    sine = math.sin(CAR_FORWARD_IN_LIDAR)
    forward = points[:, 0] * cosine + points[:, 1] * sine
    left = -points[:, 0] * sine + points[:, 1] * cosine
    return np.column_stack((forward + LIDAR_FORWARD_OF_BASE, left))


def _empty(reason, section="unknown", mode="none"):
    blank = np.empty((0, 2))
    return LanePrediction(False, blank, blank, blank, blank,
                          float("nan"), 0.0, 0.0, reason, mode, blank,
                          section)


def _trace_wall(forward, lateral, side, max_lateral=2.20,
                max_ahead=2.05):
    """Track one physical wall as a bend carries it across the car's x axis."""
    rows = []
    previous = None
    missed = 0
    for station in np.arange(0.02, max_ahead + 0.001, 0.08):
        values = lateral[np.abs(forward - station) < 0.065]
        # A trace must stay on the nearby track wall. Distant room returns
        # cannot establish the side of the lane.
        values = values[np.abs(values) < max_lateral]
        if previous is None:
            values = values[(values > 0.16) if side > 0 else (values < -0.16)]
            if len(values) < 2:
                continue
            edge = float(np.quantile(values, 0.20 if side > 0 else 0.80))
        else:
            slope = ((rows[-1][1] - rows[-2][1]) /
                     (rows[-1][0] - rows[-2][0])) if len(rows) > 1 else 0.0
            stride = station + LIDAR_FORWARD_OF_BASE - rows[-1][0]
            predicted = previous + float(np.clip(slope * stride, -0.24, 0.24))
            values = values[np.abs(values - predicted) < 0.30]
            if len(values) < 2:
                missed += 1
                # Never bridge a long blind gap onto the opposite wall after
                # it has wrapped around a corner.
                if missed > 3:
                    break
                continue
            edge = float(np.median(values))
        rows.append((station + LIDAR_FORWARD_OF_BASE, edge))
        previous = edge
        missed = 0
    return np.asarray(rows).reshape(-1, 2)


def _clusters(values, gap=0.32):
    """Return separated lateral groups, merging one round tube's thick arc."""
    values = np.sort(np.asarray(values, dtype=float))
    if not len(values):
        return []
    splits = np.flatnonzero(np.diff(values) > gap) + 1
    return np.split(values, splits)


def _trace_identified_wall(points, wall_id, expected_width,
                           max_lateral=2.20, max_ahead=2.05):
    """Trace one already-identified physical wall through forward stations."""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if not len(points):
        return np.empty((0, 2))
    forward = points[:, 0] - LIDAR_FORWARD_OF_BASE
    lateral = points[:, 1]
    expected = (-0.5 * expected_width if wall_id == 1
                else 0.5 * expected_width)
    rows, previous, missed = [], None, 0
    for station in np.arange(0.02, max_ahead + 0.001, 0.08):
        values = lateral[(np.abs(forward - station) < 0.075)
                         & (np.abs(lateral) < max_lateral)]
        candidates = []
        for group in _clusters(values):
            # Use the surface facing the drivable corridor, not the middle of
            # a circular tube or a specular return on its far side.
            edge = float(np.quantile(group, 0.75 if wall_id == 1 else 0.25))
            candidates.append((edge, len(group)))
        if previous is None:
            if not candidates:
                continue
            centre, _ = min(candidates,
                            key=lambda item: abs(item[0] - expected)
                            - 0.015 * min(item[1], 10))
            if abs(centre - expected) > 0.95:
                continue
        else:
            slope = ((rows[-1][1] - rows[-2][1]) /
                     (rows[-1][0] - rows[-2][0])) if len(rows) > 1 else 0.0
            stride = station + LIDAR_FORWARD_OF_BASE - rows[-1][0]
            predicted = previous + float(np.clip(slope * stride, -0.28, 0.28))
            if not candidates:
                missed += 1
                if missed > 4:
                    break
                continue
            centre, _ = min(candidates,
                            key=lambda item: abs(item[0] - predicted)
                            - 0.015 * min(item[1], 10))
            if abs(centre - predicted) > 0.38:
                missed += 1
                if missed > 4:
                    break
                continue
        # A single LaserScan beam may be all that survives on a reflective
        # tube. Temporal confirmation is handled by CoherentWallIdentifier.
        rows.append((station + LIDAR_FORWARD_OF_BASE, centre))
        previous, missed = centre, 0
    return np.asarray(rows, dtype=float).reshape(-1, 2)


def _offset_wall(rows, wall_id, distance):
    """Offset a forward wall trace along its local normal toward lane centre."""
    rows = np.asarray(rows, dtype=float).reshape(-1, 2)
    if not len(rows):
        return rows.copy()
    slopes = (np.zeros(1) if len(rows) == 1
              else np.gradient(rows[:, 1], rows[:, 0]))
    norm = np.sqrt(1.0 + slopes * slopes)
    # For a forward-directed tangent, (-slope, +1) is the left normal.
    direction = 1.0 if wall_id == 1 else -1.0
    output = rows.copy()
    output[:, 0] += direction * distance * (-slopes / norm)
    output[:, 1] += direction * distance * (1.0 / norm)
    return output[np.argsort(output[:, 0])]


def predict_centerline(lidar_points, lookahead_m=0.70, expected_width=2.30,
                       min_width=1.30, max_width=3.80,
                       wall_observation=None, section="straight"):
    points = chassis_points(lidar_points)
    if len(points) < 12:
        return _empty("too few wall returns", section)
    forward = points[:, 0] - LIDAR_FORWARD_OF_BASE
    left = points[:, 1]
    # Exclude the repeatable narrow reflection behind the LiDAR. A real wall
    # entering the front footprint is not removed here.
    self_return = ((forward > -0.48) & (forward < -0.15)
                   & (np.abs(left) < 0.06))
    points = points[~self_return]
    forward = forward[~self_return]
    left = left[~self_return]
    if wall_observation is not None:
        # wall1 is the physical right/outer boundary; wall2 is left/inner.
        # Identity therefore survives a U-turn even when a distant wall
        # crosses the car's instantaneous x/y axes.
        lower_rows = _trace_identified_wall(
            wall_observation.wall1, 1, expected_width)
        upper_rows = _trace_identified_wall(
            wall_observation.wall2, 2, expected_width)
        wall1_count = len(wall_observation.wall1)
        wall2_count = len(wall_observation.wall2)
        confidence1 = wall_observation.confidence1
        confidence2 = wall_observation.confidence2
    else:
        # Compatibility fallback when no section-aware identifier is passed.
        upper_rows = _trace_wall(forward, left, 1)
        lower_rows = _trace_wall(forward, left, -1)
        wall1_count = int(np.count_nonzero(left < -0.16))
        wall2_count = int(np.count_nonzero(left > 0.16))
        confidence1 = confidence2 = 0.0

    left_by_x = {round(row[0], 3): row[1] for row in upper_rows}
    right_by_x = {round(row[0], 3): row[1] for row in lower_rows}
    centers, widths, weights, left_rows, right_rows = [], [], [], [], []
    for key in sorted(set(left_by_x) & set(right_by_x)):
        left_edge, right_edge = left_by_x[key], right_by_x[key]
        width_at_station = left_edge - right_edge
        if not min_width <= width_at_station <= max_width:
            continue
        centers.append((key, (left_edge + right_edge) / 2.0))
        widths.append(width_at_station)
        left_rows.append((key, left_edge))
        right_rows.append((key, right_edge))
        weights.append(6.0)

    paired = (len(centers) >= 4
              and centers[-1][0] - centers[0][0] >= 0.30)
    left_rows = np.asarray(left_rows, dtype=float).reshape(-1, 2)
    right_rows = np.asarray(right_rows, dtype=float).reshape(-1, 2)
    observed_width = float(np.median(widths)) if paired else float("nan")
    width = float(np.clip(observed_width if paired else expected_width,
                          min_width, max_width))
    if paired and observed_width < CAR_WIDTH + 0.10:
        return _empty("visible lane is narrower than the car margin", section)

    if paired:
        centers = np.asarray(centers, dtype=float)
        inferred_centers = np.empty((0, 2))
        fit_rows = centers
        fit_weights = np.asarray(weights, dtype=float)
        mode = "paired"
        observed_ahead = float(centers[-1, 0] - LIDAR_FORWARD_OF_BASE)
        # A wall often continues around a corner after its opposite wall
        # disappears. Extend the measured paired center by half the measured
        # lane width, so the turn is visible before the car reaches it.
        extensions = []
        for edge, wall_id, name in (
                (upper_rows, 2, "paired_plus_wall2"),
                (lower_rows, 1, "paired_plus_wall1")):
            continuation = edge[edge[:, 0] > centers[-1, 0] + 0.04]
            if len(continuation) < 3 or continuation[-1, 0] < centers[-1, 0] + 0.16:
                continue
            proposed = _offset_wall(continuation, wall_id, width * 0.5)
            proposed = proposed[proposed[:, 0] > centers[-1, 0] - 0.02]
            if (len(proposed) < 3
                    or abs(proposed[0, 1] - centers[-1, 1]) > 0.35):
                continue
            extensions.append((proposed[-1, 0], proposed, name))
        if extensions:
            _, extension, mode = max(extensions, key=lambda item: item[0])
            inferred_centers = extension
            fit_rows = np.vstack((centers, extension))
            fit_weights = np.r_[fit_weights, np.full(len(extension), 3.0)]
            observed_ahead = float(extension[-1, 0] - LIDAR_FORWARD_OF_BASE)
    else:
        # At a corner the far wall can leave the forward view. Continue the
        # longer continuous wall inward by half of the recently seen width.
        # This is a local measurement, never an offline-map lookup.
        choices = []
        for rows, wall_id, name, confirmed in (
                (upper_rows, 2, "single_wall2",
                 wall_observation is None
                 or wall_observation.wall2_confirmed),
                (lower_rows, 1, "single_wall1",
                 wall_observation is None
                 or wall_observation.wall1_confirmed)):
            if not confirmed:
                continue
            if len(rows) < 5 or rows[-1, 0] - LIDAR_FORWARD_OF_BASE < 0.36:
                continue
            jumps = np.abs(np.diff(rows[:, 1]))
            if np.count_nonzero(jumps > 0.38) > 1:
                continue
            score = rows[-1, 0] + 0.025 * len(rows)
            choices.append((score, rows, wall_id, name))
        if not choices:
            return _empty("no confirmed forward wall trace", section)
        _, edge, wall_id, mode = max(choices, key=lambda item: item[0])
        centers = _offset_wall(edge, wall_id, width * 0.5)
        centers = centers[centers[:, 0] > 0.03]
        if len(centers) < 4:
            return _empty("wall offset has insufficient forward support",
                          section)
        inferred_centers = centers
        fit_rows = centers
        fit_weights = np.ones(len(centers))
        observed_ahead = float(edge[-1, 0] - LIDAR_FORWARD_OF_BASE)

    # A quadratic captures the near bend without forcing a fresh steering
    # decision every scan. Weighted rows favour portions seen on both sides.
    x = fit_rows[:, 0]
    fit = np.polyfit(x, fit_rows[:, 1], deg=2,
                     w=np.sqrt(fit_weights))
    polynomial = np.poly1d(fit)
    derivative = np.polyder(polynomial)
    far_delta = float(fit_rows[-1, 1] - fit_rows[0, 1])
    observed_line = np.poly1d(np.polyfit(centers[:, 0], centers[:, 1], 1))
    signed_bend = float(np.polyfit(centers[:, 0], centers[:, 1], 2)[0])
    observed_bend = abs(signed_bend)
    extension_bend = (abs(float(inferred_centers[-1, 1]
                                - observed_line(inferred_centers[-1, 0])))
                      if mode.startswith("paired_plus") and len(inferred_centers)
                      else 0.0)
    straight_like = (observed_ahead >= 0.32 and
                     observed_bend < 0.80 and extension_bend < 0.30 and
                     abs(far_delta) < 0.35)
    if straight_like:
        # A short diagonal slice of straight corridor can put the midpoint
        # off to one side. Pure pursuit at 0.45 m then demands full lock,
        # even though the measured walls show almost no curvature. Use the
        # measured line heading and lateral error with gentle gains instead.
        line = observed_line
        join_x = 0.55
        future_x = np.arange(0.0, 1.201, 0.05)
        future_y = line(future_x)
        joining = future_x <= join_x
        t = future_x[joining] / join_x
        future_y[joining] = ((-2*t**3 + 3*t**2) * float(line(join_x))
                             + (t**3 - t**2) * join_x * float(line.c[0]))
        predicted = np.column_stack((future_x, future_y))
        steering = (0.18 * math.atan(float(line.c[0]))
                    + 0.20 * float(line(0.42)))
        # A small heading gain is useful on an ordinary straight, but it is
        # too weak when the car is already beside a wall. Use the measured
        # near-side body clearance to move away before entering the bend.
        near_slice = (forward >= 0.02) & (forward <= 0.35)
        near_left = left[near_slice & (left > 0.16) & (left < 2.20)]
        near_right = left[near_slice & (left < -0.16) & (left > -2.20)]
        left_gap = (float(np.quantile(near_left, 0.10)) - CAR_WIDTH * 0.5
                    if len(near_left) >= 8 else float("inf"))
        right_gap = (-float(np.quantile(near_right, 0.90)) - CAR_WIDTH * 0.5
                     if len(near_right) >= 8 else float("inf"))
        if left_gap < 0.18 and (not np.isfinite(right_gap)
                                or right_gap > left_gap + 0.10):
            steering -= min(0.22, 2.0 * (0.18 - left_gap))
        elif right_gap < 0.18 and (not np.isfinite(left_gap)
                                    or left_gap > right_gap + 0.10):
            steering += min(0.22, 2.0 * (0.18 - right_gap))
        # The far end of a one-wall trace shortens as the corner wraps away
        # from the scanner. Its measured curvature is visible before the
        # straight/corner classifier changes mode. Start that turn while the
        # bend is still ahead of the front bumper, then let the normal tight
        # corner branch take over. The short trace alone is noisy, so the
        # tracker also takes a three-scan median of single-wall requests.
        if (mode.startswith("single") and observed_ahead < 0.68
                and observed_bend > 0.45 and abs(far_delta) > 0.12
                and signed_bend * far_delta > 0):
            proximity = float(np.clip((0.68 - observed_ahead) / 0.14,
                                      0.0, 1.0))
            bend_steer = math.atan(2.0 * WHEELBASE * signed_bend)
            bend_steer = float(np.clip(bend_steer, -0.35, 0.25))
            steering += proximity * bend_steer
            predicted[:, 1] += (proximity * signed_bend
                                * np.maximum(future_x - 0.25, 0.0)**2)
        steering = float(np.clip(steering, -MAX_STEER, 0.26))
        return LanePrediction(True, predicted, centers, upper_rows, lower_rows,
                              width, observed_ahead, steering, "", mode,
                              inferred_centers, section, wall1_count,
                              wall2_count, confidence1, confidence2)
    # Look only a short distance past currently supported wall geometry.
    future_end = min(1.60, max(0.75, fit_rows[-1, 0] + 0.08))
    future_x = np.arange(0.0, future_end + 0.001, 0.05)
    future_y = polynomial(future_x)

    # When the nearby measured center is already far from the car, the old
    # 0.48 m join delayed the correction until the front reached the wall.
    # Keep that longer, smooth join only while the car is near lane center.
    # Use actual nearby wall measurements here. A global quadratic over a
    # tight corner can invent a lateral offset at the car despite the local
    # wall points showing the car centered.
    near_error = abs(float(np.median(centers[:min(3, len(centers)), 1])))
    join_x = min(0.48, max(0.29, observed_ahead - 0.04))
    if near_error > 0.075:
        join_x = min(join_x, 0.29)
    target_y = float(polynomial(join_x))
    target_slope = float(derivative(join_x))
    joining = future_x <= join_x
    t = future_x[joining] / join_x
    future_y[joining] = ((-2*t**3 + 3*t**2) * target_y
                         + (t**3 - t**2) * join_x * target_slope)
    if (mode.startswith("single") and near_error <= 0.075
            and abs(far_delta) > 0.15):
        # A global quadratic can bulge briefly *opposite* a one-wall bend.
        # The wall trace already shows which way this corner turns.
        future_y = (np.minimum.accumulate(future_y) if far_delta < 0
                    else np.maximum.accumulate(future_y))
    future_y = np.clip(future_y, -2.00, 2.00)
    predicted = np.column_stack((future_x, future_y))

    turning_ahead = (mode.startswith("single") and abs(far_delta) > 0.15)
    if near_error > 0.075 and not turning_ahead:
        lookahead_m = min(lookahead_m, 0.45)
    elif mode.startswith("paired_plus"):
        lookahead_m = max(lookahead_m, 0.80)
    elif turning_ahead:
        # Follow the visible bend ahead even if the near wall center is
        # offset: steering toward that offset first delays a tight turn.
        lookahead_m = max(lookahead_m, 0.85)
    target = predicted[np.argmin(np.abs(predicted[:, 0] - lookahead_m))]
    alpha = math.atan2(target[1], target[0])
    steering = math.atan2(2 * WHEELBASE * math.sin(alpha),
                          max(float(np.linalg.norm(target)), 0.05))
    use_preview = ((mode.startswith("paired_plus") and near_error <= 0.075)
                   or turning_ahead)
    if use_preview:
        # Pure pursuit alone waits until the far bend reaches its lookahead
        # point. Preview the inferred centerline curvature while the car is
        # still on the straight approaching that bend.
        preview_x = min(1.00, max(0.60, observed_ahead * 0.70))
        slope = float(derivative(preview_x))
        curvature = float(np.polyder(polynomial, 2)(preview_x)) / (
            1.0 + slope * slope) ** 1.5
        feedforward = math.atan(WHEELBASE * curvature)
        if feedforward * far_delta > 0:
            weight = 0.65 if mode.startswith("single") else 0.45
            steering = (1.0 - weight) * steering + weight * feedforward
    # VESC's servo_min=0.10, gain=-1.2135 and offset=0.42 leave only
    # 0.264 rad of physical left steering. Keep prediction reachable.
    steering = float(np.clip(steering, -MAX_STEER, 0.26))
    return LanePrediction(True, predicted, centers, upper_rows, lower_rows,
                          width, observed_ahead, steering, "", mode,
                          inferred_centers, section, wall1_count, wall2_count,
                          confidence1, confidence2)
