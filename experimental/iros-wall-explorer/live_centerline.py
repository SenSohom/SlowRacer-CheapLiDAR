#!/usr/bin/env python3
"""Extract and project the road center from a single low-wall LiDAR cloud.

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


def chassis_points(lidar_points):
    """Convert Unitree LiDAR XYZ into x/y from the car's rear axle."""
    points = np.asarray(lidar_points, dtype=float)
    cosine = math.cos(CAR_FORWARD_IN_LIDAR)
    sine = math.sin(CAR_FORWARD_IN_LIDAR)
    forward = points[:, 0] * cosine + points[:, 1] * sine
    left = -points[:, 0] * sine + points[:, 1] * cosine
    return np.column_stack((forward + LIDAR_FORWARD_OF_BASE, left))


def _empty(reason):
    blank = np.empty((0, 2))
    return LanePrediction(False, blank, blank, blank, blank,
                          float("nan"), 0.0, 0.0, reason)


def _trace_wall(forward, lateral, side):
    """Track one physical wall as a bend carries it across the car's x axis."""
    rows = []
    previous = None
    missed = 0
    for station in np.arange(0.02, 1.03, 0.06):
        values = lateral[np.abs(forward - station) < 0.052]
        # A trace must stay on the nearby track wall. Distant room returns
        # cannot establish the side of the lane.
        values = values[np.abs(values) < 1.10]
        if previous is None:
            values = values[(values > 0.16) if side > 0 else (values < -0.16)]
            if len(values) < 2:
                continue
            edge = float(np.quantile(values, 0.20 if side > 0 else 0.80))
        else:
            slope = ((rows[-1][1] - rows[-2][1]) /
                     (rows[-1][0] - rows[-2][0])) if len(rows) > 1 else 0.0
            stride = station + LIDAR_FORWARD_OF_BASE - rows[-1][0]
            predicted = previous + float(np.clip(slope * stride, -0.16, 0.16))
            values = values[np.abs(values - predicted) < 0.18]
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


def predict_centerline(lidar_points, lookahead_m=0.45, expected_width=0.77):
    points = chassis_points(lidar_points)
    if len(points) < 80:
        return _empty("too few low-wall returns")
    forward = points[:, 0] - LIDAR_FORWARD_OF_BASE
    left = points[:, 1]
    # Exclude the repeatable narrow reflection behind the LiDAR. A real wall
    # entering the front footprint is not removed here.
    self_return = ((forward > -0.48) & (forward < -0.15)
                   & (np.abs(left) < 0.06))
    points = points[~self_return]
    forward = forward[~self_return]
    left = left[~self_return]

    left_rows, right_rows, centers, widths, weights = [], [], [], [], []
    upper_rows, lower_rows = [], []
    for station in np.arange(0.02, 0.91, 0.08):
        nearby = np.abs(forward - station) < 0.075
        left_side = left[nearby & (left > 0.16) & (left < 1.10)]
        right_side = left[nearby & (left < -0.16) & (left > -1.10)]
        if len(left_side) >= 2:
            upper_rows.append((station + LIDAR_FORWARD_OF_BASE,
                               float(np.quantile(left_side, 0.20))))
        if len(right_side) >= 2:
            lower_rows.append((station + LIDAR_FORWARD_OF_BASE,
                               float(np.quantile(right_side, 0.80))))
        if len(left_side) < 2 or len(right_side) < 2:
            continue
        left_edge = float(np.quantile(left_side, 0.20))
        right_edge = float(np.quantile(right_side, 0.80))
        width = left_edge - right_edge
        if not 0.43 <= width <= 1.15:
            continue
        left_rows.append((station + LIDAR_FORWARD_OF_BASE, left_edge))
        right_rows.append((station + LIDAR_FORWARD_OF_BASE, right_edge))
        centers.append((station + LIDAR_FORWARD_OF_BASE,
                        (left_edge + right_edge) / 2))
        widths.append(width)
        weights.append(min(len(left_side), len(right_side), 12))

    paired = (len(centers) >= 5 and
              centers[-1][0] - LIDAR_FORWARD_OF_BASE >= 0.25)
    upper_rows = _trace_wall(forward, left, 1)
    lower_rows = _trace_wall(forward, left, -1)
    left_rows = np.asarray(left_rows).reshape(-1, 2)
    right_rows = np.asarray(right_rows).reshape(-1, 2)
    # A slice at fixed car-forward x spans more than the true track width
    # through a diagonal corner. Keep a fixed nominal width for one-wall
    # continuation rather than letting those slices widen the inferred lane.
    observed_width = float(np.median(widths)) if paired else float("nan")
    width = float(expected_width)
    if paired and observed_width < CAR_WIDTH + 0.10:
        return _empty("visible lane is narrower than the car margin")

    if paired:
        centers = np.asarray(centers)
        inferred_centers = np.empty((0, 2))
        fit_rows = centers
        fit_weights = np.asarray(weights, dtype=float)
        mode = "paired"
        observed_ahead = float(centers[-1, 0] - LIDAR_FORWARD_OF_BASE)
        # A wall often continues around a corner after its opposite wall
        # disappears. Extend the measured paired center by half the measured
        # lane width, so the turn is visible before the car reaches it.
        extensions = []
        for edge, sign, name in ((upper_rows, -1, "paired_plus_left"),
                                 (lower_rows, 1, "paired_plus_right")):
            continuation = edge[edge[:, 0] > centers[-1, 0] + 0.04]
            if len(continuation) < 3 or continuation[-1, 0] < centers[-1, 0] + 0.16:
                continue
            proposed = continuation.copy()
            proposed[:, 1] += sign * width * 0.5
            if abs(proposed[0, 1] - centers[-1, 1]) > 0.16:
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
        for rows, sign, name in ((upper_rows, -1, "single_left"),
                                 (lower_rows, 1, "single_right")):
            if len(rows) < 5 or rows[-1, 0] - LIDAR_FORWARD_OF_BASE < 0.36:
                continue
            jumps = np.abs(np.diff(rows[:, 1]))
            if np.count_nonzero(jumps > 0.23) > 1:
                continue
            score = rows[-1, 0] + 0.025 * len(rows)
            choices.append((score, rows, sign, name))
        if not choices:
            return _empty("no forward wall trace beyond the corner")
        _, edge, sign, mode = max(choices, key=lambda item: item[0])
        centers = edge.copy()
        centers[:, 1] += sign * width * 0.5
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
        join_x = 0.45
        future_x = np.arange(0.0, 0.751, 0.05)
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
        near_left = left[near_slice & (left > 0.16) & (left < 1.10)]
        near_right = left[near_slice & (left < -0.16) & (left > -1.10)]
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
        return LanePrediction(True, predicted, centers, left_rows, right_rows,
                              width, observed_ahead, steering, "", mode,
                              inferred_centers)
    # Look only a short distance past currently supported wall geometry.
    future_end = min(0.90, max(0.62, fit_rows[-1, 0] + 0.05))
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
    future_y = np.clip(future_y, -0.80, 0.80)
    predicted = np.column_stack((future_x, future_y))

    turning_ahead = (mode.startswith("single") and abs(far_delta) > 0.15)
    if near_error > 0.075 and not turning_ahead:
        lookahead_m = min(lookahead_m, 0.32)
    elif mode.startswith("paired_plus"):
        lookahead_m = max(lookahead_m, 0.55)
    elif turning_ahead:
        # Follow the visible bend ahead even if the near wall center is
        # offset: steering toward that offset first delays a tight turn.
        lookahead_m = max(lookahead_m, 0.65)
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
        preview_x = min(0.65, max(0.50, observed_ahead * 0.70))
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
    return LanePrediction(True, predicted, centers, left_rows, right_rows,
                          width, observed_ahead, steering, "", mode,
                          inferred_centers)
