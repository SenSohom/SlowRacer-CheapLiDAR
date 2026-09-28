#!/usr/bin/env python3
"""Fuse a few recent scans into the current car frame for wall inference."""

import math

import numpy as np

from live_centerline import chassis_points
from track_autonomy_core import CAR_FORWARD_IN_LIDAR, LIDAR_FORWARD_OF_BASE


def fuse_recent_scans(current, history, distance, turn, now):
    """Return current plus motion-corrected recent static-wall returns.

    History rows are ``(xyz, monotonic_time, forward_distance,
    unwrapped_turn)``.  This fused cloud is for wall inference only.  Collision
    checks must continue to use ``current`` so stale returns cannot hide a new
    obstacle.
    """
    chunks = [np.asarray(current, dtype=float)]
    for old, stamp, past_distance, past_turn in list(history)[-5:]:
        age = now - stamp
        travel = distance - past_distance
        yaw = turn - past_turn
        if (age < 0.03 or age > 0.55 or travel < -0.02 or travel > 0.45
                or abs(yaw) > 0.65):
            continue
        old = np.asarray(old, dtype=float)
        car = chassis_points(old)
        if abs(yaw) < 1e-5:
            shift = np.array([travel, 0.0])
        else:
            radius = travel / yaw
            shift = np.array([
                radius * math.sin(yaw),
                radius * (1.0 - math.cos(yaw)),
            ])
        relative = car - shift
        cosine, sine = math.cos(yaw), math.sin(yaw)
        forward = (cosine * relative[:, 0] + sine * relative[:, 1]
                   - LIDAR_FORWARD_OF_BASE)
        lateral = -sine * relative[:, 0] + cosine * relative[:, 1]
        sensor_cosine = math.cos(CAR_FORWARD_IN_LIDAR)
        sensor_sine = math.sin(CAR_FORWARD_IN_LIDAR)
        lidar_x = sensor_cosine * forward - sensor_sine * lateral
        lidar_y = sensor_sine * forward + sensor_cosine * lateral
        chunks.append(np.column_stack((lidar_x, lidar_y, old[:, 2])))
    return chunks[0] if len(chunks) == 1 else np.concatenate(chunks)
