#!/usr/bin/env python3
"""Scale the validated drawing-space wall classifier to the ROS IROS map."""

import math

from wall_finder import LAP_LEN, R_EFF, section_at


MAP_RESOLUTION_M_PER_PIXEL = 0.05
REFERENCE_PIXELS_PER_FOOT = 3.0
METRES_PER_REFERENCE_FOOT = (
    MAP_RESOLUTION_M_PER_PIXEL * REFERENCE_PIXELS_PER_FOOT
)
IROS_MAP_LAP_LENGTH_M = LAP_LEN * METRES_PER_REFERENCE_FOOT


def metres_per_reference_foot(lap_length_m=IROS_MAP_LAP_LENGTH_M):
    """Scale drawing feet to metres using a measured/configured lap length."""
    if lap_length_m <= 0.0:
        raise ValueError("lap_length_m must be positive")
    return float(lap_length_m) / LAP_LEN


def reference_distance(distance_m, lap_length_m=IROS_MAP_LAP_LENGTH_M):
    return float(distance_m) / metres_per_reference_foot(lap_length_m)


def section_at_m(distance_m, lap_length_m=IROS_MAP_LAP_LENGTH_M,
                 offset_m=0.0):
    """Section lookup for metre odometry using the validated normalized lap."""
    scale = metres_per_reference_foot(lap_length_m)
    # Decimal map boundaries such as 482.8 ft should not remain in the prior
    # section because their metre conversion lands one floating ULP below it.
    reference_position = round(
        (float(distance_m) + float(offset_m)) / scale, 9)
    return section_at(reference_position,
                      lap_len=LAP_LEN)


def turn_sign(section):
    if section in ("left", "U_left"):
        return 1.0
    if section in ("right", "U_right"):
        return -1.0
    return 0.0


def effective_radius_m(section, lap_length_m=IROS_MAP_LAP_LENGTH_M):
    if section == "straight":
        return float("inf")
    return R_EFF[section] * metres_per_reference_foot(lap_length_m)


def nominal_steering(section, wheelbase,
                     lap_length_m=IROS_MAP_LAP_LENGTH_M):
    sign = turn_sign(section)
    if sign == 0.0:
        return 0.0
    return sign * math.atan(float(wheelbase)
                            / effective_radius_m(section, lap_length_m))
