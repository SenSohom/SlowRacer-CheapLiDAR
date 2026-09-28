#!/usr/bin/env python3
"""Synthetic geometry tests; run before replaying or enabling motors."""

import math
import os
import tempfile
import unittest

import numpy as np

from coherent_wall_tracker import (
    CoherentWallIdentifier,
    polar_scan_from_chassis,
)
from identified_centerline import predict_centerline
from iros_track import (
    IROS_MAP_LAP_LENGTH_M,
    effective_radius_m,
    section_at_m,
)
from track_autonomy_core import CAR_FORWARD_IN_LIDAR, LIDAR_FORWARD_OF_BASE
from wall_lidar_core import ScanPathTracker, lidar_overload_reason
from wall_mapper import IncrementalWallMap, save_classified_ply


def raw_from_chassis(xy):
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    forward = xy[:, 0] - LIDAR_FORWARD_OF_BASE
    lateral = xy[:, 1]
    cosine = math.cos(CAR_FORWARD_IN_LIDAR)
    sine = math.sin(CAR_FORWARD_IN_LIDAR)
    return np.column_stack((
        cosine * forward - sine * lateral,
        sine * forward + cosine * lateral,
        np.zeros(len(xy)),
    ))


def straight_corridor(width=2.0, thick=False):
    forward = np.linspace(0.30, 3.20, 240)
    offsets = (-0.10, -0.04, 0.0, 0.04, 0.10) if thick else (0.0,)
    rows = []
    for offset in offsets:
        rows.append(np.column_stack((forward,
                                     np.full_like(forward,
                                                  -width / 2.0 + offset))))
        rows.append(np.column_stack((forward,
                                     np.full_like(forward,
                                                  width / 2.0 + offset))))
    return np.vstack(rows)


def turn_walls(section, width=2.0):
    radius = effective_radius_m(section)
    sweep = np.linspace(-0.25, 0.95, 420)
    if section in ("left", "U_left"):
        inner = np.column_stack(((radius - width / 2.0) * np.sin(sweep),
                                 radius - (radius - width / 2.0)
                                 * np.cos(sweep)))
        outer = np.column_stack(((radius + width / 2.0) * np.sin(sweep),
                                 radius - (radius + width / 2.0)
                                 * np.cos(sweep)))
        return outer, inner  # wall1, wall2
    inner = np.column_stack(((radius - width / 2.0) * np.sin(sweep),
                             -radius + (radius - width / 2.0)
                             * np.cos(sweep)))
    outer = np.column_stack(((radius + width / 2.0) * np.sin(sweep),
                             -radius + (radius + width / 2.0)
                             * np.cos(sweep)))
    return inner, outer  # wall1, wall2


class WallExplorerTests(unittest.TestCase):
    def confirmed(self, points, section):
        identifier = CoherentWallIdentifier()
        observation = None
        for _ in range(3):
            observation = identifier.update(points, section)
        return observation

    def test_iros_scale_and_section_order(self):
        self.assertAlmostEqual(IROS_MAP_LAP_LENGTH_M, 208.83, places=2)
        self.assertEqual(section_at_m(0.0), "straight")
        self.assertEqual(section_at_m(482.8 * 0.15), "U_left")
        self.assertAlmostEqual(effective_radius_m("U_right"), 2.565,
                               places=3)

    def test_straight_both_walls_and_round_tube_thickness(self):
        points = straight_corridor(width=2.0, thick=True)
        observation = self.confirmed(points, "straight")
        self.assertTrue(observation.wall1_confirmed)
        self.assertTrue(observation.wall2_confirmed)
        lane = predict_centerline(
            raw_from_chassis(points), expected_width=2.30,
            wall_observation=observation, section="straight")
        self.assertTrue(lane.valid, lane.reason)
        self.assertEqual(lane.mode, "paired")
        self.assertAlmostEqual(lane.width_m, 1.90, delta=0.18)
        self.assertLess(abs(lane.steering_rad), 0.05)

    def test_temporal_range_bin_rejects_one_specular_glint(self):
        ranges = np.array([0.45, 1.96, 1.98, 2.00, 2.02, 2.04])
        points = np.column_stack((ranges + LIDAR_FORWARD_OF_BASE,
                                  np.zeros(len(ranges))))
        scan = polar_scan_from_chassis(points)
        self.assertEqual(len(scan), 1)
        self.assertGreater(scan[0][0], 1.90)

    def test_u_left_outer_wall_only_keeps_left_turn(self):
        wall1, _ = turn_walls("U_left")
        observation = self.confirmed(wall1, "U_left")
        self.assertTrue(observation.wall1_confirmed)
        self.assertFalse(observation.wall2_confirmed)
        lane = predict_centerline(
            raw_from_chassis(wall1), expected_width=2.0,
            wall_observation=observation, section="U_left")
        self.assertTrue(lane.valid, lane.reason)
        self.assertEqual(lane.mode, "single_wall1")
        self.assertGreater(lane.steering_rad, 0.03)

    def test_u_right_outer_wall_only_keeps_right_turn(self):
        _, wall2 = turn_walls("U_right")
        observation = self.confirmed(wall2, "U_right")
        self.assertTrue(observation.wall2_confirmed)
        lane = predict_centerline(
            raw_from_chassis(wall2), expected_width=2.0,
            wall_observation=observation, section="U_right")
        self.assertTrue(lane.valid, lane.reason)
        self.assertEqual(lane.mode, "single_wall2")
        self.assertLess(lane.steering_rad, -0.03)

    def test_no_wall_guess_is_slow_and_bounded(self):
        tracker = ScanPathTracker(initial_width_m=2.30,
                                  survey_zigzag=False)
        points = raw_from_chassis(straight_corridor())
        for index in range(4):
            command, lane, reason = tracker.update(
                points, 10.0 + 0.1 * index,
                lap_distance=0.03 * index, turn=0.0)
            self.assertIsNotNone(command, reason)
        empty = np.empty((0, 3))
        command, lane, reason = tracker.update(
            empty, 10.55, lap_distance=0.16, turn=0.01)
        self.assertEqual(reason, "")
        self.assertEqual(command[0], 0.22)
        self.assertTrue(lane.mode.startswith("reacquire"))
        self.assertFalse(lane.valid)
        command, _, reason = tracker.update(
            empty, 12.0, lap_distance=0.60, turn=0.02)
        self.assertIsNone(command)
        self.assertEqual(reason, "wall reacquisition limit exceeded")

    def test_fused_history_cannot_hide_current_wall_loss(self):
        tracker = ScanPathTracker(initial_width_m=2.30,
                                  survey_zigzag=False)
        points = raw_from_chassis(straight_corridor())
        for index in range(4):
            command, lane, reason = tracker.update(
                points, 20.0 + 0.1 * index,
                planning_points=points,
                lap_distance=0.03 * index, turn=0.0)
            self.assertIsNotNone(command, reason)
        command, lane, reason = tracker.update(
            np.empty((0, 3)), 20.45,
            planning_points=points,
            lap_distance=0.15, turn=0.0)
        self.assertEqual(reason, "")
        self.assertEqual(command[0], 0.22)
        self.assertFalse(lane.valid)
        self.assertTrue(lane.mode.startswith("reacquire"))

    def test_one_current_wall_caps_fused_pair_speed(self):
        tracker = ScanPathTracker(initial_width_m=2.30,
                                  survey_zigzag=True)
        corridor = straight_corridor()
        both = raw_from_chassis(corridor)
        wall1_only = raw_from_chassis(corridor[:len(corridor) // 2])
        for index in range(4):
            command, _, reason = tracker.update(
                both, 30.0 + 0.1 * index,
                planning_points=both,
                lap_distance=0.03 * index, turn=0.0)
            self.assertIsNotNone(command, reason)
        command, lane, reason = tracker.update(
            wall1_only, 30.45,
            planning_points=both,
            lap_distance=0.15, turn=0.0)
        self.assertEqual(reason, "")
        self.assertTrue(lane.valid)
        self.assertEqual(command[0], 0.34)

    def test_u_left_blind_guess_keeps_section_handedness(self):
        tracker = ScanPathTracker(initial_width_m=2.30,
                                  survey_zigzag=False)
        wall1, wall2 = turn_walls("U_left")
        points = raw_from_chassis(np.vstack((wall1, wall2)))
        distance = 482.8 * 0.15
        for index in range(4):
            command, _, reason = tracker.update(
                points, 40.0 + 0.1 * index,
                planning_points=points,
                lap_distance=distance + 0.02 * index,
                turn=0.02 * index)
            self.assertIsNotNone(command, reason)
        command, lane, reason = tracker.update(
            np.empty((0, 3)), 40.45,
            planning_points=points,
            lap_distance=distance + 0.10, turn=0.10)
        self.assertEqual(reason, "")
        self.assertEqual(command[0], 0.22)
        self.assertGreater(command[1], 0.0)
        self.assertEqual(lane.mode, "reacquire_U_left")

    def test_map_records_two_colored_wall_identities(self):
        points = straight_corridor()
        observation = self.confirmed(points, "straight")
        wall_map = IncrementalWallMap(voxel_m=0.05)
        wall_map.update(observation, np.array([1.0, 2.0]), 0.0,
                        distance=0.0, stamp=1.0)
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "classified_walls.npz")
            wall_map.save(path)
            self.assertTrue(os.path.exists(path))
            ply = os.path.splitext(path)[0] + ".ply"
            self.assertTrue(os.path.exists(ply))
            with open(ply, "r", encoding="utf-8") as stream:
                text = stream.read()
            self.assertIn("wall1=outer/right", text)
            self.assertIn("property uchar wall_id", text)
            piece_ply = os.path.join(directory, "pieces.ply")
            save_classified_ply(
                piece_ply,
                np.array([[0.0, -1.0], [0.0, 1.0]]),
                np.array([1, 2]), np.array([0, 1]),
                np.array([1.0, 0.8]))
            with open(piece_ply, "r", encoding="utf-8") as stream:
                piece_text = stream.read()
            self.assertIn("property ushort piece_id", piece_text)
            fault_ply = os.path.join(directory, "overload.ply")
            save_classified_ply(
                fault_ply,
                np.array([[0.1, 0.2, 0.05], [0.3, 0.4, 0.06]]),
                np.array([0, 0]), np.array([0, 1]),
                np.array([0.0, 1.0]))
            with open(fault_ply, "r", encoding="utf-8") as stream:
                fault_text = stream.read()
            self.assertIn("gray=unclassified red=contact", fault_text)

    def test_overload_detects_point_flood_and_dense_contact(self):
        flood = np.zeros((101, 3), dtype=float)
        reason = lidar_overload_reason(
            flood, raw_count=101, max_retained_points=100,
            contact_point_limit=80)
        self.assertIn("retained point count", reason)

        reason = lidar_overload_reason(
            np.empty((0, 3)), raw_count=501, max_retained_points=100,
            max_raw_points=500, contact_point_limit=80)
        self.assertIn("driver flood limit", reason)

        chassis_contact = np.tile(np.array([[0.30, 0.20]]), (81, 1))
        contact = raw_from_chassis(chassis_contact)
        reason = lidar_overload_reason(
            contact, raw_count=len(contact), max_retained_points=1000,
            contact_point_limit=80)
        self.assertIn("possible track strike", reason)

        mount_reflection = raw_from_chassis(
            np.tile(np.array([[0.0, 0.0]]), (100, 1)))
        reason = lidar_overload_reason(
            mount_reflection, raw_count=len(mount_reflection),
            max_retained_points=1000, contact_point_limit=80)
        self.assertEqual(reason, "")


if __name__ == "__main__":
    unittest.main(verbosity=2)
