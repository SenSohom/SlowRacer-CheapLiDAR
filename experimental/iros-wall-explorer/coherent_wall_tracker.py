#!/usr/bin/env python3
"""Persistent wall identity over current and motion-corrected LiDAR scans."""

import math
from dataclasses import dataclass

import numpy as np

from iros_track import IROS_MAP_LAP_LENGTH_M, metres_per_reference_foot
from track_autonomy_core import LIDAR_FORWARD_OF_BASE
from wall_finder import WallFinder, WallTracker


@dataclass
class WallPiece:
    piece_id: int
    wall: int
    confidence: float
    points: np.ndarray


@dataclass
class WallObservation:
    section: str
    wall1: np.ndarray
    wall2: np.ndarray
    confidence1: float
    confidence2: float
    fix1: object = None
    fix2: object = None
    pieces: object = None
    current_wall1: object = None
    current_wall2: object = None
    current_confidence1: float = 0.0
    current_confidence2: float = 0.0

    @property
    def wall1_confirmed(self):
        return self.fix1 is not None

    @property
    def wall2_confirmed(self):
        return self.fix2 is not None

    @property
    def current_wall1_visible(self):
        return self.current_wall1 is not None and len(self.current_wall1) > 0

    @property
    def current_wall2_visible(self):
        return self.current_wall2 is not None and len(self.current_wall2) > 0


def polar_scan_from_chassis(points, range_scale=1.0,
                            angular_resolution=math.radians(0.25)):
    """Collapse a 3-D/fused cloud to one nearest 2-D return per bearing.

    ``points`` are XY positions in rear-axle coordinates.  The returned polar
    scan is measured at the LiDAR, with 0 forward and positive angles left.
    Collapsing elevation layers prevents the piece splitter from interpreting
    repeated Unitree rings as separate walls.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if not len(points):
        return []
    forward = points[:, 0] - LIDAR_FORWARD_OF_BASE
    lateral = points[:, 1]
    ranges = np.hypot(forward, lateral)
    angles = np.arctan2(lateral, forward)
    valid = np.isfinite(ranges) & np.isfinite(angles) & (ranges > 0.0)
    ranges, angles = ranges[valid], angles[valid]
    if not len(ranges):
        return []
    bins = np.rint(angles / angular_resolution).astype(np.int64)
    output = []
    for angular_bin in np.unique(bins):
        members = np.flatnonzero(bins == angular_bin)
        member_ranges = ranges[members]
        if len(members) >= 3:
            # A motion-corrected history gives several samples of the same
            # tube. The lower quartile follows its inward/visible surface but
            # does not let one specular short-range glint win the whole bin.
            distance = float(np.quantile(member_ranges, 0.25))
            representative = members[np.argmin(
                np.abs(member_ranges - distance))]
        else:
            representative = members[np.argmin(member_ranges)]
            distance = float(ranges[representative])
        output.append((distance / range_scale,
                       float(angles[representative])))
    return output


def _piece_xy(piece, range_scale):
    if not piece:
        return np.empty((0, 2), dtype=float)
    polar = np.asarray(piece, dtype=float)
    return np.column_stack((
        range_scale * polar[:, 0] * np.cos(polar[:, 1])
        + LIDAR_FORWARD_OF_BASE,
        range_scale * polar[:, 0] * np.sin(polar[:, 1]),
    ))


class CoherentWallIdentifier:
    """Classify scan pieces and confirm each wall across several frames."""

    def __init__(self, lap_length_m=IROS_MAP_LAP_LENGTH_M,
                 max_range_m=3.5, min_piece_confidence=0.55):
        self.range_scale = metres_per_reference_foot(lap_length_m)
        # Keep the validated classifier untouched. Convert metre scans to its
        # reference drawing feet at this boundary, then convert pieces back.
        self.finder = WallFinder(
            units="ft",
            min_range=0.10 / self.range_scale,
            max_range=max_range_m / self.range_scale,
        )
        self.tracker1 = WallTracker(self.finder, which=1)
        self.tracker2 = WallTracker(self.finder, which=2)
        self.min_piece_confidence = float(min_piece_confidence)
        self.last = None

    def _stable_labels(self, labels):
        stable = []
        for piece, wall, confidence in labels:
            xy = _piece_xy(piece, self.range_scale)
            span = (float(np.max(np.linalg.norm(xy - xy[0], axis=1)))
                    if len(xy) > 1 else 0.0)
            if len(piece) >= 3 and span >= 0.12:
                stable.append((piece, wall, confidence))
        return stable

    def _collect(self, labels, which, minimum_confidence):
        accepted = []
        for piece, wall, confidence in labels:
            if wall != which or confidence < minimum_confidence:
                continue
            xy = _piece_xy(piece, self.range_scale)
            # Reject tiny isolated glints. A real tube may be sparse, but a
            # usable wall piece must cover a short physical span; temporal
            # tracking then confirms it over three scans.
            span = (float(np.max(np.linalg.norm(xy - xy[0], axis=1)))
                    if len(xy) > 1 else 0.0)
            if len(piece) < 3 or span < 0.12:
                continue
            accepted.append((piece, confidence, xy))
        if not accepted:
            return np.empty((0, 2), dtype=float), 0.0
        points = np.concatenate([xy for _, _, xy in accepted])
        total = sum(len(piece) for piece, _, _ in accepted)
        confidence = sum(len(piece) * value
                         for piece, value, _ in accepted) / total
        return points, float(confidence)

    def update(self, chassis_xy, section, current_chassis_xy=None):
        scan = polar_scan_from_chassis(chassis_xy, self.range_scale)
        labels = self._stable_labels(
            self.finder.label_scan(scan, section))
        if current_chassis_xy is None:
            current_labels = labels
        else:
            current_scan = polar_scan_from_chassis(
                current_chassis_xy, self.range_scale)
            current_labels = self._stable_labels(
                self.finder.label_scan(current_scan, section))
        fix1 = self.tracker1.update(scan, section, labels)
        fix2 = self.tracker2.update(scan, section, labels)
        wall1, confidence1 = self._collect(
            labels, 1, self.min_piece_confidence)
        wall2, confidence2 = self._collect(
            labels, 2, self.min_piece_confidence)
        current_wall1, current_confidence1 = self._collect(
            current_labels, 1, self.min_piece_confidence)
        current_wall2, current_confidence2 = self._collect(
            current_labels, 2, self.min_piece_confidence)
        pieces = [WallPiece(index, wall, float(confidence),
                            _piece_xy(piece, self.range_scale))
                  for index, (piece, wall, confidence)
                  in enumerate(current_labels)]
        self.last = WallObservation(
            section=section,
            wall1=wall1,
            wall2=wall2,
            confidence1=confidence1,
            confidence2=confidence2,
            fix1=fix1,
            fix2=fix2,
            pieces=pieces,
            current_wall1=current_wall1,
            current_wall2=current_wall2,
            current_confidence1=current_confidence1,
            current_confidence2=current_confidence2,
        )
        return self.last
