#!/usr/bin/env python3
"""Replay a wall-explorer stop/overload NPZ without ROS or motor output."""

import argparse
import collections
import json
import os

import numpy as np

from temporal_scan_v1 import fuse_recent_scans
from wall_lidar_core import ScanPathTracker
from wall_mapper import IncrementalWallMap


def load_frames(path):
    saved = np.load(path, allow_pickle=False)
    if "points" in saved and "offsets" in saved:
        points = np.asarray(saved["points"], dtype=float)
        offsets = np.asarray(saved["offsets"], dtype=int)
        frames = [points[offsets[index]:offsets[index + 1]]
                  for index in range(len(offsets) - 1)]
        count = len(frames)
        times = np.asarray(saved.get("times", np.arange(count) * 0.05),
                           dtype=float)
        distances = np.asarray(saved.get("distances", np.zeros(count)),
                               dtype=float)
        turns = np.asarray(saved.get("turns", np.zeros(count)), dtype=float)
        return frames, times, distances, turns
    if "current_points" in saved:
        return ([np.asarray(saved["current_points"], dtype=float)],
                np.array([0.0]),
                np.array([float(saved.get("odom_distance", 0.0))]),
                np.array([float(saved.get("imu_turn", 0.0))]))
    raise ValueError("NPZ is not a wall-explorer stop or overload dump")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input")
    parser.add_argument("--lap-length-m", type=float, default=208.83)
    parser.add_argument("--lap-offset-m", type=float, default=0.0)
    parser.add_argument("--initial-width-m", type=float, default=2.30)
    parser.add_argument("--min-width-m", type=float, default=1.30)
    parser.add_argument("--max-width-m", type=float, default=3.80)
    parser.add_argument("--output-prefix")
    args = parser.parse_args()

    frames, times, distances, turns = load_frames(args.input)
    prefix = (args.output_prefix or
              os.path.splitext(os.path.abspath(args.input))[0] + "_replay")
    tracker = ScanPathTracker(
        lap_length_m=args.lap_length_m,
        lap_offset_m=args.lap_offset_m,
        initial_width_m=args.initial_width_m,
        min_width_m=args.min_width_m,
        max_width_m=args.max_width_m,
        survey_zigzag=True,
    )
    wall_map = IncrementalWallMap(voxel_m=0.05)
    history = collections.deque(maxlen=6)
    modes, sections, failures = collections.Counter(), collections.Counter(), []
    rows = []
    position = np.zeros(2)
    previous_distance = float(distances[0]) if len(distances) else 0.0
    previous_turn = float(turns[0]) if len(turns) else 0.0

    for index, points in enumerate(frames):
        now = float(times[index])
        distance = float(distances[index])
        turn = float(turns[index])
        travel = max(0.0, distance - previous_distance)
        heading = 0.5 * (previous_turn + turn)
        position += travel * np.array([np.cos(heading), np.sin(heading)])
        planning = fuse_recent_scans(points, history, distance, turn, now)
        command, lane, reason = tracker.update(
            points, now, planning_points=planning,
            lap_distance=distance, turn=turn)
        history.append((points.copy(), now, distance, turn))
        wall_map.update(tracker.last_observation, position, turn,
                        distance, now)
        modes[lane.mode] += 1
        sections[tracker.section] += 1
        if reason:
            failures.append({"frame": index, "reason": reason})
        rows.append({
            "frame": index,
            "time": now,
            "distance": distance,
            "turn": turn,
            "section": tracker.section,
            "mode": lane.mode,
            "lane_valid": bool(lane.valid),
            "width_m": float(lane.width_m),
            "wall1_points": int(lane.wall1_count),
            "wall2_points": int(lane.wall2_count),
            "steering": (float(command[1]) if command is not None else None),
            "speed": (float(command[0]) if command is not None else None),
            "reason": reason,
        })
        previous_distance, previous_turn = distance, turn

    map_path = wall_map.save(prefix + "_map.npz",
                             lap_length_m=np.asarray(args.lap_length_m),
                             estimated_width_m=np.asarray(tracker.width))
    summary = {
        "input": os.path.abspath(args.input),
        "frames": len(frames),
        "modes": dict(modes),
        "sections": dict(sections),
        "failures": failures,
        "final_width_m": tracker.width,
        "width_locked": tracker.width_locked,
        "map_npz": map_path,
        "map_ply": os.path.splitext(map_path)[0] + ".ply",
        "frame_results": rows,
    }
    json_path = prefix + "_summary.json"
    with open(json_path, "w", encoding="utf-8") as stream:
        json.dump(summary, stream, indent=2)
        stream.write("\n")
    print("frames:", len(frames))
    print("modes:", dict(modes))
    print("failures:", len(failures))
    print("summary:", json_path)
    print("colored map:", summary["map_ply"])


if __name__ == "__main__":
    main()
