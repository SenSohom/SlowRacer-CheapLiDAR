#!/usr/bin/env python3
"""Small odometry-frame map of confirmed wall1/wall2 observations."""

import os

import numpy as np


def save_classified_ply(path, points, wall_ids, piece_ids, confidence):
    """Save classified or fault points with colors and metadata."""
    path = os.path.abspath(os.path.expanduser(path))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    points = np.asarray(points, dtype=float)
    if points.size == 0:
        points = np.empty((0, 3), dtype=float)
    elif points.ndim != 2 or points.shape[1] not in (2, 3):
        raise ValueError("classified PLY points must be Nx2 or Nx3")
    elif points.shape[1] == 2:
        points = np.column_stack((points, np.zeros(len(points))))
    wall_ids = np.asarray(wall_ids).reshape(-1)
    piece_ids = np.asarray(piece_ids).reshape(-1)
    confidence = np.asarray(confidence, dtype=float).reshape(-1)
    if not (len(points) == len(wall_ids) == len(piece_ids)
            == len(confidence)):
        raise ValueError("classified PLY arrays must have equal lengths")
    temporary = path + ".tmp"
    with open(temporary, "w", encoding="utf-8") as stream:
        stream.write("ply\nformat ascii 1.0\n")
        stream.write("comment warm=wall1 cool=wall2 gray=unclassified "
                     "red=contact\n")
        stream.write("element vertex %d\n" % len(points))
        stream.write("property float x\nproperty float y\nproperty float z\n")
        stream.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        stream.write("property uchar wall_id\nproperty ushort piece_id\n")
        stream.write("property float confidence\nend_header\n")
        for point, wall_id, piece_id, value in zip(
                points, wall_ids, piece_ids, confidence):
            wall_id = int(wall_id)
            piece_id = int(piece_id)
            if wall_id == 1:
                base = (245, 65 + 18 * (int(piece_id) % 4), 30)
            elif wall_id == 2:
                base = (30, 105 + 20 * (int(piece_id) % 4), 255)
            elif piece_id == 1:
                base = (255, 25, 25)
            else:
                base = (145, 145, 145)
            shade = (1.0 if wall_id == 0 else
                     0.55 + 0.45 * float(np.clip(value, 0.0, 1.0)))
            color = [int(channel * shade) for channel in base]
            stream.write("%.5f %.5f %.5f %d %d %d %d %d %.4f\n" %
                         (point[0], point[1], point[2], color[0], color[1],
                          color[2], wall_id, piece_id, float(value)))
    os.replace(temporary, path)
    return path


class IncrementalWallMap:
    """Voxel-average confirmed classified walls without mixing identities."""

    def __init__(self, voxel_m=0.05, max_cells=120000):
        self.voxel_m = float(voxel_m)
        self.max_cells = int(max_cells)
        self.cells = {}
        self.trajectory = []
        self.last_trajectory_position = None

    def _add(self, points, wall_id, confidence, stamp):
        if not len(points):
            return
        indices = np.floor(points / self.voxel_m).astype(np.int64)
        for point, index in zip(points, indices):
            key = (int(wall_id), int(index[0]), int(index[1]))
            cell = self.cells.get(key)
            if cell is None:
                self.cells[key] = [float(point[0]), float(point[1]), 1,
                                   float(confidence), float(stamp)]
            else:
                cell[0] += float(point[0])
                cell[1] += float(point[1])
                cell[2] += 1
                cell[3] += float(confidence)
                cell[4] = float(stamp)
        if len(self.cells) > self.max_cells:
            # Drop the least recently observed cells. This is infrequent and
            # bounds memory on long Jetson sessions.
            excess = len(self.cells) - self.max_cells
            oldest = sorted(self.cells, key=lambda key: self.cells[key][4])[:excess]
            for key in oldest:
                self.cells.pop(key, None)

    def update(self, observation, position_xy, yaw, distance, stamp):
        if observation is None or position_xy is None or yaw is None:
            return
        position = np.asarray(position_xy, dtype=float).reshape(2)
        cosine, sine = np.cos(yaw), np.sin(yaw)
        rotation = np.array([[cosine, -sine], [sine, cosine]])

        if observation.wall1_confirmed and observation.confidence1 >= 0.65:
            global_points = observation.wall1 @ rotation.T + position
            self._add(global_points, 1, observation.confidence1, stamp)
        if observation.wall2_confirmed and observation.confidence2 >= 0.65:
            global_points = observation.wall2 @ rotation.T + position
            self._add(global_points, 2, observation.confidence2, stamp)

        if (self.last_trajectory_position is None
                or np.linalg.norm(position - self.last_trajectory_position) >= 0.08):
            self.trajectory.append((position[0], position[1], float(yaw),
                                    float(distance), float(stamp)))
            self.last_trajectory_position = position.copy()

    def arrays(self):
        walls = {1: [], 2: []}
        counts = {1: [], 2: []}
        confidence = {1: [], 2: []}
        for (wall_id, _, _), cell in self.cells.items():
            walls[wall_id].append((cell[0] / cell[2], cell[1] / cell[2]))
            counts[wall_id].append(cell[2])
            confidence[wall_id].append(cell[3] / cell[2])
        output = {}
        for wall_id in (1, 2):
            output["wall%d" % wall_id] = np.asarray(
                walls[wall_id], dtype=float).reshape(-1, 2)
            output["wall%d_count" % wall_id] = np.asarray(
                counts[wall_id], dtype=np.int32)
            output["wall%d_confidence" % wall_id] = np.asarray(
                confidence[wall_id], dtype=float)
        output["trajectory"] = np.asarray(
            self.trajectory, dtype=float).reshape(-1, 5)
        return output

    def save(self, path, **metadata):
        """Atomically replace NPZ data and a colored PLY beside it."""
        path = os.path.abspath(os.path.expanduser(path))
        os.makedirs(os.path.dirname(path), exist_ok=True)
        temporary = path + ".tmp.npz"
        payload = self.arrays()
        payload.update(metadata)
        np.savez_compressed(temporary, **payload)
        os.replace(temporary, path)
        root, _ = os.path.splitext(path)
        self.save_colored_ply(root + ".ply", payload)
        return path

    def save_colored_ply(self, path, payload=None):
        """Write wall1 warm/red and wall2 cool/blue for MeshLab/RViz tools."""
        payload = self.arrays() if payload is None else payload
        rows = []
        for wall_id, base_color in ((1, (245, 75, 35)),
                                    (2, (35, 125, 255))):
            points = payload["wall%d" % wall_id]
            counts = payload["wall%d_count" % wall_id]
            confidence = payload["wall%d_confidence" % wall_id]
            for point, count, value in zip(points, counts, confidence):
                brightness = 0.55 + 0.45 * float(np.clip(value, 0.0, 1.0))
                red, green, blue = [int(channel * brightness)
                                    for channel in base_color]
                rows.append((point[0], point[1], 0.0, red, green, blue,
                             wall_id, int(count), float(value)))
        temporary = path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            stream.write("ply\nformat ascii 1.0\n")
            stream.write("comment wall1=outer/right warm wall2=inner/left cool\n")
            stream.write("element vertex %d\n" % len(rows))
            stream.write("property float x\nproperty float y\nproperty float z\n")
            stream.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
            stream.write("property uchar wall_id\nproperty uint observations\n")
            stream.write("property float confidence\nend_header\n")
            for row in rows:
                stream.write("%.5f %.5f %.5f %d %d %d %d %d %.4f\n" % row)
        os.replace(temporary, path)
        return path
