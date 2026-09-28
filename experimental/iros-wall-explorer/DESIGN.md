# IROS Section-Aware Wall Explorer

## Status

This is an **experimental, dry-run-by-default variant** beside the preserved
V1 controller. The original `live_centerline.py`, `lidar_only_core.py`,
`track_lidar_only_node.py`, and `run_v1.sh` still match the release archive.
No physical drive validation has been performed for this variant.

## Driving model

The planner uses the validated `wall_finder.py` calculation without changing
its classification logic:

- `wall1`: physical outer/right wall while driving the reference lap.
- `wall2`: physical inner/left wall.
- Straight: identify a piece using signed wall-line geometry.
- Turn/U-turn: identify whether the piece is inside or outside the known turn
  centre and convert that to wall1/wall2.
- Track section comes from normalized lap odometry. The default `208.83 m`
  lap is the brief's `1392.2 ft × 3 px/ft × 0.05 m/px` scale.

The start convention is the centre of the bottom straight, pointing right,
and driving counter-clockwise. Set `--lap-offset-m` if the vehicle starts
elsewhere. Replace `--lap-length-m` with a measured lap when available.

### Both walls visible

1. Split the scan into continuous pieces.
2. Classify each piece as wall1 or wall2 by section-aware majority vote.
3. Confirm and smooth each wall over three scans.
4. Pair forward stations and follow their measured midpoint.
5. Smooth the observed width and keep it within `1.3–3.8 m`.

### One wall visible

1. Require the wall identity to be temporally confirmed.
2. Trace the visible wall continuously through forward stations.
3. Offset it toward the corridor by half the last reliable paired width.
4. Offset along the wall's local normal, not only the car's lateral axis.
5. During U-turns, retain the section's known handedness and nominal
   curvature so an inner/outer wall cannot cause a turn-direction swap.

### No wall visible

The vehicle does not drive indefinitely blind. It:

1. Motion-corrects the last confirmed path using odometry travel and IMU turn.
2. Continues at a `0.22 m/s` planning target.
3. On a straight, adds a small steering sweep to search for tube returns.
4. In a turn/U-turn, holds the known section direction and nominal curvature.
5. Uses **current returns only** for swept-body collision checks.
6. Stops if no wall is reacquired within `1.25 s` or `0.38 m` after a width
   lock. Before the first paired-width lock, limits are `0.45 s` and `0.12 m`.

### Survey zigzag

On paired-wall straight sections only, the planner adds a gentle `5 m`
wavelength lateral survey bias. Its amplitude is clearance-dependent and
capped at `0.18 m`. It is disabled in turns, in one-wall mode, and where the
corridor is too narrow. Use `--no-survey-zigzag` to disable it.

## Circular metal-tube filtering

The HVAC tubes can produce thick arcs, sparse glints, and inconsistent short
clusters. The experimental pipeline therefore:

- Uses the proven `/scan` projection by default.
- Motion-corrects up to five recent scans for wall inference.
- Separately checks current-scan wall support, so fused history cannot delay
  entry into the slow bounded no-wall protocol.
- Collapses elevation/history returns into angular bins.
- Uses a lower range quartile when a bin has repeated support, preventing one
  short-range specular glint from becoming a wall.
- Rejects tiny pieces shorter than `0.12 m`.
- Merges lateral subclusters separated by less than `0.32 m`, then uses the
  tube surface facing the corridor rather than its apparent centre/far side.
- Requires piece confidence plus three-scan wall confirmation.
- Enforces station-to-station continuity so a trace cannot jump onto an
  adjacent hairpin wall.
- Uses fused returns for inference but only the current scan for collision.

## Colored wall data

Live understood pieces are published at:

```text
/wall_explorer/classified_points   sensor_msgs/PointCloud2, frame base_link
```

- wall1/outer/right: warm red/orange.
- wall2/inner/left: cool blue/cyan.
- Shade changes by scan piece and confidence.
- Fields: `x`, `y`, `z`, `rgb`, `wall_id`, `piece_id`, `confidence`.

The global odometry-frame map is checkpointed every 30 seconds and saved on
stop as:

```text
/tmp/iros_wall_map_latest.npz
/tmp/iros_wall_map_latest.ply
```

The PLY is directly viewable in MeshLab and colors the two wall identities.
Use `--map-output PATH.npz` for persistent storage. Record the live piece
topic in a ROS bag when per-frame piece history is needed.

## LiDAR overload/track-strike fault

The node latches a fatal stop when either:

- retained points exceed `--max-retained-points` (default `12000`),
- pre-filter points exceed `--max-raw-points` (default `96000`), or
- near-body returns reach `--contact-point-limit` (default `80`).

It commands zero and brake **before any file I/O**, publishes a fatal status,
then saves:

- `/tmp/wall_explorer_overload.npz`: original LaserScan ranges or pre-filter
  cloud, filtered/base-frame triggering points, contact mask,
  pose/turn/counts, and the last understood wall pieces;
- `/tmp/wall_explorer_overload.ply`: triggering scan in the base frame, with
  ordinary points gray and possible body/track contact points red;
- `/tmp/wall_explorer_overload_classified.ply`: last understood wall pieces,
  retaining the warm wall1 and cool wall2 colors;
- `/tmp/wall_explorer_overload.txt`: persistent fatal reason, counts, and
  artifact paths;
- the current NPZ/colored-PLY global wall map.

It reports the counts and artifact paths on `/wall_explorer/status`, keeps the
brake asserted for one second, then shuts the node down. Set
`--overload-output` to change the dump location. Tune both thresholds from
stationary bags before motors are enabled.

## Files

| File | Role |
|---|---|
| `wall_finder.py` | Supplied and validated wall calculation; logic unchanged |
| `iros_track.py` | ROS-map scale and metre/section adapter |
| `coherent_wall_tracker.py` | Tube filtering, piece labels, confidence tracking |
| `identified_centerline.py` | Both-wall and local-normal one-wall path creation |
| `temporal_scan_v1.py` | Motion-correct recent-scan fusion |
| `wall_lidar_core.py` | Steering, survey weave, no-wall protocol, overload test |
| `wall_mapper.py` | Classified odometry-frame NPZ and colored PLY map |
| `track_wall_explorer_node.py` | Separate ROS node retaining V1 safety/recovery logic |
| `run_wall_explorer.sh` | Dry-run launcher |
| `replay_wall_explorer.py` | ROS-free replay, JSON summary and colored wall map |
| `test_wall_explorer.py` | Synthetic straight, tube, U-turn, dropout, map and fault tests |

## Validation and tuning sequence

Run pure geometry tests:

```bash
conda run -n wadman python -m unittest -v test_wall_explorer.py
```

`wadman` is the offline validation environment on this workstation. The
Jetson launcher still uses the ROS 2 Python environment sourced by
`run_wall_explorer.sh` so that `rclpy` and the ROS message packages resolve.

Run stationary/dry on the Jetson using the working `/scan` topic:

```bash
./run_wall_explorer.sh --input scan \
  --lap-length-m 208.83 --initial-width-m 2.30
```

In RViz, display `/wall_explorer/classified_points` using the RGB color
transform. Confirm red points remain wall1 and blue points remain wall2 through
all five U-turns. Also confirm tube glints do not create distant wall jumps.

Record tuning evidence:

```bash
ros2 bag record /scan /odom /unilidar/imu /sensors/core \
  /wall_explorer/status /wall_explorer/classified_points
```

Replay a saved stop/overload NPZ without motors or ROS:

```bash
conda run -n wadman python replay_wall_explorer.py \
  /tmp/wall_explorer_last_stop_scans.npz
```

This writes a per-frame JSON report plus a classified NPZ/colored PLY map.

Only after stationary and bag replay validation should an operator explicitly
add `--drive`. Begin at low speed with a physical emergency stop and direct
supervision. The drawing-derived sections/radii and all synthetic tests remain
approximations until real competition-track data validates them.
