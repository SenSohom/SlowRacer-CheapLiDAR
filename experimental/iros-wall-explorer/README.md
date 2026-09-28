# IROS Wall Explorer — Experimental Car Test Package

This folder is a self-contained, **dry-run-by-default** controller for testing
the section-aware wall classifier and slow wall-exploration planner on the
car. It is separate from the preserved V1/V2 releases.

> This controller has synthetic and offline replay validation only. It has
> not completed a physical lap. Do not add `--drive` until the stationary
> checks below pass with current track data and an operator is holding the
> physical emergency stop.

See [DESIGN.md](DESIGN.md) for the complete algorithm, filtering, wall-color,
mapping, and fault behavior.

## What it does

- Classifies the physical right/outer boundary as wall1 and left/inner
  boundary as wall2 using the supplied `wall_finder.py` calculation.
- Builds a center path from two walls or offsets one confirmed wall by half
  the last reliable width.
- Retains the known left/right or U-turn direction when only one wall remains.
- Uses a slow, distance- and time-bounded dead-reckoning/search path when the
  current scan has no stable wall piece.
- Fuses a short, motion-corrected scan history for inference, while using only
  the current scan for collision checks.
- Filters thick/sparse reflections from round HVAC tubing and requires
  temporal wall confirmation.
- Publishes current understood pieces as a colored point cloud and records a
  colored odometry-frame wall map.
- Immediately stops, brakes, records evidence, reports, and shuts down on a
  point flood or dense near-body/track-contact cluster.

## Included files

| File | Purpose |
|---|---|
| `track_wall_explorer_node.py` | ROS 2 node, safety state machine, recording and publishers |
| `wall_finder.py` | Supplied wall1/wall2 classifier; classification logic unchanged |
| `coherent_wall_tracker.py` | Piece confidence, temporal identity and current-vs-history support |
| `identified_centerline.py` | Two-wall and local-normal one-wall center paths |
| `wall_lidar_core.py` | Steering, U-turn guidance, bounded blind search and overload checks |
| `temporal_scan_v1.py` | Motion-corrected recent-scan fusion |
| `wall_mapper.py` | Classified NPZ and colored PLY map output |
| `track_autonomy_core.py`, `live_centerline.py` | Preserved V1 geometry/helpers required by this package |
| `run_wall_explorer.sh` | Jetson launcher; does not enable motors unless `--drive` is supplied |
| `replay_wall_explorer.py` | Motor-free replay of saved stop/overload NPZ files |
| `test_wall_explorer.py` | Synthetic straight, tube, U-turn, dropout, mapping and fault tests |
| `IROS2026.png`, `IROS2026.yaml` | Track reference/map metadata; not loaded by the controller at runtime |

## Assumed car interfaces

The launcher expects the existing Jetson installation described by this
repository: ROS 2 Humble, Unitree LiDAR workspace, F1TENTH workspace, VESC,
PS4 controller, and these topics:

```text
Input:  /scan (preferred) or /unilidar/cloud
        /unilidar/imu /odom /sensors/core
        /commands/servo/position /joy
Output: /drive /commands/motor/brake
        /wall_explorer/status
        /wall_explorer/classified_points
```

`/scan` must use `0` radians forward and positive angles to the car's left,
matching the projection that worked with `scan_snapshot.py`. The raw-cloud Z
band is provisional, so start with `/scan`.

`run_wall_explorer.sh` sources these existing Jetson paths:

```text
/opt/ros/humble/setup.bash
/home/autoracer/unilidar_sdk/unitree_lidar_ros2/install/setup.bash
/home/autoracer/f1tenth_ws/install/setup.bash
```

Update the launcher only if the car uses different installation paths.

## 1. Get the folder on the Jetson

For an existing checkout:

```bash
cd /home/autoracer/SlowRacer-CheapLiDAR
git pull --ff-only
cd experimental/iros-wall-explorer
```

For a new checkout:

```bash
cd /home/autoracer
git clone https://github.com/SenSohom/SlowRacer-CheapLiDAR.git
cd SlowRacer-CheapLiDAR/experimental/iros-wall-explorer
```

The LiDAR/VESC/odometry/joystick bringup must already be running. Do not run
`joy_teleop` at the same time: its high-priority drive-mux output can conflict
with autonomy.

Check the required streams before launching:

```bash
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /unilidar/imu
ros2 topic hz /sensors/core
ros2 topic hz /joy
```

## 2. Run the motor-free tests

On the development workstation, use the existing `wadman` environment:

```bash
conda run -n wadman python -m unittest -v test_wall_explorer.py
```

On the Jetson, use the ROS Python environment after sourcing the workspaces:

```bash
python3 -m unittest -v test_wall_explorer.py
```

All tests must pass before continuing.

## 3. Stationary dry run and recording

Place the car on the track without enabling motors. The reference start is
the middle of the bottom straight in `IROS2026.png`, pointing right and
driving counter-clockwise.

Run one 60-second dry session. `--remote-run` is safe here because `--drive`
is absent; it exercises the active planner without publishing drive/brake:

```bash
./run_wall_explorer.sh \
  --input scan \
  --remote-run --exit-on-stop --max-seconds 60 \
  --lap-length-m 208.83 \
  --initial-width-m 2.30 --min-width-m 1.30 --max-width-m 3.80
```

In another terminal, watch status and record evidence:

```bash
ros2 topic echo /wall_explorer/status
```

```bash
ros2 bag record /scan /odom /unilidar/imu /sensors/core /joy \
  /wall_explorer/status /wall_explorer/classified_points
```

In RViz, display `/wall_explorer/classified_points` with its RGB transformer:

- red/orange must remain wall1, the physical right/outer wall;
- blue/cyan must remain wall2, the physical left/inner wall;
- an empty current scan publishes an empty cloud instead of leaving a stale
  wall image visible.

The status line includes `points=RAW/RETAINED contact=N`. Record the largest
normal values while the car is stationary and while it is manually carried
through representative straights and U-turns. Tune overload limits with
margin from those measurements; do not lower them blindly.

## 4. Check saved data and replay

Normal map/stop artifacts are written to:

```text
/tmp/iros_wall_map_latest.npz
/tmp/iros_wall_map_latest.ply
/tmp/wall_explorer_last_stop_scans.npz
```

Replay the stop file without ROS or motors:

```bash
conda run -n wadman python replay_wall_explorer.py \
  /tmp/wall_explorer_last_stop_scans.npz
```

On the Jetson, replace the first line with `python3`. Review the generated
JSON summary and colored PLY map. Confirm the section order and steering signs
through every left/right/U-turn before a physical run.

The default `208.83 m` lap length is drawing-derived. Replace it with a
measured centerline lap using `--lap-length-m`; use `--lap-offset-m` if the
physical start is not the reference start.

## 5. First controlled physical test

Only proceed after the dry run, RViz colors, threshold counts, and replay are
credible.

1. Use a clear track with no people in the vehicle envelope.
2. Keep one operator at the car with the physical emergency stop.
3. Keep `joy_teleop` stopped and verify no second `/drive` publisher exists.
4. Do not use `--remote-run` for the first moving test.
5. Start with a short time limit and press PS4 **L1** once the node reports
   ready. PS4 **R1** stops autonomy.

```bash
./run_wall_explorer.sh \
  --drive --input scan --exit-on-stop --max-seconds 15 \
  --lap-length-m 208.83 \
  --initial-width-m 2.30 --min-width-m 1.30 --max-width-m 3.80
```

The package caps positive motor requests at `0.45 m/s`; one-wall and U-turn
planning are slower, and current-wall loss requests `0.22 m/s`. These are
software requests, not verified physical ground speeds.

## LiDAR overload/track-contact shutdown

Defaults:

```text
--max-retained-points 12000
--max-raw-points      96000
--contact-point-limit 80
```

When any limit trips, the node latches the fault, sends zero speed and brake
before file I/O, reports through ROS status and logs, records the original
input plus pose/classification data, holds the brake for one second, and shuts
itself down. Default evidence files are:

```text
/tmp/wall_explorer_overload.npz
/tmp/wall_explorer_overload.ply
/tmp/wall_explorer_overload_classified.ply
/tmp/wall_explorer_overload.txt
```

The trigger PLY colors ordinary points gray and near-body/contact candidates
red. Use `--overload-output /persistent/path/fault.npz` when `/tmp` is not
appropriate.

## Known limitations

- The classifier depends on the correct measured lap length, start offset,
  travel direction, LiDAR mounting convention, and section sequence.
- Drawing-derived radii and synthetic tube tests are not substitutes for a
  recorded physical-track replay.
- The planner performs bounded wall reacquisition, not general obstacle
  avoidance or full SLAM.
- Cloud-mode height bounds require validation on the actual seven-inch LiDAR
  mount.
- Treat every initial moving run as supervised experimental testing.
