# V1 Changes for the IROS 2026 Track

## Main finding

The LiDAR was probably seeing the HVAC-tube walls. V1 rejected most of those returns because its filters and lane-width assumptions were designed for the old, approximately `0.77–0.84 m` wide track.

`scan_snapshot.py` displayed the walls because it:

- Reads `/scan` instead of the raw `/unilidar/cloud` point cloud.
- Keeps nearly every finite return beyond `0.05 m`.
- Does not apply V1's height, lateral-distance, lane-width, station-support, or minimum-frame-point checks.
- Accumulates scans for approximately five seconds, making sparse tube returns much easier to see.

Seeing walls in `scan_snapshot.py` therefore confirms useful LiDAR returns exist; it does not confirm that V1's wall detector accepts them.

## Why V1 failed

| Check | V1 behavior | Effect on the new track |
|---|---|---|
| Input | Raw `/unilidar/cloud` | Different preprocessing from the working `/scan` display |
| Range | Keeps `0.10–2.0 m` | Can discard farther wall returns |
| Height | Keeps sensor-frame `z = 0.04–0.15 m` | At a 7-inch LiDAR height and 11-inch wall top, only a thin upper portion of the tube can pass |
| Frame point count | Drops scans with fewer than 50 retained points | Sparse tube returns can invalidate a scan |
| Prediction point count | Requires at least 80 retained points | A visible scan can still be rejected by the controller |
| Lateral gate | Keeps walls only at `0.16 < |y| < 1.10 m` | Both walls of a centered `3.5 m` section are near `±1.75 m` and are removed |
| Accepted lane width | `0.43–1.15 m` | Even a `1.5 m` section is rejected as too wide |
| Expected width | Fixed at `0.8382 m` in the latest V1 (`0.77 m` in older V1) | Single-wall center estimates are offset by the wrong half-width |
| Look-ahead | Wall stations cover only about `0.02–0.91 m` | Too short for stable estimation on a wide, curving course |
| Wall support | Multiple points and stations required in every frame | Reflective curved tubes may not provide enough consistent samples |

## Required perception changes

### Preferred input path

Use the working `/scan` data path in V1:

1. Subscribe to `/scan`.
2. Retain finite ranges from approximately `0.10–3.5 m`.
3. Convert polar samples to XY points.
4. Transform the points into `base_link` using TF.
5. Do not reapply the old raw-cloud `z = 0.04–0.15 m` filter.

If V1 must continue using `/unilidar/cloud`:

- Inspect the height limits used by the PointCloud-to-LaserScan converter that produced the successful `/scan` data.
- Apply the same limits to the raw cloud.
- As an initial test only, evaluate a sensor-relative height band near `-0.14 to +0.11 m`.
- Confirm the Unitree point-cloud axis directions before using those values.
- Remove ground points separately if required.

### Starting parameters

These values are initial test settings, not final competition tuning:

| Parameter | Suggested starting value |
|---|---:|
| Maximum usable range | `3.5 m` |
| Maximum lateral wall distance | `2.2 m` |
| Accepted paired-wall width | `1.3–3.8 m` |
| Forward wall trace/look-ahead | About `2.0 m` |
| Recent scans for lane inference | `3–5` motion-corrected scans |

Additional changes:

- Remove the fixed `0.8382 m` expected track width.
- When both walls are reliable, measure the width and smooth it over time, clamped to approximately `1.3–3.8 m`.
- For single-wall tracking, use the last reliable measured width or, preferably, the local width from the map.
- Replace the global 80-point requirement with a wall-station coverage/support test.
- Tune point-count requirements using recorded competition-track data.
- Use accumulated scans only for wall/lane inference. Collision and footprint checks must use the current scan so an old observation cannot hide a new obstacle.

Wider gates alone are unsafe on this course because nearby portions of a hairpin can be mistaken for the current corridor. Wall pairing must also require:

- Spatial continuity.
- Consistency with previous frames.
- Agreement with the mapped corridor and vehicle pose.
- Rejection of sudden wall or width jumps.

## Map and path changes

V1 does not load `IROS2026.yaml` or `IROS2026.png`. Editing the YAML alone will not change V1 behavior; map loading, localization, and map-based path use must be integrated explicitly.

The new course is not suitable for a simple radial/ring centerline. Generate an ordered centerline from the free corridor between the two wall loops:

1. Load the occupancy map.
2. Isolate the drivable corridor.
3. Extract its medial axis or distance-transform center.
4. Convert it into one ordered closed path.
5. Smooth it while preserving clearance and respecting steering limits.
6. Store local corridor width/clearance along the path.
7. Track ordered progress so geometrically adjacent hairpin sections are not confused.

Do not use V2's clockwise/right-turn-only steering restriction for this map. The track contains substantial left and right turns. Verify that the path curvature is feasible with the car's asymmetric steering limits, approximately `+0.26 rad` left and `-0.38 rad` right.

Use the full vehicle footprint during planning and collision checks:

- Body: approximately `0.508 × 0.2667 m`.
- Wheelbase: approximately `0.3302 m`.

## `IROS2026.yaml` and image checks

Current map settings:

```yaml
image: "IROS2026.png"
resolution: 0.05
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.45
free_thresh: 0.196
```

Recommended checks and changes:

- Keep `negate: 0`; black walls and white free space are interpreted correctly.
- Because the PNG is binary black/white, the current occupancy thresholds are not the main issue.
- At `0.05 m/pixel`, the `1036 × 631` image represents approximately `51.8 × 31.55 m`. Keep this resolution only if it matches the measured venue.
- Otherwise set `resolution = measured physical map width / 1036` and cross-check it using `measured physical map height / 631`.
- Keep origin `[0, 0, 0]` only if the lower-left map corner is the intended world origin; otherwise use a surveyed origin.
- The PNG contains 20 black dots, each approximately `0.65 m` in diameter at the current scale. Remove them if they are reference/grid marks; retain them only if they represent real obstacles.
- The drawn walls are approximately `0.16 m` thick at the current scale. If the HVAC tubes are actually about 11 inches in diameter, render/dilate them to roughly `0.279 m` (`5–6 pixels` at the current resolution), or account for their radius consistently through the footprint/inflation model. If 11 inches describes only wall height, do not infer tube diameter from it.

## Files to update

- `extracted_v1/AutoRacer_V1_20260926/newtrack/track_lidar_only_node.py`
  - Change the sensor input/preprocessing path and remove incompatible scan rejection.
- `extracted_v1/AutoRacer_V1_20260926/newtrack/live_centerline.py`
  - Widen wall search limits and improve wall association/temporal stability.
- `extracted_v1/AutoRacer_V1_20260926/newtrack/lidar_only_core.py`
  - Remove the fixed narrow-track width and update center estimation.
- `extracted_v1/AutoRacer_V1_20260926/newtrack/track_autonomy_core.py`
  - Integrate the new centerline, local width, footprint, and steering constraints.
- `IROS2026.png` and `IROS2026.yaml`
  - Correct scale, origin, nonphysical dots, and wall representation as needed.

## Validation sequence

1. Record synchronized `/scan`, `/unilidar/cloud`, TF, and odometry at narrow/wide straights and both turn directions.
2. Log the point count after every filter: raw, range, height, transform, lateral gate, station support, and width pairing.
3. Verify `/scan` frame ID and transforms; replace any hard-coded cloud yaw correction with TF when possible.
4. Replay the data with motor output disabled and inspect detected walls, measured width, centerline mode, and commanded steering.
5. Verify map resolution/origin and remove nonphysical obstacles before generating the centerline.
6. Check the complete vehicle rectangle and steering radius along the full planned path.
7. Perform the first physical tests at low speed, under supervision, while recording all sensor and controller data.
8. Stop the vehicle on stale sensor data, invalid corridor estimates, or localization failure.

## Bottom line

The primary failure was not LiDAR mounting height alone. V1 discarded the new track because of its narrow lateral gate, `1.15 m` maximum accepted width, fixed old-track width, restrictive height slice, short look-ahead, and strict per-frame point requirements. The robust fix is to reuse the proven `/scan` preprocessing, support a locally varying `1.5–3.5 m` corridor, add temporally and map-consistent wall association, and generate the reference centerline from the occupancy map.
