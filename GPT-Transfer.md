# AutoRacer GPT Transfer — V1 and V2

**Prepared:** 2026-09-26, America/New_York.  
**Purpose:** Give the next operator or assistant enough context to restore the car, choose the correct version, understand the map and planner, and continue development without repeating the earlier diagnosis.  
**Scope:** Local packaging and documentation. No physical drive or Jetson deployment was performed while preparing these ZIP files.

**Public copy:** Jetson login credentials are omitted. Public ZIP checksums differ from the original local transfer ZIPs because their documentation was sanitized; runtime code, maps, and recordings are unchanged. Use the checksums attached to the [GitHub release](https://github.com/SenSohom/SlowRacer-CheapLiDAR/releases/tag/v1-v2-20260926).

## 1. Current status and the most important distinctions

The project has two preserved releases and two newer working copies:

| Copy | Role | Physical validation | Important distinction |
|---|---|---|---|
| Frozen V1, 2026-09-24 | Proven live-LiDAR baseline | 60 seconds, 17.07 m forward odometry, five guarded reverse recoveries | No offline-map input; original 0.77 m lane setting |
| Frozen V2, 2026-09-24 | Proven live-LiDAR controller with history and optional map preview | 120 seconds, 58.00 m forward odometry, five guarded recoveries | Uses the earlier track map; clockwise direction guard |
| New-track V1, 2026-09-25 | Slower mapping copy of V1 | Mapping run completed; first lap retained | Lane setting 33 inches; positive motor command capped at 0.45 m/s |
| New-track V2, latest working copy | Controller for the rebuilt track; latest diagnostics and fixes | Earlier revisions completed two 120-second runs; latest corner fixes have replay validation only | New map, controller freshness checks, revised recovery, corner timing fixes, obstacle mocks |

**Do not describe the latest new-track V2 copy as fully physically validated.** The successful 120-second tests preceded its last corner-timing and test-guard changes. The missed corner has not been physically retested with those changes.

**Neither V1 nor V2 currently implements dependable obstacle bypass.** Both can brake and, in normal mode, attempt guarded reverse. V2 can retain a valid wall-map match despite an unmapped object. A map mismatch is therefore not a reliable trigger for going around an object.

The latest user request before this handover was to mock obstacle behavior using the actual offline map. That mock was completed locally. The new request was to create separate V1/V2 ZIPs and this comprehensive handover. No physical test is implied by the packaging request.

## 2. Deliverables and archive organization

The original PC output directory (private transfer copies) is:

```text
/home/carla1000/Downloads/autoracer_transfer_20260926/
  AutoRacer_V1_20260926.zip
  AutoRacer_V2_20260926.zip
  ARCHIVE-SHA256SUMS
  ARCHIVE-VALIDATION.json
  build_transfer_archives.py
```

The original standalone handover is `/home/carla1000/Downloads/GPT-Transfer.md`. The public handover is `/home/carla1000/Downloads/SlowRacer-CheapLiDAR/GPT-Transfer.md` and is copied into both public ZIPs. Public ZIPs and their checksums are stored in that repository directory under `.release-assets/` and published as GitHub release attachments.

Each ZIP extracts into its own top-level directory:

```text
AutoRacer_V1_20260926/             AutoRacer_V2_20260926/
  frozen/                          frozen/
  newtrack/                        newtrack/
  support/launch/                  support/launch/
  GPT-Transfer.md                  GPT-Transfer.md
  README-FIRST.md                  README-FIRST.md
  NEWTRACK-PROGRESS.md              NEWTRACK-PROGRESS.md
  HANDOVER-20260923.md              HANDOVER-20260923.md
  TRANSFER-MANIFEST.json            TRANSFER-MANIFEST.json
  TRANSFER-SHA256SUMS               TRANSFER-SHA256SUMS
                                   evidence/v2_test_120s_20260924_213145/
```

- `frozen/` preserves the corresponding 2026-09-24 release, including its original checksums and README.
- `newtrack/` is the corresponding 2026-09-25 working directory, including local maps, scripts, plots, and available recordings. Python bytecode caches are omitted.
- `support/launch/` contains the LiDAR launch and the autonomous bringup that starts `joy_linux` without `joy_teleop`.
- The V2 ZIP also includes the full locally available ROS bag for the final successful frozen-V2 120-second run.
- `TRANSFER-MANIFEST.json` identifies source directories, roles, and hashes. `TRANSFER-SHA256SUMS` is authoritative for the entire transferred package.
- The inherited `newtrack/VERSION`, `newtrack/README.md`, and, where present, `newtrack/SHA256SUMS` describe the parent release and may be stale. They are preserved as provenance; do not use them to identify or verify the current working copy. Read this document, `NEWTRACK-PROGRESS.md`, and the transfer manifest instead.

ROS workspaces, operating system packages, serial-device rules, and the existing Jetson installation are external dependencies. These are project bundles, not Jetson system images.

## 3. Source directories and remote destinations

| Content | Source on this PC | Existing/historical location on Jetson |
|---|---|---|
| Frozen V1 | `/home/carla1000/Downloads/autoracer_V1_20260924` | `/home/autoracer/releases/V1_20260924` |
| Frozen V2 | `/home/carla1000/Downloads/autoracer_V2_20260924` | Intended `/home/autoracer/releases/V2_20260924`; historical transfer was pending |
| Earlier V2 development and full final bag | `/home/carla1000/Downloads/autoracer_V2` | `/home/autoracer/v2` |
| New-track V1 | `/home/carla1000/Downloads/autoracer_newtrack_20260925/v1` | `/home/autoracer/newtrack_20260925/v1` |
| New-track V2 | `/home/carla1000/Downloads/autoracer_newtrack_20260925/v2` | `/home/autoracer/newtrack_20260925/v2` |
| New-track progress notes | `/home/carla1000/Downloads/autoracer_newtrack_20260925/README.md` | PC notes; included in both ZIPs |
| Original handover | `/home/carla1000/Downloads/HANDOVER.md` | Historical context only |

The frozen PC directories are read-only and their original checksums were verified during packaging. Leave them intact. Develop in a separate working directory rather than editing the preserved releases.

The Jetson was unreachable during the preceding mock work (`No route to host`). Packaging uses local sources and does not certify the Jetson's current files, running processes, or power state.

## 4. Project objective and operator preferences

The car should drive continuously around a small indoor cardboard-wall track, turn early enough to clear tight right corners, steer smoothly, and recover from a blocked corner when a guarded reverse is clear. The operator favors practical progress and direct execution within authorized scope over repeated repositioning or approval questions.

The development sequence was:

1. Diagnose incorrect LiDAR maps and wall visibility.
2. Obtain a useful low-wall map from a slow single lap.
3. Establish a live-LiDAR controller independent of map matching.
4. Freeze that functioning baseline as V1.
5. Add short scan history and optional offline-map preview, while retaining the live path when map preview is unavailable; freeze the functioning version as V2.
6. Rebuild the track, update the minimum lane width to 33 inches, and collect a new first-lap map using a slow V1 copy.
7. Run new-track V2 tests, then investigate an unmapped obstacle and a missed turn.
8. Mock side-obstacle cases on the actual saved map, then package the project for transfer.

Specific preferences and corrections to preserve:

- Use the front part of current LiDAR geometry for decisions; do not let rear geometry dictate forward turning.
- Use one visible wall and the fixed lane width when the other wall disappears.
- The operator briefly considered adaptive width, then explicitly chose fixed width. The earlier track used 0.77 m; the new track uses 33 inches = 0.8382 m.
- Keep the measured car body geometry. An earlier suspected body error was retracted by the operator.
- Start turns earlier and avoid false left turns in a right corner.
- Favor a short, smooth preview supported by current observations; historical/map information supplements it.
- Guarded reverse is useful and should allow the run to continue when clearance exists. A touching point that moves clear is treated differently from an entering collision.
- PS4 **L1**, not a keyboard key, is the requested start control. R1 is the autonomous override; it is not the autonomous start button.
- The obstacle under discussion occupies one-third to one-half of lane width and is intended to leave a side passage. Both side placements were mocked because its exact position and dimensions were not supplied.
- An earlier usage constraint asked to stop if weekly usage reaches 30% remaining. No reliable usage measurement is recorded in these project files; do not invent a reported quota value.

## 5. Hardware, geometry, and coordinate conventions

| Item | Current project value / context |
|---|---|
| Computer on car | NVIDIA Jetson NX, Ubuntu 22.04 aarch64, ROS 2 Humble |
| Motor/servo controller | VESC MK6, firmware 7.0 in original handover |
| LiDAR | Unitree L1 3D LiDAR, `/dev/sensors/unilidar` |
| VESC serial path | `/dev/sensors/vesc` |
| Human controller | PS4 DualShock over Bluetooth; `/dev/input/js0` in saved config |
| Overall car length | 20 inches = 0.5080 m, bumper to bumper |
| Overall car width | 10.5 inches = 0.2667 m |
| Axle-to-axle wheelbase | 13 inches = 0.3302 m |
| LiDAR from extreme rear bumper | 14 inches = 0.3556 m; centered laterally |
| Rear bumper behind rear axle in software | 3.5 inches = 0.0889 m |
| Front bumper ahead of rear axle | 16.5 inches = 0.4191 m |
| LiDAR ahead of rear axle | 10.5 inches = 0.2667 m |
| LiDAR physical height, last explicit correction | 7 inches = 0.1778 m above floor |
| New-track measured minimum width | 33 inches = 0.8382 m |
| Earlier track fixed lane width | 0.77 m |
| LiDAR yaw mounting calibration | Car-forward direction is +61 degrees in LiDAR XY coordinates |
| Geometric maximum steering magnitude | 0.38 rad |
| General positive/left actuator limit | 0.26 rad in the local tracker |
| V2 clockwise-course left correction limit | +0.05 rad after direction guard |

The operator initially supplied a 17-inch length and later corrected the extreme bumper length to 20 inches. The software uses 20 inches. The rear-axle offset in the table is the software interpretation of the measurements; retain it unless remeasured.

`track_autonomy_core.py` defines the authoritative planning geometry. `live_centerline.chassis_points()` converts LiDAR returns into car axes: rear axle at `(0, 0)`, **x forward, y left**. In a plot using those axes, the black cross at the origin is the rear axle reference, not the LiDAR. The LiDAR is `(0.2667, 0)` in those axes. Sensor-frame plots and map-frame plots have different origins; do not interpret a plotting marker as a measured chassis position without reading its label.

In software steering coordinates, negative is right and positive is left. The VESC servo calibration is:

```text
speed_to_erpm_gain = 4614.0
steering_angle_to_servo_gain = -1.2135
steering_angle_to_servo_offset = 0.42
servo_min = 0.10
servo_max = 0.91
```

The negative servo gain converts the software steering angle to servo position; it is not a reason to reverse the planner sign. Reverse VESC speed is negative. Historical odometry velocity sign has been inconsistent enough that recovery checks combine raw VESC speed with odometry motion.

**Known configuration mismatch:** saved `config/vesc.yaml` still has `vesc_to_odom_node.wheelbase: .25`, while the planner uses the measured 0.3302 m. That config is preserved, not silently corrected. It can affect odometry yaw/closure. Reconcile it deliberately in a future calibrated working copy, with a test; IMU and scan matching were used to avoid trusting wheel-only closure.

The original `HANDOVER.md` contains older wheelbase, track-width, TF, controller, and map information. Current geometry and launch/code behavior take precedence. The historical static TF has a different height/orientation; these controllers directly transform LiDAR points with the constants above rather than relying on that TF for centerline extraction.

## 6. LiDAR visibility and why the original maps were poor

The initial images showed wall smearing, inconsistent lap trajectories, and duplicate boundaries. The investigation separated raw wall visibility from errors introduced by motion estimation and map accumulation.

Physical LiDAR height changed from approximately 9 inches to 8 inches and finally 7 inches. The operator said the adjacent Jetson prevented lowering it further until the mounting arrangement was changed. Cardboard visibility tests then compared vertical point bands. The most useful stationary wall image was the **0.04–0.15 m band in the point-cloud's z coordinates**. Wider/higher bands included more room returns and did not isolate the low track boundaries as well.

Current controllers retain finite points satisfying:

```text
0.10 <= hypot(x, y) <= 2.0 metres
0.04 <= z < 0.15 metres, in the incoming LiDAR coordinates
at least 50 retained returns before accepting a new control scan
```

Do not equate the selected sensor z band with physical height above the floor without checking the Unitree frame convention and mounting. The selection is based on the observed useful wall returns. An obstacle that does not intersect this band can be missed even if it exists physically.

The mapping/replay configuration uses KISS-ICP with low-wall point filtering, a 0.04 m map voxel setting, and no deskew in the saved replay script. Parallel short wall segments can allow ICP sliding or drift. Accumulating several laps without adequate closure correction creates duplicated walls. Merely collecting more laps is not a fix.

The useful current map was built by finding the **first** return to the stationary starting scan, using that first lap only, distributing the end-pose correction over the trajectory, and then fitting/cleaning the two boundary rings. This is not a general SLAM pose-graph optimizer. Radial boundary fitting can invent narrow/wide sections and assumes a ring-like course.

## 7. V1 algorithm

Core files are `live_centerline.py`, `lidar_only_core.py`, `track_autonomy_core.py`, and `track_lidar_only_node.py`.

1. Convert the latest low-wall cloud into rear-axle car axes and remove selected fixed returns from the car/mount.
2. Trace nearby left and right wall segments through forward stations. When both are visible, use their measured midpoint. When only one continues, offset it by the fixed lane-width estimate to infer the center.
3. Fit a short smooth center path. Distinguish an approximately straight corridor from a visible bend so a diagonal wall slice does not produce alternating large corrections on a straight.
4. Join the path smoothly to the rear axle. Use a short lookahead, measured lateral error, and visible forward bend cues to request steering.
5. Smooth requests and retain recent corner direction through brief ambiguous one-wall frames.
6. Check a short full-body sweep against **current** returns. Candidate steering angles near the requested angle can be tried; there is no separate search for an obstacle bypass route.
7. Convert the planner target to a VESC speed request using measured wheel motion, a launch request to overcome friction, and speed caps/coasting.
8. If the forward path blocks, send zero immediately. After persistent blockage, attempt guarded reverse only if rear visibility, direction, and swept clearance permit it.

Frozen V1 planner targets are **0.73 m/s paired walls, 0.69 m/s single/extended wall, and 0.64 m/s tight turns**. These targets are not a promise of actual ground speed. The new-track V1 preserves those planning targets but clamps every positive published motor command to **0.45 m/s**. Its width setting is 0.8382 m rather than 0.77 m.

The frozen V1 success establishes that the car can continue for a timed run with recoveries. It does not establish optimal path following, obstacle passing, or collision-free driving.

## 8. V2 additions and fallback behavior

V2 keeps the V1-style live planner as its primary source of current lane geometry. It adds `temporal_scan_v2.py` and `map_preview_v2.py`.

### Recent scan history

Up to four recent scan entries are transformed into the current car frame using odometry distance and IMU heading change. Accepted history is at most 0.50 seconds old, at most 0.38 m away in travel, and at most 0.55 rad away in turn. History improves the visible lane preview. **Current returns alone are used for footprint collision checks.** A dynamic object should not be treated as a static historical wall.

### Offline-map localization and preview

The map contains walls, a rear-axle path, path headings, and the recorded mapping trajectory. Background global search considers path poses and, in the new-track copy, the recorded mapping trajectory as seeds. This reduces the chance that a different symmetric part of the oval is selected when the car is off the smoothed path.

Once localized, wheel speed and IMU propagate the pose, then moving-scan ICP corrects it. Weak/rejected matches do not enable mapped preview. Three tracking misses can clear localization and require a new search. New-track search results may be propagated and revalidated when up to 2.5 seconds old; foreground callbacks do not run the complete global search.

Representative working-copy checks include:

- moving-scan match fraction at least 0.50, median wall distance at most 0.05 m, and rear-axle path error at most 0.50 m;
- path error at most 0.25 m before extending the preview;
- current visible path overlap around x = 0.35–0.70 m with median map/live lateral discrepancy at most 0.16 m;
- a short enough join to the existing live-path end;
- no strong disagreement in turn direction;
- a current-cloud footprint check at the blended steering angle.

If these fail, V2 returns the live command and live path. The extension is added after the live path, rather than replacing the measured near segment. Steering blends 75% live and 25% mapped request and changes the live request by no more than **0.06 rad**. The node applies the clockwise steering bound again after blending.

**This is optional map preview, not a controller that blindly follows the complete global route.** The NPZ's saved speed profile is not the runtime speed command: `MapPreview.refine()` returns the live speed. Do not infer the physical test speed from the path's stored 0.19–0.31 m/s preview profile.

Normal V2 planning targets are **0.83/0.79/0.74 m/s** for paired/single/tight sections. `--slow-map` selects **0.53/0.49/0.44 m/s** and a slower motor governor. The latest slow profile caps the rolling request to 0.40 m/s on a detected right-turn approach and briefly coasts above 0.42 m/s there. Launch and rolling commands differ, so consult `/sensors/core`, `/odom`, and the status log for actual speed.

### Clockwise-course direction guard

V2 was tuned for these clockwise tracks. False left requests after a right turn were observed physically. V2 holds a recent right cue through ambiguous frames and restricts positive left correction to **+0.05 rad**. This is useful on this course but can prevent a genuine left corner or a necessary leftward obstacle bypass. Revisit the guard before using a course containing left turns.

## 9. Controller buttons, arming, stopping, and recovery

The PS4 controller and VESC are different devices: PS4 provides operator input; VESC controls motor and servo.

| Action / mode | Actual behavior |
|---|---|
| L1, `buttons[4]`, without `--remote-run` | A new press while `ready` requests autonomous start; not hold-to-drive |
| R1, `buttons[5]` | Interrupts active/recovering/settling autonomy; named manual override in logs |
| `--remote-run` | Automatically requests one start when prerequisites are ready; L1 is not required for that initial start |
| Without `--drive` | Dry run; no drive/brake publication by this node |
| `--exit-on-stop` | Exits after a completed start has stopped and its brief brake period finishes |
| Normal persistent blocked path | Immediate zero command, then guarded reverse after three blocked scans if the prerequisites pass |
| `--obstacle-test` | Persistent projected-lane obstruction causes a stop; reverse recovery is disabled |
| `--reposition-only`, latest V2 | One existing guarded reverse capped to 0.08 m, then stop; no forward resumption |

Do not hold R1 expecting autonomy to move. Do not tell the operator to press keyboard F1. A stopped node without `--exit-on-stop` can return to `ready` after buttons are released; a new L1 press can start again. `--remote-run` automatically arms only once per process.

Arming requires a valid lane, at least 12 accepted scans, stationary wheel/odometry readings, fresh scan/IMU/odometry/VESC data, only one `/drive` publisher, and an attached brake subscriber. After reboot, the node may seed a zero-speed servo/drive command to make VESC odometry start publishing.

The **latest new-track V2** also requires a fresh PS4 message within 0.50 seconds when motors are enabled and stops on stale controller data even in `--remote-run`. Frozen V1, frozen V2, and new-track V1 retain older behavior that bypasses joystick freshness checks in remote-run mode. Do not generalize the new V2 requirement to all archives.

Saved joystick settings use `autorepeat_rate: 20.0`. With no autorepeat, an idle Bluetooth controller may appear stale. Check the actual incoming `/joy` rate instead of repeatedly restarting the drive process.

**Teleop conflict:** the saved `joy_teleop.yaml` assigns its manual deadman to L1, while this autonomous node also uses L1 to start. `/teleop` has mux priority 100 and `/drive` priority 10. Running `joy_teleop` during autonomy can therefore override autonomous commands and reproduce the earlier “steering moves but the car does not go” / manual-mode confusion. Use the provided bringup that keeps `joy_linux` and omits `joy_teleop`. R1 stops this node; actual joystick driving additionally requires an appropriate teleop process and mapping.

Normal V2 recovery brakes until wheel motion settles, checks current/recent rear-wall visibility, and searches reverse arcs. It can choose 0.15 m, 0.07 m, or 0.035 m reverse targets according to clearance. It publishes -0.35 m/s during reverse, settles for 0.4 seconds, and requires several clear forward frames before resuming. Three unsuccessful attempts stop the run; successful forward travel can reset that attempt budget. The latest recovery also tolerates a short blocked reverse observation while braked and reevaluates forward clearance.

The footprint logic recognizes narrowly defined existing contacts that monotonically move clear. This implements the user's request to escape a wall rather than treating every rear-side touching return as a reason to remain stopped. It is not a blanket instruction to drive through walls or ignore the car body.

## 10. Current map, path, and file formats

Only the first lap of the new-track slow recording was used even though the recording contained roughly three laps. Total V1 mapping-run forward odometry was 27.04 m.

The first strong return to the stationary start scan was at **29.755744333 s**, with approximately **95.8%** of nearby wall points within 8 cm. Its matched end LiDAR pose relative to the initial scan was:

```text
x   = -0.0337771260082687 m
y   = +0.2088246705201064 m
yaw = +0.44997122073576146 rad
```

That return need not be exactly the same chassis position; the earlier operator estimated 10–30 cm difference. Closure correction preserves the matched end pose rather than forcing an arbitrary identical position.

| Artifact | Contents / use |
|---|---|
| V1 `maps/newtrack_scans.npz` | 684 low-band clouds, concatenated XYZ with offsets, cloud times, odom and IMU samples |
| V1 `maps/newtrack_kiss_replay.npz` | KISS times, 4×4 poses, accumulated wall points |
| V1 `maps/newtrack_single_lap.npz` / `.xml` | 209 corrected poses and 6,012 raw first-lap wall points |
| V1 `maps/newtrack_single_lap_clean.npz` / `.xml` / `.png` | 209 poses, 4,545 retained fitted boundary returns |
| V2 `maps/newtrack_single_lap_clean.npz` | Same clean map input for the path builder |
| V2 `maps/newtrack_v2_path.npz` / `.csv` / `.png` | 180-point clockwise smoothed rear-axle path, map walls, recorded LiDAR trajectory, geometric metadata |
| V2 `maps/newtrack_v2_path_metrics.json` | Path and body-clearance diagnostics |
| V2 `maps/newtrack_v2_preview_replay.json` | Original first-lap preview replay counts |

The new planned lap is **6.3928 m**, maximum geometric steering about **0.3627 rad**, minimum mapped full-body clearance about **0.0455 m**. The fitted radial corridor width ranges from about **0.721 to 1.410 m**. The apparent minimum below the measured 0.8382 m is a fitting artifact/uncertainty, not a replacement physical measurement. Those clearance figures are computed against the map and do not guarantee physical clearance or a globally optimal route.

The path NPZ has `path[N,2]` in map coordinates for the rear axle, `yaw[N]` for chassis heading, `curvature`, `body_gap`, `speed_profile`, `walls[M,3]`, and `reference_trajectory[K,3]` for recorded LiDAR poses. Converting between a LiDAR pose and the rear axle uses the 61-degree calibration and 0.2667 m forward offset.

Start a future physical run on a centered straight pointing in the established clockwise travel direction. An exact preassigned map point is not mandatory for live fallback; mapped preview waits for an acceptable localization. If the track walls, mounting, or dimensions change, the saved map/calibration may no longer represent the track.

## 11. Physical test history and what it proves

| Date / run | Outcome | Evidence / limitation |
|---|---|---|
| Sep 24, frozen V1 60 s | Time limit; 17.07 m; five reverse recoveries | `V1/frozen/evidence/v1_60s_run.log`, final scans; timed continuation, not clean racing |
| Sep 24, V2 at V1-speed 60 s | 26.61 m; time limit; measured wheel speed zero afterward | Frozen V2 `recordings/v2_test_20260924_205314.controller.log` |
| Sep 24, faster early V2 attempts | False left commands after right turns | Earlier logs/stop scans in frozen V2; direction guard added afterward |
| Sep 24, final frozen V2 120 s | 58.00 m; five guarded recoveries; time limit | `v2_test_120s_20260924_213145`; full final bag included in V2 ZIP |
| Sep 25, V1 slow mapping | 27.04 m total; approximately three laps recorded | Remote `v1_slow_map_20260925_172827`; only first lap used |
| Sep 25, first new-track V2 attempt | Stopped at 0.73 m, initially facing barrier; inadequate rear returns | `newtrack_v2_first_stop*` plots/scans |
| Sep 25, second new-track V2 attempt | Stopped at 1.11 m in first right corner | `newtrack_v2_second_stop*`, `newtrack_v2_turn_timing.png` |
| Sep 25, 18:23 V2 120 s | 32.19 m; live fallback; three guarded reverses; time limit | Local `newtrack_v2_test_120s_20260925_182303` bag and controller log |
| Sep 25, 18:38 V2 120 s | 42.38 m; mapped preview active at multiple points; time limit | Full bag remains remote at `newtrack_v2_test_120s_20260925_183827`; a recovery occurred, so do not label it recovery-free |
| Sep 25, 18:45 obstacle attempt | 1.83 m, missed first right corner; operator reported wall contact; obstacle not reached | Local `newtrack_v2_obstacle_20260925_184554` bag, missed-turn scan plot |
| Subsequent latest-code work | Recorded-scan replay and stationary clearance checks only | No new physical obstacle pass or retest of the fixed corner |

The final frozen-V2 recording had 1,527 forward drive messages and none above +0.05 rad left after the course direction guard. Recorded faster unguarded runs had a right-to-left flip around +0.20 rad and a corresponding IMU response; the correction was motivated by real behavior rather than just a plot.

For the first-lap new-map preview replay, map preview was used on 53 of 209 scans and live fallback on 156. Added steering stayed within 0.056 rad. These original replay counts are evidence for that revision, not a fresh test of every subsequent change.

Distance figures are forward odometry, not an independent motion-capture measurement or guaranteed completed lap count. Successful timed tests included recoveries. Preserve that distinction in future summaries.

## 12. Missed corner: diagnosis and latest untested physical changes

The first obstacle-test implementation used a long 0.45 m swept-body guard. It counted the turning wall at the first tight corner and stopped there before seeing the placed object. The operator reported wall contact. The frame sequence also showed that right-turn steering was late: at about 0.97 m forward travel, the continuing left wall already bent right but the command was only about -0.04 rad and remained weak until the front reached the corner.

Wheel speed was about 0.47 m/s, and recorded odometry moved another **0.077 m in roughly 0.25 seconds** after the zero-speed command. A planner stop is not an instantaneous physical stop.

Latest working-copy changes, present in the V2 ZIP:

1. `live_centerline.py`: recognizes a sustained right bend in `paired_plus_left` geometry before the near segment stops looking straight. The cue uses several inferred stations, far lateral displacement below -0.17 m, extension deviation below -0.12 m, and continuation reaching at least x = 0.95 m.
2. `lidar_only_core.py`: holds a recent strong right preview through one weak continuation scan for about 0.35 seconds.
3. `track_lidar_only_node.py`: in slow mode, reduces/coasts the rolling motor request on the right-turn approach rather than arriving at the corner at the preceding straight's speed.
4. The optional obstacle-test guard now counts returns in the projected lane interior over x = 0.52–0.95 m and within the existing predicted length. Three or more points within ±0.18 m of the predicted center for two scans cause a stop. It is a stop guard, not a bypass planner.
5. Optional `--reposition-only` can request a short existing guarded reverse and then stop. That mode was prepared but not physically used.

Replay of the failed corner changes the request near 0.97 m from about -0.04 to -0.19 rad and near 1.12 m from about -0.03 to -0.31 rad. Replay against 1,051 scans from the earlier successful fallback run showed limited command changes. Neither replay establishes the physical car will clear the corner.

At the last confirmed contact position, the operator reported the car intact, facing the wall, and unmoved. Eight fresh frames had 23–38 rear returns and zero counted hits for the checked 0.10 m reverse arcs. No reverse was actually performed. That is historical state; inspect live state on reconnection rather than assuming the car is still there.

## 13. Unmapped obstacles and the completed mocks

### Existing behavior

A new object does not necessarily prevent wall localization: existing walls may account for most returns and ICP can reject object returns as outliers. Even when mapped preview is rejected, V1-style fallback is a wall/centerline estimator with short steering alternatives. It is not an obstacle-aware route search.

In normal mode, the first blocked scan already produces zero speed. The third consecutive blocked scan attempts recovery, which may stop if turn direction or rear clearance is missing. The optional obstacle-test mode stops and disables reverse. Do not describe a future third-blocked position as the first braking position.

A straight-lane synthetic example in `mock_unmapped_obstacle.py` found normal first braking with the near object face about 0.22 m beyond the bumper. The separate obstacle-test guard stopped at about 0.27 m in that particular synthetic example. An earlier 0.12–0.13 m figure corresponded to a hypothetical third blocked scan if the car ignored braking and kept advancing; it is not the normal first stop clearance. These are mock geometry results, not physical stopping guarantees.

### Actual offline-map side-object mock

`newtrack/mock_offline_map_obstacle.py` uses **`maps/newtrack_v2_path.npz`**, the actual current offline map. It inserts an opaque object into synthetic current scans while leaving the offline walls and route unchanged. It accounts for simple angular occlusion so the object hides returns behind it.

Assumptions: object depth 0.25 m; width one-third or one-half of the local mapped 0.829 m lane; placed near either wall; visible in the retained low-wall z band; initial localization known. The synthetic sensor viewpoints advance along the original map path at 0.4 m/s. The mock runs the actual live tracker and map-preview code, but **does not apply returned steering or simulate braking dynamics**. It calls the tracker directly and does not reproduce the ROS node's four-scan fusion, recovery state machine, or speed governor. It is an open-loop planner replay, not a completed obstacle pass or lap.

| Placement | Object width | Remaining cross-section opening | Approach replay result |
|---|---:|---:|---|
| Left, one-third | 0.276 m | 0.543 m | First blocked command at map-path frame 17 |
| Left, one-half | 0.414 m | 0.404 m | First blocked command at frame 15 |
| Right, one-third | 0.276 m | 0.543 m | No blocked command in the 38-frame approach replay |
| Right, one-half | 0.414 m | 0.404 m | First blocked command at frame 24 |

The no-object baseline has no blocked command over the same approach. At the common diagnostic viewpoint, scan matching initialized near the known pose has 92–97% inliers despite the object. This supports the conclusion that an unmapped object need not force localization failure. It does not validate global relocalization from an unknown pose.

The opening exceeds the 0.267 m car width in all four cases, but full length, turning radius, approach position, and a reachable steering sequence still matter. The right-third case's lack of blocked commands is **not proof of a successful pass** because the simulated sensor follows the old path rather than the returned commands.

Outputs included in V2:

- `maps/offline_map_obstacle_mock.png`: actual map and four placement comparisons; red dashed projected car bodies intersect the object at those sampled poses.
- `maps/offline_map_obstacle_mock_approach.png`: three viewpoints leading up to the first block in the left-third case.
- `maps/offline_map_obstacle_mock.json`: assumptions, baseline, frame-by-frame commands, preview status, and diagnostic matching.
- `maps/unmapped_obstacle_mock.png` / `.json`: earlier synthetic straight-lane diagnostic.

The next obstacle feature should treat localization confidence and route obstruction as separate questions. A future working-copy implementation can retain a good pose match, temporarily shift the local path into observed free space, check the full-body reachable curve, and rejoin the normal route afterward. That is future work, not implemented behavior in these ZIPs. A centered half-width object in a 0.838 m lane would leave only about 0.21 m on each side, which is narrower than the car; do not assume every obstacle occupying half the total width leaves a passable side.

## 14. Transfer to Jetson and verify

Connection supplied by the operator:

```text
SSH account: autoracer@192.168.0.105
Authentication: obtain credentials privately from the car owner
```

From this PC, transfer the files when the Jetson is powered and reachable:

```bash
scp /home/carla1000/Downloads/SlowRacer-CheapLiDAR/.release-assets/AutoRacer_V1_20260926.zip autoracer@192.168.0.105:/home/autoracer/
scp /home/carla1000/Downloads/SlowRacer-CheapLiDAR/.release-assets/AutoRacer_V2_20260926.zip autoracer@192.168.0.105:/home/autoracer/
scp /home/carla1000/Downloads/SlowRacer-CheapLiDAR/.release-assets/ARCHIVE-SHA256SUMS autoracer@192.168.0.105:/home/autoracer/
scp /home/carla1000/Downloads/SlowRacer-CheapLiDAR/GPT-Transfer.md autoracer@192.168.0.105:/home/autoracer/
ssh autoracer@192.168.0.105
```

On Jetson, verify and extract into a separate transfer directory:

```bash
cd /home/autoracer
sha256sum -c ARCHIVE-SHA256SUMS
mkdir -p /home/autoracer/transfers/20260926
unzip AutoRacer_V1_20260926.zip -d /home/autoracer/transfers/20260926
unzip AutoRacer_V2_20260926.zip -d /home/autoracer/transfers/20260926
cd /home/autoracer/transfers/20260926/AutoRacer_V1_20260926
sha256sum -c TRANSFER-SHA256SUMS
cd /home/autoracer/transfers/20260926/AutoRacer_V2_20260926
sha256sum -c TRANSFER-SHA256SUMS
```

Verification and extraction do not start the car. The scripts resolve their code directory relative to themselves, so they do not require overwriting `/home/autoracer/newtrack_20260925` or the frozen releases. ROS workspace paths still assume the existing `/home/autoracer` installation.

If only one ZIP is transferred, verify that specific ZIP against its line in `ARCHIVE-SHA256SUMS` or use its package's `TRANSFER-SHA256SUMS`; the combined outer checksum file otherwise reports the missing second ZIP.

## 15. ROS setup and stack startup

Existing dependencies:

```text
/opt/ros/humble
/home/autoracer/unilidar_sdk/unitree_lidar_ros2
/home/autoracer/f1tenth_ws
Python: numpy, scipy, matplotlib for plots, kiss-icp 1.3.0 for offline replay
ROS: rclpy, sensor_msgs_py, ackermann_msgs, nav_msgs, sensor_msgs, std_msgs,
     vesc_msgs, rosbag2_py, rosidl_runtime_py, launch/launch_ros,
     unitree_lidar_ros2, f1tenth_stack, joy_linux, vesc_driver,
     vesc_ackermann, ackermann_mux
```

Source the environment in every new ROS terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/autoracer/unilidar_sdk/unitree_lidar_ros2/install/setup.bash
source /home/autoracer/f1tenth_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export PYTHONDONTWRITEBYTECODE=1
```

Use either transferred package as the stack-support directory; the following examples use V2. Check existing nodes first so you do not launch duplicate drivers or drive publishers.

Terminal A, if the LiDAR driver is not already running:

```bash
ros2 launch /home/autoracer/transfers/20260926/AutoRacer_V2_20260926/support/launch/lidar_launch.py
```

Terminal B, if the VESC/joystick/mux stack is not already running:

```bash
ros2 launch /home/autoracer/transfers/20260926/AutoRacer_V2_20260926/support/launch/v1_slow_mapping_bringup.py \
  vesc_config:=/home/autoracer/transfers/20260926/AutoRacer_V2_20260926/newtrack/config/vesc.yaml \
  mux_config:=/home/autoracer/transfers/20260926/AutoRacer_V2_20260926/newtrack/config/mux.yaml \
  joy_config:=/home/autoracer/transfers/20260926/AutoRacer_V2_20260926/newtrack/config/joy_teleop.yaml
```

This support launch starts `joy_linux`, VESC driver, Ackermann-to-VESC, VESC odometry, and mux. It deliberately omits the `joy_teleop` action. Config paths are supplied explicitly so installed workspace defaults do not silently differ from the saved files.

LiDAR launch uses `cloud_scan_num: 18`. The prior low-band recordings were around 10 Hz, but rate depends on aggregation/load. Measure the current rate instead of assuming it. These direct-cloud controllers do not need a new static TF publisher to start; other visualization tools may have their own TF requirements.

Read-only checks:

```bash
ros2 node list
ros2 topic info /drive --verbose
ros2 topic info /commands/motor/brake --verbose
ros2 topic hz /unilidar/cloud
ros2 topic hz /unilidar/imu
ros2 topic hz /joy
ros2 topic echo /lidar_only_lap/status
```

Stop each `topic hz` command with Ctrl-C before continuing in that terminal. Expected sensor/actuation topics are:

| Topic | Use |
|---|---|
| `/unilidar/cloud` | PointCloud2, low-wall points selected in code |
| `/unilidar/imu` | Heading and freshness |
| `/odom` | Forward travel and recovery motion |
| `/sensors/core` | VESC wheel speed |
| `/joy` | L1/R1 and controller freshness |
| `/commands/servo/position` | Reported servo command |
| `/drive` | Autonomous Ackermann speed/steering |
| `/commands/motor/brake` | Brake command, 8.0 in node |
| `/lidar_only_lap/status` | State, steering, map state, speed, distance, reason |
| `/teleop` | Higher-priority mux channel; avoid autonomous conflict |

## 16. Choose and run the intended version

All motor-enabled commands below require a clear physical track and live supervision. They are operational instructions for a later authorized test, not a command to start a run as part of this file transfer.

### A. Latest new-track V1, controller-started slow lap

After sourcing ROS and starting the stack:

```bash
cd /home/autoracer/transfers/20260926/AutoRacer_V1_20260926/newtrack
python3 -u track_lidar_only_node.py --drive --min-seconds 0 --max-seconds 90 --exit-on-stop
```

Wait for ready sensor status, then press/release **L1**. This uses the positive command cap of 0.45 m/s and no offline map. Its estimated lap completion uses forward distance at least 5 m, accumulated absolute turn at least 6.20 rad, and odometry closure below 0.35 m. Because wheel closure can drift, it may not stop after exactly one physical lap.

For a recorded remote-armed mapping run:

```bash
bash collect_map_v1_slow.sh
```

This script auto-arms after prerequisites and records up to 90 seconds, with stationary lead/trailing intervals. It is not guaranteed to terminate at the first real lap. Extract the first lap offline as described below. Do not promise the operator it always performs exactly one lap.

### B. Latest new-track V2, controller-started bounded run

```bash
cd /home/autoracer/transfers/20260926/AutoRacer_V2_20260926/newtrack
python3 -u track_lidar_only_node.py --drive --min-seconds 120 --max-seconds 120 --exit-on-stop \
  --slow-map --map maps/newtrack_v2_path.npz
```

Press/release L1 when ready. The equal minimum/maximum disables the earlier estimated single-lap stop until the timed test ends, subject to other stops. This command does not record a bag; use another terminal to record or use the existing recorded-test script:

```bash
bash test_v2_120s.sh
```

That script records a bag and auto-arms using `--remote-run`. It still requires fresh controller data in the latest new-track V2.

**Avoid the inherited `newtrack/run_v2.sh` as the default for the new course.** It names `maps/v2_autonomy_20260924.npz`, the earlier course's path, which is absent from the new-track map directory. Use the explicit new-map command or `test_v2_120s.sh` instead. It is preserved, not silently rewritten during packaging.

To run latest V2 purely live, omit `--map`. To diagnose without motor output, omit `--drive`. Do not use a frozen V2 old-track map on the rebuilt track.

### C. Restore the frozen baseline

```bash
cd /home/autoracer/transfers/20260926/AutoRacer_V1_20260926/frozen
sha256sum -c SHA256SUMS
bash run_v1.sh
```

Frozen V1 `run_v1.sh` auto-arms and uses the original speeds/0.77 m setting. It is the historical baseline, not the slow 33-inch mapping copy.

Frozen V2 is available at the corresponding V2 `frozen/` path. Verify its original checksums there. Its `run_v2.sh` uses the bundled earlier-track map and auto-arms for a 60-second run. For a controller-started frozen test, source ROS and invoke its node directly without `--remote-run`, with the desired time arguments and its bundled map. Keep logs/new recordings in a writable experiment copy so the frozen release remains preserved.

## 17. Rebuild or inspect maps offline

The scripts require ROS bag deserialization packages for raw bags; the actual-map mock only needs the local Python numerical/plotting dependencies. No physical motion is needed for these operations.

For a new recorded bag, from V1 `newtrack/`:

```bash
python3 extract_scans_v2.py recordings/NEW_BAG maps/NEW_scans.npz
python3 replay_kiss_v2.py recordings/NEW_BAG maps/NEW_kiss.npz --start 4.0 --end 90.0
python3 find_closure_v2.py maps/NEW_scans.npz maps/NEW_kiss.npz --window-start 20 --window-end 45 --step 0.4
```

Adjust the motion start/end and first-return window from the new recording. The stationary-start target is built from the early recording interval. Inspect matching coverage and wall geometry; use the first plausible full-lap return, not merely a later score from a second lap.

`build_closed_map_v2.py` still has closure defaults from the **earlier Sep 24 lap**. Supply all four measured closure parameters for a new recording. To reproduce the existing Sep 25 first-lap map from the included NPZ inputs without overwriting it:

```bash
python3 build_closed_map_v2.py maps/newtrack_scans.npz maps/newtrack_kiss_replay.npz maps/rebuilt_single_lap.npz \
  --closure-time 29.755744333 \
  --closure-x -0.0337771260082687 \
  --closure-y 0.2088246705201064 \
  --closure-yaw 0.44997122073576146
python3 clean_map_walls.py maps/rebuilt_single_lap.npz maps/rebuilt_single_lap_clean.npz
```

The builder writes corresponding XML as well; cleaning writes visualization. Do not overwrite the established map until its replacement is reviewed.

V2 `newtrack/build_v2_autonomy_path.py` reads `maps/newtrack_single_lap_clean.npz` and writes `maps/newtrack_v2_path.*` relative to its own directory. For a new track, work in a separate directory and update/replace that input deliberately. Running the builder in the established directory overwrites the saved path outputs.

To regenerate the actual-map mock from V2 `newtrack/`:

```bash
python3 mock_offline_map_obstacle.py
```

It writes the comparison PNG, approach PNG, and JSON under `maps/`. The original map is unchanged. The mock imports this working copy's controller modules; later controller changes can change replay results.

## 18. Diagnostics, recordings, and missing remote-only evidence

Most useful current V2 map-directory files:

```text
newtrack_v2_path.png                        current map and geometric route
newtrack_v2_initial_alignment.png           initial map/scan alignment diagnostic
newtrack_v2_120s_result.png                  earlier 18:23 live-fallback run visualization
newtrack_v2_first_stop.png                  first failed start
newtrack_v2_second_stop.png                 second failed corner
newtrack_v2_turn_timing.png                  turn onset diagnostic
obstacle_missed_turn_scans_20260925.png      failed 18:45 turn sequence
obstacle_early_stop_scans_20260925.npz       57 saved clouds around that attempt
obstacle_early_stop_telemetry_20260925.npz   drive/wheel/odom samples around it
obstacle_preflight_scan_20260925.*           stationary preflight before obstacle attempt
obstacle_postcontact_stationary_20260925.npz stationary clouds after reported contact
offline_map_obstacle_mock.png               four side-object comparisons
offline_map_obstacle_mock_approach.png      approach to first blocked command
```

`newtrack_v2_initial_alignment.png` is the filename present locally; do not confuse it with older notes using the word “initial alignment” without the exact filename. The `.png` for `newtrack_v2_120s_result` represents the fallback run and is not the missing 18:38 map-enabled run's full visualization.

Locally available new-track V2 raw bags included in the ZIP are `newtrack_v2_test_120s_20260925_182303`, `newtrack_v2_test_120s_20260925_183310`, and `newtrack_v2_obstacle_20260925_184554`. The 18:33 bag is an intermediate attempt, not the successful 18:38 run.

Full bags known to remain only on the Jetson and not included as full raw bags:

```text
/home/autoracer/newtrack_20260925/v1/recordings/v1_slow_map_20260925_172827
/home/autoracer/newtrack_20260925/v2/recordings/newtrack_v2_test_120s_20260925_183827
```

Their map/replay derivatives or recorded outcome notes are retained locally. Retrieve the raw data later if needed; do not claim those full bags were packaged.

At every stop, the node writes its most recent scan history to:

```text
/tmp/lidar_only_last_stop_scans.npz
```

Copy that file to a timestamped experiment location before another stop or reboot overwrites/removes it. Raw clouds are concatenated with offsets; timestamps are monotonic for stop-history files rather than wall-clock UTC. Keep the controller log and a ROS bag for a physical change so steering, sensor age, wheel speed, and wall contact can be compared.

`inspect_reverse_clearance.py` is read-only. It samples fresh clouds and reports rear returns and hit counts for tested reverse arcs; it does not reposition the car. A clear sampled arc is evidence for a future guarded move, not proof that every physical motion is clear.

## 19. Troubleshooting and recommended continuation

| Symptom | First evidence to inspect |
|---|---|
| Car stays ready / does not move | `/lidar_only_lap/status`, new L1 edge vs remote-run, R1 released, accepted scans, stationary speeds, brake subscriber, `/drive` publisher count |
| L1 appears to select manual mode | Check whether `joy_teleop` is running and publishing higher-priority `/teleop` |
| Latest V2 stops after controller connection | `/joy` freshness and 20 Hz autorepeat; distinguish PS4 loss from a VESC failure |
| No odometry after reboot | Whether the zero-speed initialization reached VESC/mux; current servo topic and driver state |
| Steering moves, wheels do not | Actual VESC wheel response, command speed, static friction, brake/mux ownership; do not infer a localization problem from this alone |
| Large alternating steering on a straight | Current wall trace, single-wall ambiguity, history age, map/live agreement; do not just increase speed |
| Right turn starts too late | Far left-wall continuation, `paired_plus_left` corner cue, scan delay, current turn approach speed |
| False left after a right turn | Direction guard outputs, recent right preview, map blending and final actuator bound |
| Reverse unavailable | Current/recent rear visibility, blocked swept arc, missing turn direction, motion settling; normal mode vs obstacle-test flag |
| Map smears after repeated laps | First-lap crop/closure, filtering, raw KISS drift; inspect a single lap before increasing accumulated data |
| Obstacle has a visible side gap but car stops | Full-body reachable path versus centerline only; current controller has no reliable bypass route search |
| `run_v2.sh` fails in new-track copy | Inherited old-map filename; use `test_v2_120s.sh` or the explicit new map command |

Suggested next development sequence:

1. Verify transferred archive hashes and inspect live Jetson/car state. Preserve the frozen releases.
2. Recheck current low-band visibility, controller freshness, and unique drive ownership before an authorized physical run.
3. Retest the replay-only early-right-corner changes at the existing slow profile without an added obstacle. Record the run and current stop scans if it fails.
4. For obstacle work, first add a local reachable bypass in a separate working copy and run **closed-loop** map simulation where steering changes the simulated car pose. Include the full 0.508 × 0.267 m body, wheelbase, steering limits, scan delay, and braking. The existing open-loop mocks cannot establish a pass.
5. Keep good wall localization while detecting route obstruction; use current free space to shift the local path and merge back. Review the clockwise left-correction guard if the bypass requires left steering.
6. Only then compare a bounded physical obstacle test against the mock. Preserve failed-turn logs as well as successful runs.

Do not rebuild the map solely because an object was added: the operator explicitly wanted to add the obstacle to the existing mapped track. Rebuild when the wall layout or sensor calibration changes.

## 20. Handover checklist for the next assistant

- Read this document and the package's `README-FIRST.md` before treating an inherited README as current.
- Select **frozen** or **newtrack** explicitly; cite which code copy a result used.
- Separate physical evidence, recorded-scan replay, open-loop synthetic mock, and future work.
- Preserve the measured body, LiDAR offset/yaw, and 33-inch setting for this rebuilt track.
- Remember L1 is PS4 start; R1 is override. Existing auto-arm scripts do not wait for L1.
- Do not confuse “map matches walls” with “the mapped route is clear.”
- Do not claim a complete obstacle bypass, globally optimal route, or collision-free lap from the current evidence.
- Use local work for packaging/offline tasks; a documentation request is not a request to start the physical car.
- Keep communication concise during execution and avoid repeated approval questions for already authorized reversible work.

The project can be resumed from the packages alone for code, maps, diagnostics, and the available evidence, with the existing Jetson ROS installation as its runtime dependency. The two explicitly listed remote-only raw bags are the main evidence gap.
