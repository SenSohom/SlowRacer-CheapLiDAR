# SlowRacer-CheapLiDAR

Low-speed autonomous RoboRacer using a Unitree LiDAR.

- **V1:** Live-LiDAR lane following and guarded recovery.
- **V2:** Adds recent-scan history and optional offline-map preview.

Download [V1](https://github.com/SenSohom/SlowRacer-CheapLiDAR/releases/download/v1-v2-20260926/AutoRacer_V1_20260926.zip) or [V2](https://github.com/SenSohom/SlowRacer-CheapLiDAR/releases/download/v1-v2-20260926/AutoRacer_V2_20260926.zip). Each ZIP includes the frozen release and latest track-specific working copy.

See [GPT-Transfer.md](GPT-Transfer.md) for setup, test results, and known limitations. Obstacle bypass remains unresolved.

An experimental, dry-run-by-default IROS wall classifier, mapper and slow
exploration controller is available in
[experimental/iros-wall-explorer](experimental/iros-wall-explorer/README.md).
It includes Jetson deployment steps, offline tests, colored wall output, and a
LiDAR point-flood/track-contact shutdown recorder. It has not yet completed a
physical validation run.
