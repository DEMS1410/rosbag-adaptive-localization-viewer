# Rosbag Adaptive Localization Viewer

Visual trajectory analysis for ROS 2 bags with OptiTrack ground truth.

## Product direction

This project pivots from a Python-only dashboard to a two-part system:

- `Python` for ingestion and normalization of experiment data
- `React + Vite` for a cleaner, more polished visualization frontend

That matches your goal better: graphs that are much more attractive, closer in spirit to [dgist-slam/se2-diff-odom-uncertainty](https://github.com/dgist-slam/se2-diff-odom-uncertainty), but adapted to real ROS 2 bags and OptiTrack CSV exports.

## What we are building

The viewer will load:

- one ROS 2 bag (`.db3`)
- one OptiTrack CSV export
- one session config file describing which trajectory topic to use and how to align everything

It will then render:

- estimated trajectory vs ground truth
- multiple trajectory variants from the bag, such as `/odom`, `/odom_raw`, `/odom_raw_adapted`
- timeline scrubbing
- error-colored path overlays
- trajectory metrics such as RMSE, maximum error, duration, and path length

## Why this architecture

Your actual data format makes this separation the right choice:

- the ROS bag is a structured binary source with multiple candidate topics
- the OptiTrack export uses a multi-row header format
- the frontend should stay focused on rendering and interaction, not file parsing complexity

So the system becomes:

```text
ROS 2 bag + OptiTrack CSV + session.yaml
                |
                v
        Python normalization layer
                |
                v
       normalized experiment JSON
                |
                v
          React visualization app
```

## Real input formats observed

From the sample files you provided:

- bag topics include `/odom`, `/odom_raw`, `/odom_raw_adapted`, `/tf`, `/scan`, `/imu/data`, `/lstm_predictions`
- OptiTrack exports provide a multi-row header where the `ROBOT` rigid body has:
  - quaternion rotation as `X Y Z W`
  - position as `X Y Z`
  - time as `Time (Seconds)`

This repo is now aligned to those formats.

## Repository layout

```text
docs/
examples/
frontend/
src/rosbag_adaptive_localization_viewer/
```

## Current implementation status

Already included:

- updated system design
- normalized data format proposal
- real OptiTrack CSV parser for rigid body exports
- ROS 2 bag trajectory extractor skeleton using `rosbags`
- JSON export model for the frontend
- React frontend scaffold with a polished trajectory dashboard mock

## Quick start

### Python side

```bash
pip install -e .
python -m rosbag_adaptive_localization_viewer.cli inspect-bag path/to/bag.db3
python -m rosbag_adaptive_localization_viewer.cli inspect-optitrack path/to/optitrack.csv --rigid-body ROBOT
```

To build a frontend-ready JSON from a local session file in this workspace:

```bash
powershell -ExecutionPolicy Bypass -File scripts/build_experiment.ps1 examples/local-session.yaml frontend/public/demo-experiment.json
```

### Frontend side

```bash
cd frontend
npm install
npm run dev
```

## Next step

The next real milestone is connecting the frontend to normalized JSON exported from one of your actual experiments.

That connection is now in place through `frontend/public/demo-experiment.json`, and the frontend will try to load it automatically.

See:

- [docs/system-design.md](H:/Tesis/rosbag-adaptive-localization-viewer/docs/system-design.md)
- [docs/data-format.md](H:/Tesis/rosbag-adaptive-localization-viewer/docs/data-format.md)
- [examples/session.yaml](H:/Tesis/rosbag-adaptive-localization-viewer/examples/session.yaml)
