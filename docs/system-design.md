# System Design

## Goal

Build an offline trajectory analysis system for ROS 2 localization experiments that feels closer to a polished interactive web demo than to RViz.

## Final architecture choice

The project now uses a split architecture:

- `Python` for extraction, normalization, metrics, and export
- `React + Vite` for the interactive frontend

This is the best compromise between:

- visual quality
- development speed
- flexibility for your changing bag and OptiTrack formats

## High-level flow

```text
ROS 2 bag (.db3) -----------\
                             \
OptiTrack CSV --------------- ---> Python normalization ---> experiment.json ---> React frontend
                              /
session.yaml ----------------/
```

## Why not RViz

RViz is excellent for robotic debugging, but it is not the best fit for this product goal.

For your use case, a custom web frontend is better because it gives:

- cleaner presentation
- richer comparison views
- easier metric summaries
- easier future export to reports or GitHub Pages
- more freedom for layout and visual identity

## Input reality from your data

### ROS 2 bag

Your sample bag already shows multiple useful trajectory candidates:

- `/odom`
- `/odom_raw`
- `/odom_raw_adapted`
- `/tf`

That means the viewer should not assume only one possible estimate source. The session config must declare which topic is the primary estimate and which optional comparison topics to overlay.

### OptiTrack CSV

Your OptiTrack export is not a simple flat CSV. It uses:

- metadata rows
- multi-row headers
- rigid body sections
- quaternion orientation columns
- position columns

The parser therefore needs to explicitly extract one rigid body such as `ROBOT`.

## Core backend modules

### 1. Session config loader

Reads experiment metadata and alignment choices from `session.yaml`.

### 2. ROS bag loader

Reads candidate trajectory topics from `.db3` bags using `rosbags`.

First target message types:

- `nav_msgs/msg/Odometry`
- `geometry_msgs/msg/PoseStamped`
- `tf2_msgs/msg/TFMessage`

### 3. OptiTrack loader

Parses multi-header CSV exports and extracts the rigid body trajectory into a normalized pose table.

### 4. Alignment and metrics

Transforms all sources into the same timeline and frame convention:

- timestamp normalization
- origin alignment
- interpolation
- per-sample position error
- trajectory-level summary metrics

### 5. Export layer

Produces a frontend-friendly `experiment.json`.

## Normalized experiment model

The frontend should receive one compact JSON structure:

```text
experiment
  metadata
  trajectories[]
  metrics
  timeline
```

Each trajectory contains:

- source id
- display name
- timestamps
- positions
- optional yaw
- optional error values

## Frontend responsibilities

The React app is responsible for:

- experiment loading
- layer toggles
- polished trajectory overlays
- timeline scrubbing
- summary cards
- responsive layout

It should not parse raw bags or raw OptiTrack exports directly.

## Planned visual experience

Inspired by the DGIST demo, but adapted to real experiment data:

- strong side control panel
- large trajectory canvas
- distinct styling for `/odom`, `/odom_raw`, `/odom_raw_adapted`, and GT
- animated focus marker for current time
- metric cards with clear hierarchy
- optional error heat strip and error timeline

## Near-term roadmap

1. Normalize one real bag and one OptiTrack CSV.
2. Export `experiment.json`.
3. Feed that into the frontend.
4. Add topic toggles and time scrubber.
5. Compute and render error metrics.
