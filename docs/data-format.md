# Standard Data Format

## Goal

Define a stable experiment contract that matches how your real data arrives:

- ROS 2 bag as `.db3`
- OptiTrack export as multi-header CSV
- session config that resolves topic and alignment differences

## Experiment folder

```text
experiment_name/
  bag/
    metadata.yaml
    experiment_0.db3
  gt/
    optitrack.csv
  config/
    session.yaml
```

Required:

- `bag/*.db3`
- `config/session.yaml`

Optional:

- `gt/optitrack.csv`

## Session config

```yaml
experiment_name: prueba_adaptativo_lidar_01
bag_path: ../bag/prueba_adaptativo_lidar_20260508_181907_0.db3

estimate:
  primary_topic: /odom
  comparison_topics:
    - /odom_raw
    - /odom_raw_adapted

ground_truth:
  enabled: true
  csv_path: ../gt/Take 2026-05-04 12.46.07 PM.csv
  rigid_body_name: ROBOT

alignment:
  mode: first_sample
  time_offset_sec: 0.0

frames:
  estimate_frame: odom
  ground_truth_frame: optitrack_world
```

## Observed ROS 2 bag topics

From your sample bag, relevant trajectory-related topics include:

- `/odom`
- `/odom_raw`
- `/odom_raw_adapted`
- `/tf`

This means version 1 should support:

- one primary estimated trajectory
- zero or more comparison trajectories
- one optional ground truth trajectory

## OptiTrack CSV contract

Your OptiTrack file uses several header rows. For the rigid body `ROBOT`, the meaningful columns are:

- `Time (Seconds)`
- `Rotation / X`
- `Rotation / Y`
- `Rotation / Z`
- `Rotation / W`
- `Position / X`
- `Position / Y`
- `Position / Z`

So the normalized internal schema becomes:

```text
timestamp_sec
qx
qy
qz
qw
x_m
y_m
z_m
```

## Normalized frontend JSON

The backend should export:

```json
{
  "metadata": {
    "experiment_name": "prueba_adaptativo_lidar_01"
  },
  "trajectories": [
    {
      "id": "odom",
      "label": "/odom",
      "source_type": "estimate",
      "samples": [
        {
          "t_sec": 0.0,
          "x": 0.0,
          "y": 0.0,
          "z": 0.0,
          "yaw_rad": 0.0
        }
      ]
    }
  ],
  "metrics": {}
}
```

## Alignment assumptions

Keep alignment explicit in `session.yaml` because your setups may vary:

- different estimate topics
- different frame conventions
- different run start times
- occasional axis changes in OptiTrack interpretation

Initial supported alignment modes:

- `none`
- `first_sample`

## Best practice

1. Keep raw bags untouched.
2. Keep raw OptiTrack CSV untouched.
3. Create one `session.yaml` per experiment.
4. Put experiment-specific decisions only in `session.yaml`, not in code.
