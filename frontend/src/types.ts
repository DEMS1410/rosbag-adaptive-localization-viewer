export type TrajectorySample = {
  t_sec: number;
  x: number;
  y: number;
  z?: number | null;
  yaw_rad?: number | null;
  error_m?: number | null;
  cov_xx?: number | null;
  cov_xy?: number | null;
  cov_yy?: number | null;
  yaw_var?: number | null;
};

export type TrajectorySeries = {
  id: string;
  label: string;
  source_type: "estimate" | "comparison" | "ground_truth";
  samples: TrajectorySample[];
};

export type ExperimentData = {
  metadata: {
    experiment_name: string;
  };
  trajectories: TrajectorySeries[];
  metrics: Record<string, number>;
  scene?: {
    map?: {
      resolution: number;
      width: number;
      height: number;
      origin: [number, number];
      occupied_points: [number, number][];
      grid: number[][];
    } | null;
    scans?: {
      t_sec: number;
      points: [number, number][];
    }[];
  };
};
