export type TrajectorySample = {
  t_sec: number;
  x: number;
  y: number;
  z?: number | null;
  yaw_rad?: number | null;
  error_m?: number | null;
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
};
