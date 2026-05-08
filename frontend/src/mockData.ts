import type { ExperimentData, TrajectorySeries } from "./types";

const time = Array.from({ length: 180 }, (_, index) => index * 0.1);

function buildSeries(
  id: string,
  label: string,
  sourceType: TrajectorySeries["source_type"],
  color: string,
  xShift: number,
  yShift: number,
): TrajectorySeries {
  return {
    id,
    label,
    source_type: sourceType,
    samples: time.map((t) => {
      const x = t * 0.14 + Math.sin(t * 0.6) * 0.45 + xShift;
      const y = Math.cos(t * 0.28) * 1.8 + Math.sin(t * 0.12) * 0.5 + yShift;
      return {
        t_sec: t,
        x,
        y,
        z: 0,
        yaw_rad: 0,
        error_m: Math.abs(xShift) * 0.8 + Math.abs(yShift) * 0.3,
      };
    }),
  };
}

export const mockExperiment: ExperimentData = {
  metadata: {
    experiment_name: "prueba_adaptativo_lidar_01",
  },
  metrics: {
    rmse_position_m: 0.118,
    max_position_error_m: 0.324,
    duration_sec: 53.2,
    path_length_m: 8.4,
  },
  trajectories: [
  buildSeries("gt", "OptiTrack GT", "ground_truth", "#0f172a", 0, 0),
  buildSeries("odom", "/odom", "estimate", "#f97316", 0.08, -0.04),
  buildSeries("odom_raw", "/odom_raw", "comparison", "#38bdf8", 0.22, -0.12),
  buildSeries("odom_raw_adapted", "/odom_raw_adapted", "comparison", "#10b981", 0.12, -0.05),
  ],
};
