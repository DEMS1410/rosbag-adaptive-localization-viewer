import Plot from "react-plotly.js";
import { useEffect, useMemo, useRef, useState } from "react";
import type { ChangeEvent, MouseEvent } from "react";

import { mockExperiment } from "./mockData";
import type { ExperimentData, TrajectorySeries } from "./types";

const settingsStorageKey = "adaptive-localization-viewer-settings";

const defaultLayerColors: Record<string, string> = {
  ground_truth: "#0f172a",
  map_base: "#16a34a",
  odom: "#f97316",
  odom_raw: "#38bdf8",
  odom_raw_adapted: "#10b981",
  map: "#f8fafc",
  lidar: "#22d3ee",
  covariance: "#f472b6",
  yaw: "#fb923c",
};

const colorPresets: Record<string, Record<string, string>> = {
  dgist: defaultLayerColors,
  contrast: {
    ground_truth: "#f8fafc",
    map_base: "#22c55e",
    odom: "#f97316",
    odom_raw: "#22d3ee",
    odom_raw_adapted: "#34d399",
    map: "#94a3b8",
    lidar: "#f472b6",
    covariance: "#fb7185",
    yaw: "#fbbf24",
  },
  soft: {
    ground_truth: "#cbd5e1",
    map_base: "#4ade80",
    odom: "#fb923c",
    odom_raw: "#60a5fa",
    odom_raw_adapted: "#2dd4bf",
    map: "#e5e7eb",
    lidar: "#67e8f9",
    covariance: "#f9a8d4",
    yaw: "#fdba74",
  },
};

type ViewerSettings = {
  layerColors?: Record<string, string>;
  visibleIds?: string[];
  appMode?: "analysis" | "simulation";
  playbackMode?: "progressive" | "full";
  showMap?: boolean;
  showLidar?: boolean;
  showGhostTrajectories?: boolean;
  showCurrentMarkers?: boolean;
  showErrorColoredEstimate?: boolean;
  showCovarianceEllipses?: boolean;
  showErrorChart?: boolean;
  showWedge?: boolean;
  showKeyboardHints?: boolean;
  covarianceMode?: "current" | "trail";
  ellipseStride?: number;
  sigmaMultiplier?: number;
  tailWindowSec?: number;
  covarianceTrailCount?: number;
  playbackRate?: number;
};

function loadSettings(): ViewerSettings {
  if (typeof window === "undefined") {
    return {};
  }
  try {
    const raw = window.localStorage.getItem(settingsStorageKey);
    return raw ? (JSON.parse(raw) as ViewerSettings) : {};
  } catch {
    return {};
  }
}

function metricValue(experiment: ExperimentData, label: string): string {
  const metrics = experiment.metrics;
  const values: Record<string, string> = {
    "RMSE Position": metrics.rmse_position_m != null ? `${metrics.rmse_position_m.toFixed(3)} m` : "-",
    "Max Error": metrics.max_position_error_m != null ? `${metrics.max_position_error_m.toFixed(3)} m` : "-",
    Duration: metrics.duration_sec != null ? `${metrics.duration_sec.toFixed(1)} s` : "-",
    "Path Length": metrics.path_length_m != null ? `${metrics.path_length_m.toFixed(2)} m` : "-",
  };
  return values[label] ?? "-";
}

function currentSample(series: TrajectorySeries, timeSec: number) {
  if (series.samples.length === 0) {
    return null;
  }
  let nearest = series.samples[0];
  for (const sample of series.samples) {
    if (Math.abs(sample.t_sec - timeSec) < Math.abs(nearest.t_sec - timeSec)) {
      nearest = sample;
    }
  }
  return nearest;
}

function currentScan(
  scans: { t_sec: number; points: [number, number][] }[] | undefined,
  timeSec: number,
) {
  if (!scans || scans.length === 0) {
    return null;
  }
  let nearest = scans[0];
  for (const scan of scans) {
    if (Math.abs(scan.t_sec - timeSec) < Math.abs(nearest.t_sec - timeSec)) {
      nearest = scan;
    }
  }
  return nearest;
}

function samplesUntilTime(series: TrajectorySeries, timeSec: number) {
  const visible = series.samples.filter((sample) => sample.t_sec <= timeSec);
  if (visible.length > 0) {
    return visible;
  }
  return series.samples.length > 0 ? [series.samples[0]] : [];
}

function samplesInWindow(series: TrajectorySeries, timeSec: number, windowSec: number) {
  const start = Math.max(0, timeSec - windowSec);
  return series.samples.filter((sample) => sample.t_sec >= start && sample.t_sec <= timeSec);
}

function fmt(value: number | null | undefined, digits = 3, suffix = ""): string {
  if (value == null || Number.isNaN(value)) {
    return "-";
  }
  return `${value.toFixed(digits)}${suffix}`;
}

function ellipseTrace(
  x: number,
  y: number,
  covXx: number,
  covXy: number,
  covYy: number,
  sigma: number,
) {
  const traceX: number[] = [];
  const traceY: number[] = [];
  const mean = (covXx + covYy) / 2;
  const diff = (covXx - covYy) / 2;
  const root = Math.sqrt(Math.max(diff * diff + covXy * covXy, 0));
  const lambda1 = Math.max(mean + root, 1e-9);
  const lambda2 = Math.max(mean - root, 1e-9);
  const angle = 0.5 * Math.atan2(2 * covXy, covXx - covYy);
  const rx = sigma * Math.sqrt(lambda1);
  const ry = sigma * Math.sqrt(lambda2);

  for (let i = 0; i <= 36; i += 1) {
    const t = (Math.PI * 2 * i) / 36;
    const ex = rx * Math.cos(t);
    const ey = ry * Math.sin(t);
    traceX.push(x + ex * Math.cos(angle) - ey * Math.sin(angle));
    traceY.push(y + ex * Math.sin(angle) + ey * Math.cos(angle));
  }

  return { x: traceX, y: traceY };
}

function wedgeTrace(
  x: number,
  y: number,
  yaw: number,
  yawVar: number,
  radius: number,
  sigma: number,
) {
  const sigmaYaw = sigma * Math.sqrt(Math.max(yawVar, 1e-9));
  const start = yaw - sigmaYaw;
  const end = yaw + sigmaYaw;
  const xs = [x];
  const ys = [y];
  const segments = 20;
  for (let i = 0; i <= segments; i += 1) {
    const a = start + ((end - start) * i) / segments;
    xs.push(x + radius * Math.cos(a));
    ys.push(y + radius * Math.sin(a));
  }
  xs.push(x);
  ys.push(y);
  return { x: xs, y: ys };
}

export default function App() {
  const persistedSettings = useMemo(loadSettings, []);
  const pageRef = useRef<HTMLDivElement | null>(null);
  const apiBaseUrl = "http://127.0.0.1:8765";
  const [experiment, setExperiment] = useState<ExperimentData>(mockExperiment);
  const [selectedTime, setSelectedTime] = useState(0);
  const [visibleIds, setVisibleIds] = useState<string[]>(
    persistedSettings.visibleIds ?? mockExperiment.trajectories.map((trajectory) => trajectory.id),
  );
  const [isPlaying, setIsPlaying] = useState(false);
  const [playbackRate, setPlaybackRate] = useState(persistedSettings.playbackRate ?? 1);
  const [ellipseStride, setEllipseStride] = useState(persistedSettings.ellipseStride ?? 12);
  const [sigmaMultiplier, setSigmaMultiplier] = useState(persistedSettings.sigmaMultiplier ?? 3);
  const [showWedge, setShowWedge] = useState(persistedSettings.showWedge ?? true);
  const [showGhostTrajectories, setShowGhostTrajectories] = useState(
    persistedSettings.showGhostTrajectories ?? false,
  );
  const [showCurrentMarkers, setShowCurrentMarkers] = useState(persistedSettings.showCurrentMarkers ?? true);
  const [showErrorColoredEstimate, setShowErrorColoredEstimate] = useState(
    persistedSettings.showErrorColoredEstimate ?? true,
  );
  const [showCovarianceEllipses, setShowCovarianceEllipses] = useState(
    persistedSettings.showCovarianceEllipses ?? false,
  );
  const [showErrorChart, setShowErrorChart] = useState(persistedSettings.showErrorChart ?? true);
  const [playbackMode, setPlaybackMode] = useState<"progressive" | "full">(
    persistedSettings.playbackMode ?? "progressive",
  );
  const [appMode, setAppMode] = useState<"analysis" | "simulation">(persistedSettings.appMode ?? "analysis");
  const [covarianceMode, setCovarianceMode] = useState<"current" | "trail">(
    persistedSettings.covarianceMode ?? "current",
  );
  const [tailWindowSec, setTailWindowSec] = useState(persistedSettings.tailWindowSec ?? 8);
  const [covarianceTrailCount, setCovarianceTrailCount] = useState(
    persistedSettings.covarianceTrailCount ?? 6,
  );
  const [showMap, setShowMap] = useState(persistedSettings.showMap ?? true);
  const [showLidar, setShowLidar] = useState(persistedSettings.showLidar ?? false);
  const [showKeyboardHints, setShowKeyboardHints] = useState(persistedSettings.showKeyboardHints ?? true);
  const [layerColors, setLayerColors] = useState({
    ...defaultLayerColors,
    ...(persistedSettings.layerColors ?? {}),
  });
  const [apiStatus, setApiStatus] = useState<"checking" | "online" | "offline">("checking");
  const [bagUploadFile, setBagUploadFile] = useState<File | null>(null);
  const [optitrackUploadFile, setOptitrackUploadFile] = useState<File | null>(null);
  const [uploadExperimentName, setUploadExperimentName] = useState("uploaded_experiment");
  const [uploadPrimaryTopic, setUploadPrimaryTopic] = useState("/odom");
  const [uploadComparisonTopics, setUploadComparisonTopics] = useState("/odom_raw,/odom_raw_adapted");
  const [uploadRigidBody, setUploadRigidBody] = useState("ROBOT");
  const [includeOptitrackUpload, setIncludeOptitrackUpload] = useState(true);
  const [isBuildingUpload, setIsBuildingUpload] = useState(false);
  const [uploadError, setUploadError] = useState<string | null>(null);

  useEffect(() => {
    setVisibleIds((current) => {
      const availableIds = experiment.trajectories.map((trajectory) => trajectory.id);
      const persistedVisible = current.filter((id) => availableIds.includes(id));
      return persistedVisible.length > 0 ? persistedVisible : availableIds;
    });
  }, [experiment]);

  useEffect(() => {
    if (typeof window === "undefined") {
      return;
    }
    const settings: ViewerSettings = {
      layerColors,
      visibleIds,
      appMode,
      playbackMode,
      showMap,
      showLidar,
      showGhostTrajectories,
      showCurrentMarkers,
      showErrorColoredEstimate,
      showCovarianceEllipses,
      showErrorChart,
      showWedge,
      showKeyboardHints,
      covarianceMode,
      ellipseStride,
      sigmaMultiplier,
      tailWindowSec,
      covarianceTrailCount,
      playbackRate,
    };
    window.localStorage.setItem(settingsStorageKey, JSON.stringify(settings));
  }, [
    layerColors,
    visibleIds,
    appMode,
    playbackMode,
    showMap,
    showLidar,
    showGhostTrajectories,
    showCurrentMarkers,
    showErrorColoredEstimate,
    showCovarianceEllipses,
    showErrorChart,
    showWedge,
    showKeyboardHints,
    covarianceMode,
    ellipseStride,
    sigmaMultiplier,
    tailWindowSec,
    covarianceTrailCount,
    playbackRate,
  ]);

  useEffect(() => {
    let cancelled = false;
    fetch("/demo-experiment.json")
      .then((response) => {
        if (!response.ok) {
          throw new Error(`HTTP ${response.status}`);
        }
        return response.json() as Promise<ExperimentData>;
      })
      .then((data) => {
        if (!cancelled) {
          setExperiment(data);
          setSelectedTime(0);
        }
      })
      .catch(() => {
        // Keep mock data fallback.
      });

    return () => {
      cancelled = true;
    };
  }, []);

  useEffect(() => {
    let cancelled = false;
    fetch(`${apiBaseUrl}/health`)
      .then((response) => {
        if (!response.ok) {
          throw new Error("offline");
        }
        return response.json();
      })
      .then(() => {
        if (!cancelled) {
          setApiStatus("online");
        }
      })
      .catch(() => {
        if (!cancelled) {
          setApiStatus("offline");
        }
      });
    return () => {
      cancelled = true;
    };
  }, [apiBaseUrl]);

  const visibleSeries = useMemo(
    () => experiment.trajectories.filter((trajectory) => visibleIds.includes(trajectory.id)),
    [experiment, visibleIds],
  );

  const isProgressiveMode = playbackMode === "progressive";
  const isSimulationMode = appMode === "simulation";
  const maxTime = useMemo(
    () =>
      Math.max(
        0,
        ...experiment.trajectories.map((trajectory) =>
          trajectory.samples.length > 0 ? trajectory.samples[trajectory.samples.length - 1].t_sec : 0,
        ),
      ),
    [experiment],
  );

  const estimateSeries = useMemo(
    () => experiment.trajectories.find((trajectory) => trajectory.source_type === "estimate") ?? null,
    [experiment],
  );
  const gtSeries = useMemo(
    () => experiment.trajectories.find((trajectory) => trajectory.source_type === "ground_truth") ?? null,
    [experiment],
  );
  const estimateSample = estimateSeries ? currentSample(estimateSeries, selectedTime) : null;
  const gtSample = gtSeries ? currentSample(gtSeries, selectedTime) : null;
  const currentSceneScan = useMemo(
    () => currentScan(experiment.scene?.scans, selectedTime),
    [experiment, selectedTime],
  );
  const progressPct = maxTime > 0 ? (selectedTime / maxTime) * 100 : 0;
  const activeSceneBadges = [
    appMode === "simulation" ? "Simulation" : "Analysis",
    playbackMode === "progressive" ? "Progressive" : "Full view",
    showMap ? "Map" : null,
    showLidar ? "Lidar" : null,
    showCovarianceEllipses ? "Covariance" : null,
  ].filter(Boolean) as string[];

  useEffect(() => {
    if (!isPlaying) {
      return;
    }
    const timer = window.setInterval(() => {
      setSelectedTime((current) => {
        const next = current + 0.08 * playbackRate;
        if (next >= maxTime) {
          setIsPlaying(false);
          return maxTime;
        }
        return next;
      });
    }, 80);
    return () => window.clearInterval(timer);
  }, [isPlaying, playbackRate, maxTime]);

  useEffect(() => {
    if (!isSimulationMode) {
      return;
    }
    pageRef.current?.focus();
    function onKeyDown(event: KeyboardEvent) {
      if (event.target instanceof HTMLInputElement || event.target instanceof HTMLSelectElement) {
        return;
      }
      if (event.code === "Space") {
        event.preventDefault();
        setIsPlaying((value) => !value);
        return;
      }
      if (event.key === "ArrowRight" || event.key.toLowerCase() === "d") {
        event.preventDefault();
        setSelectedTime((value) => Math.min(maxTime, value + 0.5));
        return;
      }
      if (event.key === "ArrowLeft" || event.key.toLowerCase() === "a") {
        event.preventDefault();
        setSelectedTime((value) => Math.max(0, value - 0.5));
        return;
      }
      if (event.key.toLowerCase() === "r") {
        setSelectedTime(0);
        setIsPlaying(false);
        return;
      }
      if (event.key.toLowerCase() === "m") {
        setShowMap((value) => !value);
        return;
      }
      if (event.key.toLowerCase() === "l") {
        setShowLidar((value) => !value);
        return;
      }
      if (event.key.toLowerCase() === "c") {
        setShowCovarianceEllipses((value) => !value);
      }
    }
    window.addEventListener("keydown", onKeyDown);
    return () => window.removeEventListener("keydown", onKeyDown);
  }, [isSimulationMode, maxTime]);

  const sceneTraces = useMemo(() => {
    const traces: object[] = [];
    const map = experiment.scene?.map;
    if (showMap && map?.grid?.length) {
      const heatmapRows = [...map.grid].reverse();
      traces.push({
        z: heatmapRows,
        type: "heatmap" as const,
        name: "map",
        x0: map.origin[0] + map.resolution / 2,
        dx: map.resolution,
        y0: map.origin[1] + map.resolution / 2,
        dy: map.resolution,
        zmin: -1,
        zmax: 100,
        colorscale: [
          [0.0, "#f8fafc"],
          [0.009, "#f8fafc"],
          [0.01, "#2b0a4a"],
          [0.6, "#2b0a4a"],
          [0.61, "#fde047"],
          [1.0, "#fde047"],
        ],
        showscale: false,
        opacity: 0.92,
        hovertemplate: "map x=%{x:.2f}<br>y=%{y:.2f}<br>occ=%{z}<extra></extra>",
      });
    } else if (showMap && map?.occupied_points?.length) {
      traces.push({
        x: map.occupied_points.map((point) => point[0]),
        y: map.occupied_points.map((point) => point[1]),
        type: "scattergl" as const,
        mode: "markers",
        name: "map",
        marker: {
          size: 7,
          color: layerColors.map,
          symbol: "square",
        },
        opacity: 0.9,
        hoverinfo: "skip" as const,
      });
    }
    if (showLidar && currentSceneScan?.points?.length) {
      traces.push({
        x: currentSceneScan.points.map((point) => point[0]),
        y: currentSceneScan.points.map((point) => point[1]),
        type: "scattergl" as const,
        mode: "markers",
        name: "lidar",
        marker: {
          size: isSimulationMode ? 5 : 4,
          color: layerColors.lidar,
        },
      });
    }
    return traces;
  }, [experiment, showMap, showLidar, currentSceneScan, isSimulationMode, layerColors]);

  const plotBounds = useMemo(() => {
    const xs: number[] = [];
    const ys: number[] = [];

    for (const trajectory of visibleSeries) {
      for (const sample of trajectory.samples) {
        xs.push(sample.x);
        ys.push(sample.y);
      }
    }

    if (showMap) {
      for (const point of experiment.scene?.map?.occupied_points ?? []) {
        xs.push(point[0]);
        ys.push(point[1]);
      }
    }

    if (showLidar) {
      for (const point of currentSceneScan?.points ?? []) {
        xs.push(point[0]);
        ys.push(point[1]);
      }
    }

    if (xs.length === 0 || ys.length === 0) {
      return null;
    }

    let xmin = Math.min(...xs);
    let xmax = Math.max(...xs);
    let ymin = Math.min(...ys);
    let ymax = Math.max(...ys);
    const xspan = Math.max(xmax - xmin, 1);
    const yspan = Math.max(ymax - ymin, 1);
    const pad = Math.max(xspan, yspan) * 0.12;

    xmin -= pad;
    xmax += pad;
    ymin -= pad;
    ymax += pad;

    return {
      x: [xmin, xmax] as [number, number],
      y: [ymin, ymax] as [number, number],
    };
  }, [visibleSeries, experiment, showMap, showLidar, currentSceneScan]);

  const plotData = visibleSeries.flatMap((trajectory) => {
    const color = layerColors[trajectory.id] ?? "#8b5cf6";
    const focus = currentSample(trajectory, selectedTime);
    const playedSamples = samplesUntilTime(trajectory, selectedTime);
    const visibleSamples = isSimulationMode
      ? samplesInWindow(trajectory, selectedTime, tailWindowSec)
      : isProgressiveMode
        ? playedSamples
        : trajectory.samples;
    const traces: object[] = [];

    if (!isSimulationMode && isProgressiveMode && showGhostTrajectories) {
      traces.push({
        x: trajectory.samples.map((sample) => sample.x),
        y: trajectory.samples.map((sample) => sample.y),
        type: "scattergl" as const,
        mode: "lines",
        name: `${trajectory.label} full`,
        line: {
          color:
            trajectory.source_type === "ground_truth" ? "rgba(148,163,184,0.25)" : "rgba(71,85,105,0.35)",
          width: trajectory.source_type === "ground_truth" ? 3 : 2,
          dash: "dot",
        },
        hoverinfo: "skip" as const,
        showlegend: false,
      });
    }

    if (
      showErrorColoredEstimate &&
      trajectory.source_type === "estimate" &&
      trajectory.samples.some((sample) => sample.error_m != null)
    ) {
      traces.push({
        x: visibleSamples.map((sample) => sample.x),
        y: visibleSamples.map((sample) => sample.y),
        type: "scattergl" as const,
        mode: "markers",
        name: `${trajectory.label} error`,
        marker: {
          size: isSimulationMode ? 9 : 8,
          color: visibleSamples.map((sample) => sample.error_m ?? 0),
          colorscale: [
            [0, "#10b981"],
            [0.5, "#f59e0b"],
            [1, "#ef4444"],
          ],
          cmin: 0,
          cmax: Math.max(...trajectory.samples.map((sample) => sample.error_m ?? 0), 0.1),
          colorbar: {
            title: "Err [m]",
            thickness: 12,
            bgcolor: "rgba(10,14,26,0.2)",
            outlinecolor: "#334155",
            tickcolor: "#94a3b8",
          },
        },
        hovertemplate:
          "<b>%{text}</b><br>x=%{x:.3f} m<br>y=%{y:.3f} m<br>err=%{marker.color:.3f} m<extra></extra>",
        text: visibleSamples.map(() => trajectory.label),
        showlegend: false,
      });
    }

    traces.push({
      x: visibleSamples.map((sample) => sample.x),
      y: visibleSamples.map((sample) => sample.y),
      type: "scattergl" as const,
      mode: "lines",
      name: trajectory.label,
      line: {
        color,
        width: trajectory.source_type === "ground_truth" ? (isSimulationMode ? 6 : 5) : isSimulationMode ? 4 : 3,
        dash: trajectory.source_type === "comparison" ? "dot" : "solid",
      },
    });

    if (showCurrentMarkers && focus) {
      traces.push({
        x: [focus.x],
        y: [focus.y],
        type: "scatter" as const,
        mode: "markers",
        name: `${trajectory.label} @ t`,
        marker: {
          color,
          size: isSimulationMode ? 15 : 12,
          line: { color: "#ffffff", width: 2 },
        },
        showlegend: false,
      });
    }

    return traces;
  });

  const covarianceTraces = useMemo(() => {
    if (!estimateSeries || !showCovarianceEllipses) {
      return [];
    }
    const played = isSimulationMode
      ? samplesInWindow(estimateSeries, selectedTime, tailWindowSec)
      : isProgressiveMode
        ? samplesUntilTime(estimateSeries, selectedTime)
        : estimateSeries.samples;
    const traces: object[] = [];
    const sourceSamples =
      covarianceMode === "current" && played.length > 0
        ? [played[played.length - 1]]
        : played.filter((_, index) => index % ellipseStride === 0).slice(-covarianceTrailCount);

    for (let index = 0; index < sourceSamples.length; index += 1) {
      const sample = sourceSamples[index];
      if (
        sample.cov_xx == null ||
        sample.cov_xy == null ||
        sample.cov_yy == null ||
        !Number.isFinite(sample.cov_xx) ||
        !Number.isFinite(sample.cov_xy) ||
        !Number.isFinite(sample.cov_yy)
      ) {
        continue;
      }
      const ellipse = ellipseTrace(
        sample.x,
        sample.y,
        sample.cov_xx,
        sample.cov_xy,
        sample.cov_yy,
        sigmaMultiplier,
      );
      traces.push({
        x: ellipse.x,
        y: ellipse.y,
        type: "scattergl" as const,
        mode: "lines",
        name: "xy Cov",
        line: {
          color: covarianceMode === "current" ? layerColors.covariance : `${layerColors.covariance}88`,
          width: covarianceMode === "current" ? 2.5 : 1.5,
        },
        fill: "toself" as const,
        fillcolor: covarianceMode === "current" ? `${layerColors.covariance}22` : `${layerColors.covariance}12`,
        showlegend: index === 0,
        hoverinfo: "skip" as const,
      });
    }

    const focus = currentSample(estimateSeries, selectedTime);
    if (showWedge && focus?.yaw_rad != null && focus?.yaw_var != null) {
      const wedge = wedgeTrace(focus.x, focus.y, focus.yaw_rad, focus.yaw_var, 0.45, sigmaMultiplier);
      traces.push({
        x: wedge.x,
        y: wedge.y,
        type: "scattergl" as const,
        mode: "lines",
        name: "yaw ±sigma",
        line: {
          color: layerColors.yaw,
          width: 1.5,
        },
        fill: "toself" as const,
        fillcolor: `${layerColors.yaw}20`,
      });
    }

    return traces;
  }, [
    estimateSeries,
    selectedTime,
    ellipseStride,
    sigmaMultiplier,
    showWedge,
    showCovarianceEllipses,
    isProgressiveMode,
    isSimulationMode,
    covarianceMode,
    covarianceTrailCount,
    tailWindowSec,
    layerColors,
  ]);

  const errorSeries = useMemo(
    () =>
      experiment.trajectories.find(
        (trajectory) =>
          trajectory.source_type === "estimate" && trajectory.samples.some((sample) => sample.error_m != null),
      ),
    [experiment],
  );

  const errorPlotData =
    errorSeries == null
      ? []
      : [
          {
            x: errorSeries.samples.map((sample) => sample.t_sec),
            y: errorSeries.samples.map((sample) => sample.error_m ?? null),
            type: "scatter" as const,
            mode: "lines",
            name: "Position Error",
            line: {
              color: "#ef4444",
              width: 3,
            },
            fill: "tozeroy" as const,
            fillcolor: "rgba(239, 68, 68, 0.14)",
          },
        ];

  async function onFileSelected(event: ChangeEvent<HTMLInputElement>) {
    const file = event.target.files?.[0];
    if (!file) {
      return;
    }
    const text = await file.text();
    const parsed = JSON.parse(text) as ExperimentData;
    setExperiment(parsed);
    setSelectedTime(0);
  }

  async function onBuildFromUploads() {
    if (!bagUploadFile) {
      setUploadError("Select a ROS 2 .db3 bag first.");
      return;
    }

    setIsBuildingUpload(true);
    setUploadError(null);

    const formData = new FormData();
    formData.append("bag_file", bagUploadFile);
    formData.append("experiment_name", uploadExperimentName);
    formData.append("primary_topic", uploadPrimaryTopic);
    formData.append("comparison_topics", uploadComparisonTopics);
    formData.append("rigid_body_name", uploadRigidBody);
    formData.append("include_optitrack", includeOptitrackUpload ? "true" : "false");
    if (includeOptitrackUpload && optitrackUploadFile) {
      formData.append("optitrack_file", optitrackUploadFile);
    }

    try {
      const response = await fetch(`${apiBaseUrl}/build-experiment`, {
        method: "POST",
        body: formData,
      });
      const payload = (await response.json()) as ExperimentData & { error?: string };
      if (!response.ok || payload.error) {
        throw new Error(payload.error ?? `HTTP ${response.status}`);
      }
      setExperiment(payload);
      setSelectedTime(0);
      setApiStatus("online");
    } catch (error) {
      setApiStatus("offline");
      setUploadError(error instanceof Error ? error.message : "Upload failed");
    } finally {
      setIsBuildingUpload(false);
    }
  }

  function onShellClick(event: MouseEvent<HTMLDivElement>) {
    if (!isSimulationMode) {
      return;
    }
    if (event.target === event.currentTarget) {
      pageRef.current?.focus();
    }
  }

  return (
    <div
      ref={pageRef}
      className="page-shell"
      tabIndex={0}
      onClick={onShellClick}
    >
      <aside className="sidebar">
        <div className="panel hero-panel">
          <p className="eyebrow">SE(2) Experiment Viewer</p>
          <h1>Adaptive Localization</h1>
          <p className="lede">
            A trajectory analysis cockpit for ROS 2 odometry, OptiTrack ground truth, playback,
            and error inspection.
          </p>
          <div className="hero-pills">
            <span>Offline</span>
            <span>ROS 2 Bag</span>
            <span>OptiTrack GT</span>
          </div>
        </div>

        <div className="panel">
          <div className="panel-title-row">
            <h2>Experiment</h2>
            <span className={`status-badge ${apiStatus === "online" ? "status-ok" : "status-muted"}`}>
              {apiStatus === "online" ? "API Online" : apiStatus === "checking" ? "Checking API" : "API Offline"}
            </span>
          </div>
          <div className="meta-row">
            <span>Name</span>
            <strong>{experiment.metadata.experiment_name}</strong>
          </div>
          <div className="meta-row">
            <span>Bag</span>
            <strong>prueba_adaptativo_lidar</strong>
          </div>
          <div className="meta-row">
            <span>GT rigid body</span>
            <strong>ROBOT</strong>
          </div>
          <div className="meta-row">
            <span>Primary topic</span>
            <strong>/odom</strong>
          </div>
          <div className="meta-row">
            <span>Visible layers</span>
            <strong>{visibleSeries.length}</strong>
          </div>
          {experiment.scene?.map_base_method ? (
            <div className="meta-row">
              <span>Map/base TF</span>
              <strong>{experiment.scene.map_base_method}</strong>
            </div>
          ) : null}
          <label className="upload-box">
            <span>Load experiment JSON</span>
            <input accept=".json,application/json" type="file" onChange={onFileSelected} />
          </label>
          <div className="upload-divider">or build from source files</div>
          <div className="upload-form">
            <label className="field-block">
              <span>Experiment name</span>
              <input
                className="text-input"
                type="text"
                value={uploadExperimentName}
                onChange={(event) => setUploadExperimentName(event.target.value)}
              />
            </label>
            <label className="field-block">
              <span>ROS 2 bag (.db3)</span>
              <input
                accept=".db3"
                type="file"
                onChange={(event) => setBagUploadFile(event.target.files?.[0] ?? null)}
              />
            </label>
            <label className="field-block">
              <span>Primary topic</span>
              <input
                className="text-input"
                type="text"
                value={uploadPrimaryTopic}
                onChange={(event) => setUploadPrimaryTopic(event.target.value)}
              />
            </label>
            <label className="field-block">
              <span>Comparison topics</span>
              <input
                className="text-input"
                type="text"
                value={uploadComparisonTopics}
                onChange={(event) => setUploadComparisonTopics(event.target.value)}
              />
            </label>
            <label className="check-row compact-check">
              <input
                checked={includeOptitrackUpload}
                type="checkbox"
                onChange={(event) => setIncludeOptitrackUpload(event.target.checked)}
              />
              <span>Include OptiTrack CSV</span>
            </label>
            {includeOptitrackUpload ? (
              <>
                <label className="field-block">
                  <span>OptiTrack CSV</span>
                  <input
                    accept=".csv,text/csv"
                    type="file"
                    onChange={(event) => setOptitrackUploadFile(event.target.files?.[0] ?? null)}
                  />
                </label>
                <label className="field-block">
                  <span>Rigid body</span>
                  <input
                    className="text-input"
                    type="text"
                    value={uploadRigidBody}
                    onChange={(event) => setUploadRigidBody(event.target.value)}
                  />
                </label>
              </>
            ) : null}
            <button
              className="control-btn primary build-upload-btn"
              type="button"
              disabled={isBuildingUpload || apiStatus === "checking"}
              onClick={onBuildFromUploads}
            >
              {isBuildingUpload ? "Building..." : "Build from bag + CSV"}
            </button>
            {uploadError ? <p className="upload-error">{uploadError}</p> : null}
            <p className="panel-help">
              Start the local API with <code>python -m rosbag_adaptive_localization_viewer.cli serve</code>.
            </p>
          </div>
        </div>

        <div className="panel">
          <h2>Layers</h2>
          <div className="toggle-list">
            {experiment.trajectories.map((trajectory) => {
              const checked = visibleIds.includes(trajectory.id);
              const color = layerColors[trajectory.id] ?? "#8b5cf6";
              return (
                <label key={trajectory.id} className="toggle-item">
                  <input
                    checked={checked}
                    type="checkbox"
                    onChange={() =>
                      setVisibleIds((current) =>
                        checked
                          ? current.filter((id) => id !== trajectory.id)
                          : [...current, trajectory.id],
                      )
                    }
                  />
                  <span className="swatch" style={{ backgroundColor: color }} />
                  <span>{trajectory.label}</span>
                  <span className="toggle-meta">{trajectory.samples.length}</span>
                </label>
              );
            })}
          </div>
        </div>

        <div className="panel">
          <h2>Colors</h2>
          <p className="panel-help">Choose the color of each trajectory and overlay.</p>
          <div className="preset-row">
            <button className="preset-btn" type="button" onClick={() => setLayerColors(colorPresets.dgist)}>
              DGIST
            </button>
            <button className="preset-btn" type="button" onClick={() => setLayerColors(colorPresets.contrast)}>
              High Contrast
            </button>
            <button className="preset-btn" type="button" onClick={() => setLayerColors(colorPresets.soft)}>
              Soft
            </button>
          </div>
          <div className="color-grid">
            {[
              { id: "odom", label: "/odom" },
              { id: "odom_raw", label: "/odom_raw" },
              { id: "odom_raw_adapted", label: "/odom_raw_adapted" },
              { id: "map_base", label: "map -> base" },
              { id: "ground_truth", label: "ROBOT / GT" },
              { id: "map", label: "Map" },
              { id: "lidar", label: "Lidar" },
              { id: "covariance", label: "XY covariance" },
              { id: "yaw", label: "Yaw wedge" },
            ].map((item) => (
              <label key={item.id} className="color-row">
                <span>{item.label}</span>
                <input
                  aria-label={`Color for ${item.label}`}
                  type="color"
                  value={layerColors[item.id] ?? "#ffffff"}
                  onChange={(event) =>
                    setLayerColors((current) => ({
                      ...current,
                      [item.id]: event.target.value,
                    }))
                  }
                />
              </label>
            ))}
          </div>
          <button className="control-btn reset-colors-btn" type="button" onClick={() => setLayerColors(defaultLayerColors)}>
            Reset colors
          </button>
        </div>

        <div className="panel">
          <div className="panel-title-row">
            <h2>Mode</h2>
            <span className="time-chip">{isSimulationMode ? "Live-style" : "Study"}</span>
          </div>
          <div className="mode-row">
            <button
              className={`mode-btn ${appMode === "analysis" ? "active" : ""}`}
              type="button"
              onClick={() => setAppMode("analysis")}
            >
              Analysis
            </button>
            <button
              className={`mode-btn ${appMode === "simulation" ? "active" : ""}`}
              type="button"
              onClick={() => setAppMode("simulation")}
            >
              Simulation
            </button>
          </div>
          {isSimulationMode ? (
            <div className="slider-block">
              <div className="slider-label-row">
                <span>Visible tail window</span>
                <strong>{tailWindowSec.toFixed(0)} s</strong>
              </div>
              <input
                className="scrubber"
                min={2}
                max={30}
                step={1}
                type="range"
                value={tailWindowSec}
                onChange={(event) => setTailWindowSec(Number(event.target.value))}
              />
            </div>
          ) : null}
          {isSimulationMode && showKeyboardHints ? (
            <div className="keyboard-card">
              <strong>Keyboard</strong>
              <span>`Space` play/pause</span>
              <span>`A / D` or arrows step</span>
              <span>`R` reset</span>
              <span>`M` map</span>
              <span>`L` lidar</span>
              <span>`C` covariance</span>
            </div>
          ) : null}
        </div>

        <div className="panel">
          <div className="panel-title-row">
            <h2>Playback</h2>
            <span className="time-chip">{selectedTime.toFixed(2)} s</span>
          </div>
          <div className="playback-row">
            <button className="control-btn" type="button" onClick={() => setSelectedTime(0)}>
              Reset
            </button>
            <button className="control-btn primary" type="button" onClick={() => setIsPlaying((v) => !v)}>
              {isPlaying ? "Pause" : "Play"}
            </button>
            <button
              className="control-btn"
              type="button"
              onClick={() => setSelectedTime((v) => Math.min(maxTime, v + 1))}
            >
              +1s
            </button>
          </div>
          <input
            className="scrubber"
            max={maxTime}
            min={0}
            step={0.1}
            type="range"
            value={selectedTime}
            onChange={(event) => setSelectedTime(Number(event.target.value))}
          />
          <div className="progress-rail">
            <div className="progress-fill" style={{ width: `${progressPct}%` }} />
          </div>
          <div className="meta-row compact">
            <span>Rate</span>
            <select
              className="rate-select"
              value={playbackRate}
              onChange={(event) => setPlaybackRate(Number(event.target.value))}
            >
              <option value={0.5}>0.5x</option>
              <option value={1}>1.0x</option>
              <option value={2}>2.0x</option>
              <option value={4}>4.0x</option>
            </select>
          </div>
          <div className="mode-row">
            <button
              className={`mode-btn ${playbackMode === "progressive" ? "active" : ""}`}
              type="button"
              onClick={() => setPlaybackMode("progressive")}
            >
              Progressive
            </button>
            <button
              className={`mode-btn ${playbackMode === "full" ? "active" : ""}`}
              type="button"
              onClick={() => setPlaybackMode("full")}
            >
              Full View
            </button>
          </div>
        </div>

        <div className="panel">
          <h2>Scene Controls</h2>
          <label className="check-row">
            <input
              checked={showGhostTrajectories}
              type="checkbox"
              onChange={(event) => setShowGhostTrajectories(event.target.checked)}
            />
            <span>Show full trajectory ghost</span>
          </label>
          <label className="check-row">
            <input
              checked={showCurrentMarkers}
              type="checkbox"
              onChange={(event) => setShowCurrentMarkers(event.target.checked)}
            />
            <span>Show current markers</span>
          </label>
          <label className="check-row">
            <input
              checked={showErrorColoredEstimate}
              type="checkbox"
              onChange={(event) => setShowErrorColoredEstimate(event.target.checked)}
            />
            <span>Show error-colored estimate</span>
          </label>
          <label className="check-row">
            <input
              checked={showCovarianceEllipses}
              type="checkbox"
              onChange={(event) => setShowCovarianceEllipses(event.target.checked)}
            />
            <span>Show pink XY covariance ellipses</span>
          </label>
          <label className="check-row">
            <input checked={showMap} type="checkbox" onChange={(event) => setShowMap(event.target.checked)} />
            <span>Show map</span>
          </label>
          <label className="check-row">
            <input checked={showLidar} type="checkbox" onChange={(event) => setShowLidar(event.target.checked)} />
            <span>Show lidar</span>
          </label>
          <label className="check-row">
            <input checked={showWedge} type="checkbox" onChange={(event) => setShowWedge(event.target.checked)} />
            <span>Show orange yaw wedge</span>
          </label>
          <label className="check-row">
            <input
              checked={showErrorChart}
              type="checkbox"
              onChange={(event) => setShowErrorChart(event.target.checked)}
            />
            <span>Show error chart</span>
          </label>
          <label className="check-row">
            <input
              checked={showKeyboardHints}
              type="checkbox"
              onChange={(event) => setShowKeyboardHints(event.target.checked)}
            />
            <span>Show keyboard hints</span>
          </label>
        </div>

        <div className="panel">
          <h2>Covariance</h2>
          <p className="panel-help">
            Pink ellipses are XY position uncertainty from `/odom`. Orange wedge shows yaw uncertainty.
          </p>
          <div className="meta-row compact">
            <span>Covariance mode</span>
            <select
              className="rate-select"
              value={covarianceMode}
              onChange={(event) => setCovarianceMode(event.target.value as "current" | "trail")}
            >
              <option value="current">Current only</option>
              <option value="trail">Recent trail</option>
            </select>
          </div>
          <div className="slider-block">
            <div className="slider-label-row">
              <span>Draw every K steps</span>
              <strong>{ellipseStride}</strong>
            </div>
            <input
              className="scrubber"
              min={4}
              max={40}
              step={1}
              type="range"
              value={ellipseStride}
              onChange={(event) => setEllipseStride(Number(event.target.value))}
            />
          </div>
          {covarianceMode === "trail" ? (
            <div className="slider-block">
              <div className="slider-label-row">
                <span>Trail ellipse count</span>
                <strong>{covarianceTrailCount}</strong>
              </div>
              <input
                className="scrubber"
                min={1}
                max={20}
                step={1}
                type="range"
                value={covarianceTrailCount}
                onChange={(event) => setCovarianceTrailCount(Number(event.target.value))}
              />
            </div>
          ) : null}
          <div className="slider-block">
            <div className="slider-label-row">
              <span>Sigma multiplier</span>
              <strong>{sigmaMultiplier.toFixed(1)} sigma</strong>
            </div>
            <input
              className="scrubber"
              min={1}
              max={4}
              step={0.5}
              type="range"
              value={sigmaMultiplier}
              onChange={(event) => setSigmaMultiplier(Number(event.target.value))}
            />
          </div>
        </div>
      </aside>

      <main className="main-stage">
        <section className="headline-bar">
          <div>
            <p className="eyebrow">Trajectory Analysis</p>
            <h2 className="headline-title">Estimate, adapted odometry, ground truth, map, and lidar in one synchronized view</h2>
            <div className="headline-badges">
              {activeSceneBadges.map((badge) => (
                <span key={badge} className="headline-badge">
                  {badge}
                </span>
              ))}
            </div>
          </div>
          <div className="headline-meta">
            <span>{experiment.trajectories.length} tracks</span>
            <span>{fmt(maxTime, 1, " s")} horizon</span>
          </div>
        </section>

        <section className="top-control-rack">
          <article className="rack-card">
            <span className="rack-label">Mode</span>
            <div className="rack-actions">
              <button
                className={`mode-btn ${appMode === "analysis" ? "active" : ""}`}
                type="button"
                onClick={() => setAppMode("analysis")}
              >
                Analysis
              </button>
              <button
                className={`mode-btn ${appMode === "simulation" ? "active" : ""}`}
                type="button"
                onClick={() => setAppMode("simulation")}
              >
                Simulation
              </button>
            </div>
          </article>

          <article className="rack-card">
            <span className="rack-label">Playback</span>
            <div className="rack-actions rack-actions-triple">
              <button className="control-btn" type="button" onClick={() => setSelectedTime(0)}>
                Reset
              </button>
              <button className="control-btn primary" type="button" onClick={() => setIsPlaying((v) => !v)}>
                {isPlaying ? "Pause" : "Play"}
              </button>
              <button
                className="control-btn"
                type="button"
                onClick={() => setSelectedTime((v) => Math.min(maxTime, v + 1))}
              >
                +1s
              </button>
            </div>
          </article>

          <article className="rack-card">
            <span className="rack-label">Rate</span>
            <div className="rack-inline">
              <span className="time-chip">{selectedTime.toFixed(2)} s</span>
              <select
                className="rate-select"
                value={playbackRate}
                onChange={(event) => setPlaybackRate(Number(event.target.value))}
              >
                <option value={0.5}>0.5x</option>
                <option value={1}>1.0x</option>
                <option value={2}>2.0x</option>
                <option value={4}>4.0x</option>
              </select>
            </div>
          </article>
        </section>

        <section className="panel chart-panel">
          <div className="panel-head">
            <div>
              <p className="eyebrow">Spatial View</p>
              <h2>Trajectory Overlay</h2>
            </div>
            <div className="legend-note">
              <strong>{selectedTime.toFixed(2)} s</strong>
              <span>Simulation adds map/lidar overlays and keyboard control</span>
            </div>
          </div>

          <Plot
            data={[...sceneTraces, ...plotData, ...covarianceTraces]}
            layout={{
              autosize: true,
              paper_bgcolor: "rgba(0,0,0,0)",
              plot_bgcolor: "#07111f",
              font: { color: "#dbeafe", family: "Inter, Segoe UI, sans-serif" },
              margin: { l: 50, r: 50, t: 20, b: 50 },
              xaxis: {
                title: "X [m]",
                gridcolor: "#1e293b",
                zerolinecolor: "#334155",
                range: plotBounds?.x,
              },
              yaxis: {
                title: "Y [m]",
                gridcolor: "#1e293b",
                zerolinecolor: "#334155",
                scaleanchor: "x",
                scaleratio: 1,
                range: plotBounds?.y,
              },
              legend: {
                orientation: "h",
                x: 0,
                y: 1.12,
                bgcolor: "rgba(7,17,31,0.65)",
              },
            }}
            style={{ height: "560px", width: "100%" }}
            useResizeHandler
          />
        </section>

        <section className="metric-grid">
          {["RMSE Position", "Max Error", "Duration", "Path Length"].map((label) => (
            <article key={label} className="metric-card">
              <span>{label}</span>
              <strong>{metricValue(experiment, label)}</strong>
            </article>
          ))}
        </section>

        <section className="bottom-grid">
          {showErrorChart ? (
            <section className="panel compact-panel">
              <div className="panel-head">
                <div>
                  <p className="eyebrow">Temporal Error</p>
                  <h2>Error vs Time</h2>
                </div>
              </div>

              <Plot
                data={errorPlotData}
                layout={{
                  autosize: true,
                  paper_bgcolor: "rgba(0,0,0,0)",
                  plot_bgcolor: "#160b0a",
                  font: { color: "#fde68a", family: "Inter, Segoe UI, sans-serif" },
                  margin: { l: 50, r: 24, t: 16, b: 44 },
                  xaxis: {
                    title: "Time [s]",
                    gridcolor: "#4a1d1f",
                  },
                  yaxis: {
                    title: "Position Error [m]",
                    gridcolor: "#4a1d1f",
                  },
                  shapes: [
                    {
                      type: "line",
                      x0: selectedTime,
                      x1: selectedTime,
                      y0: 0,
                      y1: 1,
                      yref: "paper",
                      line: {
                        color: "#f59e0b",
                        width: 2,
                        dash: "dot",
                      },
                    },
                  ],
                }}
                style={{ height: "280px", width: "100%" }}
                useResizeHandler
              />
            </section>
          ) : null}

          <section className="panel compact-panel">
            <div className="panel-head">
              <div>
                <p className="eyebrow">State Snapshot</p>
                <h2>Current Samples</h2>
              </div>
            </div>

            <div className="snapshot-grid">
              <article className="snapshot-card">
                <span>Estimate XY</span>
                <strong>
                  {fmt(estimateSample?.x, 2)}, {fmt(estimateSample?.y, 2)}
                </strong>
                <small>yaw {fmt(estimateSample?.yaw_rad, 2, " rad")}</small>
              </article>
              <article className="snapshot-card">
                <span>Ground Truth XY</span>
                <strong>
                  {fmt(gtSample?.x, 2)}, {fmt(gtSample?.y, 2)}
                </strong>
                <small>time-sync view</small>
              </article>
              <article className="snapshot-card accent">
                <span>Instant Error</span>
                <strong>{fmt(estimateSample?.error_m, 3, " m")}</strong>
                <small>estimate vs GT</small>
              </article>
              <article className="snapshot-card">
                <span>Scene</span>
                <strong>
                  {experiment.scene?.map?.occupied_points?.length ?? 0} map pts / {currentSceneScan?.points.length ?? 0} lidar pts
                </strong>
                <small>current scan overlay</small>
              </article>
            </div>

            <div className="inventory-list">
              {experiment.trajectories.map((trajectory) => (
                <article key={trajectory.id} className="inventory-card">
                  <div className="inventory-title">
                    <span
                      className="swatch"
                      style={{ backgroundColor: layerColors[trajectory.id] ?? "#8b5cf6" }}
                    />
                    <strong>{trajectory.label}</strong>
                  </div>
                  <span>{trajectory.samples.length} samples</span>
                </article>
              ))}
            </div>
          </section>
        </section>
      </main>
    </div>
  );
}
