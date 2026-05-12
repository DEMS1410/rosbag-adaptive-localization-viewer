import Plot from "react-plotly.js";
import { useEffect, useMemo, useRef, useState } from "react";
import type { ChangeEvent, MouseEvent, ReactNode } from "react";

import { mockExperiment } from "./mockData";
import type { ExperimentData, TrajectorySeries } from "./types";

const settingsStorageKey = "adaptive-localization-viewer-settings";

const defaultLayerColors: Record<string, string> = {
  ground_truth: "#e2e8f0",
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
  if (typeof window === "undefined") return {};
  try {
    const raw = window.localStorage.getItem(settingsStorageKey);
    return raw ? (JSON.parse(raw) as ViewerSettings) : {};
  } catch {
    return {};
  }
}

function metricValue(experiment: ExperimentData, label: string): string {
  const m = experiment.metrics;
  const values: Record<string, string> = {
    "RMSE Position": m.rmse_position_m != null ? `${m.rmse_position_m.toFixed(3)} m` : "—",
    "Max Error": m.max_position_error_m != null ? `${m.max_position_error_m.toFixed(3)} m` : "—",
    Duration: m.duration_sec != null ? `${m.duration_sec.toFixed(1)} s` : "—",
    "Path Length": m.path_length_m != null ? `${m.path_length_m.toFixed(2)} m` : "—",
  };
  return values[label] ?? "—";
}

function metricColor(label: string, metrics: Record<string, number>): string {
  if (label === "RMSE Position") {
    const v = metrics.rmse_position_m;
    if (v == null) return "";
    if (v < 0.05) return "good";
    if (v < 0.15) return "warn";
    return "bad";
  }
  if (label === "Max Error") {
    const v = metrics.max_position_error_m;
    if (v == null) return "";
    if (v < 0.1) return "good";
    if (v < 0.3) return "warn";
    return "bad";
  }
  return "";
}

function currentSample(series: TrajectorySeries, timeSec: number) {
  if (series.samples.length === 0) return null;
  let nearest = series.samples[0];
  for (const s of series.samples) {
    if (Math.abs(s.t_sec - timeSec) < Math.abs(nearest.t_sec - timeSec)) nearest = s;
  }
  return nearest;
}

function currentScan(
  scans: { t_sec: number; points: [number, number][] }[] | undefined,
  timeSec: number,
) {
  if (!scans || scans.length === 0) return null;
  let nearest = scans[0];
  for (const s of scans) {
    if (Math.abs(s.t_sec - timeSec) < Math.abs(nearest.t_sec - timeSec)) nearest = s;
  }
  return nearest;
}

function samplesUntilTime(series: TrajectorySeries, timeSec: number) {
  const visible = series.samples.filter((s) => s.t_sec <= timeSec);
  return visible.length > 0 ? visible : series.samples.length > 0 ? [series.samples[0]] : [];
}

function samplesInWindow(series: TrajectorySeries, timeSec: number, windowSec: number) {
  const start = Math.max(0, timeSec - windowSec);
  return series.samples.filter((s) => s.t_sec >= start && s.t_sec <= timeSec);
}

function fmt(value: number | null | undefined, digits = 3, suffix = ""): string {
  if (value == null || Number.isNaN(value)) return "—";
  return `${value.toFixed(digits)}${suffix}`;
}

function ellipseTrace(x: number, y: number, covXx: number, covXy: number, covYy: number, sigma: number) {
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
  for (let i = 0; i <= 36; i++) {
    const t = (Math.PI * 2 * i) / 36;
    const ex = rx * Math.cos(t);
    const ey = ry * Math.sin(t);
    traceX.push(x + ex * Math.cos(angle) - ey * Math.sin(angle));
    traceY.push(y + ex * Math.sin(angle) + ey * Math.cos(angle));
  }
  return { x: traceX, y: traceY };
}

function wedgeTrace(x: number, y: number, yaw: number, yawVar: number, radius: number, sigma: number) {
  const sigmaYaw = sigma * Math.sqrt(Math.max(yawVar, 1e-9));
  const start = yaw - sigmaYaw;
  const end = yaw + sigmaYaw;
  const xs = [x];
  const ys = [y];
  for (let i = 0; i <= 20; i++) {
    const a = start + ((end - start) * i) / 20;
    xs.push(x + radius * Math.cos(a));
    ys.push(y + radius * Math.sin(a));
  }
  xs.push(x);
  ys.push(y);
  return { x: xs, y: ys };
}

// ── Small reusable components ───────────────────────────────────────────────
function Section({ title, children }: { title: string; children: ReactNode }) {
  return (
    <div className="settings-section">
      <h3 className="settings-section-title">{title}</h3>
      {children}
    </div>
  );
}

// ── App ─────────────────────────────────────────────────────────────────────
export default function App() {
  const persistedSettings = useMemo(loadSettings, []);
  const pageRef = useRef<HTMLDivElement | null>(null);
  const apiBaseUrl = "http://127.0.0.1:8765";

  // Core data
  const [experiment, setExperiment] = useState<ExperimentData>(mockExperiment);
  const [selectedTime, setSelectedTime] = useState(0);

  // Visibility
  const [visibleIds, setVisibleIds] = useState<string[]>(
    persistedSettings.visibleIds ?? mockExperiment.trajectories.map((t) => t.id),
  );

  // Playback
  const [isPlaying, setIsPlaying] = useState(false);
  const [playbackRate, setPlaybackRate] = useState(persistedSettings.playbackRate ?? 1);
  const [playbackMode, setPlaybackMode] = useState<"progressive" | "full">(
    persistedSettings.playbackMode ?? "progressive",
  );

  // App mode
  const [appMode, setAppMode] = useState<"analysis" | "simulation">(persistedSettings.appMode ?? "analysis");

  // Scene toggles
  const [showMap, setShowMap] = useState(persistedSettings.showMap ?? true);
  const [showLidar, setShowLidar] = useState(persistedSettings.showLidar ?? false);
  const [showGhostTrajectories, setShowGhostTrajectories] = useState(persistedSettings.showGhostTrajectories ?? false);
  const [showCurrentMarkers, setShowCurrentMarkers] = useState(persistedSettings.showCurrentMarkers ?? true);
  const [showErrorColoredEstimate, setShowErrorColoredEstimate] = useState(persistedSettings.showErrorColoredEstimate ?? true);
  const [showCovarianceEllipses, setShowCovarianceEllipses] = useState(persistedSettings.showCovarianceEllipses ?? false);
  const [showErrorChart, setShowErrorChart] = useState(persistedSettings.showErrorChart ?? true);
  const [showWedge, setShowWedge] = useState(persistedSettings.showWedge ?? true);
  const [showKeyboardHints, setShowKeyboardHints] = useState(persistedSettings.showKeyboardHints ?? true);

  // Covariance
  const [covarianceMode, setCovarianceMode] = useState<"current" | "trail">(persistedSettings.covarianceMode ?? "current");
  const [ellipseStride, setEllipseStride] = useState(persistedSettings.ellipseStride ?? 12);
  const [sigmaMultiplier, setSigmaMultiplier] = useState(persistedSettings.sigmaMultiplier ?? 3);
  const [tailWindowSec, setTailWindowSec] = useState(persistedSettings.tailWindowSec ?? 8);
  const [covarianceTrailCount, setCovarianceTrailCount] = useState(persistedSettings.covarianceTrailCount ?? 6);

  // Colors
  const [layerColors, setLayerColors] = useState({ ...defaultLayerColors, ...(persistedSettings.layerColors ?? {}) });

  // API / upload
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

  // UI state
  const [settingsOpen, setSettingsOpen] = useState(false);

  // ── Persist settings ────────────────────────────────────────────────────
  useEffect(() => {
    setVisibleIds((current) => {
      const avail = experiment.trajectories.map((t) => t.id);
      const keep = current.filter((id) => avail.includes(id));
      return keep.length > 0 ? keep : avail;
    });
  }, [experiment]);

  useEffect(() => {
    if (typeof window === "undefined") return;
    const s: ViewerSettings = {
      layerColors, visibleIds, appMode, playbackMode,
      showMap, showLidar, showGhostTrajectories, showCurrentMarkers,
      showErrorColoredEstimate, showCovarianceEllipses, showErrorChart,
      showWedge, showKeyboardHints, covarianceMode, ellipseStride,
      sigmaMultiplier, tailWindowSec, covarianceTrailCount, playbackRate,
    };
    window.localStorage.setItem(settingsStorageKey, JSON.stringify(s));
  }, [
    layerColors, visibleIds, appMode, playbackMode,
    showMap, showLidar, showGhostTrajectories, showCurrentMarkers,
    showErrorColoredEstimate, showCovarianceEllipses, showErrorChart,
    showWedge, showKeyboardHints, covarianceMode, ellipseStride,
    sigmaMultiplier, tailWindowSec, covarianceTrailCount, playbackRate,
  ]);

  // ── Load demo data ──────────────────────────────────────────────────────
  useEffect(() => {
    let cancelled = false;
    fetch("/demo-experiment.json")
      .then((r) => { if (!r.ok) throw new Error(); return r.json() as Promise<ExperimentData>; })
      .then((data) => { if (!cancelled) { setExperiment(data); setSelectedTime(0); } })
      .catch(() => {});
    return () => { cancelled = true; };
  }, []);

  // ── API health check ────────────────────────────────────────────────────
  useEffect(() => {
    let cancelled = false;
    fetch(`${apiBaseUrl}/health`)
      .then((r) => { if (!r.ok) throw new Error(); return r.json(); })
      .then(() => { if (!cancelled) setApiStatus("online"); })
      .catch(() => { if (!cancelled) setApiStatus("offline"); });
    return () => { cancelled = true; };
  }, [apiBaseUrl]);

  // ── Derived ─────────────────────────────────────────────────────────────
  const visibleSeries = useMemo(
    () => experiment.trajectories.filter((t) => visibleIds.includes(t.id)),
    [experiment, visibleIds],
  );

  const isProgressiveMode = playbackMode === "progressive";
  const isSimulationMode = appMode === "simulation";

  const maxTime = useMemo(
    () => Math.max(0, ...experiment.trajectories.map((t) =>
      t.samples.length > 0 ? t.samples[t.samples.length - 1].t_sec : 0
    )),
    [experiment],
  );

  const estimateSeries = useMemo(
    () => experiment.trajectories.find((t) => t.source_type === "estimate") ?? null,
    [experiment],
  );
  const gtSeries = useMemo(
    () => experiment.trajectories.find((t) => t.source_type === "ground_truth") ?? null,
    [experiment],
  );
  const estimateSample = estimateSeries ? currentSample(estimateSeries, selectedTime) : null;
  const gtSample = gtSeries ? currentSample(gtSeries, selectedTime) : null;
  const currentSceneScan = useMemo(() => currentScan(experiment.scene?.scans, selectedTime), [experiment, selectedTime]);
  const progressPct = maxTime > 0 ? (selectedTime / maxTime) * 100 : 0;

  // ── Playback ticker ─────────────────────────────────────────────────────
  useEffect(() => {
    if (!isPlaying) return;
    const timer = window.setInterval(() => {
      setSelectedTime((t) => {
        const next = t + 0.08 * playbackRate;
        if (next >= maxTime) { setIsPlaying(false); return maxTime; }
        return next;
      });
    }, 80);
    return () => window.clearInterval(timer);
  }, [isPlaying, playbackRate, maxTime]);

  // ── Keyboard ────────────────────────────────────────────────────────────
  useEffect(() => {
    if (!isSimulationMode) return;
    pageRef.current?.focus();
    function onKeyDown(e: KeyboardEvent) {
      if (e.target instanceof HTMLInputElement || e.target instanceof HTMLSelectElement) return;
      if (e.code === "Space") { e.preventDefault(); setIsPlaying((v) => !v); return; }
      if (e.key === "ArrowRight" || e.key.toLowerCase() === "d") { e.preventDefault(); setSelectedTime((v) => Math.min(maxTime, v + 0.5)); return; }
      if (e.key === "ArrowLeft" || e.key.toLowerCase() === "a") { e.preventDefault(); setSelectedTime((v) => Math.max(0, v - 0.5)); return; }
      if (e.key.toLowerCase() === "r") { setSelectedTime(0); setIsPlaying(false); return; }
      if (e.key.toLowerCase() === "m") { setShowMap((v) => !v); return; }
      if (e.key.toLowerCase() === "l") { setShowLidar((v) => !v); return; }
      if (e.key.toLowerCase() === "c") { setShowCovarianceEllipses((v) => !v); }
    }
    window.addEventListener("keydown", onKeyDown);
    return () => window.removeEventListener("keydown", onKeyDown);
  }, [isSimulationMode, maxTime]);

  // ── Scene traces ────────────────────────────────────────────────────────
  const sceneTraces = useMemo(() => {
    const traces: object[] = [];
    const map = experiment.scene?.map;
    if (showMap && map?.grid?.length) {
      traces.push({
        z: [...map.grid].reverse(), type: "heatmap" as const, name: "map",
        x0: map.origin[0] + map.resolution / 2, dx: map.resolution,
        y0: map.origin[1] + map.resolution / 2, dy: map.resolution,
        zmin: -1, zmax: 100,
        colorscale: [[0.0,"#f8fafc"],[0.009,"#f8fafc"],[0.01,"#2b0a4a"],[0.6,"#2b0a4a"],[0.61,"#fde047"],[1.0,"#fde047"]],
        showscale: false, opacity: 0.92,
        hovertemplate: "map x=%{x:.2f}<br>y=%{y:.2f}<br>occ=%{z}<extra></extra>",
      });
    } else if (showMap && map?.occupied_points?.length) {
      traces.push({
        x: map.occupied_points.map((p) => p[0]),
        y: map.occupied_points.map((p) => p[1]),
        type: "scattergl" as const, mode: "markers", name: "map",
        marker: { size: 7, color: layerColors.map, symbol: "square" },
        opacity: 0.9, hoverinfo: "skip" as const,
      });
    }
    if (showLidar && currentSceneScan?.points?.length) {
      traces.push({
        x: currentSceneScan.points.map((p) => p[0]),
        y: currentSceneScan.points.map((p) => p[1]),
        type: "scattergl" as const, mode: "markers", name: "lidar",
        marker: { size: isSimulationMode ? 5 : 4, color: layerColors.lidar },
      });
    }
    return traces;
  }, [experiment, showMap, showLidar, currentSceneScan, isSimulationMode, layerColors]);

  // ── Plot bounds ─────────────────────────────────────────────────────────
  const plotBounds = useMemo(() => {
    const xs: number[] = [];
    const ys: number[] = [];
    for (const t of visibleSeries) {
      for (const s of t.samples) { xs.push(s.x); ys.push(s.y); }
    }
    if (showMap) {
      for (const p of experiment.scene?.map?.occupied_points ?? []) { xs.push(p[0]); ys.push(p[1]); }
    }
    if (showLidar) {
      for (const p of currentSceneScan?.points ?? []) { xs.push(p[0]); ys.push(p[1]); }
    }
    if (xs.length === 0 || ys.length === 0) return null;
    let xmin = Math.min(...xs); let xmax = Math.max(...xs);
    let ymin = Math.min(...ys); let ymax = Math.max(...ys);
    const pad = Math.max(xmax - xmin, ymax - ymin, 1) * 0.12;
    return { x: [xmin - pad, xmax + pad] as [number, number], y: [ymin - pad, ymax + pad] as [number, number] };
  }, [visibleSeries, experiment, showMap, showLidar, currentSceneScan]);

  // ── Trajectory traces ───────────────────────────────────────────────────
  const plotData = visibleSeries.flatMap((trajectory) => {
    const color = layerColors[trajectory.id] ?? "#8b5cf6";
    const focus = currentSample(trajectory, selectedTime);
    const playedSamples = samplesUntilTime(trajectory, selectedTime);
    const visibleSamples = isSimulationMode
      ? samplesInWindow(trajectory, selectedTime, tailWindowSec)
      : isProgressiveMode ? playedSamples : trajectory.samples;
    const traces: object[] = [];

    if (!isSimulationMode && isProgressiveMode && showGhostTrajectories) {
      traces.push({
        x: trajectory.samples.map((s) => s.x), y: trajectory.samples.map((s) => s.y),
        type: "scattergl" as const, mode: "lines", name: `${trajectory.label} full`,
        line: { color: trajectory.source_type === "ground_truth" ? "rgba(148,163,184,0.25)" : "rgba(71,85,105,0.35)", width: trajectory.source_type === "ground_truth" ? 3 : 2, dash: "dot" },
        hoverinfo: "skip" as const, showlegend: false,
      });
    }

    if (showErrorColoredEstimate && trajectory.source_type === "estimate" && trajectory.samples.some((s) => s.error_m != null)) {
      traces.push({
        x: visibleSamples.map((s) => s.x), y: visibleSamples.map((s) => s.y),
        type: "scattergl" as const, mode: "markers", name: `${trajectory.label} error`,
        marker: {
          size: isSimulationMode ? 9 : 8,
          color: visibleSamples.map((s) => s.error_m ?? 0),
          colorscale: [[0,"#10b981"],[0.5,"#f59e0b"],[1,"#ef4444"]],
          cmin: 0, cmax: Math.max(...trajectory.samples.map((s) => s.error_m ?? 0), 0.1),
          colorbar: { title: "Err [m]", thickness: 12, bgcolor: "rgba(10,14,26,0.2)", outlinecolor: "#334155", tickcolor: "#94a3b8" },
        },
        hovertemplate: "<b>%{text}</b><br>x=%{x:.3f} m<br>y=%{y:.3f} m<br>err=%{marker.color:.3f} m<extra></extra>",
        text: visibleSamples.map(() => trajectory.label),
        showlegend: false,
      });
    }

    traces.push({
      x: visibleSamples.map((s) => s.x), y: visibleSamples.map((s) => s.y),
      type: "scattergl" as const, mode: "lines", name: trajectory.label,
      line: { color, width: trajectory.source_type === "ground_truth" ? (isSimulationMode ? 6 : 5) : isSimulationMode ? 4 : 3, dash: trajectory.source_type === "comparison" ? "dot" : "solid" },
    });

    if (showCurrentMarkers && focus) {
      traces.push({
        x: [focus.x], y: [focus.y], type: "scatter" as const, mode: "markers",
        name: `${trajectory.label} @ t`,
        marker: { color, size: isSimulationMode ? 15 : 12, line: { color: "#ffffff", width: 2 } },
        showlegend: false,
      });
    }
    return traces;
  });

  // ── Covariance traces ───────────────────────────────────────────────────
  const covarianceTraces = useMemo(() => {
    if (!estimateSeries || !showCovarianceEllipses) return [];
    const played = isSimulationMode
      ? samplesInWindow(estimateSeries, selectedTime, tailWindowSec)
      : isProgressiveMode ? samplesUntilTime(estimateSeries, selectedTime) : estimateSeries.samples;
    const traces: object[] = [];
    const sourceSamples = covarianceMode === "current" && played.length > 0
      ? [played[played.length - 1]]
      : played.filter((_, i) => i % ellipseStride === 0).slice(-covarianceTrailCount);

    for (let i = 0; i < sourceSamples.length; i++) {
      const s = sourceSamples[i];
      if (s.cov_xx == null || s.cov_xy == null || s.cov_yy == null || !Number.isFinite(s.cov_xx) || !Number.isFinite(s.cov_xy) || !Number.isFinite(s.cov_yy)) continue;
      const ellipse = ellipseTrace(s.x, s.y, s.cov_xx, s.cov_xy, s.cov_yy, sigmaMultiplier);
      traces.push({
        x: ellipse.x, y: ellipse.y, type: "scattergl" as const, mode: "lines", name: "xy Cov",
        line: { color: covarianceMode === "current" ? layerColors.covariance : `${layerColors.covariance}88`, width: covarianceMode === "current" ? 2.5 : 1.5 },
        fill: "toself" as const,
        fillcolor: covarianceMode === "current" ? `${layerColors.covariance}22` : `${layerColors.covariance}12`,
        showlegend: i === 0, hoverinfo: "skip" as const,
      });
    }
    const focus = currentSample(estimateSeries, selectedTime);
    if (showWedge && focus?.yaw_rad != null && focus?.yaw_var != null) {
      const wedge = wedgeTrace(focus.x, focus.y, focus.yaw_rad, focus.yaw_var, 0.45, sigmaMultiplier);
      traces.push({
        x: wedge.x, y: wedge.y, type: "scattergl" as const, mode: "lines", name: "yaw ±σ",
        line: { color: layerColors.yaw, width: 1.5 },
        fill: "toself" as const, fillcolor: `${layerColors.yaw}20`,
      });
    }
    return traces;
  }, [estimateSeries, selectedTime, ellipseStride, sigmaMultiplier, showWedge, showCovarianceEllipses, isProgressiveMode, isSimulationMode, covarianceMode, covarianceTrailCount, tailWindowSec, layerColors]);

  // ── Error plot ──────────────────────────────────────────────────────────
  const errorSeries = useMemo(
    () => experiment.trajectories.find((t) => t.source_type === "estimate" && t.samples.some((s) => s.error_m != null)),
    [experiment],
  );
  const errorPlotData = errorSeries == null ? [] : [{
    x: errorSeries.samples.map((s) => s.t_sec),
    y: errorSeries.samples.map((s) => s.error_m ?? null),
    type: "scatter" as const, mode: "lines", name: "Position Error",
    line: { color: "#ef4444", width: 3 },
    fill: "tozeroy" as const, fillcolor: "rgba(239,68,68,0.14)",
  }];

  // ── Event handlers ──────────────────────────────────────────────────────
  async function onFileSelected(event: ChangeEvent<HTMLInputElement>) {
    const file = event.target.files?.[0];
    if (!file) return;
    const parsed = JSON.parse(await file.text()) as ExperimentData;
    setExperiment(parsed);
    setSelectedTime(0);
  }

  async function onBuildFromUploads() {
    if (!bagUploadFile) { setUploadError("Select a ROS 2 .db3 bag first."); return; }
    setIsBuildingUpload(true);
    setUploadError(null);
    const formData = new FormData();
    formData.append("bag_file", bagUploadFile);
    formData.append("experiment_name", uploadExperimentName);
    formData.append("primary_topic", uploadPrimaryTopic);
    formData.append("comparison_topics", uploadComparisonTopics);
    formData.append("rigid_body_name", uploadRigidBody);
    formData.append("include_optitrack", includeOptitrackUpload ? "true" : "false");
    if (includeOptitrackUpload && optitrackUploadFile) formData.append("optitrack_file", optitrackUploadFile);
    try {
      const res = await fetch(`${apiBaseUrl}/build-experiment`, { method: "POST", body: formData });
      const payload = (await res.json()) as ExperimentData & { error?: string };
      if (!res.ok || payload.error) throw new Error(payload.error ?? `HTTP ${res.status}`);
      setExperiment(payload);
      setSelectedTime(0);
      setApiStatus("online");
    } catch (err) {
      setApiStatus("offline");
      setUploadError(err instanceof Error ? err.message : "Upload failed");
    } finally {
      setIsBuildingUpload(false);
    }
  }

  function onShellClick(event: MouseEvent<HTMLDivElement>) {
    if (isSimulationMode && event.target === event.currentTarget) pageRef.current?.focus();
  }

  // ── Render ──────────────────────────────────────────────────────────────
  return (
    <div ref={pageRef} className="page-shell" tabIndex={0} onClick={onShellClick}>

      {/* ══ NAVBAR ══════════════════════════════════════════════════════════ */}
      <nav className="page-nav">
        <div className="nav-brand">
          <span className="nav-lab">DGIST RT604</span>
          <span className="nav-sep">·</span>
          <span className="nav-title">Adaptive Localization</span>
        </div>
        <div className="nav-right">
          <span className={`api-badge ${apiStatus === "online" ? "online" : apiStatus === "checking" ? "checking" : "offline"}`}>
            <span className="api-dot" />
            {apiStatus === "online" ? "API Online" : apiStatus === "checking" ? "Checking…" : "API Offline"}
          </span>
          <span className="nav-exp-name">{experiment.metadata.experiment_name}</span>
        </div>
      </nav>

      {/* ══ HERO ════════════════════════════════════════════════════════════ */}
      <header className="hero-section">
        <p className="hero-eyebrow">SE(2) Trajectory Analysis Tool</p>
        <h1 className="hero-title">Adaptive Localization Viewer</h1>
        <p className="hero-desc">
          Synchronized visualization of ROS 2 odometry, OptiTrack ground truth, position error,
          and covariance uncertainty for adaptive localization experiments.
        </p>
        <div className="hero-tags">
          <span className="hero-tag">Offline</span>
          <span className="hero-tag">ROS 2 Bag</span>
          <span className="hero-tag">OptiTrack GT</span>
          <span className="hero-tag">SE(2) Geometry</span>
          <span className="hero-tag">{experiment.trajectories.length} tracks</span>
          <span className="hero-tag">{fmt(maxTime, 1, " s")} horizon</span>
        </div>
      </header>

      {/* ══ METRICS ═════════════════════════════════════════════════════════ */}
      <section className="metric-grid">
        {["RMSE Position", "Max Error", "Duration", "Path Length"].map((label) => {
          const cls = metricColor(label, experiment.metrics);
          return (
            <article key={label} className={`metric-card${cls ? ` ${cls}` : ""}`}>
              <span className="metric-label">{label}</span>
              <strong className="metric-value">{metricValue(experiment, label)}</strong>
            </article>
          );
        })}
      </section>

      {/* ══ LAYER TOOLBAR ═══════════════════════════════════════════════════ */}
      <div className="layer-toolbar">
        <div className="layer-pill-row">
          {experiment.trajectories.map((trajectory) => {
            const active = visibleIds.includes(trajectory.id);
            const color = layerColors[trajectory.id] ?? "#8b5cf6";
            return (
              <button
                key={trajectory.id}
                className={`layer-pill${active ? " active" : ""}`}
                style={{ "--pill-color": color } as React.CSSProperties}
                type="button"
                onClick={() =>
                  setVisibleIds((cur) =>
                    active ? cur.filter((id) => id !== trajectory.id) : [...cur, trajectory.id]
                  )
                }
              >
                <span className="pill-dot" style={{ backgroundColor: active ? color : "transparent", borderColor: color }} />
                {trajectory.label}
              </button>
            );
          })}
        </div>
        <div className="quick-toggles">
          <button className={`toggle-chip${showMap ? " active" : ""}`} type="button" onClick={() => setShowMap((v) => !v)}>Map</button>
          <button className={`toggle-chip${showLidar ? " active" : ""}`} type="button" onClick={() => setShowLidar((v) => !v)}>Lidar</button>
          <button className={`toggle-chip${showCovarianceEllipses ? " active" : ""}`} type="button" onClick={() => setShowCovarianceEllipses((v) => !v)}>Covariance</button>
          <button className={`settings-trigger-btn${settingsOpen ? " active" : ""}`} type="button" onClick={() => setSettingsOpen((v) => !v)}>
            ⚙ Settings {settingsOpen ? "▲" : "▼"}
          </button>
        </div>
      </div>

      {/* ══ MAIN PLOT ═══════════════════════════════════════════════════════ */}
      <section className="panel chart-panel">
        <div className="chart-header">
          <div>
            <p className="eyebrow">Spatial View</p>
            <h2>Trajectory Overlay</h2>
          </div>
          <div className="chart-header-right">
            <span className="time-readout">{selectedTime.toFixed(2)} s</span>
            {isSimulationMode && showKeyboardHints && (
              <span className="keyboard-hint">Space · A/D · R · M · L · C</span>
            )}
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
            xaxis: { title: "X [m]", gridcolor: "#1e293b", zerolinecolor: "#334155", range: plotBounds?.x },
            yaxis: { title: "Y [m]", gridcolor: "#1e293b", zerolinecolor: "#334155", scaleanchor: "x", scaleratio: 1, range: plotBounds?.y },
            legend: { orientation: "h", x: 0, y: 1.12, bgcolor: "rgba(7,17,31,0.65)" },
          }}
          style={{ height: "580px", width: "100%" }}
          useResizeHandler
        />
      </section>

      {/* ══ TRANSPORT BAR ═══════════════════════════════════════════════════ */}
      <section className="transport-bar">
        {/* Left: Mode */}
        <div className="transport-left">
          <span className="rack-label">Mode</span>
          <div className="transport-mode-row">
            <button className={`mode-btn${appMode === "analysis" ? " active" : ""}`} type="button" onClick={() => setAppMode("analysis")}>Analysis</button>
            <button className={`mode-btn${appMode === "simulation" ? " active" : ""}`} type="button" onClick={() => setAppMode("simulation")}>Simulation</button>
          </div>
        </div>

        {/* Center: Controls + scrubber */}
        <div className="transport-center">
          <div className="transport-controls">
            <button className="transport-btn" type="button" title="Reset" onClick={() => { setSelectedTime(0); setIsPlaying(false); }}>⏮</button>
            <button className={`transport-btn play-btn${isPlaying ? " playing" : ""}`} type="button" title={isPlaying ? "Pause" : "Play"} onClick={() => setIsPlaying((v) => !v)}>
              {isPlaying ? "⏸" : "▶"}
            </button>
            <button className="transport-btn" type="button" title="+1 second" onClick={() => setSelectedTime((v) => Math.min(maxTime, v + 1))}>⏭</button>
          </div>
          <div className="transport-scrubber-row">
            <span className="transport-time">{selectedTime.toFixed(1)} s</span>
            <input className="scrubber" max={maxTime} min={0} step={0.1} type="range" value={selectedTime} onChange={(e) => setSelectedTime(Number(e.target.value))} />
            <span className="transport-time transport-time-end">{maxTime.toFixed(1)} s</span>
          </div>
          <div className="progress-rail"><div className="progress-fill" style={{ width: `${progressPct}%` }} /></div>
        </div>

        {/* Right: Rate + Playback mode */}
        <div className="transport-right">
          <div className="transport-rate-row">
            <span className="rack-label">Rate</span>
            <select className="rate-select" value={playbackRate} onChange={(e) => setPlaybackRate(Number(e.target.value))}>
              <option value={0.5}>0.5×</option>
              <option value={1}>1×</option>
              <option value={2}>2×</option>
              <option value={4}>4×</option>
            </select>
          </div>
          <div className="transport-mode-row">
            <button className={`mode-btn${playbackMode === "progressive" ? " active" : ""}`} type="button" onClick={() => setPlaybackMode("progressive")}>Progressive</button>
            <button className={`mode-btn${playbackMode === "full" ? " active" : ""}`} type="button" onClick={() => setPlaybackMode("full")}>Full View</button>
          </div>
        </div>
      </section>

      {/* ══ BOTTOM GRID ═════════════════════════════════════════════════════ */}
      <section className="bottom-grid">
        {/* Error chart */}
        {showErrorChart ? (
          <section className="panel">
            <p className="eyebrow">Temporal Error</p>
            <h2>Error vs Time</h2>
            <Plot
              data={errorPlotData}
              layout={{
                autosize: true,
                paper_bgcolor: "rgba(0,0,0,0)",
                plot_bgcolor: "#160b0a",
                font: { color: "#fde68a", family: "Inter, Segoe UI, sans-serif" },
                margin: { l: 50, r: 24, t: 16, b: 44 },
                xaxis: { title: "Time [s]", gridcolor: "#4a1d1f" },
                yaxis: { title: "Position Error [m]", gridcolor: "#4a1d1f" },
                shapes: [{ type: "line", x0: selectedTime, x1: selectedTime, y0: 0, y1: 1, yref: "paper", line: { color: "#f59e0b", width: 2, dash: "dot" } }],
              }}
              style={{ height: "260px", width: "100%" }}
              useResizeHandler
            />
          </section>
        ) : null}

        {/* State snapshot */}
        <section className="panel snapshot-panel">
          <p className="eyebrow">State Snapshot</p>
          <h2>Current Samples</h2>
          <div className="snapshot-grid">
            <article className="snapshot-card">
              <span>Estimate XY</span>
              <strong>{fmt(estimateSample?.x, 2)}, {fmt(estimateSample?.y, 2)}</strong>
              <small>yaw {fmt(estimateSample?.yaw_rad, 2, " rad")}</small>
            </article>
            <article className="snapshot-card">
              <span>Ground Truth XY</span>
              <strong>{fmt(gtSample?.x, 2)}, {fmt(gtSample?.y, 2)}</strong>
              <small>time-sync view</small>
            </article>
            <article className="snapshot-card accent">
              <span>Instant Error</span>
              <strong>{fmt(estimateSample?.error_m, 3, " m")}</strong>
              <small>estimate vs GT</small>
            </article>
            <article className="snapshot-card">
              <span>Scene</span>
              <strong>{experiment.scene?.map?.occupied_points?.length ?? 0} map · {currentSceneScan?.points.length ?? 0} lidar</strong>
              <small>current overlay</small>
            </article>
          </div>
          <div className="inventory-list">
            {experiment.trajectories.map((t) => (
              <article key={t.id} className="inventory-card">
                <div className="inventory-title">
                  <span className="swatch" style={{ backgroundColor: layerColors[t.id] ?? "#8b5cf6" }} />
                  <strong>{t.label}</strong>
                </div>
                <span>{t.samples.length} samples</span>
              </article>
            ))}
          </div>
        </section>
      </section>

      {/* ══ SETTINGS DRAWER ═════════════════════════════════════════════════ */}
      <div className={`settings-drawer${settingsOpen ? " open" : ""}`}>
        <div className="settings-grid">

          {/* ── Load / Build ── */}
          <Section title="Load Experiment">
            <label className="upload-box">
              <span>Load experiment JSON</span>
              <input accept=".json,application/json" type="file" onChange={onFileSelected} />
            </label>
            <div className="upload-divider">or build from source files</div>
            <div className="upload-form">
              <label className="field-block">
                <span>Experiment name</span>
                <input className="text-input" type="text" value={uploadExperimentName} onChange={(e) => setUploadExperimentName(e.target.value)} />
              </label>
              <label className="field-block">
                <span>ROS 2 bag (.db3)</span>
                <input accept=".db3" type="file" onChange={(e) => setBagUploadFile(e.target.files?.[0] ?? null)} />
              </label>
              <label className="field-block">
                <span>Primary topic</span>
                <input className="text-input" type="text" value={uploadPrimaryTopic} onChange={(e) => setUploadPrimaryTopic(e.target.value)} />
              </label>
              <label className="field-block">
                <span>Comparison topics</span>
                <input className="text-input" type="text" value={uploadComparisonTopics} onChange={(e) => setUploadComparisonTopics(e.target.value)} />
              </label>
              <label className="check-row compact-check">
                <input checked={includeOptitrackUpload} type="checkbox" onChange={(e) => setIncludeOptitrackUpload(e.target.checked)} />
                <span>Include OptiTrack CSV</span>
              </label>
              {includeOptitrackUpload && (
                <>
                  <label className="field-block">
                    <span>OptiTrack CSV</span>
                    <input accept=".csv,text/csv" type="file" onChange={(e) => setOptitrackUploadFile(e.target.files?.[0] ?? null)} />
                  </label>
                  <label className="field-block">
                    <span>Rigid body</span>
                    <input className="text-input" type="text" value={uploadRigidBody} onChange={(e) => setUploadRigidBody(e.target.value)} />
                  </label>
                </>
              )}
              <button className="control-btn primary build-upload-btn" type="button" disabled={isBuildingUpload || apiStatus === "checking"} onClick={onBuildFromUploads}>
                {isBuildingUpload ? "Building…" : "Build from bag + CSV"}
              </button>
              {uploadError && <p className="upload-error">{uploadError}</p>}
              <p className="panel-help">Run <code>python -m rosbag_adaptive_localization_viewer.cli serve</code> to start the API.</p>
            </div>
          </Section>

          {/* ── Colors ── */}
          <Section title="Layer Colors">
            <div className="preset-row">
              <button className="preset-btn" type="button" onClick={() => setLayerColors(colorPresets.dgist)}>DGIST</button>
              <button className="preset-btn" type="button" onClick={() => setLayerColors(colorPresets.contrast)}>High Contrast</button>
              <button className="preset-btn" type="button" onClick={() => setLayerColors(colorPresets.soft)}>Soft</button>
            </div>
            <div className="color-grid">
              {[
                { id: "odom", label: "/odom" }, { id: "odom_raw", label: "/odom_raw" },
                { id: "odom_raw_adapted", label: "/odom_raw_adapted" }, { id: "map_base", label: "map → base" },
                { id: "ground_truth", label: "ROBOT / GT" }, { id: "map", label: "Map" },
                { id: "lidar", label: "Lidar" }, { id: "covariance", label: "XY covariance" },
                { id: "yaw", label: "Yaw wedge" },
              ].map((item) => (
                <label key={item.id} className="color-row">
                  <span>{item.label}</span>
                  <input aria-label={`Color for ${item.label}`} type="color" value={layerColors[item.id] ?? "#ffffff"}
                    onChange={(e) => setLayerColors((c) => ({ ...c, [item.id]: e.target.value }))} />
                </label>
              ))}
            </div>
            <button className="control-btn reset-colors-btn" type="button" onClick={() => setLayerColors(defaultLayerColors)}>Reset colors</button>
          </Section>

          {/* ── Mode + Scene + Covariance ── */}
          <div style={{ display: "flex", flexDirection: "column", gap: "20px" }}>
            <Section title="Simulation Mode">
              <div className="mode-row" style={{ marginTop: 0 }}>
                <button className={`mode-btn${appMode === "analysis" ? " active" : ""}`} type="button" onClick={() => setAppMode("analysis")}>Analysis</button>
                <button className={`mode-btn${appMode === "simulation" ? " active" : ""}`} type="button" onClick={() => setAppMode("simulation")}>Simulation</button>
              </div>
              {isSimulationMode && (
                <div className="slider-block">
                  <div className="slider-label-row"><span>Tail window</span><strong>{tailWindowSec.toFixed(0)} s</strong></div>
                  <input className="scrubber" min={2} max={30} step={1} type="range" value={tailWindowSec} onChange={(e) => setTailWindowSec(Number(e.target.value))} />
                </div>
              )}
              <label className="check-row" style={{ marginTop: 10 }}>
                <input checked={showGhostTrajectories} type="checkbox" onChange={(e) => setShowGhostTrajectories(e.target.checked)} />
                <span>Ghost trajectories</span>
              </label>
              <label className="check-row">
                <input checked={showCurrentMarkers} type="checkbox" onChange={(e) => setShowCurrentMarkers(e.target.checked)} />
                <span>Current position markers</span>
              </label>
              <label className="check-row">
                <input checked={showErrorColoredEstimate} type="checkbox" onChange={(e) => setShowErrorColoredEstimate(e.target.checked)} />
                <span>Error-colored estimate</span>
              </label>
              <label className="check-row">
                <input checked={showErrorChart} type="checkbox" onChange={(e) => setShowErrorChart(e.target.checked)} />
                <span>Show error chart</span>
              </label>
              <label className="check-row">
                <input checked={showKeyboardHints} type="checkbox" onChange={(e) => setShowKeyboardHints(e.target.checked)} />
                <span>Show keyboard hints</span>
              </label>
            </Section>

            <Section title="Covariance">
              <div className="meta-row compact">
                <span>Mode</span>
                <select className="rate-select" value={covarianceMode} onChange={(e) => setCovarianceMode(e.target.value as "current" | "trail")}>
                  <option value="current">Current only</option>
                  <option value="trail">Recent trail</option>
                </select>
              </div>
              <div className="slider-block">
                <div className="slider-label-row"><span>Draw every K steps</span><strong>{ellipseStride}</strong></div>
                <input className="scrubber" min={4} max={40} step={1} type="range" value={ellipseStride} onChange={(e) => setEllipseStride(Number(e.target.value))} />
              </div>
              {covarianceMode === "trail" && (
                <div className="slider-block">
                  <div className="slider-label-row"><span>Trail count</span><strong>{covarianceTrailCount}</strong></div>
                  <input className="scrubber" min={1} max={20} step={1} type="range" value={covarianceTrailCount} onChange={(e) => setCovarianceTrailCount(Number(e.target.value))} />
                </div>
              )}
              <div className="slider-block">
                <div className="slider-label-row"><span>Sigma multiplier</span><strong>{sigmaMultiplier.toFixed(1)} σ</strong></div>
                <input className="scrubber" min={1} max={4} step={0.5} type="range" value={sigmaMultiplier} onChange={(e) => setSigmaMultiplier(Number(e.target.value))} />
              </div>
              <label className="check-row">
                <input checked={showWedge} type="checkbox" onChange={(e) => setShowWedge(e.target.checked)} />
                <span>Show yaw wedge</span>
              </label>
            </Section>
          </div>

        </div>
      </div>

    </div>
  );
}
