import Plot from "react-plotly.js";
import { useEffect, useMemo, useState } from "react";

import { mockExperiment } from "./mockData";
import type { ExperimentData, TrajectorySeries } from "./types";

const seriesColors: Record<string, string> = {
  ground_truth: "#0f172a",
  odom: "#f97316",
  odom_raw: "#38bdf8",
  odom_raw_adapted: "#10b981",
};

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

export default function App() {
  const [experiment, setExperiment] = useState<ExperimentData>(mockExperiment);
  const [selectedTime, setSelectedTime] = useState(9.4);
  const [visibleIds, setVisibleIds] = useState<string[]>(
    mockExperiment.trajectories.map((trajectory) => trajectory.id),
  );

  useEffect(() => {
    setVisibleIds(experiment.trajectories.map((trajectory) => trajectory.id));
  }, [experiment]);

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
        // Keep the mock dataset when the demo JSON is not available.
      });

    return () => {
      cancelled = true;
    };
  }, []);

  const visibleSeries = useMemo(
    () => experiment.trajectories.filter((trajectory) => visibleIds.includes(trajectory.id)),
    [experiment, visibleIds],
  );

  const plotData = visibleSeries.flatMap((trajectory) => {
    const color = seriesColors[trajectory.id] ?? "#8b5cf6";
    const focus = currentSample(trajectory, selectedTime);
    const traces = [
      {
        x: trajectory.samples.map((sample) => sample.x),
        y: trajectory.samples.map((sample) => sample.y),
        type: "scatter" as const,
        mode: "lines",
        name: trajectory.label,
        line: {
          color,
          width: trajectory.source_type === "ground_truth" ? 5 : 3,
          dash: trajectory.source_type === "comparison" ? "dot" : "solid",
        },
      },
    ];
    if (focus) {
      traces.push({
        x: [focus.x],
        y: [focus.y],
        type: "scatter" as const,
        mode: "markers",
        name: `${trajectory.label} @ t`,
        marker: {
          color,
          size: 12,
          line: { color: "#ffffff", width: 2 },
        },
        showlegend: false,
      });
    }
    return traces;
  });

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

  async function onFileSelected(event: React.ChangeEvent<HTMLInputElement>) {
    const file = event.target.files?.[0];
    if (!file) {
      return;
    }
    const text = await file.text();
    const parsed = JSON.parse(text) as ExperimentData;
    setExperiment(parsed);
    setSelectedTime(0);
  }

  return (
    <div className="page-shell">
      <aside className="sidebar">
        <div className="panel hero-panel">
          <p className="eyebrow">Adaptive Localization</p>
          <h1>Trajectory Viewer</h1>
          <p className="lede">
            Offline comparison of ROS 2 odometry against OptiTrack ground truth with a cleaner,
            more report-ready visual language.
          </p>
        </div>

        <div className="panel">
          <h2>Experiment</h2>
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
          <label className="upload-box">
            <span>Load experiment JSON</span>
            <input accept=".json,application/json" type="file" onChange={onFileSelected} />
          </label>
        </div>

        <div className="panel">
          <h2>Layers</h2>
          <div className="toggle-list">
            {experiment.trajectories.map((trajectory) => {
              const checked = visibleIds.includes(trajectory.id);
              const color = seriesColors[trajectory.id] ?? "#8b5cf6";
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
                </label>
              );
            })}
          </div>
        </div>

        <div className="panel">
          <h2>Timeline</h2>
          <input
            className="scrubber"
            max={maxTime}
            min={0}
            step={0.1}
            type="range"
            value={selectedTime}
            onChange={(event) => setSelectedTime(Number(event.target.value))}
          />
          <div className="time-readout">{selectedTime.toFixed(1)} s</div>
        </div>
      </aside>

      <main className="main-stage">
        <section className="metric-grid">
          {["RMSE Position", "Max Error", "Duration", "Path Length"].map((label) => (
            <article key={label} className="metric-card">
              <span>{label}</span>
              <strong>{metricValue(experiment, label)}</strong>
            </article>
          ))}
        </section>

        <section className="panel chart-panel">
          <div className="panel-head">
            <div>
              <p className="eyebrow">Main Plot</p>
              <h2>Trajectory Overlay</h2>
            </div>
            <div className="legend-note">GT vs estimate vs adapted variants</div>
          </div>

          <Plot
            data={plotData}
            layout={{
              autosize: true,
              paper_bgcolor: "rgba(0,0,0,0)",
              plot_bgcolor: "#f8fafc",
              font: { color: "#0f172a", family: "Inter, Segoe UI, sans-serif" },
              margin: { l: 50, r: 30, t: 20, b: 50 },
              xaxis: {
                title: "X [m]",
                gridcolor: "#dbe4f0",
                zerolinecolor: "#cbd5e1",
              },
              yaxis: {
                title: "Y [m]",
                gridcolor: "#dbe4f0",
                zerolinecolor: "#cbd5e1",
                scaleanchor: "x",
                scaleratio: 1,
              },
              legend: {
                orientation: "h",
                x: 0,
                y: 1.12,
              },
            }}
            style={{ height: "560px", width: "100%" }}
            useResizeHandler
          />
        </section>

        <section className="bottom-grid">
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
                plot_bgcolor: "#fff7ed",
                font: { color: "#0f172a", family: "Inter, Segoe UI, sans-serif" },
                margin: { l: 50, r: 24, t: 16, b: 44 },
                xaxis: {
                  title: "Time [s]",
                  gridcolor: "#fed7aa",
                },
                yaxis: {
                  title: "Position Error [m]",
                  gridcolor: "#fed7aa",
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
                      color: "#9a3412",
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

          <section className="panel compact-panel">
            <div className="panel-head">
              <div>
                <p className="eyebrow">Run Summary</p>
                <h2>Trajectory Inventory</h2>
              </div>
            </div>

            <div className="inventory-list">
              {experiment.trajectories.map((trajectory) => (
                <article key={trajectory.id} className="inventory-card">
                  <div className="inventory-title">
                    <span
                      className="swatch"
                      style={{ backgroundColor: seriesColors[trajectory.id] ?? "#8b5cf6" }}
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
