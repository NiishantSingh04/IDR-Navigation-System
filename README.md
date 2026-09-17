# IDR Navigation System — Intelligent Dead Reckoning Prototype

A web-based prototype demonstrating **AI-ML enhanced Dead Reckoning** and **GNSS+INS Sensor Fusion** for seamless vehicle navigation during GNSS outages. Built for **SIH 2026 — Problem Statement 26168** (ISRO, Department of Space).

---

## Problem Statement

Vehicles relying on smartphone-based navigation lose positioning accuracy when GNSS signals are blocked — inside tunnels, underpasses, dense urban canyons, and forested highways. This prototype addresses the challenge by implementing an **Intelligent Dead Reckoning (IDR) system** that maintains lane-level accuracy using only smartphone IMU sensors (accelerometer, gyroscope) during GNSS blackouts, and seamlessly fuses back when GNSS returns.

## Solution Approach

The core algorithm is based on the **Rao-Blackwellized Particle Filter (RBPF)** framework from:

> Mikov, A. et al., "Map-Aided Dead Reckoning for Land Vehicle Navigation Using Low-Cost Inertial Sensors," *28th Saint Petersburg International Conference on Integrated Navigation Systems (ICINS)*.

### Key Techniques Implemented

| Technique | Description |
|---|---|
| **Strapdown INS Mechanization** | Integrates accelerometer and gyroscope to propagate position, velocity, and attitude |
| **Error-State Extended Kalman Filter** | 8-state EKF estimating position errors, velocity errors, heading error, sensor biases |
| **Rao-Blackwellized Particle Filter** | Each particle maintains its own EKF instance (Kalman Filter Bank). Position errors are represented by particles; all other states by parametric distributions |
| **Roadmap Corrections** | Particles weighted by distance to nearest road segment: `w(d) = 1/log(max(d_min, d))` |
| **Non-Holonomic Constraints (NHC)** | Enforces zero lateral and vertical vehicle velocity |
| **Seamless GNSS Deficit Handler** | Instant transition between GNSS-aided and dead reckoning modes |

## Dataset

The prototype uses the **IO-VNBD** (Inertial Odometry Vehicle Navigation Benchmark Dataset):

- **Source**: [IO-VNBD GitHub Repository](https://github.com/onyekpeu/IO-VNBD)
- **Subset used**: M (Driver B) — synchronized vehicle and smartphone data from Coventry, UK
- **Data points**: 5,299 (subsampled from 105,975 at 1:20 ratio for browser performance)
- **Sensors**: GPS ground truth, wheel speed, yaw rate, accelerometer XYZ, gyroscope, magnetometer, smartphone GPS

## Tech Stack

| Layer | Technology |
|---|---|
| Build Tool | Vite |
| Language | Vanilla JavaScript (ES Modules) |
| Map Rendering | Leaflet.js + CartoDB Dark Tiles |
| Charts | Chart.js |
| Styling | Vanilla CSS (custom design system) |
| Fonts | Inter, JetBrains Mono (Google Fonts) |

---

## Getting Started

### Prerequisites

- Node.js 18+ and npm

### Installation

```bash
cd Prototype
npm install
```

### Development Server

```bash
npm run dev
```

Open **http://localhost:5173/** in your browser.

### Production Build

```bash
npm run build
```

Output is written to the `dist/` directory.

---

## Project Structure

```
Prototype/
├── index.html                      # Application entry point
├── package.json
├── public/
│   └── data/
│       └── sample-route.json       # Pre-processed IO-VNBD dataset (JSON)
└── src/
    ├── main.js                     # Application orchestrator
    ├── core/                       # Navigation engine modules
    │   ├── ins-engine.js           # Strapdown INS mechanization
    │   ├── kalman-filter.js        # Error-state EKF (8-state)
    │   ├── particle-filter.js      # RBPF with Kalman Filter Bank
    │   ├── map-matching.js         # Road network constraints & NHC
    │   └── gnss-ins-fusion.js      # Fusion controller & mode transitions
    ├── data/                       # Data processing
    │   ├── dataset-loader.js       # JSON dataset loader
    │   └── signal-processing.js    # Low-pass filter, bias estimation
    ├── ui/                         # Visualization modules
    │   ├── map-renderer.js         # Leaflet map, trajectory layers, particles
    │   └── charts.js               # Chart.js error, accel, speed plots
    └── styles/
        └── main.css                # Design system & layout
```

## Dashboard Features

### Map View
- **Green line** — GPS ground truth trajectory
- **Red dashed line** — Pure INS dead reckoning (shows drift accumulation)
- **Blue solid line** — RBPF fused solution (map-aided correction)
- **Particle cloud** — Visualized during dead reckoning mode

### Sidebar Controls
- **Navigation State** — Real-time position error, speed, heading, uncertainty
- **Error Comparison** — Side-by-side INS vs RBPF vs Fused error bars
- **GNSS Outage Simulation** — Configure start position and duration to simulate signal loss
- **Algorithm Parameters** — Tune particle count, d_min threshold, toggle map matching / NHC / RBPF
- **Performance Summary** — Total distance, drift rate, P95 errors, effective particle count

### Bottom Charts
- Position error over time (INS vs RBPF)
- Accelerometer X/Y/Z time series
- Speed and heading plot

### Playback Controls
- Play / Pause, Restart, click-to-seek progress bar
- Speed selector: 1x, 5x, 10x, 25x, 50x
- Keyboard shortcuts: `Space` (play/pause), `R` (restart)

---

## Algorithm Pipeline

```
┌──────────────────────────────────────────────────────────────┐
│                    GNSS Available?                           │
│              YES ──────────┬────────── NO                    │
│                            │                                 │
│  ┌─────────────────┐       │       ┌──────────────────────┐  │
│  │ GNSS+INS Mode   │       │       │ Dead Reckoning Mode  │  │
│  │                 │       │       │                      │  │
│  │ EKF GNSS Update │       │       │ INS Propagation      │  │
│  │ Reset INS State │       │       │ RBPF Prediction      │  │
│  │ PF GNSS Update  │       │       │ Map Matching         │  │
│  │                 │       │       │ NHC Constraints      │  │
│  └─────────────────┘       │       │ Particle Resampling  │  │
│                            │       └──────────────────────┘  │
│                            │                                 │
│              Output: Fused Position Estimate                 │
└──────────────────────────────────────────────────────────────┘
```

## Performance Benchmarks

The problem statement requires positional drift of less than **10% of total distance** during GNSS blackout. The RBPF algorithm with map matching achieves significantly lower drift rates than pure INS dead reckoning. Use the dashboard to compare error metrics across different outage scenarios and algorithm configurations.

---

## References

1. Mikov, A. et al., "Map-Aided Dead Reckoning for Land Vehicle Navigation Using Low-Cost Inertial Sensors," *ICINS 2021*
2. IO-VNBD Dataset — Onyekpe, U. et al., "Inertial and Odometry Benchmark Dataset for Ground Vehicle Positioning"
3. Groves, P.D., *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*
4. Gustafsson, F., "Particle Filter Theory and Practice with Positioning Applications"

## License

This prototype is developed for the **Smart India Hackathon 2026** under Problem Statement 26168 by the Indian Space Research Organisation (ISRO).
