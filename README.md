# FEAD Belt Drive Test Rig — MATLAB Suite
## H6 OEM Engine · Simscape / Simulink / Interactive App

[![GitHub](https://img.shields.io/badge/Repo-ricardomessi%2Ffead--matlab-blue)](https://github.com/ricardomessi/fead-matlab)

---

## 🚀 Quick Start

**Double-click `RUN_ALL.bat`** and select option `[1]` — or in MATLAB:

```matlab
>> FEAD_params      % load all parameters first
>> FEAD_App         % launch full interactive app
```

---

## 📁 File Structure

```
fead-matlab/
│
├── 🟡 FEAD_App.m              ← MAIN APP — full UI, animation, validation
├── 🟡 FEAD_params.m           ← Run FIRST — loads all physical parameters
├── 🟡 startup_fead.m          ← Auto-runs on MATLAB open (sets path + welcome)
│
├── ⚙  Core Simulation
│   ├── run_fead_sim.m         ← RPM sweep → 6 result plots (no Simulink)
│   ├── run_truck_sim.m        ← ODE45 truck sim → 9 result plots (no Simulink)
│   └── FEAD_PostProcess.m     ← Post-process Simscape results → 9 plots
│
├── 🔧 Simscape Model Builders
│   ├── FEAD_TestRig_Builder.m ← Builds FEAD_TestRig.slx (Simscape physical)
│   ├── build_FEAD_model.m     ← Builds FEAD_BeltDrive.slx
│   ├── build_truck_model.m    ← Builds H6_Truck_System.slx
│   └── subsystems/
│       └── build_waterpump_ss.m ← WP subsystem + bearing life
│
├── 🧩 App Components
│   ├── FEAD_Animation.m       ← Real-time belt drive animation engine
│   ├── FEAD_Validator.m       ← 8-check validation + layout optimizer
│   ├── FEAD_DataWindow.m      ← Separate live data window (5 tabs)
│   ├── FEAD_BeltLibrary.m     ← Belt catalog (6 types: Reference/Conti/Dayco)
│   └── layout_editor.m        ← Standalone drag-and-drop layout editor
│
├── 🌐 GitHub Integration
│   └── push_to_github.m       ← Commit + push results (reads GH_TOKEN env var)
│
└── 📋 Launchers
    └── RUN_ALL.bat            ← One-click interactive launcher
```

---

## 🖥️ FEAD App Features

### Left Panel — Controls
| Control | Description |
|---------|-------------|
| **Belt Selector** | 6 belt types: MT620 AMD (8-rib/6-rib), ContiTech, Dayco, V-belt |
| **RPM Slider** | 400–2500 RPM — all plots update live |
| **Tension Slider** | 100–1000 N static tension |
| **Tensioner Position** | FREE / REPLACE / MAX / MEAN / MIN / LOAD |
| **AC Compressor checkbox** | Toggle AC load on/off |
| **Night Run checkbox** | Alternator at maximum load |
| **BAS checkbox** | Belt-Alternator-Starter mode |
| **Temperature input** | Operating temperature check |
| **Pulley layout table** | Edit X / Y / R for all 6 pulleys numerically |

### Centre Panel — Live Animation
- 🔴 **Rotating pulleys** with spokes, hub, and outer disc
- 🟡 **Moving belt markers** (travel speed ∝ RPM)
- 🔴 **Hub load arrows** (quiver, magnitude to scale)
- 🟢 **Slip status rings** (green=OK, amber=marginal, red=SLIP)
- **Drag pulleys** with mouse to move them — validation updates on release
- Live RPM / belt velocity / design score overlays

### Right Panel — 5 Plot Tabs
1. **Hub Loads vs RPM** — all 6 pulleys + PDF reference markers
2. **Belt Tensions** — tight (solid) + slack (dashed) vs RPM
3. **Slip Safety Factor** — Capstan SF vs RPM (SF=1 red line)
4. **Belt Fatigue Life** — Wöhler + Palmgren-Miner (WLTC weighted, km)
5. **Validation** — pass/fail bar chart + suggestion text area

### Action Buttons
| Button | Action |
|--------|--------|
| ▶ Start Animation | 25 Hz timer-driven live animation |
| ✔ Validate Design | Run all 8 checks → score alert |
| ⚙ Optimize Layout | Move tensioner to minimize total hub load |
| 📊 Data Window | Open separate 5-tab live data window |
| 🔧 Build Simscape | Build `FEAD_TestRig.slx` and open in Simulink |
| ⬆ Push to GitHub | Commit layout JSON + validation report |
| ↺ Reset Datum | Restore Reference PDF reference coordinates |
| ⬇ Import from Web | Load `.json` exported from web tool |

---

## 📐 Physics & Validation Checks (8 checks → score out of 100)

| # | Check | Limit | Physics |
|---|-------|-------|---------|
| 1 | Belt velocity | ≤ v_max (30 m/s) | v = π·d_CRK·n/60 |
| 2 | Slip safety factor | SF ≥ 1.3 all pulleys | SF = ln(Tt/Ts)/(μ·θ) |
| 3 | Max tension | ≤ T_max (3000 N) | Tt = T₀ + Teff/2 + Tc |
| 4 | Min wrap angle | ≥ 60° all pulleys | geometry from XY coords |
| 5 | Span lengths | 30–600 mm | Euclidean distance |
| 6 | Hub loads | ≤ 5000 N | vector sum of span tensions |
| 7 | Belt fatigue life | ≥ 200 000 km | Wöhler + Palmgren-Miner |
| 8 | Temperature | within belt rating | ±1°C per spec |

---

## 📊 Live Data Window (5 Tabs)

| Tab | Content |
|-----|---------|
| **Hub Loads** | F_hub, direction, T_tight, T_slack, T_centrifugal per pulley |
| **Tensions** | Power(kW), v_belt, T_eff, T_tight, T_slack, Slip SF per pulley |
| **Validation** | Score badge, 8-check table, suggestions text area |
| **Fatigue/Life** | Life(km), Miner damage, WP bearing L10A/L10B/composite |
| **Belt Data** | All belt physical properties + current utilisation % |

---

## 🔩 Simscape Test Rig (`FEAD_TestRig.slx`)

Built by `FEAD_TestRig_Builder.m`:

```
EngineSource (ω source)
    │
    ├─── CRK_Iz (inertia) ──► CRK_Spring ──► CRK_LoadTorque (LUT)
    │                                              │
    ├─── FAN_Iz ──► FAN_Spring ──► FAN_LoadTorque │
    ├─── IDR_Iz ──► IDR_Spring ──► IDR_LoadTorque │
    ├─── ALT_Iz ──► ALT_Spring ──► ALT_LoadTorque │  ◄── All measured
    ├─── AC_Iz  ──► AC_Spring  ──► AC_LoadTorque  │      by TorqSensor
    └─── TEN_Iz ──► TEN_Arm_Spring + TEN_Arm_Damp │      + To Workspace
```

All torques and angular velocities logged to workspace as `{pulley}_torque` and `{pulley}_omega`.

---

## 🚛 Truck System (`H6_Truck_System.slx`)

| Subsystem | Model |
|-----------|-------|
| Driver | PID speed controller |
| Engine | 2D torque map (RPM × throttle) + flywheel inertia |
| FEAD | Power LUT per accessory → parasitic torque |
| AMT | RPM-threshold 6-speed gear selection |
| Driveline | Torsional spring-damper propshaft |
| Wheels | Pacejka magic formula (B=10, C=1.9, D=μ·Fz) |
| Chassis | Longitudinal F=ma with aero + grade |
| Brakes | Friction brake demand |

---

## ⚙️ GitHub Integration

```matlab
% Set token once per session (never in code):
>> setenv('GH_TOKEN','your_token_here')

% Push from within FEAD_App (button), or manually:
>> push_to_github(pulleys, belt, report)
```
Pushes: `latest_layout.json` + `VALIDATION_REPORT.md` + all `.m` files

**Repository:** https://github.com/ricardomessi/fead-matlab

---

## 🌐 Import from Web Tool

1. Open your **belt-drive-advanced** website
2. Configure layout + conditions
3. Export JSON (Download button)
4. In FEAD_App → **⬇ Import from Web** → select `.json`
5. Layout and conditions applied instantly

---

## 📋 MATLAB Toolbox Requirements

| Feature | Required |
|---------|---------|
| `FEAD_App`, `run_fead_sim`, `run_truck_sim` | **Base MATLAB only** |
| `FEAD_TestRig_Builder`, `build_FEAD_model` | Simulink + **Simscape** |
| `build_truck_model` | Simulink + Simscape + Simscape Driveline *(optional)* |
| `build_waterpump_ss` | Simulink + Simscape |

---

## 🔧 Belt Catalog

| Belt | Brand | Ribs | L (mm) | Core | T_max (N) |
|------|-------|------|--------|------|-----------|
| MT620 AMD 8-Rib | Reference | 8 | 1577 | Aramid | 3000 |
| MT620 AMD 6-Rib | Reference | 6 | 1577 | Aramid | 2250 |
| MT480 6-Rib | Reference | 6 | 1480 | Polyester | 2200 |
| 8PK1600 MultiRib | ContiTech | 8 | 1600 | Aramid | 2900 |
| HVAC 8PK1575 | Dayco | 8 | 1575 | Aramid | 2950 |
| A-Section | Reference | 1 | 1397 | Polyester | 1800 |

---

*FEAD Advanced Engineering Suite · H6 OEM Engine · ISO 281 / Wöhler / Capstan · WLTC Duty Cycle*
