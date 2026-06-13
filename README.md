# Gates FEAD Belt Drive – MATLAB Test Rig & Truck System
## Ashok Leyland H6 · Simscape / Simulink Model Suite

---

## 📁 File Structure

```
fead-matlab/
├── FEAD_params.m              ← Run this FIRST (loads all parameters)
├── build_FEAD_model.m         ← Builds FEAD_BeltDrive.slx (Simscape test rig)
├── build_truck_model.m        ← Builds H6_Truck_System.slx (full truck)
├── layout_editor.m            ← Interactive drag-and-drop pulley layout GUI
├── run_fead_sim.m             ← Simulate FEAD + generate 6 result plots
├── run_truck_sim.m            ← Simulate truck + generate 9 result plots
└── subsystems/
    └── build_waterpump_ss.m   ← Water pump Simscape subsystem + bearing life
```

---

## ⚡ Quick Start

### 1. Load all parameters
```matlab
>> FEAD_params
```

### 2. Run FEAD simulation (no Simulink needed)
```matlab
>> run_fead_sim
```
Outputs: hub loads, tensions, slip SF, fatigue life, bearing life, friction plots.

### 3. Run full truck simulation (no Simulink needed)
```matlab
>> run_truck_sim
```
Outputs: speed trace, RPM, gear shifts, FEAD loss, power budget, tire slip.

### 4. Open the interactive layout editor
```matlab
>> layout_editor
```
Drag pulleys, edit coordinates numerically, see live hub-load arrows update.

### 5. Build Simscape models (requires Simulink + Simscape)
```matlab
>> build_FEAD_model       % creates FEAD_BeltDrive.slx
>> build_truck_model      % creates H6_Truck_System.slx
```

### 6. Water pump subsystem
```matlab
>> build_waterpump_ss     % creates WaterPump_Subsystem.slx + plots
```

---

## 🔩 FEAD System Data (Gates PDF Datum)

| Pulley | X (mm) | Y (mm) | R (mm) | SR | Hub Force (N) | Dir (°) |
|--------|--------|--------|--------|----|---------------|---------|
| CRK | 0 | 0 | 79.57 | 1.000 | 2658.9 | 96 |
| FAN | 6 | 213.5 | 60.495 | 1.302 | 2866.4 | 258 |
| IDR | -122 | 235 | 38.7 | 2.069 | 1710.1 | 77 |
| ALT | -255 | 373.2 | 30.07 | 2.577 | 1678.1 | 279 |
| AC | -265 | 189 | 59.655 | 1.320 | 985.8 | 49 |
| TEN | -153.25 | 96.0 | 38.7 | 2.069 | 608.5 | 237 |

**Belt:** Gates MT620 AMD 8-Rib Aramid · L = 1577.3 mm · μ = 0.35 · m = 0.18 kg/m

---

## 🚛 Truck System Parameters

| Parameter | Value |
|-----------|-------|
| Engine | AL H6 6-Cyl Diesel · 175 kW @ 2400 RPM |
| Max Torque | 800 Nm @ 1200–1600 RPM |
| GVW | 16 000 kg |
| Transmission | 6-speed AMT [6.56 … 1.00] · Final drive 4.10 |
| Tyres | 315/80 R22.5 · R_loaded = 0.513 m |
| Drive config | 6×4 (4 driven wheels) |
| Wheelbase | 4.20 m |

---

## 📐 FEAD Subsystems

| Subsystem | Physics Model |
|-----------|--------------|
| **Belt Dynamics** | Capstan equation T_tight/T_slack = e^(μθ) |
| **Centrifugal Tension** | Tc = ṁ·v² |
| **Belt Slip SF** | SF = ln(Tt/Ts) / (μ·θ) |
| **Fatigue Life** | Wöhler + Palmgren-Miner (WLTC duty cycle) |
| **WP Bearing Life** | ISO 281 L10 (Ball + Roller series composite) |
| **Frictional Power** | ΔP = μ_bearing · F_hub · ω · R (AC vs no-AC) |
| **Tensioner** | Angular spring-damper: J·θ̈ + c·θ̇ + k·θ = T_belt |

---

## 🚗 Truck Subsystems

| Subsystem | Physics Model |
|-----------|--------------|
| **Driver** | PID speed controller → throttle/brake demand |
| **Engine** | 2D lookup map (RPM × throttle → Nm) + flywheel inertia |
| **FEAD** | Power lookup tables per accessory → parasitic torque |
| **Transmission** | RPM-based gear selection + final drive ratio |
| **Driveline** | Torsional spring-damper propshaft + open differential |
| **Wheels** | Pacejka magic formula (B=10, C=1.9, D=μ·Fz, E=0.97) |
| **Chassis** | Longitudinal F=ma: traction – aero – rolling – grade |
| **Brakes** | Friction brake force: F_brk = demand × μ_peak × m·g |
| **Water Pump** | Gear-driven centrifugal pump + ISO 281 L10 life |

---

## 🔧 Editing Pulley Positions

### Method 1 — Layout Editor GUI (recommended)
```matlab
>> layout_editor
```
- Drag any pulley on the canvas
- Type exact X/Y/R values in the table
- Move the RPM slider — hub-load arrows update instantly
- Click **Save to Workspace** → updates `pulleys` variable
- Click **Run Simulation** → rebuilds model and plots results

### Method 2 — Edit FEAD_params.m directly
Change the `pulleys(k).x`, `.y`, `.r` values and re-run `FEAD_params`.

### Method 3 — In the command window
```matlab
>> pulleys(1).x = 10;   % move CRK 10mm in X
>> pulleys(3).y = 240;  % move IDR
>> run_fead_sim          % recompute immediately
```

---

## 📊 Result Plots (run_fead_sim)

1. Hub Loads vs RPM (all 6 pulleys + PDF reference points)
2. Belt Tensions — tight side (solid) & slack side (dashed)
3. Slip Safety Factor vs RPM (red line = slip limit SF=1)
4. WP Bearing Life L10A / L10B / Composite vs RPM
5. FEAD Power: AC-ON vs No-AC comparison
6. Belt Fatigue Life per pulley (WLTC weighted, km)

## 📊 Result Plots (run_truck_sim)

1. Vehicle speed vs reference
2. Engine RPM trace
3. Gear shifts (AMT staircase)
4. Engine output torque
5. FEAD parasitic loss over drive cycle
6. Throttle & brake demand
7. Wheel speed vs body speed (tire slip check)
8. Power budget: Engine / Traction / FEAD loss
9. Speed trace coloured by gear

---

## ⚙️ MATLAB Toolbox Requirements

| Feature | Required Toolbox |
|---------|-----------------|
| `run_fead_sim`, `run_truck_sim` | **Base MATLAB only** (ODE45) |
| `layout_editor` | **Base MATLAB only** (uifigure) |
| `build_FEAD_model` | Simulink + **Simscape** |
| `build_truck_model` | Simulink + **Simscape** |
| `build_waterpump_ss` | Simulink + **Simscape** |
| Driveline blocks | Simscape Driveline *(optional)* |

> **Note:** The `run_*.m` scripts work in base MATLAB without Simulink.
> The `build_*.m` scripts require Simulink + Simscape to create `.slx` files.

---

## 📤 Exporting Results

```matlab
% Save workspace results to Excel
writematrix([fead_sim_results.rpm_sweep' fead_sim_results.F_hub'], ...
    'FEAD_Results.xlsx', 'Sheet','HubLoads');

% Save figures as PDF
exportgraphics(gcf, 'FEAD_Results.pdf', 'ContentType','vector');
```

---

*Gates FEAD Advanced Engineering Suite · Ashok Leyland H6 · ISO 281 / Wöhler / Capstan*
