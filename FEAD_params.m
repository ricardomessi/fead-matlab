%% FEAD_params.m  –  Gates Belt Drive & Ashok Leyland H6 Truck – All Parameters
%  Run this script before opening any model. Sets workspace variables used
%  by every Simscape/Simulink block via 'Variable' or 'From Workspace'.
%
%  Usage:  >> FEAD_params
%          >> build_FEAD_model        % then open FEAD_BeltDrive.slx
%          >> build_truck_model       % then open H6_Truck_System.slx
% ─────────────────────────────────────────────────────────────────────────────
clear; clc;
fprintf('=== Loading FEAD & Truck Parameters (Ashok Leyland H6) ===\n\n');

%% ═══════════════════════════════════════════════════════════════════════════
%  1. BELT PROPERTIES  –  Gates Micro-V MT620 AMD  8-Rib Aramid
% ═══════════════════════════════════════════════════════════════════════════
belt.name        = 'Gates MT620 AMD 8-Rib Aramid';
belt.ribs        = 8;
belt.pitch_mm    = 3.56;          % mm  rib pitch
belt.length_mm   = 1577.3;        % mm  pitch length
belt.length_m    = belt.length_mm/1000;
belt.cross_area  = 64e-6;         % m²  effective cross-section
belt.lin_mass    = 0.18;          % kg/m  linear mass density
belt.EA          = 180e3;         % N    axial stiffness (E×A)
belt.mu          = 0.35;          % –    rubber-on-steel friction coeff
belt.mu_bearing  = 0.002;         % –    bearing friction coeff
% Wöhler fatigue model (Aramid core)
belt.wohler_m    = 10;            % –    exponent
belt.wohler_Nref = 1e8;           % –    reference cycles
belt.wohler_Tref = 1200;          % N    reference tension
belt.static_tension = 480;        % N    design static (MEAN position)

%% ═══════════════════════════════════════════════════════════════════════════
%  2. PULLEY DATUM COORDINATES  (Gates PDF – CRK origin)
%     Fields: x[mm], y[mm], r[mm], eff_dia[mm], speed_ratio, cw(bool)
% ═══════════════════════════════════════════════════════════════════════════
%  Struct array:  pulleys(i).name / .x / .y / .r / .eff / .sr / .cw
pulleys(1).name = 'CRK'; pulleys(1).x =    0;     pulleys(1).y =    0;
pulleys(1).r    = 79.57; pulleys(1).eff = 159.13;  pulleys(1).sr = 1.000; pulleys(1).cw = true;
pulleys(1).mass = 4.2;   pulleys(1).Iz  = 0.5*pulleys(1).mass*(pulleys(1).r/1000)^2;

pulleys(2).name = 'FAN'; pulleys(2).x =    6;     pulleys(2).y =  213.5;
pulleys(2).r    = 60.495;pulleys(2).eff = 121.98;  pulleys(2).sr = 1.302; pulleys(2).cw = true;
pulleys(2).mass = 8.5;   pulleys(2).Iz  = 0.5*pulleys(2).mass*(pulleys(2).r/1000)^2;

pulleys(3).name = 'IDR'; pulleys(3).x = -122;     pulleys(3).y =  235;
pulleys(3).r    = 38.7;  pulleys(3).eff =  79.40;  pulleys(3).sr = 2.069; pulleys(3).cw = false;
pulleys(3).mass = 0.8;   pulleys(3).Iz  = 0.5*pulleys(3).mass*(pulleys(3).r/1000)^2;

pulleys(4).name = 'ALT'; pulleys(4).x = -255;     pulleys(4).y =  373.2;
pulleys(4).r    = 30.07; pulleys(4).eff =  61.13;  pulleys(4).sr = 2.577; pulleys(4).cw = true;
pulleys(4).mass = 1.2;   pulleys(4).Iz  = 0.5*pulleys(4).mass*(pulleys(4).r/1000)^2;

pulleys(5).name = 'AC';  pulleys(5).x = -265;     pulleys(5).y =  189;
pulleys(5).r    = 59.655;pulleys(5).eff = 120.30;  pulleys(5).sr = 1.320; pulleys(5).cw = true;
pulleys(5).mass = 3.1;   pulleys(5).Iz  = 0.5*pulleys(5).mass*(pulleys(5).r/1000)^2;

pulleys(6).name = 'TEN'; pulleys(6).x = -153.25;  pulleys(6).y =   96.0;
pulleys(6).r    = 38.7;  pulleys(6).eff =  79.40;  pulleys(6).sr = 2.069; pulleys(6).cw = false;
pulleys(6).mass = 0.9;   pulleys(6).Iz  = 0.5*pulleys(6).mass*(pulleys(6).r/1000)^2;

N_pulleys = numel(pulleys);

% Wrap angles from PDF (degrees)
wrap_angles_deg = struct('CRK',166.5,'FAN',127.6,'IDR',108.4,'ALT',145.1,'AC',105.7,'TEN',76.4);

% PDF baseline tensions & hub forces (MEAN tensioner, 1200 RPM)
pdf.CRK = struct('T',2190,'F',2658.9,'dir',96 ); 
pdf.FAN = struct('T', 945,'F',2866.4,'dir',258);
pdf.IDR = struct('T',1044,'F',1710.1,'dir',77 );
pdf.ALT = struct('T', 712,'F',1678.1,'dir',279);
pdf.AC  = struct('T', 498,'F', 985.8,'dir',49 );
pdf.TEN = struct('T', 480,'F', 608.5,'dir',237);

%% ═══════════════════════════════════════════════════════════════════════════
%  3. TENSIONER POSITIONS
% ═══════════════════════════════════════════════════════════════════════════
ten_pos(1) = struct('label','FREE',    'arm',32.0, 'x',-163.7,'y',119.7,'T',286.3);
ten_pos(2) = struct('label','REPLACE', 'arm',24.3, 'x',-158.0,'y',109.0,'T',381.2);
ten_pos(3) = struct('label','MAX',     'arm',19.1, 'x',-154.9,'y',101.4,'T',440.2);
ten_pos(4) = struct('label','MEAN',    'arm',15.4, 'x',-153.2,'y', 96.0,'T',480.0);
ten_pos(5) = struct('label','MIN',     'arm',11.9, 'x',-151.9,'y', 90.5,'T',519.2);
ten_pos(6) = struct('label','LOAD',    'arm',358.7,'x',-150.0,'y', 70.0,'T',677.9);
ten_idx    = 4;   % default = MEAN
% Tensioner spring
ten.pivot_x    = -240;       % mm
ten.pivot_y    =   72;       % mm
ten.arm_length =   90;       % mm
ten.k_spring   =  0.883;     % Nm/deg
ten.preload    = 19.49;      % Nm
ten.mean_load  = 34.12;      % Nm
ten.Iz         =  0.005;     % kg·m²

%% ═══════════════════════════════════════════════════════════════════════════
%  4. ACCESSORY POWER vs RPM  (interpolation table)
% ═══════════════════════════════════════════════════════════════════════════
load_table.rpm = [500  800 1000 1200 1400 1600 1800 2000];
load_table.CRK = [3.30 4.55 6.54 8.36 10.69 13.20 18.17 22.24];
load_table.FAN = [0.50 0.80 1.80 2.80  4.60  6.50 10.80 14.30];
load_table.IDR = [0.10 0.15 0.17 0.20  0.22  0.25  0.27  0.30];
load_table.ALT = [1.60 2.12 2.67 3.06  3.23  3.40  3.55  3.70];
load_table.AC  = [1.00 1.33 1.73 2.10  2.42  2.80  3.28  3.64];
load_table.TEN = [0.10 0.15 0.17 0.20  0.22  0.25  0.27  0.27];

%% ═══════════════════════════════════════════════════════════════════════════
%  5. WATER PUMP BEARING DATA  (C&U)
% ═══════════════════════════════════════════════════════════════════════════
wp.gear_ratio  = 1.35;
wp.F_radial    = 358;         % N   impeller radial load
wp.ball.Cr     = 19035;       % N   basic dynamic load rating
wp.ball.p      = 3;           % –   life exponent
wp.roller.Cr   = 38179;       % N
wp.roller.p    = 10/3;
wp.ref.L10A    = 17820;       % h   C&U reference life
wp.ref.L10B    =  3860;       % h
wp.ref.L10comp =  3305;       % h

%% ═══════════════════════════════════════════════════════════════════════════
%  6. ENGINE – Ashok Leyland H6  6-Cyl Diesel
% ═══════════════════════════════════════════════════════════════════════════
engine.name           = 'AL H6 6-Cyl Diesel';
engine.displacement_L = 5.1;
engine.cylinders      = 6;
engine.bore_mm        = 94;
engine.stroke_mm      = 122;
engine.comp_ratio     = 17.5;
engine.idle_rpm       = 600;
engine.rated_rpm      = 2500;
engine.max_torque_Nm  = 800;          % at 1200–1600 RPM
engine.rated_power_kW = 175;          % at 2400 RPM
engine.flywheel_Iz    = 1.8;          % kg·m²  (crankshaft + flywheel)
engine.friction_Nm    = 25;           % motoring friction torque at idle

% Engine torque map: rows=RPM, cols=throttle(0-1→0-1)
% [RPM; T_at_throttle_0, T_at_0.25, T_at_0.5, T_at_0.75, T_at_1.0]
engine.torque_rpm  = [600 800 1000 1200 1400 1600 1800 2000 2200 2400 2500];
engine.torque_thr  = [0  0.25  0.50  0.75  1.00];
engine.torque_map  = [   % Nm  (rows=RPM, cols=throttle)
%  0    0.25   0.50   0.75  1.00
   0   120    240    360   460;   % 600
   0   150    300    500   580;   % 800
   0   175    360    580   680;   % 1000
   0   200    420    650   780;   % 1200
   0   210    440    680   800;   % 1400
   0   210    440    680   800;   % 1600
   0   200    420    650   780;   % 1800
   0   185    390    610   730;   % 2000
   0   165    350    560   680;   % 2200
   0   140    300    490   600;   % 2400
   0   120    260    420   520;   % 2500
];

%% ═══════════════════════════════════════════════════════════════════════════
%  7. TRANSMISSION – AMT 6-Speed
% ═══════════════════════════════════════════════════════════════════════════
trans.ratios     = [6.56  3.83  2.37  1.59  1.19  1.00];  % 6 forward
trans.ratio_rev  = 5.48;
trans.final_drive= 4.10;
trans.clutch_Iz  = 0.15;     % kg·m²
trans.clutch_mu  = 0.35;
trans.shift_time = 0.4;      % s   AMT shift duration
trans.upshift_rpm  = 2200;   % RPM trigger up
trans.downshift_rpm=  900;   % RPM trigger down

%% ═══════════════════════════════════════════════════════════════════════════
%  8. DRIVELINE
% ═══════════════════════════════════════════════════════════════════════════
driveline.propshaft_k  = 18000; % Nm/rad  torsional stiffness
driveline.propshaft_c  =   120; % Nm·s/rad damping
driveline.Iz_shaft     =   0.8; % kg·m²
driveline.diff_ratio   =   1.0; % (split ratio, open diff assumed 50/50)

%% ═══════════════════════════════════════════════════════════════════════════
%  9. WHEELS & TYRES  (315/80 R22.5 typical truck rear)
% ═══════════════════════════════════════════════════════════════════════════
tyre.label           = '315/80 R22.5';
tyre.R_loaded_m      = 0.513;        % m  loaded radius
tyre.R_free_m        = 0.530;        % m
tyre.mass_kg         = 65;           % kg
tyre.Iz              = tyre.mass_kg * tyre.R_loaded_m^2 / 2;
tyre.mu_peak         = 0.85;         % peak friction coeff
tyre.C_alpha         = 80000;        % N/rad  cornering stiffness
tyre.rolling_res     = 0.007;        % Cr  rolling resistance coeff
% Pacejka simplified (longitudinal)
tyre.B = 10; tyre.C = 1.9; tyre.D = tyre.mu_peak; tyre.E = 0.97;
n_wheels = 6;  n_drive_wheels = 4;   % 6×4 truck

%% ═══════════════════════════════════════════════════════════════════════════
%  10. VEHICLE / CHASSIS
% ═══════════════════════════════════════════════════════════════════════════
vehicle.GVW_kg        = 16000;       % Gross Vehicle Weight
vehicle.curb_kg       = 8500;
vehicle.payload_kg    = vehicle.GVW_kg - vehicle.curb_kg;
vehicle.wheelbase_m   = 4.20;
vehicle.cg_height_m   = 1.05;
vehicle.frontal_area  = 7.5;         % m²
vehicle.Cd            = 0.65;        % drag coefficient
vehicle.rho_air       = 1.225;       % kg/m³
vehicle.Iz_yaw        = 35000;       % kg·m²  yaw inertia

%% ═══════════════════════════════════════════════════════════════════════════
%  11. SIMULATION SETTINGS
% ═══════════════════════════════════════════════════════════════════════════
sim.T_end_fead    = 30;       % s   FEAD test rig duration
sim.T_end_truck   = 120;      % s   Truck drive cycle duration
sim.dt            = 1e-4;     % s   fixed step (for Simscape)
sim.solver        = 'ode23t';
sim.rpm_init      = 1200;     % RPM  initial engine speed
sim.tension_init  = 480;      % N    initial static tension

%% ═══════════════════════════════════════════════════════════════════════════
%  12. DRIVE CYCLE  (RPM profile for FEAD test)
% ═══════════════════════════════════════════════════════════════════════════
dc_t   = [0   5    10   15   20   25   30];        % s
dc_rpm = [800 1200 1600 2000 1600 1200 800];       % RPM
dc_thr = [0.3 0.5  0.7  1.0  0.7  0.5  0.3];

%% Summary
fprintf('Pulleys loaded      : %d\n', N_pulleys);
fprintf('Belt length         : %.1f mm\n', belt.length_mm);
fprintf('Design tension      : %.0f N\n', belt.static_tension);
fprintf('Engine max torque   : %.0f Nm @ 1200-1600 RPM\n', engine.max_torque_Nm);
fprintf('Engine rated power  : %.0f kW @ 2400 RPM\n', engine.rated_power_kW);
fprintf('GVW                 : %.0f kg\n', vehicle.GVW_kg);
fprintf('Transmission        : %d-speed AMT [%.2f ... %.2f]\n', ...
        numel(trans.ratios), trans.ratios(1), trans.ratios(end));
fprintf('\nParameters ready. Run build_FEAD_model or build_truck_model.\n');
