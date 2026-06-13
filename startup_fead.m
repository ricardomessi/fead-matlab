%% startup_fead.m  –  Auto-runs when MATLAB opens the fead-matlab folder
%  Sets the working directory, adds subsystems to path, loads parameters,
%  and prints the quick-start menu.
% ─────────────────────────────────────────────────────────────────────────────

% Add subsystems folder to path
addpath(fullfile(pwd, 'subsystems'));

% Load all parameters
FEAD_params;

% Print welcome banner
clc;
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════════════════╗\n');
fprintf('║   Gates FEAD Belt Drive – MATLAB Test Rig                   ║\n');
fprintf('║   Ashok Leyland H6 · Simscape / Simulink Suite              ║\n');
fprintf('╠══════════════════════════════════════════════════════════════╣\n');
fprintf('║  Quick Start:                                                ║\n');
fprintf('║   >> run_fead_sim        (FEAD plots — no Simulink needed)   ║\n');
fprintf('║   >> run_truck_sim       (Truck sim — no Simulink needed)    ║\n');
fprintf('║   >> layout_editor       (Drag-and-drop pulley layout GUI)   ║\n');
fprintf('║   >> build_FEAD_model    (Build FEAD_BeltDrive.slx)         ║\n');
fprintf('║   >> build_truck_model   (Build H6_Truck_System.slx)        ║\n');
fprintf('║   >> build_waterpump_ss  (WP subsystem + bearing life)       ║\n');
fprintf('╚══════════════════════════════════════════════════════════════╝\n\n');
