%% startup_fead.m  –  Auto-runs when MATLAB opens the fead-matlab folder
%  Sets path, loads parameters, and prints quick-start menu.
% ─────────────────────────────────────────────────────────────────────────────

addpath(genpath(pwd));   % add all subfolders

% Load all parameters
FEAD_params;

% Welcome banner
clc;
fprintf('\n');
fprintf('╔═══════════════════════════════════════════════════════════════════╗\n');
fprintf('║   FEAD Belt Drive Test Rig  –  MATLAB Suite               ║\n');
fprintf('║   H6 OEM Engine  ·  Simscape / Simulink / App Designer       ║\n');
fprintf('╠═══════════════════════════════════════════════════════════════════╣\n');
fprintf('║                                                                   ║\n');
fprintf('║  ▶ FULL APP (recommended):                                        ║\n');
fprintf('║     >> FEAD_App            ← main interactive app + animation     ║\n');
fprintf('║                                                                   ║\n');
fprintf('║  ▶ QUICK SIMULATIONS:                                             ║\n');
fprintf('║     >> run_fead_sim        ← FEAD plots (no Simulink needed)      ║\n');
fprintf('║     >> run_truck_sim       ← Full truck ODE simulation            ║\n');
fprintf('║     >> build_waterpump_ss  ← WP bearing life + pump curves        ║\n');
fprintf('║                                                                   ║\n');
fprintf('║  ▶ SIMSCAPE MODELS:                                               ║\n');
fprintf('║     >> build_FEAD_model    ← FEAD_BeltDrive.slx                  ║\n');
fprintf('║     >> build_truck_model   ← H6_Truck_System.slx                 ║\n');
fprintf('║                                                                   ║\n');
fprintf('║  ▶ UTILITIES:                                                     ║\n');
fprintf('║     >> layout_editor       ← Drag-and-drop layout editor          ║\n');
fprintf('║     >> FEAD_DataWindow()   ← Standalone data window               ║\n');
fprintf('║     >> push_to_github(...)← Push model to GitHub                 ║\n');
fprintf('║                                                                   ║\n');
fprintf('╚═══════════════════════════════════════════════════════════════════╝\n\n');
fprintf('Parameters loaded. Type FEAD_App to launch.\n\n');
