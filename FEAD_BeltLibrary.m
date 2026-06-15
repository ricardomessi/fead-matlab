%% FEAD_BeltLibrary.m  –  Complete Gates & Alternative Belt Catalog
%  Returns a struct array of available belt types with all physical properties.
%  Used by FEAD_App.m for belt selection and design validation.
%
%  Usage:  lib = FEAD_BeltLibrary();
% ─────────────────────────────────────────────────────────────────────────────
function lib = FEAD_BeltLibrary()

% ── Gates Micro-V (Poly-V) belts ─────────────────────────────────────────────
lib(1).id          = 'MT620_AMD_8RIB';
lib(1).name        = 'Gates MT620 AMD 8-Rib Aramid';
lib(1).brand       = 'Gates';
lib(1).type        = 'Poly-V (Micro-V)';
lib(1).ribs        = 8;
lib(1).pitch_mm    = 3.56;
lib(1).length_mm   = 1577.3;
lib(1).lin_mass    = 0.18;     % kg/m
lib(1).mu          = 0.35;
lib(1).EA          = 180000;   % N  axial stiffness
lib(1).wohler_m    = 10;
lib(1).wohler_Nref = 1e8;
lib(1).wohler_Tref = 1200;    % N
lib(1).T_max       = 3000;    % N  max allowable tension
lib(1).v_max       = 30;      % m/s max belt speed
lib(1).temp_min    = -40;     % °C
lib(1).temp_max    = 120;     % °C
lib(1).core        = 'Aramid';
lib(1).cover       = 'EPDM';
lib(1).standard    = 'Gates AMD';
lib(1).part_no     = '8PK1575';
lib(1).color_rgb   = [0.96 0.62 0.04];

lib(2).id          = 'MT480_6RIB';
lib(2).name        = 'Gates MT480 6-Rib Polyester';
lib(2).brand       = 'Gates';
lib(2).type        = 'Poly-V (Micro-V)';
lib(2).ribs        = 6;
lib(2).pitch_mm    = 3.56;
lib(2).length_mm   = 1480.0;
lib(2).lin_mass    = 0.14;
lib(2).mu          = 0.35;
lib(2).EA          = 120000;
lib(2).wohler_m    = 9;
lib(2).wohler_Nref = 8e7;
lib(2).wohler_Tref = 900;
lib(2).T_max       = 2200;
lib(2).v_max       = 28;
lib(2).temp_min    = -30;
lib(2).temp_max    = 110;
lib(2).core        = 'Polyester';
lib(2).cover       = 'EPDM';
lib(2).standard    = 'Gates MT';
lib(2).part_no     = '6PK1480';
lib(2).color_rgb   = [0.55 0.36 0.96];

lib(3).id          = 'MT620_AMD_6RIB';
lib(3).name        = 'Gates MT620 AMD 6-Rib Aramid';
lib(3).brand       = 'Gates';
lib(3).type        = 'Poly-V (Micro-V)';
lib(3).ribs        = 6;
lib(3).pitch_mm    = 3.56;
lib(3).length_mm   = 1577.3;
lib(3).lin_mass    = 0.136;
lib(3).mu          = 0.35;
lib(3).EA          = 135000;
lib(3).wohler_m    = 10;
lib(3).wohler_Nref = 1e8;
lib(3).wohler_Tref = 900;
lib(3).T_max       = 2250;
lib(3).v_max       = 30;
lib(3).temp_min    = -40;
lib(3).temp_max    = 120;
lib(3).core        = 'Aramid';
lib(3).cover       = 'EPDM';
lib(3).standard    = 'Gates AMD';
lib(3).part_no     = '6PK1575';
lib(3).color_rgb   = [0.65 0.54 0.98];

% ── Continental / ContiTech belts ─────────────────────────────────────────────
lib(4).id          = 'CONTI_8PK1600';
lib(4).name        = 'ContiTech 8PK1600 MultiRib';
lib(4).brand       = 'ContiTech';
lib(4).type        = 'Poly-V (Micro-V)';
lib(4).ribs        = 8;
lib(4).pitch_mm    = 3.56;
lib(4).length_mm   = 1600.0;
lib(4).lin_mass    = 0.179;
lib(4).mu          = 0.34;
lib(4).EA          = 175000;
lib(4).wohler_m    = 9.5;
lib(4).wohler_Nref = 9e7;
lib(4).wohler_Tref = 1150;
lib(4).T_max       = 2900;
lib(4).v_max       = 30;
lib(4).temp_min    = -40;
lib(4).temp_max    = 120;
lib(4).core        = 'Aramid';
lib(4).cover       = 'EPDM';
lib(4).standard    = 'ContiTech';
lib(4).part_no     = '8PK1600';
lib(4).color_rgb   = [0.20 0.85 0.60];

% ── Dayco belts ───────────────────────────────────────────────────────────────
lib(5).id          = 'DAYCO_8PK1575';
lib(5).name        = 'Dayco 8PK1575 HVAC Aramid';
lib(5).brand       = 'Dayco';
lib(5).type        = 'Poly-V (Micro-V)';
lib(5).ribs        = 8;
lib(5).pitch_mm    = 3.56;
lib(5).length_mm   = 1575.0;
lib(5).lin_mass    = 0.182;
lib(5).mu          = 0.36;
lib(5).EA          = 178000;
lib(5).wohler_m    = 10;
lib(5).wohler_Nref = 1e8;
lib(5).wohler_Tref = 1180;
lib(5).T_max       = 2950;
lib(5).v_max       = 30;
lib(5).temp_min    = -40;
lib(5).temp_max    = 115;
lib(5).core        = 'Aramid';
lib(5).cover       = 'EPDM';
lib(5).standard    = 'Dayco';
lib(5).part_no     = 'DAY-8PK1575';
lib(5).color_rgb   = [0.96 0.28 0.71];

% ── Classical V-belts ─────────────────────────────────────────────────────────
lib(6).id          = 'GATES_A55';
lib(6).name        = 'Gates A-Section Classical V-Belt';
lib(6).brand       = 'Gates';
lib(6).type        = 'Classical V-Belt';
lib(6).ribs        = 1;
lib(6).pitch_mm    = 12.7;  % top width
lib(6).length_mm   = 1397.0;
lib(6).lin_mass    = 0.115;
lib(6).mu          = 0.45;
lib(6).EA          = 60000;
lib(6).wohler_m    = 8;
lib(6).wohler_Nref = 5e7;
lib(6).wohler_Tref = 800;
lib(6).T_max       = 1800;
lib(6).v_max       = 25;
lib(6).temp_min    = -20;
lib(6).temp_max    = 100;
lib(6).core        = 'Polyester';
lib(6).cover       = 'Rubber';
lib(6).standard    = 'ISO 22';
lib(6).part_no     = 'A55';
lib(6).color_rgb   = [0.38 0.64 0.98];

fprintf('Belt library loaded: %d belt types available.\n', numel(lib));
end
