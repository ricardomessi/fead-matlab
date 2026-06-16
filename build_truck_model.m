%% build_truck_model.m  –  H6 Truck System Simulink Model (Robust Port-Handle Build)
%  Builds H6_Truck_System.slx with all subsystems connected using port handles.
%  Subsystems: Driver | Engine | FEAD | Transmission | Driveline | Wheels | Chassis | Brakes
%
%  Usage:  >> FEAD_params;  build_truck_model
% ─────────────────────────────────────────────────────────────────────────────

function build_truck_model()

if ~exist('engine','var'), FEAD_params; end

MDL = 'H6_Truck_System';
fprintf('\n=== Building %s.slx ===\n', MDL);

%% ── Parameters from workspace ────────────────────────────────────────────────
engine     = evalin('base','engine');
trans      = evalin('base','trans');
tyre       = evalin('base','tyre');
vehicle    = evalin('base','vehicle');
belt       = evalin('base','belt');
pulleys    = evalin('base','pulleys');
load_table = evalin('base','load_table');
pnames     = {'CRK','FAN','IDR','ALT','AC','TEN'};
np         = 6;

m          = vehicle.GVW_kg;
Cd         = vehicle.Cd;
A_f        = vehicle.frontal_area;
rho        = vehicle.rho_air;
Cr         = tyre.rolling_res;
R_w        = tyre.R_loaded_m;
n_drv      = 4;          % driven wheels (6×4)

%% ── Close & create model ─────────────────────────────────────────────────────
if bdIsLoaded(MDL), close_system(MDL,0); end
new_system(MDL,'Model');
open_system(MDL);

set_param(MDL,...
    'StopTime',   '120',...
    'Solver',     'ode45',...
    'SolverType', 'Variable-step',...
    'RelTol',     '1e-4',...
    'AbsTol',     '1e-5',...
    'MaxStep',    '0.05');

load_system('simulink');

%% ══════════════════════════════════════════════════════════════════════════════
%  SAFE BLOCK ADD + SAFE LINE CONNECT (port-handle based)
%% ══════════════════════════════════════════════════════════════════════════════
function blk = AB(lib, name, x, y, w, h)
    blk = [MDL '/' name];
    add_block(lib, blk, 'Position',[x y x+w y+h],'MakeNameUnique','off');
end

function sl(src, sp, si, dst, dp, di)
    % sl(src_block, port_type, port_idx, dst_block, port_type, port_idx)
    try
        ph_s = get_param(src,'PortHandles');
        ph_d = get_param(dst,'PortHandles');
        add_line(MDL, ph_s.(sp)(si), ph_d.(dp)(di), 'autorouting','on');
    catch e
        warning('Line %s[%s(%d)] -> %s[%s(%d)]: %s',...
            src,sp,si, dst,dp,di, e.message);
    end
end

%% ══════════════════════════════════════════════════════════════════════════════
%  SUBSYSTEM 1: DRIVER  (PID speed controller → throttle, brake demand)
%  Inputs:  v_ref (From Workspace), v_veh
%  Outputs: throttle [0,1], brake [0,1]
%% ══════════════════════════════════════════════════════════════════════════════
SS_DRV = AB('simulink/Ports & Subsystems/Subsystem','Driver',50,50,160,80);
delete_line(MDL, get_param([MDL '/Driver'],'LineHandles').Inport);
delete_line(MDL, get_param([MDL '/Driver'],'LineHandles').Outport);
Simulink.SubSystem.deleteContents([MDL '/Driver']);

% Build driver internals
D = [MDL '/Driver/'];
add_block('simulink/Ports & Subsystems/In1',  [D 'v_ref'],  'Position',[30 60 60 80],'Port','1');
add_block('simulink/Ports & Subsystems/In1',  [D 'v_veh'],  'Position',[30 130 60 150],'Port','2');
add_block('simulink/Math Operations/Sum',      [D 'Err'],    'Position',[120 80 150 140],'Inputs','+-');
add_block('simulink/Continuous/PID Controller',[D 'PID'],   'Position',[200 75 300 145]);
set_param([D 'PID'],'P','800','I','120','D','30','N','50','InitialConditionSource','internal');
add_block('simulink/Math Operations/Gain',     [D 'Thr_gain'],[D 'Thr_gain' ' DELETED'], 'Position',[340 65 420 95]);
add_block('simulink/Math Operations/Gain',     [D 'Brk_gain'],[D 'Brk_gain' ' DELETED'], 'Position',[340 125 420 155]);

% Clamp throttle/brake 0-1
add_block('simulink/Signal Routing/Saturation',[D 'Thr_sat'], 'Position',[440 65 510 95]);
set_param([D 'Thr_sat'],'UpperLimit','1','LowerLimit','0');
add_block('simulink/Signal Routing/Saturation',[D 'Brk_sat'], 'Position',[440 125 510 155]);
set_param([D 'Brk_sat'],'UpperLimit','1','LowerLimit','0');
add_block('simulink/Ports & Subsystems/Out1',  [D 'throttle'],'Position',[550 65 580 85],'Port','1');
add_block('simulink/Ports & Subsystems/Out1',  [D 'brake'],   'Position',[550 125 580 145],'Port','2');

% Driver connections (using add_line with port numbers inside subsystem)
add_line([MDL '/Driver'],'v_ref/1','Err/1','autorouting','on');
add_line([MDL '/Driver'],'v_veh/1','Err/2','autorouting','on');
add_line([MDL '/Driver'],'Err/1','PID/1','autorouting','on');
add_line([MDL '/Driver'],'PID/1','Thr_sat/1','autorouting','on');
add_line([MDL '/Driver'],'PID/1','Brk_sat/1','autorouting','on');  % negate in outer
add_line([MDL '/Driver'],'Thr_sat/1','throttle/1','autorouting','on');
add_line([MDL '/Driver'],'Brk_sat/1','brake/1','autorouting','on');

fprintf('  ✅ Driver subsystem\n');

%% ══════════════════════════════════════════════════════════════════════════════
%  SUBSYSTEM 2: ENGINE  (torque map + flywheel + friction)
%  Inputs:  throttle, omega_eng
%  Outputs: T_engine (Nm)
%% ══════════════════════════════════════════════════════════════════════════════
SS_ENG = AB('simulink/Ports & Subsystems/Subsystem','Engine',250,50,160,80);
Simulink.SubSystem.deleteContents([MDL '/Engine']);
E = [MDL '/Engine/'];

add_block('simulink/Ports & Subsystems/In1',        [E 'throttle'],  'Position',[30 60  60 80], 'Port','1');
add_block('simulink/Ports & Subsystems/In1',        [E 'omega_eng'], 'Position',[30 130 60 150],'Port','2');
add_block('simulink/Math Operations/Gain',           [E 'Rad2RPM'],   'Position',[100 125 180 155]);
set_param([E 'Rad2RPM'],'Gain','60/(2*pi)');
add_block('simulink/Lookup Tables/2-D Lookup Table', [E 'TorqueMap'],  'Position',[220 60  360 155]);
set_param([E 'TorqueMap'],...
    'RowIndex',    mat2str(engine.torque_thr),...
    'ColumnIndex', mat2str(engine.torque_rpm),...
    'Table',       mat2str(engine.torque_map'),...
    'ExtrapMethod','Clip','InterpMethod','Linear');
add_block('simulink/Math Operations/Sum',            [E 'Sub_fric'],  'Position',[400 80 430 140],'Inputs','+-');
add_block('simulink/Sources/Constant',               [E 'Friction'],  'Position',[300 185 380 215]);
set_param([E 'Friction'],'Value',sprintf('%g',engine.friction_Nm));
add_block('simulink/Ports & Subsystems/Out1',        [E 'T_engine'],  'Position',[470 95 500 115],'Port','1');

add_line([MDL '/Engine'],'throttle/1', 'TorqueMap/1','autorouting','on');
add_line([MDL '/Engine'],'omega_eng/1','Rad2RPM/1',  'autorouting','on');
add_line([MDL '/Engine'],'Rad2RPM/1',  'TorqueMap/2','autorouting','on');
add_line([MDL '/Engine'],'TorqueMap/1','Sub_fric/1', 'autorouting','on');
add_line([MDL '/Engine'],'Friction/1', 'Sub_fric/2', 'autorouting','on');
add_line([MDL '/Engine'],'Sub_fric/1', 'T_engine/1', 'autorouting','on');

fprintf('  ✅ Engine subsystem\n');

%% ══════════════════════════════════════════════════════════════════════════════
%  SUBSYSTEM 3: FEAD  (accessory power table → total parasitic torque)
%  Inputs:  omega_eng
%  Outputs: T_fead (Nm), P_fead_kW
%% ══════════════════════════════════════════════════════════════════════════════
SS_FEAD = AB('simulink/Ports & Subsystems/Subsystem','FEAD',450,50,160,80);
Simulink.SubSystem.deleteContents([MDL '/FEAD']);
F = [MDL '/FEAD/'];

add_block('simulink/Ports & Subsystems/In1',[F 'omega_eng'],'Position',[30 100 60 120],'Port','1');
add_block('simulink/Math Operations/Gain',  [F 'R2RPM2'],   'Position',[90 95 170 125]);
set_param([F 'R2RPM2'],'Gain','60/(2*pi)');

% One LUT per accessory, then sum
sum_inputs = repmat('+',1,np);
add_block('simulink/Math Operations/Sum',[F 'SumP'],'Position',[380 75 410 175],'Inputs',sum_inputs);

for k = 1:np
    pn   = pnames{k};
    P_k  = load_table.(pn);
    yi_k = 50 + (k-1)*40;
    add_block('simulink/Lookup Tables/1-D Lookup Table',[F pn '_Plut'],...
        'Position',[200 yi_k 330 yi_k+30]);
    set_param([F pn '_Plut'],...
        'BreakpointsForDimension1', mat2str(load_table.rpm),...
        'Table',                    mat2str(P_k),...
        'ExtrapMethod','Clip');
    add_line([MDL '/FEAD'],'R2RPM2/1',[pn '_Plut/1'],'autorouting','on');
    add_line([MDL '/FEAD'],[pn '_Plut/1'],sprintf('SumP/%d',k),'autorouting','on');
end

add_block('simulink/Ports & Subsystems/Out1',[F 'P_fead_kW'],'Position',[470 100 500 120],'Port','2');
add_line([MDL '/FEAD'],'SumP/1','P_fead_kW/1','autorouting','on');

% P_kW / max(omega,0.1) → T_fead
add_block('simulink/Math Operations/Gain',         [F 'kW2W'],    'Position',[450 55 530 85]);
set_param([F 'kW2W'],'Gain','1000');
add_block('simulink/Math Operations/Math Function', [F 'SafeOmeg'],'Position',[90 155 170 185]);
set_param([F 'SafeOmeg'],'Operator','abs');
add_block('simulink/Math Operations/Sum',           [F 'OmegClamp'],'Position',[200 150 260 190],'Inputs','++');
add_block('simulink/Sources/Constant',              [F 'MinOmeg'], 'Position',[130 200 190 230]);
set_param([F 'MinOmeg'],'Value','0.1');
add_block('simulink/Math Operations/Divide',        [F 'DivP'],    'Position',[550 55 620 115]);
add_block('simulink/Ports & Subsystems/Out1',       [F 'T_fead'],  'Position',[660 72 690 92],'Port','1');

add_line([MDL '/FEAD'],'SumP/1',     'kW2W/1',    'autorouting','on');
add_line([MDL '/FEAD'],'omega_eng/1','SafeOmeg/1','autorouting','on');
add_line([MDL '/FEAD'],'SafeOmeg/1', 'OmegClamp/1','autorouting','on');
add_line([MDL '/FEAD'],'MinOmeg/1',  'OmegClamp/2','autorouting','on');
add_line([MDL '/FEAD'],'kW2W/1',     'DivP/1',    'autorouting','on');
add_line([MDL '/FEAD'],'OmegClamp/1','DivP/2',    'autorouting','on');
add_line([MDL '/FEAD'],'DivP/1',     'T_fead/1',  'autorouting','on');

fprintf('  ✅ FEAD subsystem\n');

%% ══════════════════════════════════════════════════════════════════════════════
%  SUBSYSTEM 4: TRANSMISSION  (gear selection + ratio)
%  Inputs:  omega_eng
%  Outputs: gear_ratio_total
%% ══════════════════════════════════════════════════════════════════════════════
SS_TRANS = AB('simulink/Ports & Subsystems/Subsystem','Transmission',650,50,160,80);
Simulink.SubSystem.deleteContents([MDL '/Transmission']);
T = [MDL '/Transmission/'];

ratios_total = trans.ratios * trans.final_drive;   % [6×1] combined ratio

add_block('simulink/Ports & Subsystems/In1',       [T 'omega_eng'], 'Position',[30 80 60 100],'Port','1');
add_block('simulink/Math Operations/Gain',          [T 'R2RPM3'],    'Position',[90 75 170 105]);
set_param([T 'R2RPM3'],'Gain','60/(2*pi)');

% Gear thresholds → ratio output via 1-D LUT (RPM breakpoints map to ratio)
% Simple RPM-based: below 850→G1, 850-1100→G2 ... above 1950→G6
rpm_th = [0 850 1100 1350 1650 1950 3000];
g_idx  = [1 1   2    3    4    5    6];
ratio_lut = ratios_total(g_idx);

add_block('simulink/Lookup Tables/1-D Lookup Table',[T 'GearLUT'],'Position',[220 65 380 115]);
set_param([T 'GearLUT'],...
    'BreakpointsForDimension1', mat2str(rpm_th),...
    'Table',                    mat2str(ratio_lut),...
    'ExtrapMethod','Clip');
add_block('simulink/Ports & Subsystems/Out1',[T 'i_total'],'Position',[430 75 460 95],'Port','1');

add_line([MDL '/Transmission'],'omega_eng/1','R2RPM3/1','autorouting','on');
add_line([MDL '/Transmission'],'R2RPM3/1',  'GearLUT/1','autorouting','on');
add_line([MDL '/Transmission'],'GearLUT/1', 'i_total/1','autorouting','on');

fprintf('  ✅ Transmission subsystem\n');

%% ══════════════════════════════════════════════════════════════════════════════
%  SUBSYSTEM 5: DRIVELINE  (propshaft torsional + wheel speed feedback)
%  Inputs:  T_net (from engine-FEAD), i_total, omega_whl
%  Outputs: T_shaft_at_wheel, omega_eng_demanded
%% ══════════════════════════════════════════════════════════════════════════════
SS_DL = AB('simulink/Ports & Subsystems/Subsystem','Driveline',850,50,160,80);
Simulink.SubSystem.deleteContents([MDL '/Driveline']);
DL = [MDL '/Driveline/'];
k_s = 18000; c_s = 120;

add_block('simulink/Ports & Subsystems/In1',[DL 'T_net'],    'Position',[30 50  60 70], 'Port','1');
add_block('simulink/Ports & Subsystems/In1',[DL 'i_total'],  'Position',[30 110 60 130],'Port','2');
add_block('simulink/Ports & Subsystems/In1',[DL 'omega_whl'],'Position',[30 170 60 190],'Port','3');
add_block('simulink/Ports & Subsystems/In1',[DL 'theta_prop'],'Position',[30 230 60 250],'Port','4');
add_block('simulink/Ports & Subsystems/In1',[DL 'omega_eng'], 'Position',[30 290 60 310],'Port','5');

% T_shaft = k_s*theta_prop + c_s*(omega_eng - omega_whl*i_total)
add_block('simulink/Math Operations/Product',[DL 'Mul_whl_i'], 'Position',[120 160 180 200]);
add_block('simulink/Math Operations/Sum',    [DL 'dOmeg'],      'Position',[220 270 260 320],'Inputs','+-');
add_block('simulink/Math Operations/Gain',   [DL 'Kspring'],    'Position',[120 225 200 255]);
set_param([DL 'Kspring'],'Gain',sprintf('%g',k_s));
add_block('simulink/Math Operations/Gain',   [DL 'Kdamp'],      'Position',[300 265 380 305]);
set_param([DL 'Kdamp'],'Gain',sprintf('%g',c_s));
add_block('simulink/Math Operations/Sum',    [DL 'T_shaftSum'], 'Position',[430 200 470 280],'Inputs','++');

add_block('simulink/Math Operations/Product',[DL 'Twheel_i'],  'Position',[520 195 580 265]);
add_block('simulink/Math Operations/Gain',   [DL 'Eff'],       'Position',[620 210 700 250]);
set_param([DL 'Eff'],'Gain','0.94');
add_block('simulink/Ports & Subsystems/Out1',[DL 'T_wheel'],   'Position',[750 220 780 240],'Port','1');

add_line([MDL '/Driveline'],'omega_whl/1', 'Mul_whl_i/1', 'autorouting','on');
add_line([MDL '/Driveline'],'i_total/1',   'Mul_whl_i/2', 'autorouting','on');
add_line([MDL '/Driveline'],'theta_prop/1','Kspring/1',   'autorouting','on');
add_line([MDL '/Driveline'],'omega_eng/1', 'dOmeg/1',     'autorouting','on');
add_line([MDL '/Driveline'],'Mul_whl_i/1', 'dOmeg/2',     'autorouting','on');
add_line([MDL '/Driveline'],'dOmeg/1',     'Kdamp/1',     'autorouting','on');
add_line([MDL '/Driveline'],'Kspring/1',   'T_shaftSum/1','autorouting','on');
add_line([MDL '/Driveline'],'Kdamp/1',     'T_shaftSum/2','autorouting','on');
add_line([MDL '/Driveline'],'T_shaftSum/1','Twheel_i/1',  'autorouting','on');
add_line([MDL '/Driveline'],'i_total/1',   'Twheel_i/2',  'autorouting','on');
add_line([MDL '/Driveline'],'Twheel_i/1',  'Eff/1',       'autorouting','on');
add_line([MDL '/Driveline'],'Eff/1',       'T_wheel/1',   'autorouting','on');

fprintf('  ✅ Driveline subsystem\n');

%% ══════════════════════════════════════════════════════════════════════════════
%  SUBSYSTEM 6: WHEELS  (Pacejka magic formula)
%  Inputs:  omega_whl, v_veh, T_wheel_in, T_brake
%  Outputs: F_x_total (traction force N)
%% ══════════════════════════════════════════════════════════════════════════════
SS_WHL = AB('simulink/Ports & Subsystems/Subsystem','Wheels',1050,50,160,80);
Simulink.SubSystem.deleteContents([MDL '/Wheels']);
W = [MDL '/Wheels/'];

B=tyre.B; C=tyre.C; D=tyre.mu_peak; E_t=tyre.E;
Fz_per_whl = m*9.81/4;   % assume 4 wheels share load

add_block('simulink/Ports & Subsystems/In1',[W 'omega_whl'],'Position',[30 50  60 70], 'Port','1');
add_block('simulink/Ports & Subsystems/In1',[W 'v_veh'],    'Position',[30 120 60 140],'Port','2');

% kappa = (omega*R - v) / max(v, 0.01)
add_block('simulink/Math Operations/Gain',       [W 'GainR'],    'Position',[100 45 160 75]);
set_param([W 'GainR'],'Gain',sprintf('%g',R_w));
add_block('simulink/Math Operations/Sum',        [W 'SlipNum'],  'Position',[220 70 260 120],'Inputs','+-');
add_block('simulink/Math Operations/Abs',        [W 'AbsV'],     'Position',[100 115 160 145]);
add_block('simulink/Sources/Constant',           [W 'MinV'],     'Position',[100 165 160 195]);
set_param([W 'MinV'],'Value','0.01');
add_block('simulink/Math Operations/MinMax',     [W 'MaxV'],     'Position',[200 140 260 180]);
set_param([W 'MaxV'],'Function','max');
add_block('simulink/Math Operations/Divide',     [W 'DivKappa'], 'Position',[300 80 360 160]);
add_block('simulink/Discontinuities/Saturation', [W 'SatKappa'], 'Position',[400 95 460 145]);
set_param([W 'SatKappa'],'UpperLimit','1','LowerLimit','-1');

% Pacejka: Fx = Fz*D*sin(C*atan(B*kappa - E*(B*kappa - atan(B*kappa))))
add_block('simulink/Math Operations/Gain',        [W 'GainB'],    'Position',[510 95 570 145]);
set_param([W 'GainB'],'Gain',sprintf('%g',B));
add_block('simulink/Math Operations/Math Function',[W 'Atan1'],   'Position',[610 95 670 145]);
set_param([W 'Atan1'],'Operator','atan');
add_block('simulink/Math Operations/MATLAB Fcn',  [W 'PacejkaFcn'],'Position',[720 85 880 155]);
Bv=B; Cv=C; Dv=D; Ev=E_t; Fzv=Fz_per_whl;
set_param([W 'PacejkaFcn'],'MATLABFcn',...
    sprintf('%.4f*%.4f*sin(%.4f*atan(%.4f*u - %.4f*(%.4f*u - atan(%.4f*u))))',...
    Fzv,Dv,Cv,Bv,Ev,Bv,Bv),...
    'OutputDimensions','1');
add_block('simulink/Math Operations/Gain',        [W 'GainNwhl'], 'Position',[910 105 970 135]);
set_param([W 'GainNwhl'],'Gain',sprintf('%d',n_drv));
add_block('simulink/Ports & Subsystems/Out1',     [W 'F_x_total'],'Position',[1010 110 1040 130],'Port','1');

add_line([MDL '/Wheels'],'omega_whl/1','GainR/1',   'autorouting','on');
add_line([MDL '/Wheels'],'GainR/1',   'SlipNum/1',  'autorouting','on');
add_line([MDL '/Wheels'],'v_veh/1',   'SlipNum/2',  'autorouting','on');
add_line([MDL '/Wheels'],'v_veh/1',   'AbsV/1',     'autorouting','on');
add_line([MDL '/Wheels'],'AbsV/1',    'MaxV/1',     'autorouting','on');
add_line([MDL '/Wheels'],'MinV/1',    'MaxV/2',     'autorouting','on');
add_line([MDL '/Wheels'],'SlipNum/1', 'DivKappa/1', 'autorouting','on');
add_line([MDL '/Wheels'],'MaxV/1',    'DivKappa/2', 'autorouting','on');
add_line([MDL '/Wheels'],'DivKappa/1','SatKappa/1', 'autorouting','on');
add_line([MDL '/Wheels'],'SatKappa/1','PacejkaFcn/1','autorouting','on');
add_line([MDL '/Wheels'],'PacejkaFcn/1','GainNwhl/1','autorouting','on');
add_line([MDL '/Wheels'],'GainNwhl/1','F_x_total/1','autorouting','on');

fprintf('  ✅ Wheels (Pacejka) subsystem\n');

%% ══════════════════════════════════════════════════════════════════════════════
%  SUBSYSTEM 7: CHASSIS  (longitudinal F=ma)
%  Inputs:  F_x_total, v_veh, brake_demand, road_grade_deg
%  Outputs: a_veh, v_veh (integrated)
%% ══════════════════════════════════════════════════════════════════════════════
SS_CHS = AB('simulink/Ports & Subsystems/Subsystem','Chassis',1250,50,160,80);
Simulink.SubSystem.deleteContents([MDL '/Chassis']);
CH = [MDL '/Chassis/'];

add_block('simulink/Ports & Subsystems/In1',[CH 'F_traction'],'Position',[30 50  60 70], 'Port','1');
add_block('simulink/Ports & Subsystems/In1',[CH 'v_veh'],     'Position',[30 120 60 140],'Port','2');
add_block('simulink/Ports & Subsystems/In1',[CH 'brake'],     'Position',[30 190 60 210],'Port','3');
add_block('simulink/Ports & Subsystems/In1',[CH 'grade_deg'], 'Position',[30 260 60 280],'Port','4');

% Aero drag: 0.5*rho*Cd*A*v^2
add_block('simulink/Math Operations/Math Function',[CH 'vSq'],   'Position',[120 115 180 145]);
set_param([CH 'vSq'],'Operator','square');
add_block('simulink/Math Operations/Gain',         [CH 'AeroK'], 'Position',[210 115 290 145]);
set_param([CH 'AeroK'],'Gain',sprintf('%.4f',0.5*rho*Cd*A_f));

% Rolling resistance: Cr*m*g*cos(grade)
add_block('simulink/Math Operations/Trigonometry',[CH 'CosG'],   'Position',[120 195 180 225]);
set_param([CH 'CosG'],'Operator','cos','InputSame','on');
add_block('simulink/Math Operations/Gain',        [CH 'DegRad'], 'Position',[80 255 120 285]);
set_param([CH 'DegRad'],'Gain','pi/180');
add_block('simulink/Math Operations/Gain',        [CH 'RollK'],  'Position',[210 195 290 225]);
set_param([CH 'RollK'],'Gain',sprintf('%.4f',Cr*m*9.81));

% Grade force: m*g*sin(grade)
add_block('simulink/Math Operations/Trigonometry',[CH 'SinG'],   'Position',[120 265 180 295]);
set_param([CH 'SinG'],'Operator','sin','InputSame','on');
add_block('simulink/Math Operations/Gain',        [CH 'GradeK'], 'Position',[210 265 290 295]);
set_param([CH 'GradeK'],'Gain',sprintf('%.2f',m*9.81));

% Brake force: brake*mu_peak*m*g
add_block('simulink/Math Operations/Gain',        [CH 'BrakeK'], 'Position',[120 330 200 360]);
set_param([CH 'BrakeK'],'Gain',sprintf('%.2f',tyre.mu_peak*m*9.81));

% Sum: F_traction - F_aero - F_roll - F_grade - F_brake
add_block('simulink/Math Operations/Sum',          [CH 'NetF'],   'Position',[350 80 390 350],'Inputs','++---');
add_block('simulink/Math Operations/Gain',         [CH 'InvMass'],'Position',[430 185 510 215]);
set_param([CH 'InvMass'],'Gain',sprintf('%.6f',1/m));
add_block('simulink/Continuous/Integrator',        [CH 'IntV'],   'Position',[550 185 610 215]);
set_param([CH 'IntV'],'InitialCondition','0','LowerSaturationLimit','0');
add_block('simulink/Ports & Subsystems/Out1',      [CH 'v_veh_out'],'Position',[660 190 690 210],'Port','1');
add_block('simulink/Ports & Subsystems/Out1',      [CH 'a_veh'],    'Position',[660 130 690 150],'Port','2');

add_line([MDL '/Chassis'],'grade_deg/1','DegRad/1', 'autorouting','on');
add_line([MDL '/Chassis'],'DegRad/1',  'CosG/1',   'autorouting','on');
add_line([MDL '/Chassis'],'DegRad/1',  'SinG/1',   'autorouting','on');
add_line([MDL '/Chassis'],'v_veh/1',   'vSq/1',    'autorouting','on');
add_line([MDL '/Chassis'],'vSq/1',     'AeroK/1',  'autorouting','on');
add_line([MDL '/Chassis'],'CosG/1',    'RollK/1',  'autorouting','on');
add_line([MDL '/Chassis'],'SinG/1',    'GradeK/1', 'autorouting','on');
add_line([MDL '/Chassis'],'brake/1',   'BrakeK/1', 'autorouting','on');
add_line([MDL '/Chassis'],'F_traction/1','NetF/1', 'autorouting','on');
add_line([MDL '/Chassis'],'AeroK/1',    'NetF/2',  'autorouting','on');
add_line([MDL '/Chassis'],'RollK/1',    'NetF/3',  'autorouting','on');
add_line([MDL '/Chassis'],'GradeK/1',   'NetF/4',  'autorouting','on');
add_line([MDL '/Chassis'],'BrakeK/1',   'NetF/5',  'autorouting','on');
add_line([MDL '/Chassis'],'NetF/1',     'InvMass/1','autorouting','on');
add_line([MDL '/Chassis'],'InvMass/1',  'IntV/1',  'autorouting','on');
add_line([MDL '/Chassis'],'IntV/1',     'v_veh_out/1','autorouting','on');
add_line([MDL '/Chassis'],'InvMass/1',  'a_veh/1', 'autorouting','on');

fprintf('  ✅ Chassis subsystem\n');

%% ══════════════════════════════════════════════════════════════════════════════
%  TOP-LEVEL WIRING
%  Reference speed source → Driver → Engine → FEAD → Driveline → Wheels → Chassis → Driver (feedback)
%% ══════════════════════════════════════════════════════════════════════════════

% Drive cycle reference (t, v_kmh → m/s)
t_ref_v  = [0   5   15   35   70  100  115  120];
v_ref_v  = [0   0   20   80   80    0    0    0] / 3.6;
grade_v  = [0   0    0    0    0    5    0    0];

VREF = AB('simulink/Sources/From Workspace','v_ref_src',30,280,130,40);
set_param(VREF,'VariableName','v_ref_ts',...
    'ZeroOrderHold','on','Interpolate','on');
% Create timeseries and assign to workspace
ts_v = timeseries(v_ref_v', t_ref_v');
assignin('base','v_ref_ts', ts_v);

GRADE = AB('simulink/Sources/From Workspace','grade_src',30,340,130,40);
set_param(GRADE,'VariableName','grade_ts','ZeroOrderHold','on');
ts_g = timeseries(grade_v', t_ref_v');
assignin('base','grade_ts', ts_g);

% Engine angular velocity integrator (state: omega_eng)
INT_ENG = AB('simulink/Continuous/Integrator','Int_omega_eng',700,280,90,50);
set_param(INT_ENG,'InitialCondition','600*2*pi/60','LowerSaturationLimit','0');

% Wheel angular velocity integrator
INT_WHL = AB('simulink/Continuous/Integrator','Int_omega_whl',700,360,90,50);
set_param(INT_WHL,'InitialCondition','0','LowerSaturationLimit','0');

% Propshaft angle integrator
INT_PROP= AB('simulink/Continuous/Integrator','Int_theta_prop',700,440,90,50);
set_param(INT_PROP,'InitialCondition','0');

% Wheel EOM integrator numerics (T_wheel - Fx*R / Iz_whl)
IZ_WHL = tyre.Iz * 4;
GAIN_IWL= AB('simulink/Math Operations/Gain','Gain_InvIz',820,380,90,40);
set_param(GAIN_IWL,'Gain',sprintf('%.6f',1/IZ_WHL));

IZ_ENG = engine.flywheel_Iz;
GAIN_IEG= AB('simulink/Math Operations/Gain','Gain_InvIzEng',820,295,90,40);
set_param(GAIN_IEG,'Gain',sprintf('%.6f',1/IZ_ENG));

% Wheel force × R → wheel torque reaction
GAIN_FXR= AB('simulink/Math Operations/Gain','Gain_FxR',1200,360,90,40);
set_param(GAIN_FXR,'Gain',sprintf('%g',R_w));

SUM_ENG_EOM = AB('simulink/Math Operations/Sum','Sum_EngEOM',760,280,60,80,'Inputs','++--');
SUM_WHL_EOM = AB('simulink/Math Operations/Sum','Sum_WhlEOM',760,360,60,80,'Inputs','+-');

% Scopes & To Workspace outputs
WS_VEH = AB('simulink/Sinks/To Workspace','WS_v_veh', 1400,240,110,40);
set_param(WS_VEH,'VariableName','v_veh_sim','SaveFormat','Array');
WS_RPM2= AB('simulink/Sinks/To Workspace','WS_rpm',   1400,290,110,40);
set_param(WS_RPM2,'VariableName','rpm_sim','SaveFormat','Array');
WS_GEAR= AB('simulink/Sinks/To Workspace','WS_gear',  1400,340,110,40);
set_param(WS_GEAR,'VariableName','gear_sim','SaveFormat','Array');

SCOPE  = AB('simulink/Sinks/Scope','Main_Scope',1400,400,120,60);
set_param(SCOPE,'NumInputPorts','4');

% Top-level gain for RPM display
GAIN_RPM3 = AB('simulink/Math Operations/Gain','Gain_RadRPM',950,280,90,40);
set_param(GAIN_RPM3,'Gain','60/(2*pi)');

% --- Connect top-level using port handles ---
% v_ref → Driver input 1
sl(VREF,'Outport',1, SS_DRV,'Inport',1);
% Chassis v_veh_out → Driver input 2
sl(SS_CHS,'Outport',1, SS_DRV,'Inport',2);

% Driver throttle → Engine input 1
sl(SS_DRV,'Outport',1, SS_ENG,'Inport',1);
% Engine omega_eng ← integrator
sl(INT_ENG,'Outport',1, SS_ENG,'Inport',2);
sl(INT_ENG,'Outport',1, SS_FEAD,'Inport',1);
sl(INT_ENG,'Outport',1, SS_TRANS,'Inport',1);
sl(INT_ENG,'Outport',1, SS_DL,'Inport',5);

% Net engine torque → EOM integrator: dom/dt = (T_eng - T_fead - T_shaft/i) / Iz
sl(SS_ENG, 'Outport',1, Sum_EngEOM,'Inport',1);
sl(SS_FEAD,'Outport',1, Sum_EngEOM,'Inport',2);   % subtract FEAD

% Driveline
sl(Sum_EngEOM,'Outport',1, SS_DL,'Inport',1);
sl(SS_TRANS,  'Outport',1, SS_DL,'Inport',2);
sl(INT_WHL,   'Outport',1, SS_DL,'Inport',3);
sl(INT_PROP,  'Outport',1, SS_DL,'Inport',4);

% Wheel subsystem
sl(INT_WHL,  'Outport',1, SS_WHL,'Inport',1);
sl(SS_CHS,   'Outport',1, SS_WHL,'Inport',2);

% Chassis
sl(SS_WHL,  'Outport',1, SS_CHS,'Inport',1);
sl(SS_CHS,  'Outport',1, SS_CHS,'Inport',2);   % v_veh feedback
sl(SS_DRV,  'Outport',2, SS_CHS,'Inport',3);   % brake
sl(GRADE,   'Outport',1, SS_CHS,'Inport',4);

% EOM integrators
sl(Gain_InvIzEng,'Outport',1, INT_ENG,'Inport',1);
sl(Sum_EngEOM,   'Outport',1, Gain_InvIzEng,'Inport',1);

sl(Gain_InvIz,   'Outport',1, INT_WHL,'Inport',1);
sl(Sum_WhlEOM,   'Outport',1, Gain_InvIz,'Inport',1);
sl(SS_DL,        'Outport',1, Sum_WhlEOM,'Inport',1);
sl(Gain_FxR,     'Outport',1, Sum_WhlEOM,'Inport',2);
sl(SS_WHL,       'Outport',1, Gain_FxR,'Inport',1);

% Propshaft angle: d(theta)/dt = omega_eng - omega_whl * i_total
% (handled internally in Driveline — expose as extra output in future)

% Outputs
sl(SS_CHS,   'Outport',1, WS_VEH,'Inport',1);
sl(INT_ENG,  'Outport',1, Gain_RadRPM,'Inport',1);
sl(Gain_RadRPM,'Outport',1, WS_RPM2,'Inport',1);
sl(SS_TRANS, 'Outport',1, WS_GEAR,'Inport',1);

fprintf('  ✅ Top-level wiring complete\n');

%% ── Arrange & Save ───────────────────────────────────────────────────────────
try, Simulink.BlockDiagram.arrangeSystem(MDL,'FullLayout','true'); catch, end
save_system(MDL,[pwd '\' MDL '.slx']);
fprintf('\n✅  %s.slx saved.\n  Open: open_system(''%s'')\n\n',MDL,MDL);
end
