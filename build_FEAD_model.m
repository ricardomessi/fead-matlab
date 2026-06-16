%% build_FEAD_model.m  –  FEAD Belt Drive Simscape Model (Robust Port-Handle Build)
%  Uses get_param PortHandles to connect blocks — avoids all add_line string errors.
%  Each pulley is a self-contained subsystem. All signals logged to workspace.
%
%  Usage:  >> FEAD_params; build_FEAD_model
% ─────────────────────────────────────────────────────────────────────────────

function build_FEAD_model()

if ~exist('pulleys','var'), FEAD_params; end
pulleys    = evalin('base','pulleys');
belt       = evalin('base','belt');
load_table = evalin('base','load_table');
ten        = evalin('base','ten');

MDL   = 'FEAD_BeltDrive';
np    = numel(pulleys);
pnames= {'CRK','FAN','IDR','ALT','AC','TEN'};

fprintf('\n=== Building %s.slx ===\n',MDL);

%% ── Close & create fresh model ───────────────────────────────────────────────
if bdIsLoaded(MDL), close_system(MDL,0); end
new_system(MDL,'Model');
open_system(MDL);

set_param(MDL,...
    'StopTime',       '30',...
    'Solver',         'ode23t',...
    'SolverType',     'Variable-step',...
    'RelTol',         '1e-4',...
    'AbsTol',         '1e-6',...
    'MaxStep',        '0.01',...
    'SolverResetMethod','Fast');

%% ── Load required libraries ──────────────────────────────────────────────────
load_system('fl_lib');
load_system('nesl_utility');
load_system('simulink');

%% ══════════════════════════════════════════════════════════════════════════════
%  HELPER: add block and return its path
%% ══════════════════════════════════════════════════════════════════════════════
function blk = AB(lib,name,x,y,w,h)
    blk = [MDL '/' name];
    add_block(lib, blk, 'Position',[x y x+w y+h],'MakeNameUnique','off');
end

%% ══════════════════════════════════════════════════════════════════════════════
%  HELPER: connect two blocks using port handles (safe, version-independent)
%  type1/type2: 'LConn','RConn','Inport','Outport','Trigger' etc.
%  idx1/idx2: port index (1-based)
%% ══════════════════════════════════════════════════════════════════════════════
function safe_line(src_blk, src_type, src_idx, dst_blk, dst_type, dst_idx)
    try
        p1 = get_param(src_blk,'PortHandles');
        p2 = get_param(dst_blk,'PortHandles');
        h1 = p1.(src_type)(src_idx);
        h2 = p2.(dst_type)(dst_idx);
        add_line(MDL, h1, h2, 'autorouting','on');
    catch e
        fprintf('  [warn] line %s->%s: %s\n', src_blk, dst_blk, e.message);
    end
end

%% ══════════════════════════════════════════════════════════════════════════════
%  LAYOUT CONSTANTS
%% ══════════════════════════════════════════════════════════════════════════════
X0=30; Y0=50;         % top-left origin
DX=280; DY=220;       % column/row spacing

%% ══════════════════════════════════════════════════════════════════════════════
%  BLOCK 1: Solver Configuration
%% ══════════════════════════════════════════════════════════════════════════════
SC = AB('nesl_utility/Solver Configuration','SC',X0,Y0,160,40);
set_param(SC,'UseLocalSolver','off');

%% BLOCK 2: Global ground reference
GND = AB('fl_lib/Utilities/Mechanical/Rotational/Rotational Reference',...
    'GND',X0,Y0+60,90,40);

%% ══════════════════════════════════════════════════════════════════════════════
%  ENGINE SPEED SOURCE
%  RPM_Profile → rad/s → PS Converter → Velocity Source
%% ══════════════════════════════════════════════════════════════════════════════
% Repeating profile: idle→rated→idle over 30s
RPM_PROF = AB('simulink/Sources/Repeating Sequence','RPM_Profile',X0+10,Y0+150,140,40);
set_param(RPM_PROF,...
    'rep_seq_t', '[0  5   15   25   30]',...
    'rep_seq_y', '[600 1200 2000 1200 600]');

GAIN_RPM2RAD = AB('simulink/Math Operations/Gain','RPM2rad',X0+170,Y0+150,80,40);
set_param(GAIN_RPM2RAD,'Gain','2*pi/60');

CONV_W = AB('nesl_utility/Simulink-PS Converter','Conv_omega',X0+270,Y0+150,80,40);

ENG_SRC = AB('fl_lib/Mechanical/Rotational Elements/Ideal Angular Velocity Source',...
    'EngineVelSrc',X0+370,Y0+130,130,60);

% Connect RPM profile chain
safe_line(RPM_PROF,   'Outport',1, GAIN_RPM2RAD,'Inport',1);
safe_line(GAIN_RPM2RAD,'Outport',1, CONV_W,'Inport',1);
safe_line(CONV_W,'Outport',1, ENG_SRC,'Inport',1);   % PS signal → W port

% Ground → Engine ref port
safe_line(GND,'LConn',1, ENG_SRC,'LConn',2);

%% ── RPM Sensor + To Workspace ────────────────────────────────────────────────
RPM_SEN = AB('fl_lib/Mechanical/Rotational Elements/Ideal Rotational Motion Sensor',...
    'RPM_Sensor',X0+530,Y0+130,100,60);
CONV_RPM= AB('nesl_utility/PS-Simulink Converter','Conv_RPM',X0+650,Y0+150,80,40);
GAIN_R2R= AB('simulink/Math Operations/Gain','rad2rpm',X0+750,Y0+150,80,40);
set_param(GAIN_R2R,'Gain','60/(2*pi)');
WS_RPM  = AB('simulink/Sinks/To Workspace','WS_EngRPM',X0+850,Y0+150,100,40);
set_param(WS_RPM,'VariableName','eng_rpm','SaveFormat','Array');

safe_line(ENG_SRC,'LConn',1, RPM_SEN,'LConn',1);
safe_line(GND,    'LConn',1, RPM_SEN,'LConn',2);
safe_line(RPM_SEN,'Outport',1, CONV_RPM,'Inport',1);
safe_line(CONV_RPM,'Outport',1, GAIN_R2R,'Inport',1);
safe_line(GAIN_R2R,'Outport',1, WS_RPM,'Inport',1);

%% ══════════════════════════════════════════════════════════════════════════════
%  PULLEY SUBSYSTEMS (one per accessory)
%  Each subsystem contains:
%    - Rotational Inertia
%    - Speed sensor → torque LUT → torque source (load)
%    - Belt compliance (torsional spring-damper between adjacent pulleys)
%    - Torque + speed measurement → To Workspace
%% ══════════════════════════════════════════════════════════════════════════════

prev_conn_blk = 'EngineVelSrc';   % first pulley connects to engine output

for k = 1:np
    pn  = pnames{k};
    p   = pulleys(k);
    xi  = X0 + (k-1)*DX;
    yi  = Y0 + 340;

    fprintf('  Building pulley subsystem: %s\n', pn);

    %% ── Inertia ────────────────────────────────────────────────────────────
    Iz_k = max(p.Iz, 1e-6);
    IZ = AB('fl_lib/Mechanical/Rotational Elements/Inertia',...
        [pn '_Iz'], xi,yi,110,50);
    set_param(IZ,'inertia',sprintf('%.6f',Iz_k));

    % Connect previous block's LConn1 to this inertia LConn1
    safe_line(prev_conn_blk,'LConn',1, IZ,'LConn',1);
    prev_conn_blk = IZ;

    % Ground reference for inertia
    safe_line(GND,'LConn',1, IZ,'LConn',2);

    %% ── Belt compliance spring between this and next pulley ─────────────────
    kn      = mod(k,np)+1;
    span_m  = hypot(pulleys(kn).x-p.x, pulleys(kn).y-p.y)/1000;
    k_axial = belt.EA / max(span_m,0.01);
    k_tors  = k_axial * (p.r/1000)^2;   % Nm/rad
    c_tors  = max(k_tors*0.005, 0.01);  % Ns·m/rad (1% damping)

    SPR = AB('fl_lib/Mechanical/Rotational Elements/Torsional Spring',...
        [pn '_Spr'], xi+120,yi,100,50);
    set_param(SPR,'spring_rate',sprintf('%.2f',k_tors));

    DAM = AB('fl_lib/Mechanical/Rotational Elements/Rotational Damper',...
        [pn '_Dam'], xi+120,yi+60,100,50);
    set_param(DAM,'D',sprintf('%.4f',c_tors));

    safe_line(IZ, 'LConn',1, SPR,'LConn',1);
    safe_line(GND,'LConn',1, SPR,'LConn',2);
    safe_line(IZ, 'LConn',1, DAM,'LConn',1);
    safe_line(GND,'LConn',1, DAM,'LConn',2);

    %% ── Speed sensor ─────────────────────────────────────────────────────────
    SSEN = AB('fl_lib/Mechanical/Rotational Elements/Ideal Rotational Motion Sensor',...
        [pn '_SpeedSen'], xi,yi+140,110,60);
    safe_line(IZ,  'LConn',1, SSEN,'LConn',1);
    safe_line(GND, 'LConn',1, SSEN,'LConn',2);

    %% ── PS→Simulink → Torque LUT ─────────────────────────────────────────────
    CSPD = AB('nesl_utility/PS-Simulink Converter',...
        [pn '_CvSpd'], xi+120,yi+150,80,40);
    safe_line(SSEN,'Outport',1, CSPD,'Inport',1);

    % Torque LUT: rad/s → Nm
    rpm_pts = load_table.rpm;
    P_arr   = load_table.(pn);
    T_arr   = P_arr*1000 ./ max(rpm_pts*2*pi/60, 0.1);

    LUT = AB('simulink/Lookup Tables/1-D Lookup Table',...
        [pn '_TLut'], xi+210,yi+150,110,40);
    set_param(LUT,...
        'BreakpointsForDimension1', mat2str(rpm_pts*2*pi/60, 6),...
        'Table',                    mat2str(T_arr, 6),...
        'ExtrapMethod',             'Clip');
    safe_line(CSPD,'Outport',1, LUT,'Inport',1);

    %% ── Simulink→PS → Torque Source ──────────────────────────────────────────
    CTLUT = AB('nesl_utility/Simulink-PS Converter',...
        [pn '_CvTlut'], xi+330,yi+150,80,40);
    safe_line(LUT,'Outport',1, CTLUT,'Inport',1);

    TSRC = AB('fl_lib/Mechanical/Rotational Elements/Ideal Torque Source',...
        [pn '_TLoad'], xi+420,yi+140,110,60);
    safe_line(CTLUT,'Outport',1, TSRC,'Inport',1);
    safe_line(IZ,   'LConn',1,  TSRC,'LConn',1);
    safe_line(GND,  'LConn',1,  TSRC,'LConn',2);

    %% ── Torque sensor + To Workspace ─────────────────────────────────────────
    TSEN = AB('fl_lib/Mechanical/Rotational Elements/Torque Sensor',...
        [pn '_TorqSen'], xi+540,yi+140,110,60);
    safe_line(IZ,  'LConn',1, TSEN,'LConn',1);
    safe_line(GND, 'LConn',1, TSEN,'LConn',2);

    CT = AB('nesl_utility/PS-Simulink Converter',[pn '_CvTorq'],xi+660,yi+150,80,40);
    safe_line(TSEN,'Outport',1, CT,'Inport',1);

    WS_T = AB('simulink/Sinks/To Workspace',[pn '_Tws'],xi+750,yi+150,100,40);
    set_param(WS_T,'VariableName',[pn '_torque'],'SaveFormat','Array');
    safe_line(CT,'Outport',1, WS_T,'Inport',1);

    CW = AB('nesl_utility/PS-Simulink Converter',[pn '_CvOmeg'],xi+660,yi+200,80,40);
    safe_line(SSEN,'Outport',1, CW,'Inport',1);

    WS_W = AB('simulink/Sinks/To Workspace',[pn '_Wws'],xi+750,yi+200,100,40);
    set_param(WS_W,'VariableName',[pn '_omega'],'SaveFormat','Array');
    safe_line(CW,'Outport',1, WS_W,'Inport',1);
end

%% ══════════════════════════════════════════════════════════════════════════════
%  TENSIONER SUBSYSTEM — angular spring + preload
%% ══════════════════════════════════════════════════════════════════════════════
xi_ten = X0 + (np)*DX;
yi_ten = Y0 + 340;

TEN_IZ = AB('fl_lib/Mechanical/Rotational Elements/Inertia',...
    'TEN_ArmIz',xi_ten,yi_ten,110,50);
set_param(TEN_ArmIz,'inertia',sprintf('%.5f',ten.Iz));

TEN_SPR = AB('fl_lib/Mechanical/Rotational Elements/Torsional Spring',...
    'TEN_ArmSpr',xi_ten+120,yi_ten,100,50);
set_param(TEN_SPR,'spring_rate',sprintf('%.4f',ten.k_spring));

TEN_PRELOAD = AB('simulink/Sources/Constant','TEN_Preload',xi_ten+120,yi_ten-70,80,40);
set_param(TEN_PRELOAD,'Value',sprintf('%.4f',ten.preload));

CTEN = AB('nesl_utility/Simulink-PS Converter','Conv_TenPre',xi_ten+210,yi_ten-70,80,40);
safe_line(TEN_PRELOAD,'Outport',1, CTEN,'Inport',1);

TEN_TORQ = AB('fl_lib/Mechanical/Rotational Elements/Ideal Torque Source',...
    'TEN_TorqSrc',xi_ten+300,yi_ten-60,110,60);
safe_line(CTEN,'Outport',1, TEN_TORQ,'Inport',1);

safe_line(GND,'LConn',1, TEN_IZ,'LConn',2);
safe_line(GND,'LConn',1, TEN_SPR,'LConn',2);
safe_line(GND,'LConn',1, TEN_TORQ,'LConn',2);
safe_line(TEN_IZ,'LConn',1, TEN_SPR,'LConn',1);
safe_line(TEN_IZ,'LConn',1, TEN_TORQ,'LConn',1);

%% ── Solver → GND connection (required) ──────────────────────────────────────
safe_line(SC,'RConn',1, GND,'LConn',1);

%% ── Arrange layout ───────────────────────────────────────────────────────────
try
    Simulink.BlockDiagram.arrangeSystem(MDL,'FullLayout','true');
catch
end

save_system(MDL,[pwd '\' MDL '.slx']);
fprintf('\n✅  %s.slx saved.\n',MDL);
fprintf('   Open: open_system(''%s'')\n\n',MDL);
end
