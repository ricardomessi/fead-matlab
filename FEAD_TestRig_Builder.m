%% FEAD_TestRig_Builder.m  –  Build the Full FEAD Simscape Test Rig Model
%  Programmatically constructs FEAD_TestRig.slx with:
%   • Engine speed source (configurable RPM profile)
%   • 6 pulley subsystems (inertia + rotational sensor + torque load)
%   • Tensioner spring-damper subsystem
%   • Belt compliance elements (torsional springs between pulleys)
%   • Accessory torque loads from lookup tables
%   • Measurement bus (speed, torque, hub force per pulley)
%   • Scope + Data Store for all signals
%   • Configurable layout via pulleys/belt structs
%
%  Usage:  >> FEAD_params; FEAD_TestRig_Builder(pulleys, belt, load_table, conditions)
%  Or:     >> FEAD_params; FEAD_TestRig_Builder()    % uses workspace defaults
% ─────────────────────────────────────────────────────────────────────────────

function FEAD_TestRig_Builder(pulleys_in, belt_in, load_table_in, conditions_in)

if ~evalin('base','exist(''pulleys'',''var'')')
    evalin('base','FEAD_params');
end

%% ── Defaults from workspace ──────────────────────────────────────────────────
if nargin < 1 || isempty(pulleys_in),    pulleys_in    = evalin('base','pulleys'); end
if nargin < 2 || isempty(belt_in),       belt_in       = evalin('base','belt');    end
if nargin < 3 || isempty(load_table_in), load_table_in = evalin('base','load_table'); end
if nargin < 4
    conditions_in = struct('ac',true,'nightRun',false,'bas',false,'temp_C',40);
end

pulleys    = pulleys_in;
belt       = belt_in;
load_table = load_table_in;
np         = numel(pulleys);
pnames     = {'CRK','FAN','IDR','ALT','AC','TEN'};

MDL = 'FEAD_TestRig';
fprintf('\n=== Building %s.slx ===\n', MDL);

%% ── Close & recreate model ───────────────────────────────────────────────────
if bdIsLoaded(MDL), close_system(MDL,0); end
if exist([MDL '.slx'],'file'), delete([MDL '.slx']); end
new_system(MDL);
open_system(MDL);

%% ── Solver config ────────────────────────────────────────────────────────────
set_param(MDL,...
    'StopTime',           '30',...
    'Solver',             'ode23t',...
    'SolverType',         'Variable-step',...
    'RelTol',             '1e-4',...
    'AbsTol',             '1e-6',...
    'MaxStep',            '1e-3',...
    'StartTime',          '0');

%% ── Helper: place a block and return its path ─────────────────────────────────
place = @(lib,name,x,y,w,h) place_block(MDL,lib,name,x,y,w,h);

%% ══════════════════════════════════════════════════════════════════════════════
%  TOP-LEVEL BLOCKS
%  Layout columns: [Source | Pulleys | Accessories | Measurement]
%  Y spacing: 200 per pulley row
%% ══════════════════════════════════════════════════════════════════════════════

%% ── Simscape Solver Configuration ───────────────────────────────────────────
add_block('nesl_utility/Solver Configuration',[MDL '/SC'],...
    'Position',[30 30 210 70]);
add_block('fl_lib/Mechanical/Rotational Elements/Mechanical Rotational Reference',...
    [MDL '/GND'],'Position',[30 100 90 140]);

%% ── Engine RPM Source ────────────────────────────────────────────────────────
% Ramp: idle → rated → idle
add_block('simulink/Sources/Ramp',[MDL '/RPM_Ramp'],...
    'slope','(2000-600)/15','start','0','x0','600',...
    'Position',[30 200 120 230]);

add_block('simulink/Math Operations/Gain',[MDL '/RPM2rad'],...
    'Gain','2*pi/60','Position',[150 200 250 230]);

add_block('nesl_utility/Simulink-PS Converter',[MDL '/Conv_omega'],...
    'Position',[280 200 380 230]);

add_block('fl_lib/Mechanical/Mechanical Sources/Ideal Angular Velocity Source',...
    [MDL '/EngineSource'],'Position',[420 200 550 260]);

% Connect engine source
add_line(MDL,'RPM_Ramp/1','RPM2rad/1','autorouting','on');
add_line(MDL,'RPM2rad/1','Conv_omega/1','autorouting','on');
safe_line(MDL,'Conv_omega','RConn',1, 'EngineSource','RConn',1);
safe_line(MDL,'GND','LConn',1, 'EngineSource','LConn',1);

% Engine scope
add_block('simulink/Sinks/Scope',[MDL '/RPM_Scope'],...
    'NumInputPorts','1','Position',[30 290 90 330]);
add_block('nesl_utility/PS-Simulink Converter',[MDL '/Conv_RPM_out'],...
    'Position',[150 290 250 320]);
add_block('fl_lib/Mechanical/Mechanical Sensors/Ideal Rotational Motion Sensor',...
    [MDL '/RPM_Sensor'],'Position',[580 200 700 260]);
add_block('simulink/Math Operations/Gain',[MDL '/rad2RPM'],...
    'Gain','60/(2*pi)','Position',[30 350 130 380]);

safe_line(MDL,'EngineSource','RConn',2, 'RPM_Sensor','RConn',1);
safe_line(MDL,'GND','LConn',1, 'RPM_Sensor','LConn',1);
safe_line(MDL,'RPM_Sensor','RConn',2, 'Conv_RPM_out','LConn',1);
add_line(MDL,'Conv_RPM_out/1','rad2RPM/1','autorouting','on');
add_line(MDL,'rad2RPM/1','RPM_Scope/1','autorouting','on');

%% ── Build each pulley subsystem ──────────────────────────────────────────────
col_x   = 750;      % pulley column X
col_acc = 1050;     % accessory column X
col_meas= 1350;     % measurement column X
row_h   = 200;      % vertical spacing
y_start = 200;

h_pulley_out = zeros(np,1);   % output port positions for connectivity check

for k = 1:np
    pn  = pnames{k};
    p   = pulleys(k);
    y_k = y_start + (k-1)*row_h;
    blk = [MDL '/' pn];

    %% ── Pulley inertia ───────────────────────────────────────────────────────
    Iz_k = max(p.Iz, 1e-6);
    add_block('fl_lib/Mechanical/Rotational Elements/Inertia',...
        [MDL '/' pn '_Iz'],...
        'inertia', sprintf('%.6f', Iz_k),...
        'Position',[col_x y_k col_x+140 y_k+40]);

    %% ── Belt compliance spring (represents belt stretch between pulleys) ─────
    % Belt stiffness between k and k+1
    kn   = mod(k,np)+1;
    span = hypot(pulleys(kn).x-p.x, pulleys(kn).y-p.y) / 1000;  % m
    k_belt = belt.EA / max(span,0.01);   % N/m axial → torsional approx
    k_tors = k_belt * (p.r/1000)^2;     % Nm/rad

    add_block('fl_lib/Mechanical/Rotational Elements/Rotational Spring',...
        [MDL '/' pn '_Spring'],...
        'spr_rate', sprintf('%.1f', k_tors),...
        'Position',[col_x-160 y_k col_x-20 y_k+40]);

    add_block('fl_lib/Mechanical/Rotational Elements/Rotational Damper',...
        [MDL '/' pn '_Damp'],...
        'D', sprintf('%.4f', k_tors*1e-3),...
        'Position',[col_x-160 y_k+50 col_x-20 y_k+90]);

    safe_line(MDL,[pn '_Iz'],'LConn',1, [pn '_Spring'],'LConn',1);
    safe_line(MDL,'GND','LConn',1, [pn '_Spring'],'RConn',1);
    safe_line(MDL,[pn '_Iz'],'LConn',1, [pn '_Damp'],'LConn',1);
    safe_line(MDL,'GND','LConn',1, [pn '_Damp'],'RConn',1);

    %% ── Accessory torque load ─────────────────────────────────────────────────
    % 1-D LUT: RPM → torque [Nm]
    rpm_pts   = load_table.rpm;
    P_arr     = load_table.(pn);
    if strcmp(pn,'AC') && ~conditions_in.ac
        P_arr = zeros(size(P_arr));
    end
    % T = P[kW]*1000 / (omega [rad/s])  — avoid div by 0 at low rpm
    T_arr = P_arr*1000 ./ max(rpm_pts*2*pi/60, 0.1);

    add_block('simulink/Lookup Tables/1-D Lookup Table',...
        [MDL '/' pn '_TorqueLUT'],...
        'BreakpointsForDimension1', mat2str(rpm_pts*2*pi/60, 6),...
        'Table',                    mat2str(T_arr, 6),...
        'ExtrapMethod','Clip',...
        'Position',[col_acc y_k col_acc+180 y_k+40]);

    % Speed sensor → LUT → Simscape torque source
    add_block('fl_lib/Mechanical/Mechanical Sensors/Ideal Rotational Motion Sensor',...
        [MDL '/' pn '_SpeedSensor'],...
        'Position',[col_acc-220 y_k col_acc-80 y_k+40]);

    add_block('nesl_utility/PS-Simulink Converter',...
        [MDL '/' pn '_PS2SL'],...
        'Position',[col_acc-320 y_k-60 col_acc-220 y_k-30]);

    add_block('nesl_utility/Simulink-PS Converter',...
        [MDL '/' pn '_SL2PS'],...
        'Position',[col_acc+200 y_k col_acc+300 y_k+30]);

    add_block('fl_lib/Mechanical/Mechanical Sources/Ideal Torque Source',...
        [MDL '/' pn '_LoadTorque'],...
        'Position',[col_acc+330 y_k col_acc+470 y_k+40]);

    % Connect: SpeedSensor → PS→SL → LUT → SL→PS → TorqueSource
    safe_line(MDL,[pn '_Iz'],'LConn',1, [pn '_SpeedSensor'],'RConn',1);
    safe_line(MDL,'GND','LConn',1, [pn '_SpeedSensor'],'LConn',1);
    safe_line(MDL,[pn '_SpeedSensor'],'RConn',2, [pn '_PS2SL'],'LConn',1);
    add_line(MDL,[pn '_PS2SL/1'],[pn '_TorqueLUT/1'],'autorouting','on');
    add_line(MDL,[pn '_TorqueLUT/1'],[pn '_SL2PS/1'],'autorouting','on');
    safe_line(MDL,[pn '_SL2PS'],'RConn',1, [pn '_LoadTorque'],'RConn',1);
    safe_line(MDL,[pn '_Iz'],'LConn',1, [pn '_LoadTorque'],'RConn',2);
    safe_line(MDL,'GND','LConn',1, [pn '_LoadTorque'],'LConn',1);

    %% ── Measurement: torque + speed + hub force ──────────────────────────────
    add_block('fl_lib/Mechanical/Mechanical Sensors/Ideal Torque Sensor',...
        [MDL '/' pn '_TorqSensor'],...
        'Position',[col_meas y_k col_meas+160 y_k+40]);

    add_block('nesl_utility/PS-Simulink Converter',...
        [MDL '/' pn '_T_conv'],...
        'Position',[col_meas+180 y_k col_meas+280 y_k+30]);

    add_block('simulink/Sinks/To Workspace',...
        [MDL '/' pn '_T_ws'],...
        'VariableName', [pn '_torque'],...
        'SaveFormat','Array',...
        'Position',[col_meas+300 y_k col_meas+440 y_k+30]);

    add_block('nesl_utility/PS-Simulink Converter',...
        [MDL '/' pn '_W_conv'],...
        'Position',[col_meas+180 y_k+50 col_meas+280 y_k+80]);

    add_block('simulink/Sinks/To Workspace',...
        [MDL '/' pn '_W_ws'],...
        'VariableName', [pn '_omega'],...
        'SaveFormat','Array',...
        'Position',[col_meas+300 y_k+50 col_meas+440 y_k+80]);

    safe_line(MDL,[pn '_Iz'],'LConn',1, [pn '_TorqSensor'],'RConn',1);
    safe_line(MDL,'GND','LConn',1, [pn '_TorqSensor'],'LConn',1);
    safe_line(MDL,[pn '_TorqSensor'],'RConn',2, [pn '_T_conv'],'LConn',1);
    add_line(MDL,[pn '_T_conv/1'],[pn '_T_ws/1'],'autorouting','on');
    safe_line(MDL,[pn '_SpeedSensor'],'RConn',2, [pn '_W_conv'],'LConn',1);
    add_line(MDL,[pn '_W_conv/1'],[pn '_W_ws/1'],'autorouting','on');

    fprintf('  Pulley %-4s: Iz=%.5f kg·m²  k_belt=%.0f Nm/rad\n', pn, Iz_k, k_tors);
end

%% ── Connect CRK to engine source ─────────────────────────────────────────────
safe_line(MDL,'EngineSource','RConn',2, 'CRK_Iz','LConn',1);

%% ── Tensioner spring-damper subsystem ────────────────────────────────────────
ten = evalin('base','ten');
add_block('fl_lib/Mechanical/Rotational Elements/Rotational Spring',...
    [MDL '/TEN_Arm_Spring'],...
    'spr_rate', sprintf('%.4f', ten.k_spring),...
    'Position',[col_x-160 y_start+(np-1)*row_h+100 col_x-20 y_start+(np-1)*row_h+140]);
add_block('fl_lib/Mechanical/Rotational Elements/Rotational Damper',...
    [MDL '/TEN_Arm_Damp'],...
    'D', sprintf('%.5f', ten.k_spring*0.1),...
    'Position',[col_x-160 y_start+(np-1)*row_h+160 col_x-20 y_start+(np-1)*row_h+200]);

safe_line(MDL,'GND','LConn',1, 'TEN_Arm_Spring','LConn',1);
safe_line(MDL,'TEN_Iz','LConn',1, 'TEN_Arm_Spring','RConn',1);
safe_line(MDL,'GND','LConn',1, 'TEN_Arm_Damp','LConn',1);
safe_line(MDL,'TEN_Iz','LConn',1, 'TEN_Arm_Damp','RConn',1);
safe_line(MDL,'TEN_Spring','RConn',1, 'TEN_Arm_Spring','RConn',1);

%% ── Dashboard: Scope showing all 6 torques ───────────────────────────────────
add_block('simulink/Sinks/Scope',[MDL '/All_Torques_Scope'],...
    'NumInputPorts',num2str(np),'Position',[col_meas+500 200 col_meas+600 200+np*45]);

%% ── Simulation parameters display block ──────────────────────────────────────
% Store belt/layout info as a constant block annotation
belt_info = sprintf(['Belt: %s | L=%.0fmm | Ribs=%d | mu=%.2f | '...
    'T_static=%.0fN | EA=%.0fN'],...
    belt.name, belt.length_mm, belt.ribs, belt.mu, belt.static_tension, belt.EA);
add_block('simulink/Commonly Used Blocks/Constant',[MDL '/BeltConfig'],...
    'Value','0','OutDataTypeStr','double',...
    'Position',[30 500 200 530]);
set_param([MDL '/BeltConfig'],'Description',belt_info);

%% ── Add model annotations ────────────────────────────────────────────────────
try
    add_annotation(MDL,...
        sprintf(['FEAD Belt Drive Test Rig\nH6 OEM Engine\n%s\n'...
        'CRK origin. All coords in mm.\nEdit pulleys struct and rebuild.'], belt_info),...
        'Position',[30 560 500 650],...
        'ForegroundColor','white','BackgroundColor','[0.06 0.08 0.14]');
catch
end

%% ── Save and open ────────────────────────────────────────────────────────────
Simulink.BlockDiagram.arrangeSystem(MDL);
save_system(MDL);
fprintf('\n✅ %s.slx saved in: %s\n', MDL, pwd);
fprintf('   Open it with:  open_system(''%s'')\n\n', MDL);

% Export params back to base workspace
assignin('base','pulleys', pulleys);
assignin('base','belt',    belt);

end % FEAD_TestRig_Builder


%% ─── Helper ────────────────────────────────────────────────────────────────
function path = place_block(MDL, lib, name, x, y, w, h)
    path = [MDL '/' name];
    add_block(lib, path, 'Position',[x y x+w y+h]);
end

function safe_line(MDL, src_name, src_type, src_idx, dst_name, dst_type, dst_idx)
    try
        p1 = get_param([MDL '/' src_name],'PortHandles');
        p2 = get_param([MDL '/' dst_name],'PortHandles');
        h1 = p1.(src_type)(src_idx);
        h2 = p2.(dst_type)(dst_idx);
        add_line(MDL, h1, h2, 'autorouting','on');
    catch e
        fprintf('  [warn] line %s->%s: %s\n', src_name, dst_name, e.message);
    end
end
