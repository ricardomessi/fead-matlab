%% build_FEAD_model.m  –  Programmatically builds the FEAD Belt Drive
%  Simscape/Simulink test rig for the Ashok Leyland H6 engine.
%
%  REQUIRES:  Simulink, Simscape (Foundation Library)
%  OPTIONAL:  Simscape Driveline (falls back to custom belt blocks)
%
%  Usage:
%    >> FEAD_params          % load all parameters first
%    >> build_FEAD_model     % creates and opens FEAD_BeltDrive.slx
% ─────────────────────────────────────────────────────────────────────────────

if ~exist('pulleys','var')
    FEAD_params;
end

mdl = 'FEAD_BeltDrive';

%% Close & delete old model if open
if bdIsLoaded(mdl), close_system(mdl,0); end
if exist([mdl '.slx'],'file'), delete([mdl '.slx']); end

%% Create new model
new_system(mdl);
open_system(mdl);
set_param(mdl,'SolverType','Fixed-step','FixedStep','1e-4',...
    'Solver','ode23t','StopTime',num2str(sim.T_end_fead),...
    'SaveTime','on','SaveOutput','on');

%% Helper – add block with position and label
pos = @(col,row) [col*220 row*140 col*220+180 row*140+60];

%% ─────────────────────────────────────────────────────────────────────────
%  SUBSYSTEM LAYOUT  (blocks placed on canvas)
% ─────────────────────────────────────────────────────────────────────────

% ── 1. Simscape Solver Configuration ──────────────────────────────────────
add_block('nesl_utility/Solver Configuration',[mdl '/SolverConfig']);
set_param([mdl '/SolverConfig'],'Position',[20 20 200 70]);
set_param([mdl '/SolverConfig'],'UseLocalSolver','off',...
    'DoFixedCost','off','MaxNonlinIter','20');

% ── 2. Mechanical Ground (reference frame) ────────────────────────────────
add_block('fl_lib/Mechanical/Rotational Elements/Mechanical Rotational Reference',...
    [mdl '/GND']);
set_param([mdl '/GND'],'Position',[20 120 80 160]);

%% ─────────────────────────────────────────────────────────────────────────
%  BUILD EACH PULLEY SUBSYSTEM
%  Each pulley has:
%    • Rotational Inertia
%    • Ideal Torque Source  (loads: fan, alt, AC etc.)
%    • Ideal Rotational Motion Sensor  (output ω, θ)
%    • PS-Simulink converter (for scope/workspace output)
% ─────────────────────────────────────────────────────────────────────────

pnames = {'CRK','FAN','IDR','ALT','AC','TEN'};
col_offset = [1 2 3 4 5 6];

for k = 1:N_pulleys
    pn   = pnames{k};
    col  = col_offset(k);
    Iz_k = pulleys(k).Iz;
    r_k  = pulleys(k).r/1000;  % m

    % ── Inertia block ──
    blk_I = [mdl '/I_' pn];
    add_block('fl_lib/Mechanical/Rotational Elements/Inertia', blk_I);
    set_param(blk_I,'Position',pos(col,1));
    set_param(blk_I,'inertia', num2str(Iz_k));

    % ── Torque source (accessory load) ──
    blk_T = [mdl '/Tload_' pn];
    add_block('fl_lib/Mechanical/Rotational Elements/Ideal Torque Source', blk_T);
    set_param(blk_T,'Position',pos(col,2));

    % ── Speed sensor ──
    blk_S = [mdl '/Spd_' pn];
    add_block('fl_lib/Mechanical/Rotational Elements/Ideal Rotational Motion Sensor', blk_S);
    set_param(blk_S,'Position',pos(col,3));

    % ── PS→Simulink (speed output) ──
    blk_C = [mdl '/Conv_' pn];
    add_block('nesl_utility/PS-Simulink Converter', blk_C);
    set_param(blk_C,'Position',pos(col,4));

    % ── Torque load source (Simulink→PS) for accessory demand ──
    blk_TI = [mdl '/TorqIn_' pn];
    add_block('nesl_utility/Simulink-PS Converter', blk_TI);
    set_param(blk_TI,'Position',pos(col,5));

    % ── Lookup table: RPM → Power → Torque  ──
    blk_LUT = [mdl '/LUT_' pn];
    add_block('simulink/Lookup Tables/1-D Lookup Table', blk_LUT);
    set_param(blk_LUT,'Position',pos(col,6));
    rpm_rad = load_table.rpm * 2*pi/60;  % rad/s
    % torque = P*1000 / omega
    P_arr = load_table.(pn);
    T_arr = P_arr*1000 ./ rpm_rad;
    set_param(blk_LUT,'BreakpointsForDimension1', mat2str(rpm_rad,6));
    set_param(blk_LUT,'Table', mat2str(T_arr,6));
    set_param(blk_LUT,'ExtrapMethod','Clip');

    % ── Connections within pulley block group ──
    % Sensor R → Inertia R
    add_line(mdl,[blk_S '/R'],[blk_I '/R'],'autorouting','on');
    % Sensor C → Torque source R
    add_line(mdl,[blk_S '/C'],[blk_T '/R'],'autorouting','on');
    % Torque source C → GND
    add_line(mdl,[blk_T '/C'],[mdl '/GND/1'],'autorouting','on');
    % Sensor omega → PS converter
    add_line(mdl,[blk_S '/W'],[blk_C '/I'],'autorouting','on');
    % PS converter output → LUT input (speed feedback for torque lookup)
    add_line(mdl,[blk_C '/1'],[blk_LUT '/1'],'autorouting','on');
    % LUT → Simulink-PS → Torque source signal input
    add_line(mdl,[blk_LUT '/1'],[blk_TI '/I'],'autorouting','on');
    add_line(mdl,[blk_TI '/O'],[blk_T '/T'],'autorouting','on');
end

%% ─────────────────────────────────────────────────────────────────────────
%  BELT DYNAMICS SUBSYSTEM
%  Belt modelled as a series of tension elements (one per span).
%  Span k connects pulley ORDER(k) → ORDER(k+1).
%  Tension in each span = T_static + effective tension component.
%  Implemented as a Simulink subsystem computing T_tight, T_slack per span.
% ─────────────────────────────────────────────────────────────────────────

build_belt_subsystem(mdl, pulleys, belt, ten_idx, ten_pos, load_table, sim);

%% ─────────────────────────────────────────────────────────────────────────
%  CRANKSHAFT DRIVE INPUT
%  Speed input from Engine RPM profile (From Workspace or Step)
% ─────────────────────────────────────────────────────────────────────────

% Engine RPM signal source (From Workspace – drive cycle)
blk_rpm = [mdl '/EngineRPM'];
add_block('simulink/Sources/From Workspace', blk_rpm);
set_param(blk_rpm,'Position',[20 700 200 740]);
t_rpm_data = timeseries(dc_rpm, dc_t);
assignin('base','t_rpm_data', t_rpm_data);
set_param(blk_rpm,'VariableName','t_rpm_data');
set_param(blk_rpm,'Interpolate','on','ZeroOrderHold','off');

% RPM to rad/s
blk_r2w = [mdl '/RPM2rads'];
add_block('simulink/Math Operations/Gain', blk_r2w);
set_param(blk_r2w,'Position',[220 700 360 740]);
set_param(blk_r2w,'Gain','2*pi/60');
add_line(mdl,[blk_rpm '/1'],[blk_r2w '/1'],'autorouting','on');

% Ideal Rotational Velocity Source for CRK
blk_vs = [mdl '/CRK_DriveSource'];
add_block('fl_lib/Mechanical/Rotational Elements/Ideal Angular Velocity Source', blk_vs);
set_param(blk_vs,'Position',[380 700 540 740]);
add_line(mdl,[blk_r2w '/1'],[blk_vs '/V'],'autorouting','on');
% connect velocity source to CRK inertia
add_line(mdl,[blk_vs '/R'],[mdl '/I_CRK/R'],'autorouting','on');
add_line(mdl,[blk_vs '/C'],[mdl '/GND/1'],'autorouting','on');

%% ─────────────────────────────────────────────────────────────────────────
%  OUTPUT SCOPES
%  • Hub Load vs time for each pulley
%  • Belt tension in each span
%  • Slip Safety Factor
%  • Bearing Life estimate
% ─────────────────────────────────────────────────────────────────────────

blk_scope1 = [mdl '/Scope_HubLoads'];
add_block('simulink/Sinks/Scope', blk_scope1);
set_param(blk_scope1,'Position',[1700 100 1800 200]);
set_param(blk_scope1,'NumInputPorts','6',...
    'AxesTitles',strjoin({'CRK','FAN','IDR','ALT','AC','TEN'},'\n'));

blk_scope2 = [mdl '/Scope_Tensions'];
add_block('simulink/Sinks/Scope', blk_scope2);
set_param(blk_scope2,'Position',[1700 260 1800 360]);
set_param(blk_scope2,'NumInputPorts','6');

blk_to_ws = [mdl '/ToWorkspace_Results'];
add_block('simulink/Sinks/To Workspace', blk_to_ws);
set_param(blk_to_ws,'Position',[1700 420 1850 460]);
set_param(blk_to_ws,'VariableName','fead_results',...
    'MaxDataPoints','inf','SaveFormat','Array');

%% ─────────────────────────────────────────────────────────────────────────
%  TENSIONER SUBSYSTEM  (spring-damper + arm rotation)
% ─────────────────────────────────────────────────────────────────────────
build_tensioner_subsystem(mdl, ten, sim);

%% Save & open
Simulink.BlockDiagram.arrangeSystem(mdl);
save_system(mdl);
fprintf('\nFEAD_BeltDrive.slx created and saved.\n');
fprintf('Run:  sim(''%s'')  to simulate.\n', mdl);
open_system(mdl);

%% ─────────────────────────────────────────────────────────────────────────
%  LOCAL HELPER FUNCTIONS
% ─────────────────────────────────────────────────────────────────────────

function build_belt_subsystem(mdl, pulleys, belt, ten_idx, ten_pos, load_table, sim)
%BUILD_BELT_SUBSYSTEM  Creates belt tension calculation subsystem.
%  Computes tight/slack tensions for each span using the Capstan equation
%  and feeds torque reactions back to pulley blocks.

    ss_name = [mdl '/BeltDynamics'];
    add_block('simulink/Ports & Subsystems/Subsystem', ss_name);
    set_param(ss_name,'Position',[880 100 1080 700]);

    open_system(ss_name);
    ss = [mdl '/BeltDynamics'];

    % Delete default inport/outport
    delete_block([ss '/In1']);
    delete_block([ss '/Out1']);

    % ── Inputs: ω of each pulley (6 inports) ──
    pnames = {'CRK','FAN','IDR','ALT','AC','TEN'};
    for k = 1:6
        p = [ss '/omega_' pnames{k}];
        add_block('simulink/Sources/In1', p);
        set_param(p,'Position',[30 (k-1)*80+30 80 (k-1)*80+60],'Port',num2str(k));
    end

    % Belt velocity  v = omega_CRK * r_CRK
    r_CRK = pulleys(1).r / 1000;  % m
    blk_v = [ss '/BeltVelocity'];
    add_block('simulink/Math Operations/Gain', blk_v);
    set_param(blk_v,'Position',[120 30 250 60]);
    set_param(blk_v,'Gain', num2str(r_CRK));
    add_line(ss,['omega_CRK/1'],[strrep(blk_v,[ss '/'],'') '/1'],'autorouting','on');

    % Centrifugal tension  Tc = m_b * v^2
    blk_v2 = [ss '/v_squared'];
    add_block('simulink/Math Operations/Math Function', blk_v2);
    set_param(blk_v2,'Position',[300 30 400 60],'Operator','square');
    blk_Tc = [ss '/T_centrifugal'];
    add_block('simulink/Math Operations/Gain', blk_Tc);
    set_param(blk_Tc,'Position',[430 30 550 60]);
    set_param(blk_Tc,'Gain', num2str(belt.lin_mass));
    add_line(ss,'BeltVelocity/1','v_squared/1','autorouting','on');
    add_line(ss,'v_squared/1','T_centrifugal/1','autorouting','on');

    % Span tensions for each of 6 spans
    % T_eff(k) = P(k)*1000 / v  (from load table already in pulley LUT)
    % T_tight  = T_static + T_eff/2
    % T_slack  = T_static - T_eff/2  (floored at 0)
    T_static = belt.static_tension;
    wrap_deg = [166.5 127.6 108.4 145.1 105.7 76.4];
    mu = belt.mu;

    for k = 1:6
        pn = pnames{k};
        % Power lookup  P = f(omega)
        blk_lut = [ss '/Plut_' pn];
        add_block('simulink/Lookup Tables/1-D Lookup Table', blk_lut);
        rpm_rad = load_table.rpm * 2*pi/60;
        P_arr   = load_table.(pn);
        set_param(blk_lut,'BreakpointsForDimension1',mat2str(rpm_rad,6),...
            'Table',mat2str(P_arr,6),'ExtrapMethod','Clip');
        set_param(blk_lut,'Position',[120 (k-1)*80+100 250 (k-1)*80+130]);
        add_line(ss,['omega_' pn '/1'],['Plut_' pn '/1'],'autorouting','on');

        % T_eff = P*1000/v  (divide block + multiply 1000)
        blk_te = [ss '/Teff_' pn];
        add_block('simulink/Math Operations/Divide', blk_te);
        set_param(blk_te,'Position',[300 (k-1)*80+100 400 (k-1)*80+130]);

        blk_k1 = [ss '/k1000_' pn];
        add_block('simulink/Math Operations/Gain', blk_k1);
        set_param(blk_k1,'Gain','1000','Position',[270 (k-1)*80+100 298 (k-1)*80+130]);
        add_line(ss,['Plut_' pn '/1'],['k1000_' pn '/1'],'autorouting','on');
        add_line(ss,['k1000_' pn '/1'],['Teff_' pn '/1'],'autorouting','on');
        add_line(ss,'BeltVelocity/1',['Teff_' pn '/2'],'autorouting','on');

        % T_tight = T_static + T_eff/2
        blk_tt = [ss '/Ttight_' pn];
        add_block('simulink/Math Operations/Sum', blk_tt);
        set_param(blk_tt,'Inputs','++','Position',[450 (k-1)*80+100 500 (k-1)*80+130]);

        blk_half = [ss '/half_' pn];
        add_block('simulink/Math Operations/Gain', blk_half);
        set_param(blk_half,'Gain','0.5','Position',[420 (k-1)*80+100 448 (k-1)*80+130]);
        add_line(ss,['Teff_' pn '/1'],['half_' pn '/1'],'autorouting','on');
        add_line(ss,['half_' pn '/1'],['Ttight_' pn '/1'],'autorouting','on');

        blk_cts = [ss '/Tstatic_' pn];
        add_block('simulink/Sources/Constant', blk_cts);
        set_param(blk_cts,'Value',num2str(T_static),'Position',[420 (k-1)*80+140 448 (k-1)*80+160]);
        add_line(ss,['Tstatic_' pn '/1'],['Ttight_' pn '/2'],'autorouting','on');

        % Slip Safety Factor  SF = ln(Ttight/Tslack) / (mu*theta)
        mu_theta = mu * wrap_deg(k) * pi/180;
        blk_sf = [ss '/SF_' pn];
        add_block('simulink/Math Operations/Fcn', blk_sf);
        set_param(blk_sf,'Position',[560 (k-1)*80+100 700 (k-1)*80+130]);
        set_param(blk_sf,'Expr',...
            sprintf('log(max(u(1),1)/max((%g - u(1)/2),0.001)) / %g', T_static, mu_theta));
        add_line(ss,['Ttight_' pn '/1'],['SF_' pn '/1'],'autorouting','on');

        % Outports: T_tight and SF
        p_out_T  = [ss '/Tout_' pn];
        p_out_SF = [ss '/SFout_' pn];
        add_block('simulink/Sinks/Out1', p_out_T);
        add_block('simulink/Sinks/Out1', p_out_SF);
        set_param(p_out_T, 'Port', num2str(k),    'Position',[750 (k-1)*80+100 800 (k-1)*80+130]);
        set_param(p_out_SF,'Port', num2str(k+6),  'Position',[750 (k-1)*80+140 800 (k-1)*80+160]);
        add_line(ss,['Ttight_' pn '/1'],['Tout_' pn '/1'],'autorouting','on');
        add_line(ss,['SF_' pn '/1'],['SFout_' pn '/1'],'autorouting','on');
    end

    close_system(ss_name);
end

function build_tensioner_subsystem(mdl, ten, sim)
%BUILD_TENSIONER_SUBSYSTEM  Spring-damper tensioner with arm rotation model.

    ss_name = [mdl '/Tensioner'];
    add_block('simulink/Ports & Subsystems/Subsystem', ss_name);
    set_param(ss_name,'Position',[1100 100 1300 300]);
    open_system(ss_name);
    ss = ss_name;
    delete_block([ss '/In1']);
    delete_block([ss '/Out1']);

    % Spring-Damper system:  J*theta_ddot + c*theta_dot + k*theta = T_belt
    % State-space:  x = [theta; theta_dot]
    J = ten.Iz;
    k = ten.k_spring * pi/180;   % convert Nm/deg → Nm/rad
    c = 2 * sqrt(k*J) * 0.15;    % 15% damping ratio

    A = [0 1; -k/J -c/J];
    B = [0; 1/J];
    C_out = eye(2);
    D_out = zeros(2,1);

    blk_ss = [ss '/TensionerSS'];
    add_block('simulink/Continuous/State-Space', blk_ss);
    set_param(blk_ss,'Position',[200 80 400 200]);
    set_param(blk_ss,'A',mat2str(A,6),'B',mat2str(B,6),...
        'C',mat2str(C_out,6),'D',mat2str(D_out,6),...
        'InitialCondition',sprintf('[%g; 0]', ten.mean_load/k));

    % Input port: belt torque
    blk_in = [ss '/T_belt'];
    add_block('simulink/Sources/In1', blk_in);
    set_param(blk_in,'Position',[30 120 80 150],'Port','1');
    add_line(ss,'T_belt/1','TensionerSS/1','autorouting','on');

    % Output: arm angle and angular velocity
    blk_out1 = [ss '/theta_arm'];
    blk_out2 = [ss '/omega_arm'];
    add_block('simulink/Sinks/Out1', blk_out1);
    add_block('simulink/Sinks/Out1', blk_out2);
    set_param(blk_out1,'Port','1','Position',[500 80 550 110]);
    set_param(blk_out2,'Port','2','Position',[500 160 550 190]);

    blk_dem1 = [ss '/Demux'];
    add_block('simulink/Signal Routing/Demux', blk_dem1);
    set_param(blk_dem1,'Outputs','2','Position',[440 100 460 200]);
    add_line(ss,'TensionerSS/1','Demux/1','autorouting','on');
    add_line(ss,'Demux/1','theta_arm/1','autorouting','on');
    add_line(ss,'Demux/2','omega_arm/1','autorouting','on');

    close_system(ss_name);
end
